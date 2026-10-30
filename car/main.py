"""
main.py — Autonomous RC Car Entry Point & Perception-Control Coordination
==========================================================================
Owned by integration-agent.

Wires together:
  Camera capture (Picamera2 / VideoCapture) → YOLOv10n (NCNN) + classical CV
  → Zone Scorer + Decision Engine → Motor Controller (serial → Arduino Uno)
  → Headless HTTP Live Stream with Diagnostic HUD overlay (port 8080)

Operating Modes:
  autonomous (default):
    Avoid all obstacles. Persons are treated as obstacles.
    Tier 1 Pi Camera Latch (PI_STOP/PI_GO) > Tier 1.5 Cliff Reflex >
    Tier 2 Ultrasonic Avoidance Reflex > Tier 3 Normal Driving.

  follow:
    Lock onto closest person at startup. Follow them with PID steering.
    Non-target obstacles still trigger Tier 1 PI_STOP.
    If tracker drops confidence → stop, enter re-acquisition.

Reference Specifications:
  arduino/PROTOCOL.md (serial command & telemetry contract)
  arduino/point_to_point_control/point_to_point_control.ino (firmware FSM)
  AGENTS.md (two-layer Pi 5 + Arduino Uno architecture)
"""

import argparse
import http.server
import io
import json
import logging
import math
import os
import signal
import socketserver
import sys
import threading
import time
import urllib.parse
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any

import cv2  
import numpy as np

from car.perception.config import (
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FPS,
    EMERGENCY_STOP_DISTANCE_CM,
    SERIAL_PORT,
    SERIAL_BAUD,
    FOLLOW_CALIBRATION_FRAMES,
    FOLLOW_SIMILARITY_THRESHOLD,
    FOLLOW_TRACK_VERIFY_THRESHOLD,
    FOLLOW_ANTI_CONFUSION_MARGIN,
    FOLLOW_SEARCH_STEP_DEG,
    FOLLOW_SEARCH_TURN_SPEED,
    FOLLOW_SEARCH_MAX_STEPS,
    FOLLOW_SEARCH_INTERVAL_S,
    LANE_ROI_Y_MIN_FRAC,
    LANE_LOOKAHEAD_Y_FRAC,
    LANE_DEFAULT_WIDTH_FRAC,
    LANE_EMA_ALPHA,
    LANE_MAX_JUMP_FRAC,
    LANE_LOST_MAX_FRAMES,
    LANE_CRUISE_SPEED,
    LANE_CURVE_SPEED,
    LANE_STEER_DEADBAND_DEG,
    LANE_MAX_STEER_DEG,
    LANE_COLOR_MODE,
)
from car.perception.detector import Detector
from car.perception.sensors import SensorManager
from car.perception.avoidance import compute_zone_scores, decide_avoidance
from car.perception.tracker import ObjectTracker
from car.perception.lane_detector import ChartLaneDetector, LaneCurve, LaneDetectionResult
from car.control.motor_controller import MotorController
from car.control.navigation import NavigationSystem
from car.control.pid import PID
from car.control.corner_recovery import CornerRecoveryController

# ── Architecture §6 Rule 2 — Bounding-box height thresholds ──────────────────
BBOX_STOP_THRESHOLD = 0.40   # > 40% of frame height → PI_STOP
BBOX_SLOW_THRESHOLD = 0.15   # 15–40% → caution / reduce PWM

# Cruise and slow PWM values (0–255)
CRUISE_SPEED = 180
SLOW_SPEED   = 180

# ── Follow-me speed modulation (bbox height vs frame height) ─────────────────
FOLLOW_TOO_CLOSE    = 0.50   # > 50% → stop / hold
FOLLOW_IDEAL_MIN    = 0.25   # 25–50% → cruise
FOLLOW_GETTING_FAR  = 0.10   # 10–25% → speed up
FOLLOW_CRUISE_SPEED = 185
FOLLOW_FAST_SPEED   = 200
FOLLOW_SLOW_SPEED   = 180

# ── Command Throttling & Cadence ─────────────────────────────────────────────
# Prevents flooding the ATmega328P 115200 baud serial rx buffer.
DRIVE_CMD_DURATION_MS = 300     # Duration in ms sent with DRIVE commands
DRIVE_THROTTLE_INTERVAL_S = 0.18 # Interval between repeated DRIVE commands (~5.5 Hz)

# A TURN command is a complete, timed skid-steer manoeuvre on the Uno, not a
# continuous steering setpoint.  Do not react to every camera frame as though
# it were a servo command: that repeatedly replaces a turn before the chassis
# has moved and produces the familiar left/right "hunting" behaviour.
LANE_STEER_EMA_ALPHA = 0.28
LANE_TURN_MAX_BURST_DEG = 18
LANE_TURN_SETTLE_S = 0.30
LANE_REVERSAL_CONFIRM_FRAMES = 3

# ── Camera Controls & AWB Definitions ────────────────────────────────────────
try:
    from libcamera import controls, Transform
    AWB_MODES = {
        "daylight": controls.AwbModeEnum.Daylight,
        "auto": controls.AwbModeEnum.Auto,
        "indoor": controls.AwbModeEnum.Indoor,
        "cloudy": controls.AwbModeEnum.Cloudy,
        "tungsten": controls.AwbModeEnum.Tungsten,
    }
except Exception:
    Transform = None
    AWB_MODES = {"auto": 0, "indoor": 4, "daylight": 5, "cloudy": 6, "tungsten": 2}


# =============================================================================
# Helper Utilities
# =============================================================================

def is_valid_us(dist: Any) -> bool:
    """Returns True if reading is a valid obstacle reflection (filters out 0.0 noise, 999 timeout, None, booleans, nan, inf)."""
    if dist is None or isinstance(dist, bool) or not isinstance(dist, (int, float)):
        return False
    if math.isnan(dist) or math.isinf(dist):
        return False
    return 1.0 < dist < 850.0


def find_daylight_ref() -> Optional[Path]:
    """Finds daylight reference image path if available."""
    candidates = [
        Path("/home/capstone/daylight.jpg"),
        Path("/home/capstone/model/daylight.jpg"),
        Path.home() / "daylight.jpg",
        Path.cwd() / "daylight.jpg",
        Path(__file__).resolve().parent.parent / "daylight.jpg",
    ]
    for p in candidates:
        if p.exists():
            return p
    return None


def set_awb_mode(cam, mode_name: str) -> bool:
    """Applies camera AWB mode to picamera2 instance."""
    mode_name = mode_name.lower().strip()
    if mode_name not in AWB_MODES:
        return False
    if cam is None:
        return True
    val = AWB_MODES[mode_name]
    try:
        cam.set_controls({"AwbMode": val})
        print(f"☀️ Camera AWB set to: {mode_name.upper()} (matches daylight.jpg)")
        return True
    except Exception as e:
        print(f"⚠️ Failed to set AwbMode on camera: {e}")
        return False


def _bbox_height_frac(bbox, frame_height: int) -> float:
    """Returns bounding box height as a fraction of frame height."""
    x1, y1, x2, y2 = bbox
    return (y2 - y1) / max(frame_height, 1)


def _bbox_center_x(bbox) -> float:
    """Returns horizontal center coordinate of a bounding box."""
    x1, y1, x2, y2 = bbox
    return (x1 + x2) / 2.0


def _check_obstacle_ahead(detections: list, frame_w: int, frame_h: int) -> Tuple[str, Optional[int]]:
    """
    Checks YOLO and pothole detections for obstacles in the forward driving path.
    Rule 1 & Rule 2:
      - Stop if obstacle is in forward path (center 20%-80%) and height > 40% of frame.
      - Slow if obstacle is in forward path and height > 15%.
      - Zone score check: if center zone is blocked and both sides dangerous -> STOP.
    """
    slow_candidate = False
    valid_dets = []

    for det in detections:
        if not isinstance(det, dict):
            continue
        bbox = det.get("bbox")
        if not bbox or len(bbox) != 4:
            continue
        try:
            frac = _bbox_height_frac(bbox, frame_h)
            cx = _bbox_center_x(bbox)
        except (TypeError, ValueError, ZeroDivisionError):
            continue

        valid_dets.append(det)
        in_center_corridor = 0.20 * frame_w <= cx <= 0.80 * frame_w

        if in_center_corridor:
            if frac > BBOX_STOP_THRESHOLD:
                return "STOP", None
            elif frac > BBOX_SLOW_THRESHOLD:
                slow_candidate = True

    if valid_dets:
        scores = compute_zone_scores(valid_dets, frame_w, frame_h)
        decision = decide_avoidance(scores)
        if decision == "STOP":
            return "STOP", None

    if slow_candidate:
        return "SLOW", SLOW_SPEED

    return "CLEAR", None


# =============================================================================
# Command Throttler
# =============================================================================

class CommandThrottler:
    """Throttles serial DRIVE, TURN, and PI_AVOID commands to prevent bus flooding and buffer delays."""

    def __init__(self, throttle_interval_s: float = DRIVE_THROTTLE_INTERVAL_S, drive_duration_ms: int = DRIVE_CMD_DURATION_MS):
        self.throttle_interval = throttle_interval_s
        self.drive_duration_ms = drive_duration_ms
        self.last_drive_time: float = 0.0
        self.last_speed: Optional[int] = None
        self.last_cmd: Optional[str] = None
        self.last_turn_time: float = 0.0
        self.last_turn_angle: Optional[int] = None
        self.last_turn_speed: Optional[int] = None
        self.last_avoid_hint: Optional[str] = None
        self.last_avoid_hint_time: float = 0.0
        self.latch_start_time: float = 0.0

    def should_send_drive(self, target_speed: int) -> bool:
        now = time.monotonic()
        if self.last_cmd != "DRIVE":
            return True
        if self.last_speed != target_speed:
            return True
        if (now - self.last_drive_time) >= self.throttle_interval:
            return True
        return False

    def record_drive(self, speed: int):
        self.last_drive_time = time.monotonic()
        self.last_speed = speed
        self.last_cmd = "DRIVE"

    def should_send_turn(self, target_angle: int, target_speed: int, interval_s: float = 0.12) -> bool:
        now = time.monotonic()
        if self.last_cmd != "TURN":
            return True
        if self.last_turn_speed is not None and self.last_turn_speed != target_speed:
            return True
        if self.last_turn_angle is None or abs(target_angle - self.last_turn_angle) >= 5:
            return True
        if (now - self.last_turn_time) >= interval_s:
            return True
        return False

    def record_turn(self, angle: int, speed: int):
        self.last_turn_time = time.monotonic()
        self.last_turn_angle = angle
        self.last_turn_speed = speed
        self.last_cmd = "TURN"
        self.last_speed = None

    def should_send_avoid_hint(self, hint: str, interval_s: float = 1.0) -> bool:
        now = time.monotonic()
        if self.last_avoid_hint != hint:
            return True
        if (now - self.last_avoid_hint_time) >= interval_s:
            return True
        return False

    def record_avoid_hint(self, hint: str):
        self.last_avoid_hint = hint
        self.last_avoid_hint_time = time.monotonic()

    def record_other(self, cmd: str):
        self.last_cmd = cmd
        self.last_speed = None


class LaneSteeringStabilizer:
    """Convert noisy visual steering estimates into safe skid-steer bursts.

    Lane detection runs at camera rate, while the Arduino executes ``TURN`` as
    a timed action.  This controller smooths the visual estimate, limits each
    correction to a small burst, and requires consecutive opposite-direction
    estimates before permitting a reversal.  It intentionally has no integral
    term: the next post-turn camera frame is the feedback that removes residual
    error.
    """

    def __init__(
        self,
        ema_alpha: float = LANE_STEER_EMA_ALPHA,
        max_turn_deg: int = LANE_TURN_MAX_BURST_DEG,
        settle_s: float = LANE_TURN_SETTLE_S,
        reversal_confirm_frames: int = LANE_REVERSAL_CONFIRM_FRAMES,
    ):
        self.ema_alpha = float(np.clip(ema_alpha, 0.01, 1.0))
        self.max_turn_deg = max(1, int(max_turn_deg))
        self.settle_s = max(0.0, float(settle_s))
        self.reversal_confirm_frames = max(1, int(reversal_confirm_frames))
        self.filtered_angle: Optional[float] = None
        self.last_direction = 0
        self.opposite_frames = 0
        self.turn_settle_until = 0.0

    def reset(self):
        self.filtered_angle = None
        self.last_direction = 0
        self.opposite_frames = 0
        self.turn_settle_until = 0.0

    def is_settling(self, now: Optional[float] = None) -> bool:
        return (time.monotonic() if now is None else now) < self.turn_settle_until

    def mark_turn_sent(self, now: Optional[float] = None):
        now = time.monotonic() if now is None else now
        self.turn_settle_until = now + self.settle_s

    def update(self, raw_angle_deg: float, deadband_deg: float) -> int:
        """Return a bounded turn burst, or zero when a reversal is unconfirmed."""
        if not math.isfinite(raw_angle_deg):
            return 0

        if self.filtered_angle is None:
            self.filtered_angle = float(raw_angle_deg)
        else:
            self.filtered_angle += self.ema_alpha * (float(raw_angle_deg) - self.filtered_angle)

        angle = self.filtered_angle
        if abs(angle) <= deadband_deg:
            self.last_direction = 0
            self.opposite_frames = 0
            return 0

        direction = 1 if angle > 0 else -1
        if self.last_direction and direction != self.last_direction:
            self.opposite_frames += 1
            if self.opposite_frames < self.reversal_confirm_frames:
                return 0
        else:
            self.opposite_frames = 0

        self.last_direction = direction
        self.opposite_frames = 0
        magnitude = min(self.max_turn_deg, max(1, int(round(abs(angle)))))
        return direction * magnitude


# =============================================================================
# System Runtime State
# =============================================================================

class SystemState:
    """Thread-safe runtime state container shared between perception and HTTP server."""

    def __init__(self):
        self.lock = threading.Lock()
        self.motors_enabled: bool = False    # Requirement R2: Initialize disarmed by default
        self.emergency_stop: bool = False    # True = Operator safety emergency stop active
        self.invert_motors: bool = False     # True = TT motor polarity inversion fix
        self.active_action_label: str = "IDLE (Safety Hold)"
        self.current_awb: str = "daylight"
        self.swap_rb: bool = True            # Requirement R2: True = BGR color swap for Pi 5 CSI camera
        self.current_mode: str = "autonomous"
        self.control_mode: str = "autonomous"  # "autonomous" or "manual"
        self.manual_override_obstacles: bool = False  # True = Ignore camera/ultrasonic obstacle stops in manual mode
        self.manual_speed: int = 185           # Manual drive PWM (180, 205, 235)
        self.manual_action: str = "IDLE"       # Current manual remote action
        self.pi_latched: bool = False
        self.fps: float = 0.0
        self.running: bool = True
        self.cam_instance: Any = None
        self.motor: Optional[MotorController] = None
        self.streaming_output: Optional['StreamingOutput'] = None

        # Follow-Me & Re-ID State
        self.follow_state: str = "WAIT_CALIBRATION"
        self.pending_select_point: Optional[Tuple[float, float]] = None
        self.pending_calibrate_center: bool = False
        self.pending_reset_target: bool = False
        self.target_bbox: Optional[Tuple[int, int, int, int]] = None
        self.target_similarity: float = 0.0
        self.calibration_progress: float = 0.0
        self.last_seen_side: str = "CENTER"
        self.non_target_persons: list = []

        # Lane Detection & Following State
        self.lane_color_mode: str = LANE_COLOR_MODE
        self.lane_result: Optional[LaneDetectionResult] = None


# =============================================================================
# Headless HTTP Live Stream with Diagnostic HUD
# =============================================================================

class StreamingOutput(io.BufferedIOBase):
    """Thread-safe MJPEG frame buffer using condition variable notification."""

    def __init__(self):
        self.frame: Optional[bytes] = None
        self.condition = threading.Condition()

    def write(self, buf: bytes):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Autonomous RC Car — Perception & Diagnostic HUD</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=Geist:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500;600;700&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg-app: #080b11;
      --bg-surface: rgba(13, 18, 28, 0.82);
      --bg-surface-elevated: rgba(20, 27, 42, 0.92);
      --border-subtle: rgba(255, 255, 255, 0.08);
      --border-glass: rgba(255, 255, 255, 0.12);
      --border-active: rgba(56, 189, 248, 0.45);

      --accent: #38bdf8;
      --accent-bg: rgba(56, 189, 248, 0.14);
      --accent-border: rgba(56, 189, 248, 0.35);

      --status-emerald: #10b981;
      --status-emerald-bg: rgba(16, 185, 129, 0.14);
      --status-emerald-border: rgba(16, 185, 129, 0.35);

      --status-amber: #f59e0b;
      --status-amber-bg: rgba(245, 158, 11, 0.14);
      --status-amber-border: rgba(245, 158, 11, 0.35);

      --status-rose: #f43f5e;
      --status-rose-bg: rgba(244, 63, 94, 0.18);
      --status-rose-border: rgba(244, 63, 94, 0.40);

      --text-main: #f1f5f9;
      --text-muted: #94a3b8;
      --text-dim: #64748b;

      --font-sans: 'Geist', -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      --font-mono: 'JetBrains Mono', ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;

      --liquid-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.12), 0 12px 32px -8px rgba(0, 0, 0, 0.75);
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    html, body {
      background: var(--bg-app); color: var(--text-main);
      font-family: var(--font-sans);
      width: 100vw; height: 100vh; height: 100dvh;
      overflow: hidden;
      display: flex; flex-direction: column;
      user-select: none;
      -webkit-font-smoothing: antialiased;
    }

    /* Structured Header Dock in Normal Document Flow */
    header#topbar {
      height: 44px; min-height: 44px; width: 100%;
      background: var(--bg-surface);
      backdrop-filter: blur(16px); -webkit-backdrop-filter: blur(16px);
      border-bottom: 1px solid var(--border-subtle);
      display: flex; align-items: center; justify-content: space-between;
      padding: 0 14px; gap: 10px;
      z-index: 100; flex-shrink: 0;
      box-shadow: 0 2px 12px rgba(0,0,0,0.4);
      transition: opacity 0.25s ease;
    }
    .brand-group {
      display: flex; align-items: center; gap: 8px; flex-shrink: 0;
      font-weight: 600; font-size: 0.82rem; letter-spacing: -0.01em; color: var(--text-main);
    }
    .brand-sub {
      font-family: var(--font-mono); font-size: 0.7rem; color: var(--text-dim); font-weight: 500;
      padding: 2px 6px; background: rgba(255, 255, 255, 0.04); border-radius: 6px; border: 1px solid var(--border-subtle);
    }
    .telemetry-bar {
      display: flex; align-items: center; gap: 6px; overflow-x: auto;
      scrollbar-width: none; -ms-overflow-style: none;
    }
    .telemetry-bar::-webkit-scrollbar { display: none; }

    /* Badges */
    .badge {
      display: inline-flex; align-items: center; gap: 5px;
      padding: 3px 8px; border-radius: 8px;
      font-size: 0.72rem; font-weight: 600;
      letter-spacing: -0.01em; white-space: nowrap;
      transition: all 0.2s cubic-bezier(0.16, 1, 0.3, 1);
    }
    .badge .num {
      font-family: var(--font-mono); font-variant-numeric: tabular-nums;
    }
    .badge-mode {
      background: var(--accent-bg); color: var(--accent); border: 1px solid var(--accent-border);
    }
    .badge-daylight {
      background: var(--status-amber-bg); color: var(--status-amber); border: 1px solid var(--status-amber-border);
    }
    .badge-motors-on {
      background: var(--status-emerald-bg); color: var(--status-emerald); border: 1px solid var(--status-emerald-border);
    }
    .badge-motors-off {
      background: var(--status-rose-bg); color: var(--status-rose); border: 1px solid var(--status-rose-border);
    }
    .badge-invert-on {
      background: var(--accent-bg); color: var(--accent); border: 1px solid var(--accent-border);
    }
    .badge-invert-off {
      background: rgba(255, 255, 255, 0.04); color: var(--text-muted); border: 1px solid var(--border-subtle);
    }
    .badge-us {
      background: var(--status-emerald-bg); color: var(--status-emerald); border: 1px solid var(--status-emerald-border);
      font-family: var(--font-mono); font-variant-numeric: tabular-nums;
    }

    /* Central Video Viewport */
    #viewport {
      flex: 1; min-height: 0; width: 100%;
      position: relative;
      display: flex; justify-content: center; align-items: center;
      background: #000; overflow: hidden;
    }
    img#stream {
      max-width: 100%; max-height: 100%; width: auto; height: auto;
      object-fit: contain; display: block;
      cursor: crosshair;
    }

    /* Bottom Controls Dock */
    .controls-dock {
      position: absolute; bottom: 14px; left: 50%; transform: translateX(-50%);
      background: var(--bg-surface);
      backdrop-filter: blur(16px); -webkit-backdrop-filter: blur(16px);
      border: 1px solid var(--border-glass); border-radius: 20px;
      padding: 5px 8px;
      display: flex; align-items: center; gap: 5px;
      box-shadow: var(--liquid-shadow); z-index: 100;
      max-width: calc(100vw - 28px); overflow-x: auto;
      scrollbar-width: none; -ms-overflow-style: none;
      transition: all 0.25s cubic-bezier(0.16, 1, 0.3, 1);
    }
    .controls-dock::-webkit-scrollbar { display: none; }
    .controls-dock.hidden {
      opacity: 0; pointer-events: none; transform: translateX(-50%) translateY(24px);
    }
    .dock-divider {
      width: 1px; height: 20px; background: var(--border-subtle); margin: 0 2px; flex-shrink: 0;
    }

    /* Buttons */
    button {
      background: rgba(255, 255, 255, 0.04); color: var(--text-main);
      border: 1px solid var(--border-subtle);
      padding: 6px 11px; border-radius: 12px;
      font-family: var(--font-sans); font-size: 0.74rem; font-weight: 500;
      letter-spacing: -0.01em; cursor: pointer; white-space: nowrap;
      display: inline-flex; align-items: center; gap: 6px;
      transition: all 0.15s cubic-bezier(0.16, 1, 0.3, 1);
      flex-shrink: 0;
    }
    button:hover {
      background: rgba(255, 255, 255, 0.09); border-color: rgba(255, 255, 255, 0.18);
    }
    button:active {
      transform: scale(0.96) translateY(1px);
    }
    button.btn-active {
      background: var(--accent-bg); border-color: var(--accent-border); color: var(--accent); font-weight: 600;
    }
    button.btn-danger {
      background: var(--status-rose-bg); border-color: var(--status-rose-border); color: #fecdd3;
    }
    button.btn-danger:hover {
      background: rgba(244, 63, 94, 0.26);
    }
    button.btn-success {
      background: var(--status-emerald-bg); border-color: var(--status-emerald-border); color: #a7f3d0;
    }
    button.btn-success:hover {
      background: rgba(16, 185, 129, 0.22);
    }

    /* SVG icon utility */
    .icon {
      width: 14px; height: 14px; flex-shrink: 0;
      stroke-width: 2; stroke-linecap: round; stroke-linejoin: round;
    }

    /* Remote D-Pad Panel */
    .remote-panel {
      position: absolute; bottom: 74px; right: 16px;
      background: var(--bg-surface-elevated);
      backdrop-filter: blur(16px); -webkit-backdrop-filter: blur(16px);
      border: 1px solid var(--border-glass); border-radius: 16px;
      padding: 12px;
      display: flex; flex-direction: column; gap: 8px; z-index: 99;
      box-shadow: var(--liquid-shadow);
      transition: all 0.25s cubic-bezier(0.16, 1, 0.3, 1);
    }
    .remote-panel.hidden {
      opacity: 0; pointer-events: none; transform: translateY(20px) scale(0.96);
    }
    .remote-header {
      display: flex; justify-content: space-between; align-items: center;
      font-size: 0.78rem; font-weight: 600; color: var(--text-main);
    }
    .speed-pills { display: flex; gap: 3px; }
    .btn-speed {
      padding: 3px 8px; font-size: 0.7rem; border-radius: 8px;
      font-family: var(--font-mono); font-variant-numeric: tabular-nums;
      background: rgba(255, 255, 255, 0.04); border: 1px solid var(--border-subtle);
      color: var(--text-muted); cursor: pointer;
    }
    .btn-speed.active {
      background: var(--accent-bg); border-color: var(--accent); color: var(--accent); font-weight: 700;
    }
    .dpad-grid {
      display: grid; grid-template-columns: repeat(3, 44px); grid-template-rows: repeat(3, 44px); gap: 6px;
      justify-content: center;
    }
    .dpad-btn {
      width: 44px; height: 44px; padding: 0; border-radius: 10px;
      background: rgba(255, 255, 255, 0.04); border: 1px solid var(--border-subtle);
      color: var(--text-main);
      display: flex; flex-direction: column; align-items: center; justify-content: center;
      cursor: pointer;
      transition: transform 0.1s cubic-bezier(0.16, 1, 0.3, 1), background 0.12s ease;
    }
    .dpad-btn:hover {
      background: rgba(255, 255, 255, 0.1); border-color: rgba(255, 255, 255, 0.18);
    }
    .dpad-btn:active, .dpad-btn.key-pressed {
      transform: scale(0.92) translateY(1px);
      background: var(--accent-bg); border-color: var(--accent); color: var(--accent);
    }
    .dpad-stop {
      background: var(--status-rose-bg); border-color: var(--status-rose-border); color: #fecdd3;
    }
    .dpad-stop:active, .dpad-stop.key-pressed {
      background: rgba(244, 63, 94, 0.35); border-color: var(--status-rose); color: #fff;
    }
    .key-hint {
      font-family: var(--font-mono); font-size: 0.58rem; color: var(--text-dim); margin-top: 1px;
    }
    .dpad-hint {
      font-size: 0.64rem; color: var(--text-dim); text-align: center;
      font-family: var(--font-mono);
    }

    .shortcut-tip {
      position: absolute; bottom: 6px; right: 14px;
      font-size: 0.66rem; color: var(--text-dim); font-family: var(--font-mono);
      opacity: 0.75; pointer-events: none; z-index: 50;
    }
    .shortcut-tip b { color: var(--text-muted); }

    /* Modal */
    .modal {
      display: none; position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
      background: rgba(4, 7, 13, 0.88); backdrop-filter: blur(8px);
      z-index: 1000; justify-content: center; align-items: center; padding: 20px;
    }
    .modal.show { display: flex; }
    .modal-box {
      background: var(--bg-surface-elevated); border: 1px solid var(--border-glass);
      border-radius: 14px; max-width: 680px; width: 100%; padding: 18px;
      box-shadow: var(--liquid-shadow);
    }
    .modal-box img { width: 100%; border-radius: 8px; margin-top: 12px; display: block; border: 1px solid var(--border-subtle); }
  </style>
</head>
<body>
  <header id="topbar">
    <div class="brand-group">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="M19 17h2c.6 0 1-.4 1-1v-3c0-.9-.7-1.7-1.5-1.9L16 10l-2-4H6L4 10.1C3.2 10.3 2.5 11.1 2.5 12v4c0 .6.4 1 1 1h2"/><circle cx="7" cy="17" r="2"/><circle cx="17" cy="17" r="2"/></svg>
      <span>Autonomous RC Car</span>
      <span class="brand-sub">Pi 5 &harr; Uno</span>
    </div>
    <div class="telemetry-bar">
      <div class="badge badge-mode" id="mode-badge" onclick="toggleControlMode()" style="cursor:pointer" title="Click to toggle Mode (C)">
        <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><rect x="4" y="4" width="16" height="16" rx="2"/><rect x="9" y="9" width="6" height="6"/><path d="M9 1v3M15 1v3M9 20v3M15 20v3M20 9h3M20 15h3M1 9h3M1 15h3"/></svg>
        <span id="mode-badge-text">MODE: AUTONOMOUS</span>
      </div>
      <div class="badge badge-invert-off" id="target-badge" onclick="calibrateTarget()" style="cursor:pointer" title="Click to Calibrate (L)">
        <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><circle cx="12" cy="12" r="10"/><circle cx="12" cy="12" r="6"/><circle cx="12" cy="12" r="2"/></svg>
        <span id="target-badge-text">TARGET: UNCALIBRATED</span>
      </div>
      <div class="badge badge-invert-off" id="override-badge" onclick="toggleOverride()" style="cursor:pointer" title="Toggle Manual Obstacle Override (O)">
        <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/></svg>
        <span id="override-badge-text">OVERRIDE: OFF</span>
      </div>
      <div class="badge badge-daylight" id="swap-rb-badge" onclick="toggleSwapRb()" style="cursor:pointer" title="Toggle Camera RGB/BGR Channel Swap (K)">
        <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="M16 3h5v5M4 20L21 3M21 16v5h-5M15 15l6 6M4 4l5 5"/></svg>
        <span id="swap-rb-badge-text">COLOR: BGR (SWAP ON)</span>
      </div>
      <div class="badge badge-daylight" id="awb-badge">
        <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M4.93 4.93l1.41 1.41M17.66 17.66l1.41 1.41M2 12h2M20 12h2M6.34 17.66l-1.41 1.41M19.07 4.93l-1.41 1.41"/></svg>
        <span id="awb-badge-text">AWB: DAYLIGHT</span>
      </div>
      <div class="badge badge-motors-off" id="motors-badge" onclick="toggleMotors()" style="cursor:pointer" title="Toggle Motor Power (M)">
        <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/></svg>
        <span id="motors-badge-text">MOTORS: BENCH SAFE [OFF]</span>
      </div>
      <div class="badge badge-invert-off" id="invert-badge" onclick="toggleInvertMotors()" style="cursor:pointer" title="Toggle Motor Polarity (I)">
        <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="m17 2 4 4-4 4M3 11V9a4 4 0 0 1 4-4h14M7 22l-4-4 4-4M21 13v2a4 4 0 0 1-4 4H3"/></svg>
        <span id="invert-badge-text">POLARITY: NORMAL</span>
      </div>
      <div class="badge badge-us" id="sonar-badge">
        <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="M2 12h4M18 12h4M8 8a6 6 0 0 1 8 0M6 5a10 10 0 0 1 12 0"/></svg>
        <span id="sonar-badge-text">US: L=--- R=---</span>
      </div>
    </div>
  </header>

  <div id="viewport">
    <img id="stream" src="stream.mjpg" alt="Autonomous Perception Stream">
  </div>

  <div class="controls-dock" id="dock">
    <button id="btn-motors" class="btn-danger" onclick="toggleMotors()">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/></svg>
      <span>Enable Motors (M)</span>
    </button>
    <button id="btn-mode" onclick="toggleControlMode()">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><rect x="4" y="4" width="16" height="16" rx="2"/><rect x="9" y="9" width="6" height="6"/><path d="M9 1v3M15 1v3M9 20v3M15 20v3M20 9h3M20 15h3M1 9h3M1 15h3"/></svg>
      <span>Mode: Toggle (C)</span>
    </button>
    <button id="btn-lane" onclick="setMode('lane')">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="M4 19L8 5M16 5l4 14M12 7v2M12 13v2M12 19v2"/></svg>
      <span>Lane Mode</span>
    </button>
    <button id="btn-lane-color" onclick="cycleLaneColor()" data-color="orange">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><circle cx="13.5" cy="6.5" r=".5" fill="currentColor"/><circle cx="17.5" cy="10.5" r=".5" fill="currentColor"/><circle cx="8.5" cy="7.5" r=".5" fill="currentColor"/><circle cx="6.5" cy="12.5" r=".5" fill="currentColor"/><path d="M12 2C6.5 2 2 6.5 2 12s4.5 10 10 10c.93 0 1.5-.67 1.5-1.5 0-.39-.15-.74-.39-1.01-.24-.26-.37-.61-.37-.99 0-.83.67-1.5 1.5-1.5H16c3.31 0 6-2.69 6-6 0-4.97-4.48-9-10-9z"/></svg>
      <span>Lane: ORANGE</span>
    </button>
    <button id="btn-follow" onclick="setMode('follow')">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><circle cx="12" cy="12" r="10"/><circle cx="12" cy="12" r="6"/><circle cx="12" cy="12" r="2"/></svg>
      <span>Follow Mode</span>
    </button>
    <button id="btn-cal-target" class="btn-active" onclick="calibrateTarget()">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><circle cx="12" cy="12" r="8"/><path d="M12 2v4M12 18v4M2 12h4M18 12h4"/></svg>
      <span>Calibrate Target (L)</span>
    </button>
    <button id="btn-reset-target" onclick="resetTarget()">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="M21.5 2v6h-6M21.34 15.57a10 10 0 1 1-.57-8.38l5.67-5.67"/></svg>
      <span>Reset Target</span>
    </button>
    <button id="btn-override" onclick="toggleOverride()">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/></svg>
      <span>Override Obstacles (O)</span>
    </button>
    <button id="btn-remote" onclick="toggleRemoteWidget()">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><rect x="2" y="6" width="20" height="12" rx="4"/><path d="M6 12h4M8 10v4M15 11h.01M18 13h.01"/></svg>
      <span>D-Pad</span>
    </button>

    <div class="dock-divider"></div>

    <button id="btn-invert" onclick="toggleInvertMotors()">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="m17 2 4 4-4 4M3 11V9a4 4 0 0 1 4-4h14M7 22l-4-4 4-4M21 13v2a4 4 0 0 1-4 4H3"/></svg>
      <span>Polarity: Normal (I)</span>
    </button>
    <button id="btn-swap-rb" class="btn-active" onclick="toggleSwapRb()">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="M16 3h5v5M4 20L21 3M21 16v5h-5M15 15l6 6M4 4l5 5"/></svg>
      <span>Swap R/B: ON (K)</span>
    </button>

    <div class="dock-divider"></div>

    <button id="btn-awb-daylight" data-awb="daylight" class="btn-active" onclick="setAwb('daylight')">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M4.93 4.93l1.41 1.41M17.66 17.66l1.41 1.41M2 12h2M20 12h2M6.34 17.66l-1.41 1.41M19.07 4.93l-1.41 1.41"/></svg>
      <span>Daylight</span>
    </button>
    <button id="btn-awb-auto" data-awb="auto" onclick="setAwb('auto')">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="M21.5 2v6h-6M21.34 15.57a10 10 0 1 1-.57-8.38l5.67-5.67"/></svg>
      <span>Auto</span>
    </button>
    <button id="btn-awb-indoor" data-awb="indoor" onclick="setAwb('indoor')">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="M9 18h6M10 22h4M12 2a7 7 0 0 0-4 12.7V17h8v-2.3A7 7 0 0 0 12 2z"/></svg>
      <span>Indoor</span>
    </button>

    <div class="dock-divider"></div>

    <button id="btn-release" class="btn-active" onclick="releaseLatch()">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="M22 11.08V12a10 10 0 1 1-5.93-9.14"/><polyline points="22 4 12 14.01 9 11.01"/></svg>
      <span>Release Latch (PI_GO)</span>
    </button>
    <button id="btn-estop" class="btn-danger" onclick="emergencyStop()">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><polygon points="7.86 2 16.14 2 22 7.86 22 16.14 16.14 22 7.86 22 2 16.14 2 7.86 7.86 2"/><rect x="9" y="9" width="6" height="6"/></svg>
      <span>E-STOP (Space)</span>
    </button>

    <div class="dock-divider"></div>

    <button id="btn-pulse" onclick="testPulse()">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><polyline points="22 12 18 12 15 21 9 3 6 12 2 12"/></svg>
      <span>Test Pulse</span>
    </button>
    <button id="btn-turn" onclick="testTurn(45)">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="m17 2 4 4-4 4M3 11V9a4 4 0 0 1 4-4h14M7 22l-4-4 4-4M21 13v2a4 4 0 0 1-4 4H3"/></svg>
      <span>Test Turn</span>
    </button>
    <button id="btn-fs" onclick="toggleFullscreen()">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="M8 3H5a2 2 0 0 0-2 2v3M21 8V5a2 2 0 0 0-2-2h-3M3 16v3a2 2 0 0 0 2 2h3M16 21h3a2 2 0 0 0 2-2v-3"/></svg>
      <span>Fullscreen</span>
    </button>
    <button id="btn-ref" onclick="openRef()">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="M23 19a2 2 0 0 1-2 2H3a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h4l2-3h6l2 3h4a2 2 0 0 1 2 2z"/><circle cx="12" cy="13" r="4"/></svg>
      <span>Ref daylight.jpg</span>
    </button>
    <button id="btn-dock-toggle" onclick="toggleDock()" style="padding:6px 9px" title="Toggle Dock (H)">
      <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"/><circle cx="12" cy="12" r="3"/></svg>
    </button>
  </div>

  <!-- Floating Remote Control Widget (D-Pad) -->
  <div class="remote-panel hidden" id="remote-panel">
    <div class="remote-header">
      <span style="display:inline-flex; align-items:center; gap:6px">
        <svg class="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor"><rect x="2" y="6" width="20" height="12" rx="4"/><path d="M6 12h4M8 10v4M15 11h.01M18 13h.01"/></svg>
        Remote Control
      </span>
      <div class="speed-pills">
        <button class="btn-speed active" onclick="setSpeed(180)" id="spd-180">180</button>
        <button class="btn-speed" onclick="setSpeed(205)" id="spd-205">205</button>
        <button class="btn-speed" onclick="setSpeed(235)" id="spd-235">235</button>
      </div>
    </div>
    <div class="dpad-grid">
      <div></div>
      <button class="dpad-btn" id="dpad-up" onmousedown="startDrive('forward')" onmouseup="stopDrive()" onmouseleave="stopDrive()" ontouchstart="startDrive('forward')" ontouchend="stopDrive()">
        <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><polyline points="18 15 12 9 6 15"/></svg>
        <span class="key-hint">W</span>
      </button>
      <div></div>
      <button class="dpad-btn" id="dpad-left" onmousedown="startDrive('left')" onmouseup="stopDrive()" onmouseleave="stopDrive()" ontouchstart="startDrive('left')" ontouchend="stopDrive()">
        <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><polyline points="15 18 9 12 15 6"/></svg>
        <span class="key-hint">A</span>
      </button>
      <button class="dpad-btn dpad-stop" id="dpad-stop" onclick="stopDrive()">
        <svg viewBox="0 0 24 24" width="14" height="14" fill="currentColor"><rect x="5" y="5" width="14" height="14" rx="2"/></svg>
        <span class="key-hint">Space</span>
      </button>
      <button class="dpad-btn" id="dpad-right" onmousedown="startDrive('right')" onmouseup="stopDrive()" onmouseleave="stopDrive()" ontouchstart="startDrive('right')" ontouchend="stopDrive()">
        <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><polyline points="9 18 15 12 9 6"/></svg>
        <span class="key-hint">D</span>
      </button>
      <div></div>
      <button class="dpad-btn" id="dpad-down" onmousedown="startDrive('reverse')" onmouseup="stopDrive()" onmouseleave="stopDrive()" ontouchstart="startDrive('reverse')" ontouchend="stopDrive()">
        <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><polyline points="6 9 12 15 18 9"/></svg>
        <span class="key-hint">S</span>
      </button>
      <div></div>
    </div>
    <div class="dpad-hint">WASD / Arrow Keys &bull; 1-3 Speed</div>
  </div>

  <div class="shortcut-tip"><b>F</b> Fullscreen &bull; <b>M</b> Motors &bull; <b>I</b> Polarity &bull; <b>K</b> Swap R/B &bull; <b>Space</b> E-Stop &bull; <b>D</b> Daylight &bull; <b>H</b> Dock</div>

  <div class="modal" id="modal" onclick="closeRef()">
    <div class="modal-box" onclick="event.stopPropagation()">
      <div style="display:flex; justify-content:space-between; align-items:center; font-weight:600; font-size:0.86rem">
        <span>Reference Image: daylight.jpg</span>
        <span style="cursor:pointer; font-size:1.2rem; line-height:1; color:var(--text-muted)" onclick="closeRef()">&times;</span>
      </div>
      <img src="daylight.jpg" alt="daylight.jpg reference">
    </div>
  </div>

  <script>
    let currentControlMode = 'autonomous';
    let manualOverride = false;
    let currentSpeed = 185;
    let driveInterval = null;
    let activeAction = null;
    const keysDown = new Set();
    window.keysDown = keysDown;

    function syncDpadVisuals(activeDir) {
      document.querySelectorAll('.dpad-btn').forEach(btn => btn.classList.remove('key-pressed'));
      if (activeDir === 'forward') {
        const b = document.getElementById('dpad-up');
        if (b) b.classList.add('key-pressed');
      } else if (activeDir === 'reverse') {
        const b = document.getElementById('dpad-down');
        if (b) b.classList.add('key-pressed');
      } else if (activeDir === 'left') {
        const b = document.getElementById('dpad-left');
        if (b) b.classList.add('key-pressed');
      } else if (activeDir === 'right') {
        const b = document.getElementById('dpad-right');
        if (b) b.classList.add('key-pressed');
      }
    }

    function toggleOverride() {
      fetch('/toggle_override').then(r => r.json()).then(d => {
        updateOverrideBadge(d.override);
      });
    }

    function updateOverrideBadge(enabled) {
      manualOverride = enabled;
      const b = document.getElementById('override-badge-text');
      const container = document.getElementById('override-badge');
      const btn = document.getElementById('btn-override');
      if (enabled) {
        if (b) b.innerText = 'OVERRIDE: ON [UNLOCKED]';
        if (container) container.className = 'badge badge-invert-on';
        if (btn) {
          const span = btn.querySelector('span');
          if (span) span.innerText = 'Override: ON (O)';
          btn.className = 'btn-active';
        }
      } else {
        if (b) b.innerText = 'OVERRIDE: OFF [ACTIVE]';
        if (container) container.className = 'badge badge-invert-off';
        if (btn) {
          const span = btn.querySelector('span');
          if (span) span.innerText = 'Override Obstacles (O)';
          btn.className = '';
        }
      }
    }

    function toggleControlMode() {
      fetch('/toggle_control_mode').then(r => r.json()).then(d => {
        currentControlMode = d.control_mode;
        updateModeBadge(d.control_mode);
      });
    }

    function setMode(mode) {
      fetch('/set_mode?mode=' + mode).then(r => r.json()).then(d => {
        currentControlMode = d.control_mode || d.mode;
        updateModeBadge(currentControlMode);
      });
    }

    function calibrateTarget() {
      fetch('/calibrate_target').then(r => r.json());
    }

    function resetTarget() {
      fetch('/reset_target').then(r => r.json());
    }

    function cycleLaneColor() {
      const colors = ['orange', 'brown', 'green', 'pink'];
      const btn = document.getElementById('btn-lane-color');
      let curr = (btn ? btn.getAttribute('data-color') : '') || 'orange';
      let nextIdx = (colors.indexOf(curr) + 1) % colors.length;
      let nextCol = colors[nextIdx];
      fetch('/set_lane_color?color=' + nextCol).then(r => r.json()).then(d => {
        if (btn && d.color) {
          btn.setAttribute('data-color', d.color);
          const span = btn.querySelector('span');
          if (span) span.innerText = 'Lane: ' + d.color.toUpperCase();
        }
      });
    }

    function updateModeBadge(mode) {
      const bText = document.getElementById('mode-badge-text');
      const badge = document.getElementById('mode-badge');
      const btn = document.getElementById('btn-mode');
      const btnSpan = btn ? btn.querySelector('span') : null;
      const panel = document.getElementById('remote-panel');
      if (mode === 'manual') {
        if (bText) bText.innerText = 'MODE: MANUAL';
        if (badge) badge.className = 'badge badge-motors-on';
        if (btnSpan) btnSpan.innerText = 'Mode: Auto (C)';
        if (btn) btn.className = 'btn-active';
        if (panel) panel.classList.remove('hidden');
      } else if (mode === 'lane') {
        if (bText) bText.innerText = 'MODE: LANE';
        if (badge) badge.className = 'badge badge-motors-on';
        if (btnSpan) btnSpan.innerText = 'Mode: Follow (C)';
        if (btn) btn.className = 'btn-active';
        if (panel) panel.classList.add('hidden');
      } else if (mode === 'follow') {
        if (bText) bText.innerText = 'MODE: FOLLOW';
        if (badge) badge.className = 'badge badge-invert-on';
        if (btnSpan) btnSpan.innerText = 'Mode: Manual (C)';
        if (btn) btn.className = '';
        if (panel) panel.classList.add('hidden');
      } else {
        if (bText) bText.innerText = 'MODE: AUTONOMOUS';
        if (badge) badge.className = 'badge badge-mode';
        if (btnSpan) btnSpan.innerText = 'Mode: Lane (C)';
        if (btn) btn.className = '';
        if (panel) panel.classList.add('hidden');
      }
    }

    window.addEventListener('load', () => {
      const streamImg = document.getElementById('stream');
      if (streamImg) {
        streamImg.addEventListener('click', (e) => {
          const rect = streamImg.getBoundingClientRect();
          const imgAspect = 640.0 / 480.0;
          const elemAspect = rect.width / rect.height;
          let renderedW = rect.width;
          let renderedH = rect.height;
          let offsetX = 0;
          let offsetY = 0;
          if (elemAspect > imgAspect) {
            renderedW = rect.height * imgAspect;
            offsetX = (rect.width - renderedW) / 2.0;
          } else {
            renderedH = rect.width / imgAspect;
            offsetY = (rect.height - renderedH) / 2.0;
          }
          const clickX = e.clientX - rect.left - offsetX;
          const clickY = e.clientY - rect.top - offsetY;
          if (clickX >= 0 && clickX <= renderedW && clickY >= 0 && clickY <= renderedH) {
            const normX = Math.max(0.0, Math.min(1.0, clickX / renderedW));
            const normY = Math.max(0.0, Math.min(1.0, clickY / renderedH));
            fetch('/select_target?x=' + normX.toFixed(4) + '&y=' + normY.toFixed(4)).then(r => r.json());
          }
        });
      }
    });

    function toggleRemoteWidget() {
      const panel = document.getElementById('remote-panel');
      if (panel) panel.classList.toggle('hidden');
    }

    function setSpeed(spd) {
      fetch('/set_speed?speed=' + spd).then(r => r.json()).then(d => {
        currentSpeed = d.manual_speed;
        [180, 205, 235].forEach(s => {
          const b = document.getElementById('spd-' + s);
          if (b) b.className = (s === currentSpeed) ? 'btn-speed active' : 'btn-speed';
        });
      });
    }

    function sendManualDrive(cmd, speed) {
      const spd = (speed !== undefined) ? speed : currentSpeed;
      if (activeAction === cmd) return;
      activeAction = cmd;
      syncDpadVisuals(cmd);
      if (driveInterval) clearInterval(driveInterval);
      fetch('/manual_drive?cmd=' + cmd + '&speed=' + spd);
      driveInterval = setInterval(() => {
        fetch('/manual_drive?cmd=' + cmd + '&speed=' + currentSpeed);
      }, 220);
    }

    function startDrive(action) {
      sendManualDrive(action, currentSpeed);
    }

    function stopDrive() {
      if (driveInterval) {
        clearInterval(driveInterval);
        driveInterval = null;
      }
      activeAction = null;
      syncDpadVisuals(null);
      keysDown.clear();
      fetch('/manual_stop');
    }

    window.sendManualDrive = sendManualDrive;
    window.startDrive = startDrive;
    window.stopDrive = stopDrive;

    function toggleFullscreen() {
      if (!document.fullscreenElement) {
        document.documentElement.requestFullscreen().catch(() => {});
      } else {
        document.exitFullscreen().catch(() => {});
      }
    }

    function toggleMotors() {
      fetch('/toggle_motors').then(r => r.json()).then(d => {
        updateMotorsBadge(d.motors_enabled);
      });
    }

    function updateMotorsBadge(enabled) {
      const btn = document.getElementById('btn-motors');
      const btnSpan = btn ? btn.querySelector('span') : null;
      const bText = document.getElementById('motors-badge-text');
      const badge = document.getElementById('motors-badge');
      if (enabled) {
        if (btnSpan) btnSpan.innerText = 'Disable Motors (M)';
        if (btn) btn.className = 'btn-success';
        if (bText) bText.innerText = 'MOTORS: ACTIVE [ON]';
        if (badge) badge.className = 'badge badge-motors-on';
      } else {
        if (btnSpan) btnSpan.innerText = 'Enable Motors (M)';
        if (btn) btn.className = 'btn-danger';
        if (bText) bText.innerText = 'MOTORS: BENCH SAFE [OFF]';
        if (badge) badge.className = 'badge badge-motors-off';
      }
    }

    function toggleInvertMotors() {
      fetch('/toggle_invert_motors').then(r => r.json()).then(d => {
        updateInvertBadge(d.invert_motors);
      });
    }

    function updateInvertBadge(inverted) {
      const btn = document.getElementById('btn-invert');
      const btnSpan = btn ? btn.querySelector('span') : null;
      const bText = document.getElementById('invert-badge-text');
      const badge = document.getElementById('invert-badge');
      if (inverted) {
        if (btnSpan) btnSpan.innerText = 'Polarity: Inverted (I)';
        if (btn) btn.className = 'btn-active';
        if (bText) bText.innerText = 'POLARITY: INVERTED (FWD FIX)';
        if (badge) badge.className = 'badge badge-invert-on';
      } else {
        if (btnSpan) btnSpan.innerText = 'Polarity: Normal (I)';
        if (btn) btn.className = '';
        if (bText) bText.innerText = 'POLARITY: NORMAL';
        if (badge) badge.className = 'badge badge-invert-off';
      }
    }

    function toggleSwapRb() {
      fetch('/toggle_swap_rb').then(r => r.json()).then(d => {
        updateSwapRbBadge(d.swap_rb);
      });
    }

    function updateSwapRbBadge(enabled) {
      const bText = document.getElementById('swap-rb-badge-text');
      const badge = document.getElementById('swap-rb-badge');
      const btn = document.getElementById('btn-swap-rb');
      const btnSpan = btn ? btn.querySelector('span') : null;
      if (enabled) {
        if (bText) bText.innerText = 'COLOR: BGR (SWAP ON)';
        if (badge) badge.className = 'badge badge-daylight';
        if (btnSpan) btnSpan.innerText = 'Swap R/B: ON (K)';
        if (btn) btn.className = 'btn-active';
      } else {
        if (bText) bText.innerText = 'COLOR: RAW RGB';
        if (badge) badge.className = 'badge badge-invert-off';
        if (btnSpan) btnSpan.innerText = 'Swap R/B: OFF (K)';
        if (btn) btn.className = '';
      }
    }

    function emergencyStop() {
      fetch('/emergency_stop').then(r => r.json()).then(d => {
        updateMotorsBadge(false);
      });
    }

    function releaseLatch() {
      fetch('/release_latch').then(r => r.json());
    }

    function testPulse() {
      fetch('/test_drive').then(r => r.json());
    }

    function testTurn(angle = 45) {
      fetch('/test_turn?angle=' + angle).then(r => r.json());
    }

    function setAwb(mode) {
      fetch('/set_awb?mode=' + mode).then(r => r.json()).then(d => {
        if (d.status === 'ok') {
          const bText = document.getElementById('awb-badge-text');
          if (bText) bText.innerText = 'AWB: ' + d.mode.toUpperCase();
          document.querySelectorAll('[data-awb]').forEach(b => {
            b.classList.toggle('btn-active', b.getAttribute('data-awb') === d.mode);
          });
        }
      });
    }

    function toggleDock() {
      const dock = document.getElementById('dock');
      const topbar = document.getElementById('topbar');
      if (dock) {
        dock.classList.toggle('hidden');
        if (topbar) topbar.style.opacity = dock.classList.contains('hidden') ? '0' : '1';
      }
    }

    function openRef() {
      const m = document.getElementById('modal');
      if (m) m.classList.add('show');
    }
    function closeRef() {
      const m = document.getElementById('modal');
      if (m) m.classList.remove('show');
    }

    function pollTelemetry() {
      fetch('/telemetry').then(r => r.json()).then(d => {
        const sb = document.getElementById('sonar-badge');
        const sbText = document.getElementById('sonar-badge-text');
        if (sb && sbText && d.sonar) {
          const vl = typeof d.sonar.left === 'number' && d.sonar.left > 1.0 && d.sonar.left < 850.0;
          const vr = typeof d.sonar.right === 'number' && d.sonar.right > 1.0 && d.sonar.right < 850.0;
          const l = vl ? d.sonar.left.toFixed(0) + 'cm' : 'CLR';
          const r = vr ? d.sonar.right.toFixed(0) + 'cm' : 'CLR';
          let usText = `US: L=${l}  R=${r}`;
          if (typeof d.sonar.rear === 'number' && d.sonar.rear > 1.0 && d.sonar.rear < 850.0) {
            usText += `  B=${d.sonar.rear.toFixed(0)}cm`;
          }
          sbText.innerText = usText;
          if ((vl && d.sonar.left < 25.0) || (vr && d.sonar.right < 25.0)) {
            sb.style.background = 'var(--status-rose-bg)';
            sb.style.borderColor = 'var(--status-rose-border)';
            sb.style.color = '#fecdd3';
          } else if ((vl && d.sonar.left < 45.0) || (vr && d.sonar.right < 45.0)) {
            sb.style.background = 'var(--status-amber-bg)';
            sb.style.borderColor = 'var(--status-amber-border)';
            sb.style.color = 'var(--status-amber)';
          } else {
            sb.style.background = 'var(--status-emerald-bg)';
            sb.style.borderColor = 'var(--status-emerald-border)';
            sb.style.color = 'var(--status-emerald)';
          }
        }
        if (d.lane_color) {
          const btn = document.getElementById('btn-lane-color');
          if (btn && btn.getAttribute('data-color') !== d.lane_color) {
            btn.setAttribute('data-color', d.lane_color);
            const span = btn.querySelector('span');
            if (span) span.innerText = 'Lane: ' + d.lane_color.toUpperCase();
          }
        }
        if (d.follow_state) {
          const tb = document.getElementById('target-badge');
          const tbText = document.getElementById('target-badge-text');
          if (tb && tbText) {
            if (d.follow_state === 'TRACKING') {
              tbText.innerText = `LOCKED (${(d.target_similarity || 0).toFixed(2)}) [${d.last_seen_side || 'CTR'}]`;
              tb.className = 'badge badge-motors-on';
            } else if (d.follow_state === 'CALIBRATING') {
              tbText.innerText = `CALIBRATING (${Math.round((d.calibration_progress || 0) * 100)}%)`;
              tb.className = 'badge badge-daylight';
            } else if (d.follow_state === 'RE_ACQUIRE') {
              tbText.innerText = `SEARCHING [${d.last_seen_side || 'CTR'}]`;
              tb.className = 'badge badge-mode';
            } else {
              tbText.innerText = 'TARGET: CLICK TO SELECT';
              tb.className = 'badge badge-invert-off';
            }
          }
        }
        if (d.control_mode) {
          currentControlMode = d.control_mode;
          updateModeBadge(d.control_mode);
        } else if (d.mode) {
          const bText = document.getElementById('mode-badge-text');
          if (bText) bText.innerText = 'MODE: ' + d.mode.toUpperCase();
        }
        if (d.manual_override_obstacles !== undefined) updateOverrideBadge(d.manual_override_obstacles);
        if (d.motors_enabled !== undefined) updateMotorsBadge(d.motors_enabled);
        if (d.invert_motors !== undefined) updateInvertBadge(d.invert_motors);
        if (d.swap_rb !== undefined) updateSwapRbBadge(d.swap_rb);
      }).catch(() => {});
    }
    setInterval(pollTelemetry, 300);

    const MOVEMENT_KEYS = {
      'KeyW': 'forward',
      'ArrowUp': 'forward',
      'KeyS': 'reverse',
      'ArrowDown': 'reverse',
      'KeyA': 'left',
      'ArrowLeft': 'left',
      'KeyD': 'right',
      'ArrowRight': 'right',
      'w': 'forward',
      's': 'reverse',
      'a': 'left',
      'd': 'right'
    };

    function getMovementKey(e) {
      if (!e) return null;
      if (e.code && MOVEMENT_KEYS[e.code]) return e.code;
      if (e.key && MOVEMENT_KEYS[e.key]) return e.key;
      if (e.key && MOVEMENT_KEYS[e.key.toLowerCase()]) return e.key.toLowerCase();
      return null;
    }

    function resolveActiveMovement() {
      const keys = Array.from(keysDown);
      for (let i = keys.length - 1; i >= 0; i--) {
        const cmd = MOVEMENT_KEYS[keys[i]];
        if (cmd) return cmd;
      }
      return null;
    }

    function removeMovementKey(e) {
      const mk = getMovementKey(e);
      if (mk) keysDown.delete(mk);
      if (e.code) keysDown.delete(e.code);
      if (e.key) {
        keysDown.delete(e.key);
        keysDown.delete(e.key.toLowerCase());
      }
    }

    window.addEventListener('keydown', (e) => {
      if (e.repeat) return;
      const k = e.key ? e.key.toLowerCase() : '';
      if (k === 'f') toggleFullscreen();
      if (k === 'm') toggleMotors();
      if (k === 'i') toggleInvertMotors();
      if (k === 'k') toggleSwapRb();
      if (k === 'c') toggleControlMode();
      if (k === 'l') calibrateTarget();
      if (k === 'o') toggleOverride();
      if (k === 'r') toggleRemoteWidget();
      if (k === 't') testPulse();
      if (k === ' ') {
        e.preventDefault();
        emergencyStop();
        keysDown.clear();
        stopDrive();
      }
      if (k === 'h') toggleDock();

      // Disarm Daylight AWB shortcut when in manual drive so 'D' turns right without toggling camera white balance
      if (k === 'd' && currentControlMode !== 'manual' && typeof currentControlMode !== 'undefined') {
        setAwb('daylight');
      }

      const isManual = (currentControlMode === 'manual' || typeof currentControlMode === 'undefined');
      if (isManual) {
        if (k === '1') setSpeed(180);
        if (k === '2') setSpeed(205);
        if (k === '3') setSpeed(235);

        const moveKey = getMovementKey(e);
        if (moveKey) {
          e.preventDefault();
          if (e.repeat) return;
          keysDown.delete(moveKey);
          keysDown.add(moveKey);
          const cmd = resolveActiveMovement();
          if (cmd) {
            sendManualDrive(cmd, currentSpeed);
          }
        }
      }
    });

    window.addEventListener('keyup', (e) => {
      const moveKey = getMovementKey(e);
      if (moveKey || (e.code && keysDown.has(e.code)) || (e.key && (keysDown.has(e.key) || keysDown.has(e.key.toLowerCase())))) {
        removeMovementKey(e);
        if (keysDown.size === 0) {
          stopDrive();
        } else {
          const nextCmd = resolveActiveMovement();
          if (nextCmd) {
            sendManualDrive(nextCmd, currentSpeed);
          } else {
            stopDrive();
          }
        }
      }
    });

    window.addEventListener('blur', () => {
      if (keysDown.size > 0) {
        keysDown.clear();
        stopDrive();
      }
    });
  </script>
</body>
</html>
"""


class DiagnosticHandler(http.server.BaseHTTPRequestHandler):
    """HTTP request handler for video stream and diagnostic controls."""

    server: 'StreamingServer'

    def log_message(self, format, *args):
        # Suppress standard logging to prevent terminal flooding
        return

    def _handle_telemetry(self):
        sys_state: SystemState = self.server.sys_state
        sonar = sys_state.motor.latest_sonar.copy() if sys_state.motor else {"left": 999.0, "right": 999.0}
        state_str = sys_state.motor.latest_state if sys_state.motor else "UNKNOWN"
        port_name = sys_state.motor.port if sys_state.motor else "DISCONNECTED"
        connected = sys_state.motor.is_connected() if sys_state.motor else False
        with sys_state.lock:
            motors_enabled = sys_state.motors_enabled
            invert_motors = sys_state.invert_motors
            action = sys_state.active_action_label
            pi_latched = sys_state.pi_latched
            mode = sys_state.current_mode
            control_mode = sys_state.control_mode
            manual_override_obstacles = sys_state.manual_override_obstacles
            manual_speed = sys_state.manual_speed
            manual_action = sys_state.manual_action
            awb = sys_state.current_awb
            swap_rb = sys_state.swap_rb
            fps = round(sys_state.fps, 1)
            follow_state = sys_state.follow_state
            target_bbox = sys_state.target_bbox
            target_similarity = round(sys_state.target_similarity, 2)
            calibration_progress = round(sys_state.calibration_progress, 2)
            last_seen_side = sys_state.last_seen_side
            lane_res_snap = sys_state.lane_result
            lane_status = lane_res_snap.status if lane_res_snap else "OFF"
            lane_offset = round(lane_res_snap.lateral_offset_pct, 1) if lane_res_snap else 0.0
            lane_steer = round(lane_res_snap.steer_angle_deg, 1) if lane_res_snap else 0.0
            lane_color = sys_state.lane_color_mode
        data = {
            "sonar": sonar,
            "state": state_str,
            "port": port_name,
            "connected": connected,
            "motors_enabled": motors_enabled,
            "invert_motors": invert_motors,
            "action": action,
            "pi_latched": pi_latched,
            "mode": mode,
            "control_mode": control_mode,
            "manual_override_obstacles": manual_override_obstacles,
            "manual_speed": manual_speed,
            "manual_action": manual_action,
            "awb": awb,
            "swap_rb": swap_rb,
            "fps": fps,
            "follow_state": follow_state,
            "target_bbox": target_bbox,
            "target_similarity": target_similarity,
            "calibration_progress": calibration_progress,
            "last_seen_side": last_seen_side,
            "lane_status": lane_status,
            "lane_offset": lane_offset,
            "lane_steer": lane_steer,
            "lane_color": lane_color,
        }
        resp = json.dumps(data).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(resp)))
        self.end_headers()
        self.wfile.write(resp)

    def _handle_toggle_swap_rb(self):
        sys_state: SystemState = self.server.sys_state
        with sys_state.lock:
            sys_state.swap_rb = not sys_state.swap_rb
            swapped = sys_state.swap_rb
        print(f"🎨 Camera color filter toggled: {'SWAP RGB<->BGR (ON)' if swapped else 'RAW RGB (OFF)'}")
        resp = json.dumps({"status": "ok", "swap_rb": swapped}).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(resp)))
        self.end_headers()
        self.wfile.write(resp)

    def _handle_set_swap_rb(self, query):
        sys_state: SystemState = self.server.sys_state
        val_str = query.get('enabled', query.get('enable', ['true']))[0].lower().strip()
        enabled = val_str in ('true', '1', 'yes', 'on')
        with sys_state.lock:
            sys_state.swap_rb = enabled
            swapped = sys_state.swap_rb
        print(f"🎨 Camera color filter set: {'SWAP RGB<->BGR (ON)' if swapped else 'RAW RGB (OFF)'}")
        resp = json.dumps({"status": "ok", "swap_rb": swapped}).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(resp)))
        self.end_headers()
        self.wfile.write(resp)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        query = urllib.parse.parse_qs(parsed.query)
        sys_state: SystemState = self.server.sys_state

        if path in ('/', '/index.html'):
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        elif path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                out = sys_state.streaming_output
                while sys_state.running and out is not None:
                    with out.condition:
                        out.condition.wait(timeout=1.0)
                        frame_bytes = out.frame
                    if frame_bytes is None:
                        continue
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(frame_bytes)))
                    self.end_headers()
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b'\r\n')
                    try:
                        self.wfile.flush()
                    except Exception:
                        pass
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as e:
                logging.warning("Stream client disconnected: %s", e)

        elif path == '/telemetry':
            self._handle_telemetry()

        elif path == '/toggle_swap_rb':
            self._handle_toggle_swap_rb()

        elif path == '/set_swap_rb':
            self._handle_set_swap_rb(query)

        elif path == '/toggle_override':
            with sys_state.lock:
                sys_state.manual_override_obstacles = not sys_state.manual_override_obstacles
                enabled = sys_state.manual_override_obstacles
            print(f"🔓 Manual obstacle override: {'ENABLED (Driver Full Authority)' if enabled else 'DISABLED (Safety Auto-Brake)'}")
            resp = json.dumps({"status": "ok", "override": enabled}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/toggle_motors':
            with sys_state.lock:
                sys_state.motors_enabled = not sys_state.motors_enabled
                enabled = sys_state.motors_enabled
                if enabled:
                    sys_state.emergency_stop = False
            if not enabled and sys_state.motor:
                sys_state.motor.stop()
            print(f"🚗 Motors toggled: {'ENABLED' if enabled else 'BENCH SAFE (DISABLED)'}")
            resp = json.dumps({"status": "ok", "motors_enabled": enabled}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/toggle_invert_motors':
            with sys_state.lock:
                sys_state.invert_motors = not sys_state.invert_motors
                inverted = sys_state.invert_motors
            if sys_state.motor:
                sys_state.motor.invert_motors = inverted
            print(f"🔄 Motor polarity toggled: {'INVERTED' if inverted else 'NORMAL'}")
            resp = json.dumps({"status": "ok", "invert_motors": inverted}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/emergency_stop':
            with sys_state.lock:
                sys_state.motors_enabled = False
                sys_state.emergency_stop = True
                sys_state.pi_latched = True
            if sys_state.motor:
                sys_state.motor.stop()
                sys_state.motor.pi_stop()
            print("🛑 Emergency Stop triggered via Web UI!")
            resp = json.dumps({"status": "ok", "motors_enabled": False, "pi_latched": True, "emergency_stop": True}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/release_latch':
            with sys_state.lock:
                sys_state.pi_latched = False
                sys_state.emergency_stop = False
            if sys_state.motor:
                sys_state.motor.pi_go()
            print("✅ Latch release (PI_GO) triggered via Web UI!")
            resp = json.dumps({"status": "ok", "pi_latched": False}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/test_drive':
            executed = False
            with sys_state.lock:
                can_drive = sys_state.motors_enabled and not sys_state.pi_latched
            if can_drive and sys_state.motor:
                sys_state.motor.drive(CRUISE_SPEED, 300)
                executed = True
            resp = json.dumps({"status": "ok", "executed": executed}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/test_turn':
            try:
                angle = int(query.get('angle', [45])[0])
            except (ValueError, TypeError):
                angle = 45
            executed = False
            with sys_state.lock:
                can_drive = sys_state.motors_enabled and not sys_state.pi_latched
            if can_drive and sys_state.motor:
                sys_state.motor.turn(angle, speed=180)
                executed = True
            resp = json.dumps({"status": "ok", "executed": executed, "angle": angle}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/toggle_control_mode':
            send_pi_go = False
            with sys_state.lock:
                # Cycle: autonomous -> lane -> follow -> manual -> autonomous
                if sys_state.control_mode == "autonomous":
                    sys_state.control_mode = "lane"
                    sys_state.current_mode = "lane"
                elif sys_state.control_mode == "lane":
                    sys_state.control_mode = "follow"
                    sys_state.current_mode = "follow"
                    if sys_state.follow_state not in ("TRACKING", "CALIBRATING"):
                        sys_state.follow_state = "WAIT_CALIBRATION"
                elif sys_state.control_mode == "follow":
                    sys_state.control_mode = "manual"
                    sys_state.current_mode = "manual"
                    if not sys_state.emergency_stop:
                        sys_state.pi_latched = False
                        send_pi_go = True
                else:
                    sys_state.control_mode = "autonomous"
                    sys_state.current_mode = "autonomous"
                mode_now = sys_state.control_mode
            if mode_now == "manual" and sys_state.motor:
                if send_pi_go:
                    sys_state.motor.pi_go()
                sys_state.motor.stop()
            print(f"🎮 Control mode toggled: {mode_now.upper()}")
            resp = json.dumps({"status": "ok", "control_mode": mode_now, "mode": mode_now}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/set_mode':
            new_mode = query.get('mode', ['autonomous'])[0].lower().strip()
            send_pi_go = False
            with sys_state.lock:
                if new_mode == "manual":
                    sys_state.control_mode = "manual"
                    sys_state.current_mode = "manual"
                    if not sys_state.emergency_stop:
                        sys_state.pi_latched = False
                        send_pi_go = True
                elif new_mode in ("autonomous", "lane", "follow"):
                    sys_state.control_mode = new_mode
                    sys_state.current_mode = new_mode
                    if new_mode == "follow" and sys_state.follow_state not in ("TRACKING", "CALIBRATING"):
                        sys_state.follow_state = "WAIT_CALIBRATION"
                mode_now = sys_state.control_mode
            if mode_now == "manual" and sys_state.motor:
                if send_pi_go:
                    sys_state.motor.pi_go()
                sys_state.motor.stop()
            print(f"🔄 Mode switched to: {mode_now.upper()}")
            resp = json.dumps({"status": "ok", "mode": mode_now, "control_mode": mode_now}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/set_lane_color':
            color = query.get('color', ['orange'])[0].lower().strip()
            valid_colors = {"orange", "brown", "green", "pink", "dark", "yellow", "blue", "white"}
            if color in valid_colors:
                with sys_state.lock:
                    sys_state.lane_color_mode = color
                    if hasattr(self.server, "lane_detector") and self.server.lane_detector:
                        self.server.lane_detector.set_color_mode(color)
                print(f"🎨 Lane color mode set to: {color.upper()}")
                resp = json.dumps({"status": "ok", "color": color}).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(resp)))
                self.end_headers()
                self.wfile.write(resp)
            else:
                resp = json.dumps({"status": "error", "message": f"Invalid lane color mode '{color}'"}).encode('utf-8')
                self.send_response(400)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(resp)))
                self.end_headers()
                self.wfile.write(resp)

        elif path == '/select_target':
            try:
                nx = float(query.get('x', [0.5])[0])
                ny = float(query.get('y', [0.5])[0])
            except (ValueError, TypeError):
                nx, ny = 0.5, 0.5
            with sys_state.lock:
                sys_state.current_mode = "follow"
                sys_state.control_mode = "follow"
                sys_state.pending_select_point = (nx, ny)
            print(f"🎯 Target select queued at ({nx:.2f}, {ny:.2f})")
            resp = json.dumps({"status": "ok", "point": [nx, ny]}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/calibrate_target':
            with sys_state.lock:
                sys_state.current_mode = "follow"
                sys_state.control_mode = "follow"
                sys_state.pending_calibrate_center = True
            print("🎯 Calibrate target queued")
            resp = json.dumps({"status": "ok"}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/reset_target':
            with sys_state.lock:
                sys_state.pending_reset_target = True
                sys_state.follow_state = "WAIT_CALIBRATION"
                sys_state.target_bbox = None
                sys_state.target_similarity = 0.0
                sys_state.calibration_progress = 0.0
                sys_state.last_seen_side = "CENTER"
            if sys_state.motor:
                sys_state.motor.stop()
            print("🔄 Target profile reset queued")
            resp = json.dumps({"status": "ok"}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/set_speed':
            try:
                spd = int(query.get('speed', [185])[0])
            except (ValueError, TypeError):
                spd = 185
            spd = max(180, min(255, spd))
            with sys_state.lock:
                sys_state.manual_speed = spd
            resp = json.dumps({"status": "ok", "manual_speed": spd}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/manual_drive':
            cmd = query.get('cmd', [''])[0].lower().strip()
            try:
                spd = int(query.get('speed', [sys_state.manual_speed])[0]) if 'speed' in query else sys_state.manual_speed
            except (ValueError, TypeError):
                spd = sys_state.manual_speed
            if 0 < spd < 180:
                spd = 180
            spd = max(0, min(255, spd))
            executed = False
            blocked = False
            reason = "ok"
            unlatch_needed = False

            with sys_state.lock:
                sys_state.control_mode = "manual"
                override = sys_state.manual_override_obstacles
                if sys_state.emergency_stop:
                    can_drive = False
                    reason = "EMERGENCY_STOP"
                elif not sys_state.motors_enabled:
                    can_drive = False
                    reason = "MOTORS_DISABLED"
                else:
                    can_drive = True
                    if sys_state.pi_latched:
                        sys_state.pi_latched = False
                        unlatch_needed = True

            if can_drive and (unlatch_needed or (sys_state.motor and sys_state.motor.latest_state in ("PI_STOPPED", "STATE_PI_STOPPED", "CLIFF_STOPPED", "STATE_CLIFF_STOPPED"))) and sys_state.motor:
                sys_state.motor.pi_go()

            if can_drive and sys_state.motor:
                sonar = sys_state.motor.latest_sonar
                us_l = sonar.get("left", 999.0)
                us_r = sonar.get("right", 999.0)
                us_rear = sonar.get("rear", 999.0)

                if cmd in ("forward", "fwd"):
                    if not override and ((is_valid_us(us_l) and us_l < EMERGENCY_STOP_DISTANCE_CM) or (is_valid_us(us_r) and us_r < EMERGENCY_STOP_DISTANCE_CM)):
                        blocked = True
                        reason = "us_front_blocked"
                        sys_state.motor.stop()
                    else:
                        sys_state.motor.drive(spd, 350)
                        executed = True
                elif cmd in ("reverse", "rev"):
                    if not override and is_valid_us(us_rear) and us_rear < EMERGENCY_STOP_DISTANCE_CM:
                        blocked = True
                        reason = "us_rear_blocked"
                        sys_state.motor.stop()
                    else:
                        sys_state.motor.reverse(spd, 350)
                        executed = True
                elif cmd in ("left", "turn_left"):
                    sys_state.motor.turn(-40, speed=spd)
                    executed = True
                elif cmd in ("right", "turn_right"):
                    sys_state.motor.turn(40, speed=spd)
                    executed = True
                elif cmd in ("stop", "halt"):
                    sys_state.motor.stop()
                    executed = True

            with sys_state.lock:
                sys_state.manual_action = cmd.upper() if executed else ("BLOCKED" if blocked else "IDLE")

            resp = json.dumps({"status": "ok" if executed else "blocked", "cmd": cmd, "executed": executed, "reason": reason}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/manual_stop':
            if sys_state.motor:
                sys_state.motor.stop()
            with sys_state.lock:
                sys_state.manual_action = "STOP"
            resp = json.dumps({"status": "ok", "action": "STOP"}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/set_awb':
            mode = query.get('mode', ['daylight'])[0]
            success = set_awb_mode(sys_state.cam_instance, mode)
            if success:
                sys_state.current_awb = mode
            resp = json.dumps({"status": "ok" if success else "error", "mode": sys_state.current_awb}).encode('utf-8')
            self.send_response(200 if success else 400)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path in ('/daylight.jpg', '/ref_image'):
            ref_path = find_daylight_ref()
            if ref_path and ref_path.exists():
                data = ref_path.read_bytes()
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_error(404, "daylight.jpg not found on filesystem")
        else:
            self.send_error(404)


class StreamingServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    """Multi-threaded HTTP Server for low-latency streaming."""
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address, RequestHandlerClass, sys_state: SystemState):
        super().__init__(server_address, RequestHandlerClass)
        self.sys_state = sys_state


# =============================================================================
# Diagnostic HUD Overlay Renderer
# =============================================================================

def draw_hud(
    frame: np.ndarray,
    detections: list,
    action_label: str,
    latest_sonar: Optional[dict],
    latest_state: Optional[str],
    port_name: Optional[str],
    fps: float,
    awb_mode: str,
    mode: str,
    motors_enabled: bool,
    invert_motors: bool,
    pi_latched: bool,
    target_bbox: Optional[Tuple[int, int, int, int]] = None,
    target_similarity: float = 0.0,
    calibration_progress: float = 0.0,
    last_seen_side: str = "CENTER",
    follow_state: str = "WAIT_CALIBRATION",
    non_target_persons: Optional[list] = None,
    lane_result: Optional[LaneDetectionResult] = None,
) -> np.ndarray:
    """Renders active bounding boxes, zone markers, and live diagnostic telemetry overlay."""
    if frame is None or getattr(frame, "size", 0) == 0:
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

    annotated = frame.copy()
    h, w = annotated.shape[:2]

    if latest_sonar is None or not isinstance(latest_sonar, dict):
        latest_sonar = {}
    if latest_state is None:
        latest_state = "UNKNOWN"
    if port_name is None:
        port_name = "DISCONNECTED"
    if fps is None or not isinstance(fps, (int, float)):
        fps = 0.0
    action_label = str(action_label or "IDLE")
    mode = str(mode or "autonomous")
    awb_mode = str(awb_mode or "daylight")

    # Dynamic scaling for resolution (supports 320x240, 640x480, etc.)
    is_hd = (w >= 480)
    box_scale  = 0.45 if is_hd else 0.35
    hud_scale1 = 0.55 if is_hd else 0.40
    hud_scale2 = 0.45 if is_hd else 0.32
    thick      = 2 if is_hd else 1

    # ── 1. Zone Divider Markers (0.33 and 0.67 of width) ─────────────────────
    z_left  = int(w * 0.33)
    z_right = int(w * 0.67)
    cv2.line(annotated, (z_left, 0), (z_left, h), (70, 70, 70), 1)
    cv2.line(annotated, (z_right, 0), (z_right, h), (70, 70, 70), 1)
    cv2.putText(annotated, "L", (10, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 100, 100), 1)
    cv2.putText(annotated, "C", (int(w * 0.48), h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 100, 100), 1)
    cv2.putText(annotated, "R", (max(10, w - 20), h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 100, 100), 1)

    # ── 1b. Render Lane Corridor, Polylines & Target Reticle (Lane Mode) ─────
    if mode == "lane" and lane_result is not None:
        if lane_result.corridor_pts is not None and len(lane_result.corridor_pts) >= 4:
            overlay = annotated.copy()
            cv2.fillPoly(overlay, [lane_result.corridor_pts], (40, 180, 50))
            cv2.addWeighted(overlay, 0.35, annotated, 0.65, 0, annotated)

        if lane_result.left_pts is not None and len(lane_result.left_pts) >= 2:
            cv2.polylines(annotated, [lane_result.left_pts], False, (255, 255, 0), thick + 1)

        if lane_result.right_pts is not None and len(lane_result.right_pts) >= 2:
            cv2.polylines(annotated, [lane_result.right_pts], False, (0, 215, 255), thick + 1)

        if lane_result.center_pts is not None and len(lane_result.center_pts) >= 2:
            cv2.polylines(annotated, [lane_result.center_pts], False, (255, 0, 255), thick)

        if lane_result.lookahead_point is not None and (lane_result.status != "LANE_LOST" or lane_result.center_coeffs is not None):
            lx, ly = lane_result.lookahead_point
            cv2.drawMarker(annotated, (lx, ly), (0, 0, 255), cv2.MARKER_CROSS, 14, thick + 1)
            cv2.circle(annotated, (lx, ly), 6, (0, 255, 255), thick)

    # ── 2. Render Detections (YOLO Obstacles, Potholes & Bystanders) ─────────
    non_target_set = set()
    if non_target_persons:
        for ntp in non_target_persons:
            if isinstance(ntp, dict) and ntp.get("bbox"):
                non_target_set.add(tuple(ntp["bbox"]))

    for det in (detections or []):
        if not isinstance(det, dict):
            continue
        bbox = det.get("bbox")
        if not bbox or len(bbox) != 4:
            continue
        try:
            x1, y1, x2, y2 = [int(v) for v in bbox]
        except (ValueError, TypeError):
            continue
        is_pothole = det.get("type") == "negative" or "pothole" in det.get("class_name", "").lower()
        is_bystander = tuple(bbox) in non_target_set or (det.get("class_name") == "person" and non_target_persons and det in non_target_persons)

        if is_bystander:
            color = (0, 140, 255)  # Orange for Bystander Obstacles
            lbl = "BYSTANDER [OBSTACLE]"
        elif is_pothole:
            color = (0, 69, 255)   # Red/Orange for Potholes
            lbl = det.get("class_name", "pothole")
        else:
            color = (0, 255, 0)    # Green for General Obstacles
            lbl = det.get("class_name", "obstacle")

        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thick)
        conf = det.get("confidence", 0.0)
        text = f"{lbl.upper()} {conf:.2f}"
        cv2.putText(annotated, text, (max(0, x1), max(12, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, box_scale, color, thick)

    # ── 3. Render Target Lock Box with Corner Reticles ────────────────────────
    if mode == "follow" and target_bbox is not None and follow_state in ("TRACKING", "CALIBRATING"):
        tx, ty, tw, th = [int(v) for v in target_bbox]
        tx2, ty2 = tx + tw, ty + th
        tgt_color = (0, 215, 255)  # Gold
        cv2.rectangle(annotated, (tx, ty), (tx2, ty2), tgt_color, thick)

        # Reticles on 4 corners
        rlen = max(6, min(22, int(min(tw, th) * 0.22)))
        cv2.line(annotated, (tx, ty), (tx + rlen, ty), (255, 255, 0), thick + 1)
        cv2.line(annotated, (tx, ty), (tx, ty + rlen), (255, 255, 0), thick + 1)
        cv2.line(annotated, (tx2, ty), (tx2 - rlen, ty), (255, 255, 0), thick + 1)
        cv2.line(annotated, (tx2, ty), (tx2, ty + rlen), (255, 255, 0), thick + 1)
        cv2.line(annotated, (tx, ty2), (tx + rlen, ty2), (255, 255, 0), thick + 1)
        cv2.line(annotated, (tx, ty2), (tx, ty2 - rlen), (255, 255, 0), thick + 1)
        cv2.line(annotated, (tx2, ty2), (tx2 - rlen, ty2), (255, 255, 0), thick + 1)
        cv2.line(annotated, (tx2, ty2), (tx2, ty2 - rlen), (255, 255, 0), thick + 1)

        if follow_state == "CALIBRATING":
            bar_w = max(50, tw)
            bar_h = 6
            bx = tx
            by = max(14, ty - 14)
            cv2.rectangle(annotated, (bx, by), (bx + bar_w, by + bar_h), (40, 40, 40), -1)
            fill_w = int(bar_w * max(0.0, min(1.0, calibration_progress)))
            cv2.rectangle(annotated, (bx, by), (bx + fill_w, by + bar_h), (0, 255, 0), -1)
            cal_lbl = f"CALIBRATING ({int(calibration_progress * FOLLOW_CALIBRATION_FRAMES)}/{FOLLOW_CALIBRATION_FRAMES})"
            cv2.putText(annotated, cal_lbl, (bx, max(12, by - 4)), cv2.FONT_HERSHEY_SIMPLEX, box_scale, (0, 255, 255), thick)
        else:
            tgt_lbl = f"TARGET (SIM: {target_similarity:.2f}) [{last_seen_side}]"
            cv2.putText(annotated, tgt_lbl, (max(0, tx), max(12, ty - 6)), cv2.FONT_HERSHEY_SIMPLEX, box_scale, (0, 255, 255), thick)

    # ── 4. Diagnostic HUD Text Overlays ──────────────────────────────────────
    head_str = f"{fps:.1f} FPS | {mode.upper()} | {awb_mode.upper()} AWB"
    cv2.putText(annotated, head_str, (14, 24 if is_hd else 18),
                cv2.FONT_HERSHEY_SIMPLEX, hud_scale1, (0, 255, 255), thick)

    # Action line
    act_color = (0, 255, 0) if "CRUISING" in action_label or "LOCKED" in action_label else (0, 165, 255)
    if "STOP" in action_label or "BRAKE" in action_label or "BLOCKED" in action_label:
        act_color = (0, 0, 255)
    cv2.putText(annotated, f"Action: {action_label}", (14, 48 if is_hd else 34),
                cv2.FONT_HERSHEY_SIMPLEX, hud_scale1, act_color, thick)

    # Directional Search Banner when in RE_ACQUIRE
    if mode == "follow" and follow_state == "RE_ACQUIRE":
        search_str = f"RE_ACQUIRE: SEARCHING {last_seen_side}"
        cv2.putText(annotated, search_str, (14, 70 if is_hd else 50),
                    cv2.FONT_HERSHEY_SIMPLEX, hud_scale1, (0, 165, 255), thick)
    elif mode == "lane" and lane_result is not None:
        if lane_result.status != "LANE_LOST":
            lane_str = f"LANE: {lane_result.status} | Off: {lane_result.lateral_offset_pct:+.1f}% | Str: {lane_result.steer_angle_deg:+.0f} deg"
            cv2.putText(annotated, lane_str, (14, 70 if is_hd else 50),
                        cv2.FONT_HERSHEY_SIMPLEX, hud_scale1, (0, 255, 200), thick)
        elif lane_result.center_coeffs is not None:
            lane_str = f"LANE: COASTING ({lane_result.lost_frames}) | Off: {lane_result.lateral_offset_pct:+.1f}% | Str: {lane_result.steer_angle_deg:+.0f} deg"
            cv2.putText(annotated, lane_str, (14, 70 if is_hd else 50),
                        cv2.FONT_HERSHEY_SIMPLEX, hud_scale1, (0, 165, 255), thick)

    # Ultrasonic telemetry display
    us_l = latest_sonar.get("left", 999.0)
    us_r = latest_sonar.get("right", 999.0)
    us_rear = latest_sonar.get("rear", 999.0)
    v_l = is_valid_us(us_l)
    v_r = is_valid_us(us_r)
    v_rear = is_valid_us(us_rear)
    us_l_disp = f"{us_l:.0f}cm" if v_l else "CLR"
    us_r_disp = f"{us_r:.0f}cm" if v_r else "CLR"
    us_str = f"US: L={us_l_disp} | R={us_r_disp}"
    if v_rear:
        us_str += f" | B={us_rear:.0f}cm"

    us_color = (0, 255, 0)
    if (v_l and us_l < EMERGENCY_STOP_DISTANCE_CM) or (v_r and us_r < EMERGENCY_STOP_DISTANCE_CM):
        us_color = (0, 0, 255)
        us_str += " [BRAKE]"
    elif (v_l and us_l < 42.0) or (v_r and us_r < 42.0):
        us_color = (0, 140, 255)
        us_str += " [AVOID]"
    elif (v_l and us_l < 60.0) or (v_r and us_r < 60.0):
        us_color = (0, 255, 255)
        us_str += " [SLOW]"

    # Bottom status lines
    cv2.putText(annotated, us_str, (14, h - (32 if is_hd else 24)),
                cv2.FONT_HERSHEY_SIMPLEX, hud_scale2, us_color, thick)

    motor_status = "ACTIVE [ON]" if motors_enabled else "BENCH SAFE [OFF]"
    ard_str = f"Arduino: {port_name} [{latest_state}] | Motors: {motor_status} | Pol: {'INV' if invert_motors else 'NORM'}"
    cv2.putText(annotated, ard_str, (14, h - (14 if is_hd else 10)),
                cv2.FONT_HERSHEY_SIMPLEX, hud_scale2, (200, 200, 200), 1)

    if pi_latched:
        cv2.putText(annotated, "[PI_STOP LATCH ACTIVE]", (max(10, w - (230 if is_hd else 160)), 24 if is_hd else 18),
                    cv2.FONT_HERSHEY_SIMPLEX, hud_scale2, (0, 0, 255), thick)

    return annotated


# =============================================================================
# Chart-Paper Lane Following Mode (Option A)
# =============================================================================

def _run_lane(
    motor: MotorController,
    sensors: SensorManager,
    lane_detector: ChartLaneDetector,
    detections: list,
    us_blocked: bool,
    frame: np.ndarray,
    pi_latched: bool,
    motors_enabled: bool,
    throttler: CommandThrottler,
    sys_state: Optional[SystemState] = None,
    lane_steering: Optional[LaneSteeringStabilizer] = None,
) -> Tuple[bool, str, LaneDetectionResult]:
    """
    Executes one loop iteration of Chart-Paper Lane Following mode (Option A).
    Priority Stack:
      Tier 1: Imminent obstacle / pothole camera stop (PI_STOP latch)
      Tier 1.5 & Tier 2: Cliff reflex & Ultrasonic auto-brake / local Arduino FSM
      Tier 2.5: In-lane active obstacle/pothole bypass (steer to clear side)
      Tier 3: Nominal chart-paper lane centering via pure pursuit
    """
    frame_h, frame_w = frame.shape[:2]
    us = sensors.read_ultrasonic()
    arduino_state = (motor.latest_state or "UNKNOWN").upper()

    # Filter valid detections
    valid_detections = [
        d for d in (detections or [])
        if isinstance(d, dict) and d.get("bbox") and len(d.get("bbox")) == 4
    ]

    # Run Lane Detection (excluding bounding boxes of active obstacles & potholes)
    lane_res = lane_detector.detect(frame, exclude_boxes=valid_detections)
    if sys_state:
        sys_state.lane_result = lane_res

    # ── 1. Tier 1: Camera Obstacle Latch (Rule 1 & Rule 2) ───────────────────
    action, slow_speed = _check_obstacle_ahead(valid_detections, frame_w, frame_h)

    if action == "STOP":
        if not pi_latched:
            print("🛑 PI_STOP: Obstacle/Pothole ahead in lane (camera latch engaged)")
            motor.pi_stop()
            pi_latched = True
            throttler.record_other("PI_STOP")
            throttler.latch_start_time = time.monotonic()
            return pi_latched, "🛑 PI_STOP: Obstacle/Pothole Ahead in Lane", lane_res
        else:
            throttler.record_other("PI_STOPPED")
            return pi_latched, "🛑 PI_STOPPED (Lane Obstacle Latch Active)", lane_res

    # ── 2. Camera Latch Release ──────────────────────────────────────────────
    if pi_latched:
        if action == "CLEAR" and not us_blocked:
            print("✅ PI_GO: Lane corridor clear, releasing camera latch.")
            motor.pi_go()
            pi_latched = False
            throttler.latch_start_time = 0.0
            arduino_state = "IDLE"
        else:
            throttler.record_other("PI_STOPPED")
            return pi_latched, "🛑 PI_STOPPED (Lane Latch Active)", lane_res

    # ── 3. Respect Arduino Autonomous Reflex States (Tier 2 & 1.5) ────────────
    if arduino_state in ("AVOIDING", "STATE_AVOIDING", "CLIFF_REVERSING", "STATE_CLIFF_REVERSING"):
        throttler.record_other(arduino_state)
        return pi_latched, f"🔀 ARDUINO {arduino_state} (Yielding)", lane_res

    if arduino_state in ("CLIFF_STOPPED", "STATE_CLIFF_STOPPED"):
        throttler.record_other(arduino_state)
        if action == "CLEAR" and not us_blocked and is_valid_us(us.get("left")) and is_valid_us(us.get("right")):
            print("✅ PI_GO: Cliff cleared, releasing latch.")
            motor.pi_go()
            return False, "✅ PI_GO: Cliff Cleared", lane_res
        return pi_latched, "⚠️ CLIFF_STOPPED (Latched Drop-Off)", lane_res

    if arduino_state in ("PI_STOPPED", "STATE_PI_STOPPED"):
        throttler.record_other(arduino_state)
        if action == "CLEAR" and not us_blocked:
            print("✅ PI_GO: Releasing stale Arduino PI_STOPPED state.")
            motor.pi_go()
            arduino_state = "IDLE"
        else:
            pi_latched = True
            return pi_latched, "🛑 PI_STOPPED (Arduino Latched)", lane_res

    # ── 4. Ultrasonic Critical Stop (< EMERGENCY_STOP_DISTANCE_CM) ───────────
    if us_blocked:
        # Check if one side has clear escape corridor
        valid_l = is_valid_us(us_l_val)
        valid_r = is_valid_us(us_r_val)
        left_clear = not valid_l or us_l_val >= EMERGENCY_STOP_DISTANCE_CM
        right_clear = not valid_r or us_r_val >= EMERGENCY_STOP_DISTANCE_CM

        if left_clear and not right_clear:
            # Right side blocked (<25cm), Left side clear: steer LEFT away from obstacle!
            throttler.record_avoid_hint("LEFT")
            if motors_enabled and throttler.should_send_turn(-35, 200):
                motor.turn(-35, speed=200)
                throttler.record_turn(-35, 200)
            return pi_latched, f"🔀 US CRITICAL ESCAPE: Right Obs ({us_r_val:.0f}cm) -> Steering LEFT"
        elif right_clear and not left_clear:
            # Left side blocked (<25cm), Right side clear: steer RIGHT away from obstacle!
            throttler.record_avoid_hint("RIGHT")
            if motors_enabled and throttler.should_send_turn(35, 200):
                motor.turn(35, speed=200)
                throttler.record_turn(35, 200)
            return pi_latched, f"🔀 US CRITICAL ESCAPE: Left Obs ({us_l_val:.0f}cm) -> Steering RIGHT"
        else:
            throttler.record_other("STOP")
            if motors_enabled:
                motor.stop()
            return pi_latched, f"🛑 US AUTO-BRAKE (L:{us.get('left', 999):.0f}cm R:{us.get('right', 999):.0f}cm)", lane_res

    # ── 5. Tier 2.5: In-Lane Active Obstacle / Pothole Bypass ────────────────
    # Check if there are active obstacles or potholes inside the driving corridor
    scores = compute_zone_scores(valid_detections, frame_w, frame_h, sonar=us)
    avoid_decision = decide_avoidance(scores)
    us_l, us_r = us.get("left", 999.0), us.get("right", 999.0)

    if avoid_decision in ("LEFT", "RIGHT"):
        hint = avoid_decision
        if throttler.should_send_avoid_hint(hint):
            motor.pi_avoid(hint)
            throttler.record_avoid_hint(hint)
        turn_angle = 32 if hint == "RIGHT" else -32
        turn_speed = LANE_CURVE_SPEED
        if motors_enabled and throttler.should_send_turn(turn_angle, turn_speed):
            motor.turn(turn_angle, speed=turn_speed)
            throttler.record_turn(turn_angle, turn_speed)
        return pi_latched, f"🔀 LANE BYPASS: Avoiding Hazard -> Steering {hint}", lane_res

    # Close ultrasonic asymmetric avoidance
    if is_valid_us(us_l) and us_l < 42.0 and (not is_valid_us(us_r) or us_l < us_r - 5.0):
        if throttler.should_send_avoid_hint("RIGHT"):
            motor.pi_avoid("RIGHT")
            throttler.record_avoid_hint("RIGHT")
        if motors_enabled and throttler.should_send_turn(32, LANE_CURVE_SPEED):
            motor.turn(32, speed=LANE_CURVE_SPEED)
            throttler.record_turn(32, LANE_CURVE_SPEED)
        return pi_latched, f"🔀 LANE BYPASS: Left Obstacle ({us_l:.0f}cm) -> Steering RIGHT", lane_res

    if is_valid_us(us_r) and us_r < 42.0 and (not is_valid_us(us_l) or us_r < us_l - 5.0):
        if throttler.should_send_avoid_hint("LEFT"):
            motor.pi_avoid("LEFT")
            throttler.record_avoid_hint("LEFT")
        if motors_enabled and throttler.should_send_turn(-32, LANE_CURVE_SPEED):
            motor.turn(-32, speed=LANE_CURVE_SPEED)
            throttler.record_turn(-32, LANE_CURVE_SPEED)
        return pi_latched, f"🔀 LANE BYPASS: Right Obstacle ({us_r:.0f}cm) -> Steering LEFT", lane_res

    # ── 6. Tier 3: Nominal Chart-Paper Lane Following ────────────────────────
    # If lane is lost and either exceeding coasting limit OR no prior lane lock:
    if lane_res.status == "LANE_LOST" and (lane_res.lost_frames > lane_detector.lost_max_frames or lane_res.center_coeffs is None):
        if motors_enabled:
            motor.stop()
        throttler.record_other("STOP")
        return pi_latched, f"⚠️ LANE LOST ({lane_res.lost_frames}f): Holding Safe Stop", lane_res

    # In production one stabilizer lives for the whole lane-follow run.  The
    # optional fallback preserves the public helper API used by offline tests.
    if lane_steering is None:
        lane_steering = LaneSteeringStabilizer()

    # Either locked onto lane (DUAL_LOCKED, SINGLE_LEFT, SINGLE_RIGHT) or active coasting with valid trajectory
    is_coasting = (lane_res.status == "LANE_LOST")

    # The vision result is intentionally held for a few frames during a brief
    # tape gap.  It is safe to coast straight through that gap, but issuing a
    # stale skid-steer burst can turn the car into the missing boundary.
    if is_coasting:
        lane_steering.update(0.0, lane_detector.steer_deadband_deg)
        target_speed = LANE_CRUISE_SPEED
        if motors_enabled and throttler.should_send_drive(target_speed):
            motor.drive(target_speed, throttler.drive_duration_ms)
            throttler.record_drive(target_speed)
        return (
            pi_latched,
            f"⚠️ LANE COASTING ({lane_res.lost_frames}/{lane_detector.lost_max_frames}): Straight",
            lane_res,
        )

    # Never replace a live Arduino turn; wait for its telemetry (and retain a
    # local settling guard for the serial round trip) before evaluating the
    # next burst.  Camera safety and ultrasonic checks above still preempt it.
    if arduino_state in ("TURNING", "STATE_TURNING") or lane_steering.is_settling():
        return pi_latched, "🛣️ LANE TURN SETTLING: Awaiting Updated Camera Pose", lane_res

    steer_cmd = lane_steering.update(
        lane_res.steer_angle_deg,
        lane_detector.steer_deadband_deg,
    )

    if steer_cmd == 0:
        target_speed = LANE_CRUISE_SPEED
        if motors_enabled:
            if throttler.should_send_drive(target_speed):
                motor.drive(target_speed, throttler.drive_duration_ms)
                throttler.record_drive(target_speed)
        else:
            target_speed = 0
        action_label = f"🛣️ LANE LOCKED ({lane_res.status}): Straight (PWM {LANE_CRUISE_SPEED})"
    else:
        target_speed = LANE_CURVE_SPEED
        if motors_enabled:
            if throttler.should_send_turn(steer_cmd, target_speed):
                motor.turn(steer_cmd, speed=target_speed)
                throttler.record_turn(steer_cmd, target_speed)
                lane_steering.mark_turn_sent()
        steer_dir = "RIGHT" if steer_cmd > 0 else "LEFT"
        action_label = f"🛣️ LANE STEER {steer_dir} ({abs(steer_cmd)}° | PWM {LANE_CURVE_SPEED})"

    return pi_latched, action_label, lane_res


# =============================================================================
# Autonomous Navigation Mode
# =============================================================================

def _run_autonomous(
    motor: MotorController,
    sensors: SensorManager,
    detections: list,
    us_blocked: bool,
    frame: np.ndarray,
    pi_latched: bool,
    motors_enabled: bool,
    throttler: CommandThrottler,
    fresh_inference: bool = True,
    corner_recovery: Optional[CornerRecoveryController] = None,
) -> Tuple[bool, str]:
    """
    One loop iteration of autonomous navigation.
    Priority stack:
      Tier 1   — Pi Camera Latch (PI_STOP / PI_GO)
      Tier 1.5 — Arduino Cliff Reflex (CLIFF_REVERSING / CLIFF_STOPPED)
      Tier 2   — Arduino Ultrasonic Avoidance (AVOIDING)
      Tier 3   — Normal Driving (DRIVE / SLOW / STOP)

    Returns:
      (updated_pi_latched, action_label)
    """
    frame_h, frame_w = frame.shape[:2]
    us = sensors.read_ultrasonic()
    arduino_state = (motor.latest_state or "UNKNOWN").upper()

    valid_detections = [
        d for d in (detections or [])
        if isinstance(d, dict) and d.get("bbox") and len(d.get("bbox")) == 4
    ]

    # ── 1. Check for Imminent Obstacles Ahead (Tier 1 Camera Latch) ───────────
    # Tier 1 camera latch is absolute: overrides everything including mid-avoidance
    # and mid-cliff reverse per PROTOCOL.md.
    action, slow_speed = _check_obstacle_ahead(valid_detections, frame_w, frame_h)

    if action == "STOP":
        if not pi_latched:
            print("🛑 PI_STOP: Imminent obstacle ahead (camera)")
            motor.pi_stop()
            pi_latched = True
            throttler.record_other("PI_STOP")
            throttler.latch_start_time = time.monotonic()
            return pi_latched, "🛑 PI_STOP: Obstacle Imminent (Camera)"
        else:
            # Check if stationary obstacle allows auto-unstick / pivot escape
            now_m = time.monotonic()
            if getattr(throttler, "latch_start_time", 0.0) == 0.0:
                throttler.latch_start_time = now_m
            latch_age = now_m - throttler.latch_start_time
            if latch_age > 1.2 and valid_detections:
                scores = compute_zone_scores(valid_detections, frame_w, frame_h, sonar=us)
                decision = decide_avoidance(scores)
                us_l, us_r = us.get("left", 999.0), us.get("right", 999.0)
                score_l, score_r = scores.get("left", 0.0), scores.get("right", 0.0)

                # Prefer side with substantially lower danger / better clearance
                if (decision == "RIGHT" or score_r < score_l) and (not is_valid_us(us_r) or us_r > us_l + 3.0 or us_r > 25.0):
                    print("🔀 Auto-Unstick: Right side clear, releasing latch and turning right")
                    motor.pi_go()
                    pi_latched = False
                    throttler.latch_start_time = 0.0
                    if motors_enabled:
                        motor.turn(45, speed=200)
                        throttler.record_turn(45, 200)
                    return pi_latched, "🔀 ESCAPE: Pivoting RIGHT around obstacle"
                elif (decision == "LEFT" or score_l < score_r) and (not is_valid_us(us_l) or us_l > us_r + 3.0 or us_l > 25.0):
                    print("🔀 Auto-Unstick: Left side clear, releasing latch and turning left")
                    motor.pi_go()
                    pi_latched = False
                    throttler.latch_start_time = 0.0
                    if motors_enabled:
                        motor.turn(-45, speed=200)
                        throttler.record_turn(-45, 200)
                    return pi_latched, "🔀 ESCAPE: Pivoting LEFT around obstacle"

            throttler.record_other("PI_STOPPED")
            return pi_latched, "🛑 PI_STOPPED (Obstacle Imminent)"

    # ── 2. Handle Existing Camera Latch ───────────────────────────────────────
    if pi_latched:
        if action == "CLEAR" and not us_blocked:
            print("✅ PI_GO: Path clear, releasing camera latch.")
            motor.pi_go()
            pi_latched = False
            throttler.latch_start_time = 0.0
            arduino_state = "IDLE"
        else:
            throttler.record_other("PI_STOPPED")
            return pi_latched, "🛑 PI_STOPPED (Camera Latch Active)"

    # ── 3. Respect Arduino Autonomous Reflex States (Tier 2 & 1.5) ────────────
    if arduino_state in ("AVOIDING", "STATE_AVOIDING", "CLIFF_REVERSING", "STATE_CLIFF_REVERSING"):
        throttler.record_other(arduino_state)
        return pi_latched, f"🔀 ARDUINO {arduino_state} (Yielding)"

    # ── 4. Handle Cliff Latched Stop (Tier 1.5) ───────────────────────────────
    if arduino_state in ("CLIFF_STOPPED", "STATE_CLIFF_STOPPED"):
        throttler.record_other(arduino_state)
        # Release cliff latch only if path is clear on camera AND ground reflection is valid
        if action == "CLEAR" and not us_blocked and is_valid_us(us.get("left")) and is_valid_us(us.get("right")):
            print("✅ PI_GO: Cliff drop-off cleared, releasing latch.")
            motor.pi_go()
            return False, "✅ PI_GO: Cliff Cleared"
        return pi_latched, "⚠️ CLIFF_STOPPED (Latched Drop-Off)"

    # ── 4b. Sync Arduino PI_STOPPED state if unlatched on Pi ─────────────────
    if arduino_state in ("PI_STOPPED", "STATE_PI_STOPPED"):
        throttler.record_other(arduino_state)
        if action == "CLEAR" and not us_blocked:
            print("✅ PI_GO: Releasing stale Arduino PI_STOPPED state.")
            motor.pi_go()
            arduino_state = "IDLE"
        else:
            pi_latched = True
            return pi_latched, "🛑 PI_STOPPED (Arduino Latched)"

    # ── 5. Ultrasonic Critical Stop / Corner Recovery (< EMERGENCY_STOP_DISTANCE_CM) ──
    us_l_val = us.get('left', 999.0)
    us_r_val = us.get('right', 999.0)
    if corner_recovery is not None:
        if corner_recovery.is_recovering or corner_recovery.is_corner_collision(us_l_val, us_r_val):
            recovering, recovery_action = corner_recovery.update(
                us_left=us_l_val,
                us_right=us_r_val,
                motor=motor,
                enabled=motors_enabled,
            )
            if recovering:
                throttler.record_other("CORNER_RECOVERY")
                if not motors_enabled:
                    return pi_latched, f"⏸️ BENCH SAFE: {recovery_action} (Suppressed)"
                return pi_latched, recovery_action

    if us_blocked:
        # Check if one side has clear escape corridor
        valid_l = is_valid_us(us_l_val)
        valid_r = is_valid_us(us_r_val)
        left_clear = not valid_l or us_l_val >= EMERGENCY_STOP_DISTANCE_CM
        right_clear = not valid_r or us_r_val >= EMERGENCY_STOP_DISTANCE_CM

        if left_clear and not right_clear:
            # Right side blocked (<25cm), Left side clear: steer LEFT away from obstacle!
            throttler.record_avoid_hint("LEFT")
            if motors_enabled and throttler.should_send_turn(-35, 200):
                motor.turn(-35, speed=200)
                throttler.record_turn(-35, 200)
            return pi_latched, f"🔀 US CRITICAL ESCAPE: Right Obstacle ({us_r_val:.0f}cm) -> Steering LEFT"
        elif right_clear and not left_clear:
            # Left side blocked (<25cm), Right side clear: steer RIGHT away from obstacle!
            throttler.record_avoid_hint("RIGHT")
            if motors_enabled and throttler.should_send_turn(35, 200):
                motor.turn(35, speed=200)
                throttler.record_turn(35, 200)
            return pi_latched, f"🔀 US CRITICAL ESCAPE: Left Obstacle ({us_l_val:.0f}cm) -> Steering RIGHT"
        else:
            throttler.record_other("STOP")
            if motors_enabled:
                motor.stop()
            return pi_latched, f"🛑 US AUTO-BRAKE (L:{us.get('left', 999):.0f}cm R:{us.get('right', 999):.0f}cm)"

    # ── 6. Active Obstacle Avoidance & Steering ──────────────────────────────
    scores = compute_zone_scores(valid_detections, frame_w, frame_h, sonar=us)
    decision = decide_avoidance(scores)
    us_l, us_r = us.get("left", 999.0), us.get("right", 999.0)

    # 6a. Fused Camera + Ultrasonic Directional Bypass
    if decision in ("LEFT", "RIGHT"):
        if throttler.should_send_avoid_hint(decision):
            motor.pi_avoid(decision)
            throttler.record_avoid_hint(decision)

        # Active skid-steer bypass turn when obstacle is detected ahead
        turn_angle = 35 if decision == "RIGHT" else -35
        turn_speed = 200
        if motors_enabled and throttler.should_send_turn(turn_angle, turn_speed):
            motor.turn(turn_angle, speed=turn_speed)
            throttler.record_turn(turn_angle, turn_speed)
            dir_str = "RIGHT" if decision == "RIGHT" else "LEFT"
            return pi_latched, f"🔀 AVOIDING OBSTACLE: Steering {dir_str} ({abs(turn_angle)}°)"

    # 6b. Close Ultrasonic Asymmetric Avoidance (sensor-only fallback)
    if is_valid_us(us_l) and us_l < 42.0 and (not is_valid_us(us_r) or us_l < us_r - 5.0):
        if throttler.should_send_avoid_hint("RIGHT"):
            motor.pi_avoid("RIGHT")
            throttler.record_avoid_hint("RIGHT")
        if motors_enabled and throttler.should_send_turn(35, 200):
            motor.turn(35, speed=200)
            throttler.record_turn(35, 200)
            return pi_latched, f"🔀 US AVOID: Left Obs ({us_l:.0f}cm) -> Steering RIGHT"

    if is_valid_us(us_r) and us_r < 42.0 and (not is_valid_us(us_l) or us_r < us_l - 5.0):
        if throttler.should_send_avoid_hint("LEFT"):
            motor.pi_avoid("LEFT")
            throttler.record_avoid_hint("LEFT")
        if motors_enabled and throttler.should_send_turn(-35, 200):
            motor.turn(-35, speed=200)
            throttler.record_turn(-35, 200)
            return pi_latched, f"🔀 US AVOID: Right Obs ({us_r:.0f}cm) -> Steering LEFT"

    # ── 7. Forward Driving (Cruising or Cautionary) ───────────────────────────
    in_us_caution = ((is_valid_us(us_l) and us_l < 60.0) or
                     (is_valid_us(us_r) and us_r < 60.0))
    if action == "SLOW" or in_us_caution:
        target_speed = slow_speed if slow_speed is not None else SLOW_SPEED
        action_label = f"🟡 CRUISING CAUTIOUSLY (PWM {target_speed})"
    else:
        target_speed = CRUISE_SPEED
        action_label = f"🟢 CRUISING FORWARD (PWM {target_speed})"

    # Execute drive with throttling and bench-safe check
    if motors_enabled:
        if throttler.should_send_drive(target_speed):
            motor.drive(target_speed, throttler.drive_duration_ms)
            throttler.record_drive(target_speed)
    else:
        action_label = f"⏸️ BENCH SAFE: DRIVE PWM {target_speed} (Suppressed)"

    return pi_latched, action_label


# =============================================================================
# Follow-Me Mode
# =============================================================================

def _run_follow(
    motor: MotorController,
    sensors: SensorManager,
    tracker: ObjectTracker,
    pid: PID,
    detections: list,
    us_blocked: bool,
    frame: np.ndarray,
    follow_state: str,
    pi_latched: bool,
    motors_enabled: bool,
    throttler: CommandThrottler,
    fresh_inference: bool = True,
    sys_state: Optional[SystemState] = None,
) -> Tuple[str, bool, str]:
    """
    One loop iteration of follow-me mode.
    Priority: Tier 1 non-target obstacle stop > PID person steering + active obstacle bypass.

    Returns:
      (updated_follow_state, updated_pi_latched, action_label)
    """
    frame_h, frame_w = frame.shape[:2]
    us = sensors.read_ultrasonic()
    arduino_state = (motor.latest_state or "UNKNOWN").upper()

    # ── Handle Web UI / Pending Triggers ─────────────────────────────────────
    if sys_state and getattr(sys_state, "pending_reset_target", False):
        tracker.reset()
        follow_state = "WAIT_CALIBRATION"
        sys_state.pending_reset_target = False
        sys_state.target_bbox = None
        sys_state.target_similarity = 0.0
        sys_state.calibration_progress = 0.0
        sys_state.last_seen_side = "CENTER"
        sys_state.follow_state = follow_state
        if motors_enabled:
            motor.stop()
        return follow_state, pi_latched, "🔄 TARGET RESET: Waiting for Selection"

    persons = [d for d in (detections or []) if isinstance(d, dict) and d.get("class_name") == "person"]
    non_person_obstacles = [
        d for d in (detections or [])
        if isinstance(d, dict) and d.get("class_name") != "person" and d.get("type") != "negative"
    ]
    pothole_obstacles = [d for d in (detections or []) if isinstance(d, dict) and d.get("type") == "negative"]

    if sys_state and getattr(sys_state, "pending_select_point", None) is not None:
        nx, ny = sys_state.pending_select_point
        sys_state.pending_select_point = None
        px, py = nx * frame_w, ny * frame_h
        best_p = None
        best_dist = float('inf')
        for p in persons:
            x1, y1, x2, y2 = [int(v) for v in p["bbox"]]
            if x1 <= px <= x2 and y1 <= py <= y2:
                best_p = p
                break
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            dist = math.hypot(cx - px, cy - py)
            if dist < best_dist and dist < (0.25 * frame_w):
                best_dist = dist
                best_p = p
        if best_p:
            x1, y1, x2, y2 = [int(v) for v in best_p["bbox"]]
            bbox = (x1, y1, x2 - x1, y2 - y1)
            tracker.start_calibration(frame, bbox)
            follow_state = "CALIBRATING"
            sys_state.follow_state = follow_state
            sys_state.target_bbox = bbox
            sys_state.calibration_progress = tracker.calibration_progress
            if motors_enabled:
                motor.stop()
            return follow_state, pi_latched, "🎯 CALIBRATING: Target Selected by Click"
        else:
            return follow_state, pi_latched, "⚠️ SELECT FAILED: No Person at Click Position"

    if sys_state and getattr(sys_state, "pending_calibrate_center", False):
        sys_state.pending_calibrate_center = False
        if persons:
            best_p = min(
                persons,
                key=lambda p: abs((p["bbox"][0] + p["bbox"][2]) / 2.0 - frame_w / 2.0)
            )
            x1, y1, x2, y2 = [int(v) for v in best_p["bbox"]]
            bbox = (x1, y1, x2 - x1, y2 - y1)
            tracker.start_calibration(frame, bbox)
            follow_state = "CALIBRATING"
            sys_state.follow_state = follow_state
            sys_state.target_bbox = bbox
            sys_state.calibration_progress = tracker.calibration_progress
            if motors_enabled:
                motor.stop()
            return follow_state, pi_latched, "🎯 CALIBRATING: Center Target Selected"
        else:
            return follow_state, pi_latched, "⚠️ CALIBRATE FAILED: No Person in View"

    # ── Respect Arduino Autonomous States ────────────────────────────────────
    if arduino_state in ("AVOIDING", "STATE_AVOIDING", "CLIFF_REVERSING", "STATE_CLIFF_REVERSING"):
        throttler.record_other(arduino_state)
        return follow_state, pi_latched, f"🔀 ARDUINO {arduino_state} (Yielding)"

    # ── Non-Target Obstacle Check for Pre-Tracking States ─────────────────────
    if follow_state in ("WAIT_CALIBRATION", "CALIBRATING", "LOCK_ON"):
        pre_obstacles = non_person_obstacles + pothole_obstacles
        obs_action, _ = _check_obstacle_ahead(pre_obstacles, frame_w, frame_h) if pre_obstacles else ("CLEAR", None)
        if obs_action == "STOP":
            if not pi_latched:
                print("🛑 PI_STOP: Non-target obstacle ahead (camera)")
                motor.pi_stop()
                pi_latched = True
                throttler.record_other("PI_STOP")
                return follow_state, pi_latched, "🛑 PI_STOP: Non-Target Obstacle"
            else:
                throttler.record_other("PI_STOPPED")
                return follow_state, pi_latched, "🛑 PI_STOPPED (Non-Target Obstacle)"

        if pi_latched:
            if obs_action == "CLEAR" and not us_blocked:
                print("✅ PI_GO: Path clear, releasing follow latch.")
                motor.pi_go()
                pi_latched = False
            else:
                throttler.record_other("PI_STOPPED")
                return follow_state, pi_latched, "🛑 PI_STOPPED (Follow Latch Active)"

    # ── State Machine ────────────────────────────────────────────────────────

    # 1. WAIT_CALIBRATION
    if follow_state == "WAIT_CALIBRATION":
        is_cal = getattr(tracker, "is_calibrated", True)
        is_trk = getattr(tracker, "is_tracking", False)
        if is_cal and is_trk:
            follow_state = "TRACKING"
            if sys_state:
                sys_state.follow_state = "TRACKING"
        else:
            if motors_enabled:
                motor.stop()
            throttler.record_other("STOP")
            return follow_state, pi_latched, "🎯 FOLLOW: Click Person or Calibrate"

    # 2. CALIBRATING
    if follow_state == "CALIBRATING":
        if motors_enabled:
            motor.stop()
        throttler.record_other("STOP")
        done = tracker.step_calibration(frame, persons) if hasattr(tracker, "step_calibration") else True
        cal_prog = getattr(tracker, "calibration_progress", 1.0)
        is_cal = getattr(tracker, "is_calibrating", False)
        is_locked = getattr(tracker, "is_calibrated", False)

        if not is_cal and not is_locked:
            # Person lost during calibration -> abort and wait for new selection
            follow_state = "WAIT_CALIBRATION"
            if sys_state:
                sys_state.follow_state = "WAIT_CALIBRATION"
                sys_state.target_bbox = None
                sys_state.calibration_progress = 0.0
            return follow_state, pi_latched, "⚠️ CALIBRATION ABORTED: Person Lost"

        if sys_state:
            sys_state.calibration_progress = cal_prog
            sys_state.target_bbox = getattr(tracker, "bbox", None)
            sys_state.last_seen_side = getattr(tracker, "last_seen_side", "CENTER")
        if done:
            follow_state = "TRACKING"
            if sys_state:
                sys_state.follow_state = "TRACKING"
            action_label = "🎯 TARGET LOCKED (Calibration Complete)"
        else:
            frames_count = getattr(tracker, "_total_calibration_frames", FOLLOW_CALIBRATION_FRAMES)
            action_label = f"🎯 CALIBRATING CLOTHES & FIGURE ({int(cal_prog * frames_count)}/{frames_count})"
        return follow_state, pi_latched, action_label

    # 3. LOCK_ON (Legacy auto-lock onto largest person)
    if follow_state == "LOCK_ON":
        if persons:
            best = max(
                persons,
                key=lambda d: (d["bbox"][2] - d["bbox"][0]) * (d["bbox"][3] - d["bbox"][1])
            )
            x1, y1, x2, y2 = [int(v) for v in best["bbox"]]
            bbox = (x1, y1, x2 - x1, y2 - y1)
            tracker.lock_on(frame, bbox)
            follow_state = "TRACKING"
            if sys_state:
                sys_state.follow_state = "TRACKING"
                sys_state.target_bbox = bbox
            action_label = "🎯 TARGET LOCKED"
        else:
            if motors_enabled:
                motor.stop()
            throttler.record_other("STOP")
            action_label = "🔍 SCANNING FOR TARGET (Owner)"
        return follow_state, pi_latched, action_label

    # 4. TRACKING
    if follow_state == "TRACKING":
        if hasattr(tracker, "update_with_detections"):
            try:
                is_tracking, tgt_box, target_det, non_target_persons, sim_score = tracker.update_with_detections(
                    frame, persons, fresh_inference=fresh_inference
                )
            except TypeError:
                is_tracking, tgt_box, target_det, non_target_persons, sim_score = tracker.update_with_detections(
                    frame, persons
                )
        else:
            is_tracking, tgt_box = tracker.update(frame)
            target_det = None
            non_target_persons = persons
            sim_score = 1.0 if is_tracking else 0.0

        if sys_state:
            sys_state.target_bbox = tgt_box
            sys_state.target_similarity = sim_score
            sys_state.last_seen_side = getattr(tracker, "last_seen_side", "CENTER")
            sys_state.non_target_persons = non_target_persons

        if not is_tracking or tgt_box is None:
            print("🤷 Tracker lost target → RE_ACQUIRE.")
            motor.pi_stop()
            pi_latched = True
            throttler.record_other("PI_STOP")
            follow_state = "RE_ACQUIRE"
            if sys_state:
                sys_state.follow_state = "RE_ACQUIRE"
            last_side = getattr(tracker, "last_seen_side", "CENTER")
            return follow_state, pi_latched, f"⚠️ TARGET LOST: Re-acquiring (Last: {last_side})"

        # Non-target persons are treated as obstacles!
        all_obstacles = non_person_obstacles + pothole_obstacles + non_target_persons

        # Evaluate obstacle path ahead (Tier 1 Pi Camera Latch)
        obs_action, obs_slow = _check_obstacle_ahead(all_obstacles, frame_w, frame_h) if all_obstacles else ("CLEAR", None)
        if obs_action == "STOP":
            if not pi_latched:
                print("🛑 PI_STOP: Non-target obstacle ahead (camera)")
                motor.pi_stop()
                pi_latched = True
                throttler.record_other("PI_STOP")
                return follow_state, pi_latched, "🛑 PI_STOP: Non-Target Obstacle"
            else:
                throttler.record_other("PI_STOPPED")
                return follow_state, pi_latched, "🛑 PI_STOPPED (Non-Target Obstacle)"

        # Handle Existing Latch Release
        if pi_latched:
            if obs_action == "CLEAR" and not us_blocked:
                print("✅ PI_GO: Path clear, releasing follow latch.")
                motor.pi_go()
                pi_latched = False
            else:
                throttler.record_other("PI_STOPPED")
                return follow_state, pi_latched, "🛑 PI_STOPPED (Follow Latch Active)"

        # Respect Arduino autonomous states
        if arduino_state in ("AVOIDING", "STATE_AVOIDING", "CLIFF_REVERSING", "STATE_CLIFF_REVERSING"):
            throttler.record_other(arduino_state)
            return follow_state, pi_latched, f"🔀 ARDUINO {arduino_state} (Yielding)"

        if arduino_state in ("CLIFF_STOPPED", "STATE_CLIFF_STOPPED"):
            throttler.record_other(arduino_state)
            if obs_action != "STOP" and not us_blocked and is_valid_us(us.get("left")) and is_valid_us(us.get("right")):
                print("✅ PI_GO: Cliff drop-off cleared, releasing latch.")
                motor.pi_go()
                return follow_state, False, "✅ PI_GO: Cliff Cleared"
            return follow_state, pi_latched, "⚠️ CLIFF_STOPPED (Latched Drop-Off)"

        if arduino_state in ("PI_STOPPED", "STATE_PI_STOPPED"):
            throttler.record_other(arduino_state)
            if obs_action == "CLEAR" and not us_blocked:
                motor.pi_go()
            else:
                pi_latched = True
                return follow_state, pi_latched, "🛑 PI_STOPPED (Arduino Latched)"

        # Critical Ultrasonic auto-brake
        if us_blocked:
            throttler.record_other("STOP")
            if motors_enabled:
                motor.stop()
            return follow_state, pi_latched, "🛑 US AUTO-BRAKE (< 25cm)"

        # Steering & Follow Speed Modulation
        x, y, w, h = tgt_box
        center_x = x + w / 2.0
        bbox_h_frac = h / max(frame_h, 1)

        pid.update(center_x)
        angle = -int(pid.output)
        angle = max(-45, min(45, angle))

        # Active Obstacle Bypass While Following
        scores = compute_zone_scores(all_obstacles, frame_w, frame_h, sonar=us) if all_obstacles else {}
        decision = decide_avoidance(scores) if scores else "CLEAR"

        avoid_override = False
        us_l = us.get("left", 999.0)
        us_r = us.get("right", 999.0)

        if decision in ("LEFT", "RIGHT"):
            avoid_override = True
            angle = -35 if decision == "LEFT" else 35
            if throttler.should_send_avoid_hint(decision):
                motor.pi_avoid(decision)
                throttler.record_avoid_hint(decision)
        elif is_valid_us(us_l) and us_l < 42.0 and (not is_valid_us(us_r) or us_r > us_l + 10.0):
            avoid_override = True
            angle = max(25, angle)
        elif is_valid_us(us_r) and us_r < 42.0 and (not is_valid_us(us_l) or us_l > us_r + 10.0):
            avoid_override = True
            angle = min(-25, angle)

        prefix = "🔀 BYPASS+FOLLOW" if avoid_override else "🎯 FOLLOWING"

        if bbox_h_frac > FOLLOW_TOO_CLOSE:
            if motors_enabled:
                motor.stop()
            throttler.record_other("STOP")
            action_label = "🎯 TARGET TOO CLOSE (Holding)"
        elif bbox_h_frac > FOLLOW_IDEAL_MIN:
            spd = FOLLOW_SLOW_SPEED if obs_action == "SLOW" else FOLLOW_CRUISE_SPEED
            action_label = f"{prefix} (Cruise PWM {spd}, Steer {angle}°)"
            if motors_enabled:
                if abs(angle) < 5:
                    if throttler.should_send_drive(spd):
                        motor.drive(spd, throttler.drive_duration_ms)
                        throttler.record_drive(spd)
                else:
                    if throttler.should_send_turn(angle, spd):
                        motor.turn(angle, speed=spd)
                        throttler.record_turn(angle, spd)
            else:
                action_label = f"⏸️ BENCH SAFE: {prefix} CRUISE (Suppressed)"
        elif bbox_h_frac > FOLLOW_GETTING_FAR:
            spd = FOLLOW_SLOW_SPEED if obs_action == "SLOW" else FOLLOW_FAST_SPEED
            action_label = f"{prefix} (Fast PWM {spd}, Steer {angle}°)"
            if motors_enabled:
                if abs(angle) < 5:
                    if throttler.should_send_drive(spd):
                        motor.drive(spd, throttler.drive_duration_ms)
                        throttler.record_drive(spd)
                else:
                    if throttler.should_send_turn(angle, spd):
                        motor.turn(angle, speed=spd)
                        throttler.record_turn(angle, spd)
            else:
                action_label = f"⏸️ BENCH SAFE: {prefix} FAST (Suppressed)"
        else:
            spd = FOLLOW_SLOW_SPEED if obs_action == "SLOW" else FOLLOW_FAST_SPEED
            action_label = f"{prefix} (PWM {spd}, Steer {angle}°)"
            if motors_enabled:
                if abs(angle) < 5:
                    if throttler.should_send_drive(spd):
                        motor.drive(spd, throttler.drive_duration_ms)
                        throttler.record_drive(spd)
                else:
                    if throttler.should_send_turn(angle, spd):
                        motor.turn(angle, speed=spd)
                        throttler.record_turn(angle, spd)
            else:
                action_label = f"⏸️ BENCH SAFE: {prefix} FAR (Suppressed)"

        return follow_state, pi_latched, action_label

    # 5. RE_ACQUIRE
    elif follow_state == "RE_ACQUIRE":
        if persons and tracker.reacquire(frame, persons):
            print("🎯 Target re-identified! Resuming TRACKING.")
            motor.pi_go()
            pi_latched = False
            if hasattr(tracker, "reset_search"):
                tracker.reset_search()
            follow_state = "TRACKING"
            if sys_state:
                sys_state.follow_state = "TRACKING"
                sys_state.target_bbox = getattr(tracker, "bbox", None)
            action_label = "🎯 TARGET RE-IDENTIFIED"
            return follow_state, pi_latched, action_label

        all_obstacles = non_person_obstacles + pothole_obstacles + persons
        obs_action, _ = _check_obstacle_ahead(all_obstacles, frame_w, frame_h) if all_obstacles else ("CLEAR", None)

        # Stepped directional search turns toward last_seen_side
        now_mono = time.monotonic()
        side = getattr(tracker, "last_seen_side", "CENTER")
        should_step = tracker.should_step_search(now_mono) if hasattr(tracker, "should_step_search") else False
        search_steps = getattr(tracker, "search_steps_taken", 0)

        if side in ("LEFT", "RIGHT") and should_step:
            side_dist = us.get("left") if side == "LEFT" else us.get("right")
            side_clear = (not is_valid_us(side_dist)) or (float(side_dist) > 35.0)

            if side_clear and obs_action != "STOP" and not us_blocked:
                if pi_latched:
                    motor.pi_go()
                    pi_latched = False
                step_angle = -FOLLOW_SEARCH_STEP_DEG if side == "LEFT" else FOLLOW_SEARCH_STEP_DEG
                if motors_enabled:
                    motor.turn(step_angle, speed=FOLLOW_SEARCH_TURN_SPEED)
                    throttler.record_turn(step_angle, FOLLOW_SEARCH_TURN_SPEED)
                if hasattr(tracker, "record_search_step"):
                    tracker.record_search_step(now_mono)
                search_steps = getattr(tracker, "search_steps_taken", 0)
                action_label = f"🔍 SEARCHING {side} (Step {search_steps}/{FOLLOW_SEARCH_MAX_STEPS})"
            else:
                if motors_enabled:
                    motor.stop()
                action_label = f"🔍 SEARCH {side} BLOCKED (Holding)"
        else:
            if motors_enabled:
                motor.stop()
            if search_steps >= FOLLOW_SEARCH_MAX_STEPS:
                action_label = f"🔍 SEARCH HOLD (Scanned {side}, awaiting target)"
            elif persons:
                action_label = "🔍 RE-ACQUIRING: Comparing Candidates"
            else:
                action_label = f"🔍 RE-ACQUIRING: Scanning ({side})"

        return follow_state, pi_latched, action_label

    return follow_state, pi_latched, action_label


# =============================================================================
# Main Pipeline
# =============================================================================

def run_pipeline(
    mode: str = "autonomous",
    port: str = "auto",
    baud: int = 115200,
    web_port: int = 8080,
    bench_safe: bool = False,
    invert_motors: bool = False,
    awb_mode: str = "daylight",
    enable_stream: bool = True,
    confidence_thresh: float = 0.4,
):
    print(f"🚀 Autonomous RC Car — mode: {mode.upper()}")
    if mode not in ("autonomous", "lane", "follow"):
        print(f"❌ Unknown mode '{mode}'. Use --mode autonomous, --mode lane, or --mode follow")
        sys.exit(1)

    system_state = SystemState()
    system_state.current_mode = mode
    system_state.control_mode = mode
    # Requirement R2: Preserve disarmed startup state (motors remain disabled until explicit user command)
    system_state.motors_enabled = False
    system_state.invert_motors = invert_motors
    system_state.current_awb = awb_mode

    # ── 1. Detector ──────────────────────────────────────────────────────────
    print("  📦 Loading YOLOv10n detector & pothole pipeline...")
    try:
        detector = Detector(conf=confidence_thresh)
    except Exception as e:
        print(f"❌ Failed to load detector: {e}")
        sys.exit(1)

    # ── 2. Hardware Interfaces (Motor Controller & Sensors) ──────────────────
    motor = MotorController(port=port, baud=baud)
    motor.invert_motors = invert_motors
    system_state.motor = motor

    if not motor.connect():
        print("⚠️  MotorController: Operating in DRY-RUN / SIMULATED mode.")
    else:
        # Await READY or bootloader settling
        ready_deadline = time.monotonic() + 3.0
        while time.monotonic() < ready_deadline:
            line = motor.read_telemetry()
            if line and "READY" in line:
                print(f"  🤖 Arduino ready: {line}")
                break
            time.sleep(0.05)
        # Configure Arduino runtime defaults per PROTOCOL.md
        motor._send("TUNE SET TELEM_STREAM 1")
        motor._send("TUNE SET CLIFF_ENABLE 0")
        motor.pi_go()
        motor.status()

    sensors = SensorManager(motor_controller=motor)
    nav = NavigationSystem(controller=motor, sensors=sensors)
    throttler = CommandThrottler()
    corner_recovery = CornerRecoveryController(
        collision_threshold=EMERGENCY_STOP_DISTANCE_CM,
        safe_threshold=EMERGENCY_STOP_DISTANCE_CM,
        backup_speed=200,
        turn_speed=200,
        backup_timeout_s=1.0,
        turn_duration_s=0.5,
        turn_angle_deg=45,
        confirm_count=2,
        motor=motor,
    )

    # ── 3. Follow-me Components ──────────────────────────────────────────────
    tracker = ObjectTracker()
    steering_pid = PID(P=0.3, I=0.01, D=0.05)
    steering_pid.SetPoint = CAMERA_WIDTH / 2.0
    steering_pid.setSampleTime(0.05)
    steering_pid.setWindup(50)
    follow_state = "LOCK_ON"

    # ── 3b. Chart-Paper Lane Following Components ────────────────────────────
    lane_detector = ChartLaneDetector(color_mode=system_state.lane_color_mode)
    lane_steering = LaneSteeringStabilizer()

    # ── 4. Camera Setup ──────────────────────────────────────────────────────
    use_picam2 = False
    picam2     = None
    cap        = None

    try:
        from picamera2 import Picamera2
        picam2 = Picamera2()
        system_state.cam_instance = picam2

        # Configure Picamera2 in BGR888 mode
        cfg = None
        if Transform is not None:
            try:
                cfg = picam2.create_video_configuration(
                    main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "BGR888"},
                    transform=Transform(hflip=1, vflip=1)
                )
            except Exception:
                cfg = None
        if cfg is None:
            cfg = picam2.create_preview_configuration(
                main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "BGR888"}
            )
        picam2.configure(cfg)
        picam2.start()
        set_awb_mode(picam2, awb_mode)
        use_picam2 = True
        print(f"  📷 CSI Camera initialised via Picamera2 ({CAMERA_WIDTH}x{CAMERA_HEIGHT}).")
    except Exception:
        print("  📷 Picamera2 unavailable; falling back to cv2.VideoCapture.")
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            cap.set(cv2.CAP_PROP_FPS,          CAMERA_FPS)
        else:
            print("  📷 No physical camera detected; generating synthetic frames.")

    # ── 5. HTTP Streaming Server ─────────────────────────────────────────────
    server_thread = None
    httpd = None
    if enable_stream:
        streaming_output = StreamingOutput()
        system_state.streaming_output = streaming_output
        try:
            httpd = StreamingServer(('', web_port), DiagnosticHandler, system_state)
            httpd.lane_detector = lane_detector
            server_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
            server_thread.start()
            print(f"  🌐 Diagnostic HUD live stream active on: http://0.0.0.0:{web_port}")
        except Exception as e:
            print(f"⚠️ Failed to start HTTP streaming server on port {web_port}: {e}")

    # ── 6. Signal Handling & Teardown ────────────────────────────────────────
    def sig_handler(sig, frame):
        system_state.running = False

    if threading.current_thread() is threading.main_thread():
        try:
            signal.signal(signal.SIGINT, sig_handler)
            signal.signal(signal.SIGTERM, sig_handler)
        except (ValueError, AttributeError):
            pass

    print("✅ System ready. Press Ctrl+C to terminate safely.\n")

    frame_count       = 0
    pi_latched        = False
    cached_detections = []
    fps               = 0.0
    t_prev            = time.time()
    last_ping_time    = time.monotonic()

    try:
        while system_state.running:
            # Sync latch state from external triggers (e.g. Web UI Emergency Stop / Latch Release)
            pi_latched = system_state.pi_latched

            # ── A. Drain incoming Arduino serial telemetry ────────────────────
            # Drains pending lines to ensure distance and state stay current
            drained_lines = motor.drain_telemetry(max_lines=40)
            for line in drained_lines:
                if "HEARTBEAT_TIMEOUT" in line:
                    print("⚠️  Arduino watchdog fired — link was quiet.")

            # Periodic ping to keep ATmega328P watchdog happy if quiet
            now_mono = time.monotonic()
            if now_mono - last_ping_time > 1.5:
                if motor.is_connected():
                    motor.status()
                last_ping_time = now_mono

            # ── B. Frame Capture ──────────────────────────────────────────────
            with system_state.lock:
                do_swap_rb = system_state.swap_rb

            if use_picam2 and picam2:
                frame_raw = picam2.capture_array()
                if len(frame_raw.shape) == 3 and frame_raw.shape[2] == 3:
                    frame = cv2.cvtColor(frame_raw, cv2.COLOR_RGB2BGR) if do_swap_rb else frame_raw.copy()
                else:
                    frame = frame_raw.copy()
            elif cap and cap.isOpened():
                ret, frame = cap.read()
                if not ret or frame is None:
                    frame = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
            else:
                frame = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
                cv2.putText(frame, "SIMULATED CAMERA STREAM", (int(CAMERA_WIDTH*0.2), int(CAMERA_HEIGHT*0.5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            frame_count += 1

            # ── C. Ultrasonic Obstacle Check ─────────────────────────────────
            us = sensors.read_ultrasonic()
            us_blocked = (is_valid_us(us.get("left")) and us["left"] < EMERGENCY_STOP_DISTANCE_CM) or \
                         (is_valid_us(us.get("right")) and us["right"] < EMERGENCY_STOP_DISTANCE_CM)

            # ── D. Inference (every 3rd frame, starting immediately on frame 1) ──
            fresh_inference = False
            if frame_count % 3 == 1:
                raw_dets = detector.detect(frame)
                cached_detections = [d for d in raw_dets if d.get("confidence", 0) >= confidence_thresh]
                fresh_inference = True

            # ── E. Decision Execution ────────────────────────────────────────
            with system_state.lock:
                current_ctrl_mode = system_state.control_mode
                override_obs = system_state.manual_override_obstacles
                motors_en = system_state.motors_enabled

            if current_ctrl_mode == "manual":
                # Ensure autonomous obstacle latch is cleared when operating in manual mode (unless in emergency stop)
                with system_state.lock:
                    e_stop = system_state.emergency_stop
                    man_act = system_state.manual_action
                    man_spd = system_state.manual_speed
                if not e_stop:
                    if pi_latched:
                        pi_latched = False
                    if motor.latest_state in ("PI_STOPPED", "STATE_PI_STOPPED", "CLIFF_STOPPED", "STATE_CLIFF_STOPPED"):
                        motor.pi_go()

                # Manual control mode: vehicle is controlled via Web UI / D-Pad
                # Safety auto-brake: ONLY if vehicle is driving FORWARD into an obstacle!
                # Reversing away or turning away from an obstacle must NEVER be auto-braked!
                if us_blocked and not override_obs and man_act in ("FORWARD", "FWD"):
                    if motors_en:
                        motor.stop()
                    action_label = f"🛑 MANUAL AUTO-BRAKE (L:{us.get('left', 999):.0f}cm R:{us.get('right', 999):.0f}cm)"
                else:
                    action_label = f"🎮 MANUAL REMOTE: {man_act} (PWM {man_spd})"
            elif current_ctrl_mode == "lane":
                pi_latched, action_label, lane_res = _run_lane(
                    motor=motor,
                    sensors=sensors,
                    lane_detector=lane_detector,
                    detections=cached_detections,
                    us_blocked=us_blocked,
                    frame=frame,
                    pi_latched=pi_latched,
                    motors_enabled=motors_en,
                    throttler=throttler,
                    sys_state=system_state,
                    lane_steering=lane_steering,
                )
            elif current_ctrl_mode == "follow":
                follow_state, pi_latched, action_label = _run_follow(
                    motor=motor,
                    sensors=sensors,
                    tracker=tracker,
                    pid=steering_pid,
                    detections=cached_detections,
                    us_blocked=us_blocked,
                    frame=frame,
                    follow_state=system_state.follow_state,
                    pi_latched=pi_latched,
                    motors_enabled=motors_en,
                    throttler=throttler,
                    fresh_inference=fresh_inference,
                    sys_state=system_state,
                )
                system_state.follow_state = follow_state
            else:
                pi_latched, action_label = _run_autonomous(
                    motor=motor,
                    sensors=sensors,
                    detections=cached_detections,
                    us_blocked=us_blocked,
                    frame=frame,
                    pi_latched=pi_latched,
                    motors_enabled=motors_en,
                    throttler=throttler,
                    fresh_inference=fresh_inference,
                    corner_recovery=corner_recovery,
                )

            # Sync state with shared container
            system_state.pi_latched = pi_latched
            system_state.active_action_label = action_label

            # ── F. FPS Calculation ───────────────────────────────────────────
            now = time.time()
            fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-6)
            t_prev = now
            system_state.fps = fps

            # ── G. HUD Rendering & Streaming ─────────────────────────────────
            if enable_stream and system_state.streaming_output is not None:
                hud_frame = draw_hud(
                    frame=frame,
                    detections=cached_detections,
                    action_label=action_label,
                    latest_sonar=motor.latest_sonar,
                    latest_state=motor.latest_state,
                    port_name=motor.port,
                    fps=fps,
                    awb_mode=system_state.current_awb,
                    mode=current_ctrl_mode,
                    motors_enabled=system_state.motors_enabled,
                    invert_motors=system_state.invert_motors,
                    pi_latched=pi_latched,
                    target_bbox=system_state.target_bbox,
                    target_similarity=system_state.target_similarity,
                    calibration_progress=system_state.calibration_progress,
                    last_seen_side=system_state.last_seen_side,
                    follow_state=system_state.follow_state,
                    non_target_persons=system_state.non_target_persons,
                    lane_result=system_state.lane_result,
                )
                ok, jpg = cv2.imencode(".jpg", hud_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    system_state.streaming_output.write(jpg.tobytes())

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n🛑 KeyboardInterrupt caught. Shutting down...")
    finally:
        print("🛑 Executing graceful hardware release...")
        system_state.running = False
        if system_state.streaming_output is not None:
            with system_state.streaming_output.condition:
                system_state.streaming_output.condition.notify_all()
        if motor:
            motor.pi_go()           # Release any latches on Arduino
            time.sleep(0.05)
            motor.stop()            # Force stop all motor outputs
            motor.disconnect()
        if sensors:
            sensors.close()
        if cap is not None:
            cap.release()
        if picam2 is not None:
            try:
                picam2.stop()
                picam2.close()
            except Exception:
                pass
        if httpd:
            try:
                httpd.shutdown()
                httpd.server_close()
            except Exception:
                pass
        print("👋 Shutdown complete. Hardware released safely.")


# =============================================================================
# CLI Entry Point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Autonomous RC Car Controller with Headless Live Stream & Telemetry HUD"
    )
    parser.add_argument(
        "--mode",
        choices=["autonomous", "lane", "follow"],
        default="autonomous",
        help="Operating mode: 'autonomous' (avoid obstacles), 'lane' (chart-paper lane following), or 'follow' (follow closest person).",
    )
    parser.add_argument(
        "--port",
        default="auto",
        help="Serial port to Arduino Uno ('auto', '/dev/ttyUSB0', '/dev/ttyACM0', etc.).",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200 per PROTOCOL.md).",
    )
    parser.add_argument(
        "--stream-port",
        "--web-port",
        type=int,
        dest="web_port",
        default=8080,
        help="Port for HTTP MJPEG live stream and web dashboard (default: 8080).",
    )
    parser.add_argument(
        "--bench-safe",
        "--safe",
        action="store_true",
        help="Run in Safe Bench-Test mode (perception & stream active, motor commands suppressed).",
    )
    parser.add_argument(
        "--invert-motors",
        action="store_true",
        help="Invert motor drive polarity (corrects reversed chassis TT motor wiring).",
    )
    parser.add_argument(
        "--awb",
        default="daylight",
        help="Camera white-balance mode: 'daylight', 'auto', 'indoor', etc. (default: daylight).",
    )
    parser.add_argument(
        "--no-stream",
        action="store_true",
        help="Disable HTTP video streaming server.",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.40,
        help="YOLO detection confidence threshold (default: 0.40).",
    )

    args = parser.parse_args()

    run_pipeline(
        mode=args.mode,
        port=args.port,
        baud=args.baud,
        web_port=args.web_port,
        bench_safe=args.bench_safe,
        invert_motors=args.invert_motors,
        awb_mode=args.awb,
        enable_stream=not args.no_stream,
        confidence_thresh=args.conf,
    )


if __name__ == "__main__":
    main()
