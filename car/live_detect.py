#!/usr/bin/env python3
"""
live_detect.py — Fullscreen Live Perception & Autonomous Control System
NMIMS MPSTME Capstone — Raspberry Pi 5 + Arduino Uno
Optimized: Pothole D40 Filtering (No Carpet/Newspaper False Positives)
"""

import io
import json
import logging
import socketserver
import threading
import time
import urllib.parse
from http import server
from pathlib import Path
from threading import Condition, Lock
from typing import List, Dict, Any, Optional

import cv2
import numpy as np
from picamera2 import Picamera2
from ultralytics import YOLO

# ── 1. Model & Path Resolution ───────────────────────────────────────────────
def find_model_path(rel_path: str) -> Optional[Path]:
    candidates = [
        Path("/home/capstone/model") / rel_path,
        Path.home() / "model" / rel_path,
        Path.home() / "model-car-capstone" / rel_path,
        Path.cwd() / rel_path,
        Path(__file__).resolve().parent / rel_path,
    ]
    for p in candidates:
        if p.exists(): return p
    return None

obs_path = find_model_path("benchmarking/models/yolov10n_ncnn_model")
if not obs_path:
    obs_path = Path("/home/capstone/model/yolov10n_ncnn_model")

pot_path = find_model_path("pothole_detection/best_ncnn_model")

print(f"📦 Loading YOLOv10n obstacle model: {obs_path}")
obs_model = YOLO(str(obs_path), task="detect")

pot_model = None
if pot_path and pot_path.exists():
    print(f"📦 Loading Pothole model: {pot_path}")
    try:
        pot_model = YOLO(str(pot_path), task="detect")
    except Exception as e:
        print(f"⚠️ Could not load NCNN pothole model: {e}")
else:
    print("ℹ️ Using classical CV Laplacian pothole detector as fallback.")

def find_daylight_ref() -> Optional[Path]:
    for p in [Path("/home/capstone/daylight.jpg"), Path("/home/capstone/model/daylight.jpg"), Path.home() / "daylight.jpg"]:
        if p.exists(): return p
    return None

CAM_WIDTH = 320
CAM_HEIGHT = 240
IMGSZ = 320
OBSTACLE_CLASSES = [0, 1, 2, 3, 5, 7]  # person, bicycle, car, motorcycle, bus, truck
CONF_OBSTACLE = 0.35
CONF_POTHOLE = 0.20                     # Sensitive threshold for actual D40 potholes
POTHOLE_ONLY = True                     # STRICT FILTER: Only detect D40 potholes; ignore alligator/longitudinal cracks
POTHOLE_EVERY = 2

BBOX_STOP_HEIGHT_FRAC = 0.38
EMERGENCY_STOP_DIST_CM = 25.0           # Critical ultrasonic auto-brake distance (< 25 cm)
US_AVOID_DIST_CM = 42.0                 # Directional ultrasonic avoidance steer zone (< 42 cm)
US_SLOW_DIST_CM = 60.0                  # Cautionary speed reduction zone (< 60 cm)
CRUISE_SPEED = 200                      # Normal cruising forward PWM
SLOW_SPEED = 135                        # Cautionary PWM when approaching obstacle

# ── 2. Camera Controls & Daylight White Balance ──────────────────────────────
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

current_awb = "daylight"
swap_rb = True
hardware_rotated = False
cam_instance = None

def set_awb_mode(cam, mode_name: str) -> bool:
    global current_awb
    mode_name = mode_name.lower().strip()
    if mode_name not in AWB_MODES: return False
    val = AWB_MODES[mode_name]
    try:
        cam.set_controls({"AwbMode": val})
        current_awb = mode_name
        print(f"☀️ Camera AWB set to: {mode_name.upper()} (matches daylight.jpg)")
        return True
    except Exception as e:
        print(f"⚠️ Failed to set AwbMode: {e}")
        return False

# ── 3. Classical CV Pothole Fallback (Enhanced Against Carpet/Text) ──────────
def cv_detect_potholes(frame: np.ndarray) -> List[Dict[str, Any]]:
    h, w = frame.shape[:2]
    roi_y = int(h * 0.40)
    roi = frame[roi_y:, :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    lap = cv2.Laplacian(blur, cv2.CV_64F)
    lap_u = np.uint8(np.clip(np.absolute(lap), 0, 255))
    _, thresh = cv2.threshold(lap_u, 32, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    dets = []
    roi_area = roi.shape[0] * roi.shape[1]
    min_area = max(150, int(roi_area * 0.0065))
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # Suppress tiny noise (text/carpet dots) and huge false areas (dynamically scaled to resolution)
        if area < min_area or area > roi_area * 0.30: continue
        x, y, cw, ch = cv2.boundingRect(cnt)
        # Potholes are round/oval cavities (aspect ratio 0.5 to 2.5) — eliminates long lines/cracks
        aspect = cw / max(ch, 1)
        if not (0.5 < aspect < 2.5): continue
        dets.append({"bbox": [x, y + roi_y, x + cw, y + roi_y + ch], "conf": 0.45, "label": "POTHOLE (CV)", "is_pothole": True})
    return dets

# ── 4. Arduino Uno Parallel Serial Bridge ────────────────────────────────────
class ArduinoBridge:
    def __init__(self):
        self.serial_port = None
        self.port_name = "SIMULATED"
        self.connected = False
        self.lock = Lock()
        self.latest_sonar = {"left": 999.0, "right": 999.0}
        self.latest_state = "IDLE"
        self.motors_enabled = False
        self.pi_latched = False
        # Chassis TT motor wiring causes forward/reverse inversion relative to standard L298N polarity.
        # Default to True so forward cruise commands actually move the car forward.
        self.invert_motors = True
        self.last_status_time = 0.0

    def auto_connect(self):
        import serial
        for p in ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyUSB1", "/dev/ttyACM1"]:
            try:
                s = serial.Serial(p, 115200, timeout=0.1)
                time.sleep(2.0)
                s.reset_input_buffer()
                s.reset_output_buffer()
                self.serial_port = s
                self.port_name = p
                self.connected = True
                print(f"🔌 Arduino connected on {p} @ 115200 baud")

                # Configure Arduino runtime parameters immediately:
                # 1. Enable continuous ultrasonic telemetry streaming
                self.send("TUNE SET TELEM_STREAM 1")
                # 2. Disable cliff detection for flat floor driving (prevents false cliff reverse bursts)
                self.send("TUNE SET CLIFF_ENABLE 0")
                # 3. Release any startup latch or cliff stop
                self.send("PI_GO")
                self.send("STATUS")
                return True
            except Exception:
                continue
        print("⚠️ No physical Arduino detected. Operating in DRY-RUN mode.")
        self.connected = False
        return False

    def send(self, cmd: str) -> bool:
        if not self.connected or not self.serial_port: return False
        with self.lock:
            try:
                self.serial_port.write((cmd.strip() + "\n").encode("utf-8"))
                self.serial_port.flush()
                return True
            except Exception:
                self.connected = False
                return False

    def read_telemetry(self):
        if not self.connected or not self.serial_port: return
        now = time.time()
        # Periodic health check / request STATUS every 1.5s if telemetry stalls
        if now - self.last_status_time > 1.5:
            self.send("STATUS")
            self.last_status_time = now

        try:
            while self.serial_port.in_waiting:
                line = self.serial_port.readline().decode("utf-8", errors="ignore").strip()
                if not line: continue
                # Parse ultrasonic telemetry: [TELEM] US LEFT=x.x RIGHT=y.y
                # or [TELEM] STATE=... US_LEFT=x.x US_RIGHT=y.y or [TELEM] CLIFF LEFT=...
                if ("LEFT=" in line or "US_LEFT=" in line) and ("RIGHT=" in line or "US_RIGHT=" in line):
                    norm = line.replace("US_LEFT=", "LEFT=").replace("US_RIGHT=", "RIGHT=")
                    for pt in norm.split():
                        if pt.startswith("LEFT="):
                            try: self.latest_sonar["left"] = float(pt.split("=")[1])
                            except: pass
                        elif pt.startswith("RIGHT="):
                            try: self.latest_sonar["right"] = float(pt.split("=")[1])
                            except: pass
                if "STATE=" in line:
                    for pt in line.split():
                        if pt.startswith("STATE="):
                            self.latest_state = pt.split("=")[1]
        except Exception:
            pass

    def pi_stop(self):
        self.pi_latched = True
        self.send("PI_STOP")

    def pi_go(self):
        self.pi_latched = False
        self.send("PI_GO")

    def pi_avoid(self, direction: str):
        eff_dir = direction.upper()
        if self.invert_motors:
            eff_dir = "RIGHT" if eff_dir == "LEFT" else "LEFT"
        self.send(f"PI_AVOID {eff_dir}")

    def drive(self, speed: int, duration_ms: int):
        if self.motors_enabled and not self.pi_latched:
            # If motors are inverted on chassis, REV moves forward physically
            cmd = f"REV {speed} {duration_ms}" if self.invert_motors else f"DRIVE {speed} {duration_ms}"
            self.send(cmd)

    def reverse(self, speed: int, duration_ms: int):
        if self.motors_enabled and not self.pi_latched:
            cmd = f"DRIVE {speed} {duration_ms}" if self.invert_motors else f"REV {speed} {duration_ms}"
            self.send(cmd)

    def turn(self, angle: int, speed: int = 150):
        if self.motors_enabled and not self.pi_latched:
            eff_angle = -angle if self.invert_motors else angle
            self.send(f"TURN {eff_angle} {speed}")

    def stop(self):
        self.send("STOP")

arduino = ArduinoBridge()
active_action_label = "IDLE (Safety Hold)"

def is_valid_us(dist: float) -> bool:
    """Returns True if reading is a valid obstacle reflection (filters out 0.0 noise & 999 timeout)."""
    return 1.0 < dist < 850.0

def evaluate_autonomous_action(frame_w: int, frame_h: int, obs_boxes: list, pot_dets: list):
    global active_action_label

    # 1. Camera Obstacle Check (YOLOv10n bounding box size & position)
    obstacle_ahead = False
    for b in obs_boxes:
        x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
        height_frac = (y2 - y1) / max(frame_h, 1)
        cx = (x1 + x2) / 2
        if 0.20 * frame_w < cx < 0.80 * frame_w and height_frac > BBOX_STOP_HEIGHT_FRAC:
            obstacle_ahead = True
            break

    if obstacle_ahead:
        active_action_label = "🛑 PI_STOP: Obstacle Imminent (Camera)"
        arduino.pi_stop()
        return

    # Release Pi latch if previously latched by camera and now clear
    if arduino.pi_latched and not obstacle_ahead:
        active_action_label = "✅ PI_GO: Path Cleared (Camera)"
        arduino.pi_go()

    # 2. Ultrasonic Sensor Active Processing
    us_l = arduino.latest_sonar["left"]
    us_r = arduino.latest_sonar["right"]
    v_l = is_valid_us(us_l)
    v_r = is_valid_us(us_r)

    # 2a. Ultrasonic Critical Safety Auto-Brake (< EMERGENCY_STOP_DIST_CM)
    if (v_l and us_l < EMERGENCY_STOP_DIST_CM) or (v_r and us_r < EMERGENCY_STOP_DIST_CM):
        active_action_label = f"🛑 US AUTO-BRAKE! (L:{us_l:.0f}cm R:{us_r:.0f}cm)"
        arduino.stop()
        return

    # 2b. Front Corridor Blocked: Both sensors detect obstacle (< US_AVOID_DIST_CM)
    if v_l and v_r and us_l < US_AVOID_DIST_CM and us_r < US_AVOID_DIST_CM:
        active_action_label = f"⚠️ US OBSTACLE AHEAD (L:{us_l:.0f}cm R:{us_r:.0f}cm) -> STOP"
        arduino.stop()
        return

    # 2c. Ultrasonic Directional Avoidance: Left obstacle closer -> Steer RIGHT
    if v_l and us_l < US_AVOID_DIST_CM and (not v_r or us_l < us_r - 6.0):
        active_action_label = f"🔀 US AVOID: Left Obs ({us_l:.0f}cm) -> Steering RIGHT"
        arduino.pi_avoid("RIGHT")
        if arduino.motors_enabled:
            arduino.turn(35, 160)
        return

    # 2d. Ultrasonic Directional Avoidance: Right obstacle closer -> Steer LEFT
    if v_r and us_r < US_AVOID_DIST_CM and (not v_l or us_r < us_l - 6.0):
        active_action_label = f"🔀 US AVOID: Right Obs ({us_r:.0f}cm) -> Steering LEFT"
        arduino.pi_avoid("LEFT")
        if arduino.motors_enabled:
            arduino.turn(-35, 160)
        return

    # 3. Camera Pothole Avoidance (Classical CV + RDD2022 D40)
    danger_left = sum(1.0 for p in pot_dets if p.get("is_pothole") and (p["bbox"][0] + p["bbox"][2]) / 2 < frame_w * 0.5)
    danger_right = sum(1.0 for p in pot_dets if p.get("is_pothole") and (p["bbox"][0] + p["bbox"][2]) / 2 >= frame_w * 0.5)

    if danger_left > 0 or danger_right > 0:
        if danger_left > danger_right:
            active_action_label = "🔀 AVOIDING POTHOLE: Steering RIGHT"
            arduino.pi_avoid("RIGHT")
            if arduino.motors_enabled:
                arduino.turn(30, 160)
        else:
            active_action_label = "🔀 AVOIDING POTHOLE: Steering LEFT"
            arduino.pi_avoid("LEFT")
            if arduino.motors_enabled:
                arduino.turn(-30, 160)
        return

    # 4. Clear Path Cruising (Speed modulated by forward ultrasonic clearance)
    if arduino.motors_enabled:
        # If an obstacle is detected in caution range (US_SLOW_DIST_CM), reduce speed
        if (v_l and us_l < US_SLOW_DIST_CM) or (v_r and us_r < US_SLOW_DIST_CM):
            active_action_label = f"🟡 CRUISING CAUTIOUSLY (PWM {SLOW_SPEED}) [US Near]"
            arduino.drive(SLOW_SPEED, 250)
        else:
            active_action_label = f"🟢 CRUISING FORWARD (PWM {CRUISE_SPEED})"
            arduino.drive(CRUISE_SPEED, 250)
    else:
        active_action_label = "⏸️ MOTORS DISABLED (Bench Test Safe)"

PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Capstone Model Car — Autonomous Perception & Control</title>
  <style>
    :root {
      --bg: #000000;
      --card-glass: rgba(18, 22, 30, 0.75);
      --border-glass: rgba(255, 255, 255, 0.12);
      --accent: #58a6ff; --success: #3fb950; --danger: #f85149; --text: #f0f6fc; --muted: #8b949e;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    html, body {
      background: var(--bg); color: var(--text);
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      width: 100vw; height: 100vh; overflow: hidden;
      display: flex; justify-content: center; align-items: center; user-select: none;
    }
    #viewport {
      position: absolute; top: 0; left: 0; width: 100vw; height: 100vh;
      display: flex; justify-content: center; align-items: center; background: #000;
    }
    img#stream { width: 100%; height: 100%; object-fit: contain; display: block; }
    header {
      position: absolute; top: 14px; left: 50%; transform: translateX(-50%);
      background: var(--card-glass); backdrop-filter: blur(12px); -webkit-backdrop-filter: blur(12px);
      border: 1px solid var(--border-glass); border-radius: 30px; padding: 6px 18px;
      display: flex; align-items: center; gap: 14px; box-shadow: 0 8px 32px rgba(0,0,0,0.6); z-index: 100;
    }
    .badge { padding: 3px 10px; border-radius: 12px; font-size: 0.75rem; font-weight: 700; }
    .badge-daylight { background: rgba(227, 179, 65, 0.2); color: #e3b341; border: 1px solid #e3b341; }
    .badge-motors-on { background: rgba(63, 185, 80, 0.2); color: var(--success); border: 1px solid var(--success); }
    .badge-motors-off { background: rgba(248, 81, 73, 0.2); color: var(--danger); border: 1px solid var(--danger); }
    .badge-invert-on { background: rgba(88, 166, 255, 0.2); color: var(--accent); border: 1px solid var(--accent); }
    .badge-invert-off { background: rgba(139, 148, 158, 0.2); color: var(--muted); border: 1px solid var(--muted); }
    .badge-us { background: rgba(63, 185, 80, 0.2); color: var(--success); border: 1px solid var(--success); }
    .controls-dock {
      position: absolute; bottom: 16px; left: 50%; transform: translateX(-50%);
      background: var(--card-glass); backdrop-filter: blur(14px); -webkit-backdrop-filter: blur(14px);
      border: 1px solid var(--border-glass); border-radius: 28px; padding: 6px 14px;
      display: flex; align-items: center; gap: 8px; box-shadow: 0 8px 32px rgba(0,0,0,0.6); z-index: 100;
      transition: all 0.3s ease;
    }
    .controls-dock.hidden { opacity: 0; pointer-events: none; transform: translateX(-50%) translateY(20px); }
    button {
      background: rgba(255, 255, 255, 0.08); color: var(--text); border: 1px solid var(--border-glass);
      padding: 7px 14px; border-radius: 18px; font-size: 0.8rem; font-weight: 600; cursor: pointer;
      display: inline-flex; align-items: center; gap: 6px;
    }
    button:hover { background: rgba(255, 255, 255, 0.18); }
    button.btn-active { background: #1f6feb; border-color: #58a6ff; }
    button.btn-danger { background: rgba(248, 81, 73, 0.25); border-color: var(--danger); color: #ff7b72; }
    button.btn-success { background: rgba(63, 185, 80, 0.25); border-color: var(--success); color: #56d364; }
    .shortcut-tip { position: absolute; bottom: 6px; right: 12px; font-size: 0.7rem; color: var(--muted); opacity: 0.6; }
    .modal {
      display: none; position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
      background: rgba(0,0,0,0.85); z-index: 1000; justify-content: center; align-items: center; padding: 20px;
    }
    .modal.show { display: flex; }
    .modal-box { background: #161b22; border: 1px solid #30363d; border-radius: 10px; max-width: 680px; width: 100%; padding: 16px; }
    .modal-box img { width: 100%; border-radius: 6px; margin-top: 10px; display: block; }
  </style>
</head>
<body>
  <div id="viewport">
    <img id="stream" src="stream.mjpg" alt="Autonomous Perception Stream">
  </div>

  <header id="topbar">
    <div style="font-weight:700; font-size:0.85rem">🏎️ NMIMS MPSTME Capstone</div>
    <div class="badge badge-daylight" id="awb-badge">AWB: DAYLIGHT ☀️</div>
    <div class="badge badge-motors-off" id="motors-badge">MOTORS: OFF ⏸️</div>
    <div class="badge badge-invert-on" id="invert-badge">POLARITY: INVERTED (FWD FIX) 🔄</div>
    <div class="badge badge-us" id="sonar-badge">US: L=--- R=---</div>
  </header>

  <div class="controls-dock" id="dock">
    <button id="btn-motors" class="btn-danger" onclick="toggleMotors()">🚗 Enable Motors (M)</button>
    <button id="btn-invert" class="btn-active" onclick="toggleInvertMotors()">🔄 Polarity: Inverted (I)</button>
    <button class="btn-active" id="btn-daylight" onclick="setAwb('daylight')">☀️ Daylight</button>
    <button onclick="setAwb('auto')">🔄 Auto</button>
    <button onclick="setAwb('indoor')">💡 Indoor</button>
    <button id="btn-pothole-mode" class="btn-active" onclick="togglePotholeMode()">🕳️ Potholes Only</button>
    <button onclick="emergencyStop()" class="btn-danger">🛑 E-STOP (Space)</button>
    <button onclick="toggleFullscreen()">⛶ Fullscreen (F)</button>
    <button onclick="openRef()">📷 Ref daylight.jpg</button>
    <button onclick="toggleDock()" style="padding:7px 10px">👁️</button>
  </div>

  <div class="shortcut-tip">Press <b>F</b> Fullscreen &bull; <b>M</b> Motors &bull; <b>I</b> Invert Polarity &bull; <b>Space</b> E-Stop &bull; <b>P</b> Potholes &bull; <b>H</b> Dock</div>

  <div class="modal" id="modal" onclick="closeRef()">
    <div class="modal-box" onclick="event.stopPropagation()">
      <div style="display:flex; justify-content:space-between; font-weight:600">
        <span>Reference Image: daylight.jpg</span>
        <span style="cursor:pointer" onclick="closeRef()">&times;</span>
      </div>
      <img src="daylight.jpg" alt="daylight.jpg reference">
    </div>
  </div>

  <script>
    function toggleFullscreen() {
      if (!document.fullscreenElement) {
        document.documentElement.requestFullscreen().catch(() => {});
      } else {
        document.exitFullscreen().catch(() => {});
      }
    }

    function toggleMotors() {
      fetch('/toggle_motors').then(r => r.json()).then(d => {
        const btn = document.getElementById('btn-motors');
        const badge = document.getElementById('motors-badge');
        if (d.motors_enabled) {
          btn.innerText = '🛑 Disable Motors (M)';
          btn.className = 'btn-success';
          badge.innerText = 'MOTORS: ACTIVE ⚡';
          badge.className = 'badge badge-motors-on';
        } else {
          btn.innerText = '🚗 Enable Motors (M)';
          btn.className = 'btn-danger';
          badge.innerText = 'MOTORS: OFF ⏸️';
          badge.className = 'badge badge-motors-off';
        }
      });
    }

    function toggleInvertMotors() {
      fetch('/toggle_invert_motors').then(r => r.json()).then(d => {
        const btn = document.getElementById('btn-invert');
        const badge = document.getElementById('invert-badge');
        if (d.invert_motors) {
          btn.innerText = '🔄 Polarity: Inverted (I)';
          btn.className = 'btn-active';
          badge.innerText = 'POLARITY: INVERTED (FWD FIX) 🔄';
          badge.className = 'badge badge-invert-on';
        } else {
          btn.innerText = '🔄 Polarity: Normal (I)';
          btn.className = '';
          badge.innerText = 'POLARITY: NORMAL ➡️';
          badge.className = 'badge badge-invert-off';
        }
      });
    }

    function emergencyStop() {
      fetch('/emergency_stop').then(r => r.json()).then(d => {
        const btn = document.getElementById('btn-motors');
        const badge = document.getElementById('motors-badge');
        btn.innerText = '🚗 Enable Motors (M)';
        btn.className = 'btn-danger';
        badge.innerText = 'MOTORS: OFF (E-STOP) 🛑';
        badge.className = 'badge badge-motors-off';
      });
    }

    function togglePotholeMode() {
      fetch('/toggle_pothole_mode').then(r => r.json()).then(d => {
        const btn = document.getElementById('btn-pothole-mode');
        if (d.pothole_only) {
          btn.innerText = '🕳️ Potholes Only';
          btn.className = 'btn-active';
        } else {
          btn.innerText = '🛣️ Potholes + Cracks';
          btn.className = '';
        }
      });
    }

    function setAwb(mode) {
      fetch('/set_awb?mode=' + mode).then(r => r.json()).then(d => {
        if (d.status === 'ok') {
          document.getElementById('awb-badge').innerText = 'AWB: ' + d.mode.toUpperCase() + (d.mode === 'daylight' ? ' ☀️' : '');
          document.querySelectorAll('.controls-dock button').forEach(b => {
            if (b.innerText.toLowerCase().includes(mode)) b.classList.add('btn-active');
            else if (b.innerText.includes('☀️') || b.innerText.includes('🔄') || b.innerText.includes('💡')) b.classList.remove('btn-active');
          });
        }
      });
    }

    function toggleDock() {
      const dock = document.getElementById('dock');
      const topbar = document.getElementById('topbar');
      dock.classList.toggle('hidden');
      topbar.style.opacity = dock.classList.contains('hidden') ? '0' : '1';
    }

    function openRef() { document.getElementById('modal').classList.add('show'); }
    function closeRef() { document.getElementById('modal').classList.remove('show'); }

    // Live Telemetry Poller (Ultrasonic & Motors)
    function pollTelemetry() {
      fetch('/telemetry').then(r => r.json()).then(d => {
        const sb = document.getElementById('sonar-badge');
        if (sb && d.sonar) {
          const l = (d.sonar.left <= 1.0 || d.sonar.left >= 850.0) ? 'CLR' : d.sonar.left.toFixed(0) + 'cm';
          const r = (d.sonar.right <= 1.0 || d.sonar.right >= 850.0) ? 'CLR' : d.sonar.right.toFixed(0) + 'cm';
          sb.innerText = `US: L=${l}  R=${r}`;
          if ((d.sonar.left > 1.0 && d.sonar.left < 25.0) || (d.sonar.right > 1.0 && d.sonar.right < 25.0)) {
            sb.style.background = 'rgba(248, 81, 73, 0.35)';
            sb.style.borderColor = 'var(--danger)';
            sb.style.color = '#ff7b72';
          } else if ((d.sonar.left > 1.0 && d.sonar.left < 45.0) || (d.sonar.right > 1.0 && d.sonar.right < 45.0)) {
            sb.style.background = 'rgba(227, 179, 65, 0.35)';
            sb.style.borderColor = '#e3b341';
            sb.style.color = '#e3b341';
          } else {
            sb.style.background = 'rgba(63, 185, 80, 0.2)';
            sb.style.borderColor = 'var(--success)';
            sb.style.color = 'var(--success)';
          }
        }
      }).catch(() => {});
    }
    setInterval(pollTelemetry, 300);

    window.addEventListener('keydown', (e) => {
      const k = e.key.toLowerCase();
      if (k === 'f') toggleFullscreen();
      if (k === 'm') toggleMotors();
      if (k === 'i') toggleInvertMotors();
      if (k === ' ') { e.preventDefault(); emergencyStop(); }
      if (k === 'h') toggleDock();
      if (k === 'p') togglePotholeMode();
      if (k === 'd') setAwb('daylight');
    });
  </script>
</body>
</html>
"""

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

def draw_boxes(frame, result, color, prefix=""):
    w = frame.shape[1]
    f_scale = 0.50 if w >= 480 else 0.38
    thick = 2 if w >= 480 else 1
    for b in result.boxes:
        x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
        label = f"{prefix}{result.names[int(b.cls[0])]} {float(b.conf[0]):.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thick)
        cv2.putText(frame, label, (x1, max(14, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, f_scale, color, thick)

def detect_loop(cam, output):
    global current_awb, swap_rb, POTHOLE_ONLY, hardware_rotated
    n = 0
    last_pot_dets = []
    fps = 0.0
    t_prev = time.time()

    print("🚀 Parallel Detection & Control Loop Active!")
    while True:
        arduino.read_telemetry()

        frame_raw = cam.capture_array()
        if swap_rb:
            frame = cv2.cvtColor(frame_raw, cv2.COLOR_RGB2BGR)
        else:
            frame = frame_raw.copy()

        # Fallback software 180° rotation if hardware ISP transform was not available
        if not hardware_rotated:
            frame = cv2.rotate(frame, cv2.ROTATE_180)

        h, w = frame.shape[:2]

        # 1. Obstacle Detection (YOLOv10n NCNN)
        r = obs_model.predict(frame, imgsz=IMGSZ, conf=CONF_OBSTACLE,
                              classes=OBSTACLE_CLASSES, verbose=False)[0]

        # 2. Pothole Detection with Strict Filtering
        if n % POTHOLE_EVERY == 0:
            last_pot_dets = []
            if pot_model is not None:
                res_p = pot_model.predict(frame, imgsz=IMGSZ, conf=CONF_POTHOLE, verbose=False)[0]
                for box in res_p.boxes:
                    cls_id = int(box.cls[0])
                    name = res_p.names.get(cls_id, "pothole").lower()
                    c_val = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                    # D40 Pothole Check (Class 0 in RDD2022)
                    is_pothole = (cls_id == 0 or "pothole" in name or "d40" in name)

                    # If POTHOLE_ONLY is on, ignore alligator/longitudinal cracks completely
                    if POTHOLE_ONLY and not is_pothole:
                        continue

                    lbl = f"POTHOLE (D40) {c_val:.2f}" if is_pothole else f"CRACK {c_val:.2f}"
                    last_pot_dets.append({
                        "bbox": [x1, y1, x2, y2],
                        "conf": c_val,
                        "label": lbl,
                        "is_pothole": is_pothole
                    })
            else:
                last_pot_dets = cv_detect_potholes(frame)

        # 3. Decision Engine
        evaluate_autonomous_action(w, h, r.boxes, last_pot_dets)

        # 4. Render Obstacle Boxes (Green)
        draw_boxes(frame, r, (0, 255, 0))

        # 5. Render Pothole Boxes (Orange/Red for Potholes, Cyan for Cracks)
        f_scale = 0.50 if w >= 480 else 0.38
        hud_scale1 = 0.65 if w >= 480 else 0.45
        hud_scale2 = 0.55 if w >= 480 else 0.40
        thick = 2 if w >= 480 else 1

        for p in last_pot_dets:
            x1, y1, x2, y2 = p["bbox"]
            color = (0, 69, 255) if p.get("is_pothole") else (255, 200, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thick)
            cv2.putText(frame, p["label"], (x1, max(14, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, f_scale, color, thick)

        now = time.time()
        fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-6)
        t_prev = now

        # HUD Overlays
        cv2.putText(frame, f"{fps:.1f} FPS (Pi 5 HW) | {current_awb.upper()} AWB", (10, 20 if w < 480 else 26),
                    cv2.FONT_HERSHEY_SIMPLEX, hud_scale1, (0, 255, 255), thick)
        cv2.putText(frame, f"Action: {active_action_label}", (10, 38 if w < 480 else 52),
                    cv2.FONT_HERSHEY_SIMPLEX, hud_scale1, (0, 255, 0) if "CRUISING" in active_action_label else (0, 165, 255), thick)

        # Ultrasonic live display with color-coded status
        us_l = arduino.latest_sonar["left"]
        us_r = arduino.latest_sonar["right"]
        us_l_disp = f"{us_l:.0f}cm" if (1.0 < us_l < 850.0) else "CLEAR"
        us_r_disp = f"{us_r:.0f}cm" if (1.0 < us_r < 850.0) else "CLEAR"
        us_str = f"US: L={us_l_disp} | R={us_r_disp}"

        us_color = (0, 255, 0)  # Green = Clear
        if (1.0 < us_l < EMERGENCY_STOP_DIST_CM) or (1.0 < us_r < EMERGENCY_STOP_DIST_CM):
            us_color = (0, 0, 255)    # Red = Critical Stop
            us_str += " [BRAKE]"
        elif (1.0 < us_l < US_AVOID_DIST_CM) or (1.0 < us_r < US_AVOID_DIST_CM):
            us_color = (0, 140, 255)  # Orange = Avoidance
            us_str += " [AVOID]"
        elif (1.0 < us_l < US_SLOW_DIST_CM) or (1.0 < us_r < US_SLOW_DIST_CM):
            us_color = (0, 255, 255)  # Yellow = Caution
            us_str += " [SLOW]"

        ard_str = f"Arduino: {arduino.port_name} [{arduino.latest_state}] | Polarity: {'INVERTED' if arduino.invert_motors else 'NORMAL'}"
        cv2.putText(frame, us_str, (10, h - (22 if w < 480 else 36)),
                    cv2.FONT_HERSHEY_SIMPLEX, hud_scale2, us_color, thick)
        cv2.putText(frame, ard_str, (10, h - (8 if w < 480 else 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, hud_scale2, (200, 200, 200), 1)

        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            output.write(jpg.tobytes())

        n += 1

# ── 7. Web Server Handler ────────────────────────────────────────────────────
class Handler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        global POTHOLE_ONLY
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        query = urllib.parse.parse_qs(parsed.query)

        if path in ('/', '/index.html'):
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        elif path == '/toggle_motors':
            arduino.motors_enabled = not arduino.motors_enabled
            if not arduino.motors_enabled:
                arduino.stop()
            print(f"🚗 Motors toggled: {'ENABLED' if arduino.motors_enabled else 'DISABLED'}")
            resp = json.dumps({"status": "ok", "motors_enabled": arduino.motors_enabled}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/toggle_invert_motors':
            arduino.invert_motors = not arduino.invert_motors
            print(f"🔄 Motor polarity toggled: {'INVERTED (FWD Fix)' if arduino.invert_motors else 'NORMAL'}")
            resp = json.dumps({"status": "ok", "invert_motors": arduino.invert_motors}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/emergency_stop':
            arduino.motors_enabled = False
            arduino.stop()
            arduino.pi_stop()
            print("🛑 Emergency Stop triggered!")
            resp = json.dumps({"status": "ok", "motors_enabled": False}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/telemetry':
            data = {
                "sonar": arduino.latest_sonar,
                "state": arduino.latest_state,
                "motors_enabled": arduino.motors_enabled,
                "invert_motors": arduino.invert_motors,
                "action": active_action_label,
            }
            resp = json.dumps(data).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/toggle_pothole_mode':
            POTHOLE_ONLY = not POTHOLE_ONLY
            print(f"🕳️ Pothole filtering mode: {'POTHOLES ONLY (D40)' if POTHOLE_ONLY else 'ALL ROAD DAMAGE'}")
            resp = json.dumps({"status": "ok", "pothole_only": POTHOLE_ONLY}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)

        elif path == '/set_awb':
            mode = query.get('mode', ['daylight'])[0]
            success = set_awb_mode(cam_instance, mode)
            resp = json.dumps({"status": "ok" if success else "error", "mode": current_awb}).encode('utf-8')
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
                self.send_error(404, "daylight.jpg not found on Raspberry Pi")

        elif path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as e:
                logging.warning('Client stream closed: %s', e)
        else:
            self.send_error(404)

class Server(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

def main():
    global output, cam_instance, hardware_rotated
    logging.basicConfig(level=logging.INFO)

    arduino.auto_connect()

    print(f"🎥 Initializing Picamera2 in BGR888 mode ({CAM_WIDTH}x{CAM_HEIGHT}, 180° rotation)...")
    cam = Picamera2()
    cam_instance = cam

    video_config = None
    hardware_rotated = False
    if Transform is not None:
        try:
            # 180° rotation natively handled by Raspberry Pi ISP with zero CPU overhead
            video_config = cam.create_video_configuration(
                main={"size": (CAM_WIDTH, CAM_HEIGHT), "format": "BGR888"},
                transform=Transform(hflip=1, vflip=1)
            )
            hardware_rotated = True
            print("🔄 Hardware 180° rotation (hflip + vflip via ISP) enabled.")
        except Exception as e:
            print(f"⚠️ Could not set hardware transform: {e}")

    if video_config is None:
        video_config = cam.create_video_configuration(main={"size": (CAM_WIDTH, CAM_HEIGHT), "format": "BGR888"})

    cam.configure(video_config)
    cam.start()

    set_awb_mode(cam, "daylight")

    output = StreamingOutput()
    worker = threading.Thread(target=detect_loop, args=(cam, output), daemon=True)
    worker.start()

    port = 8000
    print(f"🚀 Live perception ready at: http://<pi-ip>:{port}")
    try:
        Server(('', port), Handler).serve_forever()
    except KeyboardInterrupt:
        print("\n🛑 Stopping system...")
    finally:
        arduino.stop()
        cam.stop()
        cam.close()
        print("Hardware safely released.")

if __name__ == '__main__':
    main()
