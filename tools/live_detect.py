#!/usr/bin/env python3
"""
live_detect.py — Fullscreen Live Perception & Autonomous Control System
========================================================================
Combines high-speed camera streaming with parallel AI perception and
real-time Arduino Uno motor/ultrasonic actuation for teacher demonstrations.

Features:
    - 📺 Fullscreen Presentation Mode: Fills 100% of Chrome window (Press 'F' for Fullscreen)
    - ☀️ Daylight AWB Profile: Natural true-to-life lighting matching daylight.jpg
    - 🎨 True BGR Color Ordering: Eliminates the blue skin / inverted color artifact
    - 🧠 Dual-AI Perception: YOLOv10n obstacles (Green) + RDD2022/CV Potholes (Red)
    - 🤖 Parallel Arduino Uno Bridge: Auto-connects over USB-serial (115200 baud)
    - 📡 Ultrasonic Telemetry HUD: Real-time Left/Right HC-SR04P distance display
    - 🛡️ 3-Tier Priority Execution:
        Tier 1: Pi Camera Obstacle Stop (PI_STOP / PI_GO)
        Tier 1: Pothole & Zone Avoidance (PI_AVOID LEFT / RIGHT)
        Tier 2: Arduino Local Ultrasonic Auto-Brake (<25 cm reflex)
        Tier 3: Forward Cruise (DRIVE 165 300)
    - 🔒 Motor Safety Toggle: Bench-test safely; enable motors via web UI or 'M' key

Usage:
    source ~/model/.venv/bin/activate
    python3 ~/live_detect.py
    # or
    python3 tools/live_detect.py

Open in Chrome:
    http://<pi-ip>:8000
"""

import io
import json
import logging
import socketserver
import sys
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

# ── Dynamic Path Discovery ───────────────────────────────────────────────────
def find_repo_root() -> Path:
    candidates = [
        Path("/home/capstone/model"),
        Path.home() / "model",
        Path(__file__).resolve().parent.parent,
        Path(__file__).resolve().parent,
        Path.cwd(),
    ]
    for p in candidates:
        if (p / "benchmarking" / "models" / "yolov10n_ncnn_model").exists():
            return p
    return candidates[0]

REPO_ROOT = find_repo_root()
print(f"📁 Repository detected at: {REPO_ROOT}")

OBSTACLE_MODEL = str(REPO_ROOT / "benchmarking" / "models" / "yolov10n_ncnn_model")
POTHOLE_MODEL = str(REPO_ROOT / "pothole_detection" / "best_ncnn_model")

def find_daylight_ref() -> Optional[Path]:
    candidates = [
        Path("/home/capstone/daylight.jpg"),
        Path("/home/capstone/model/daylight.jpg"),
        Path.cwd() / "daylight.jpg",
        REPO_ROOT / "daylight.jpg",
        Path.home() / "daylight.jpg",
    ]
    for p in candidates:
        if p.exists():
            return p
    return None

IMGSZ = 320
OBSTACLE_CLASSES = [0, 1, 2, 3, 5, 7]  # person, bicycle, car, motorcycle, bus, truck
CONF_OBSTACLE = 0.35
CONF_POTHOLE = 0.30
POTHOLE_EVERY = 2

# Autonomous Thresholds
BBOX_STOP_HEIGHT_FRAC = 0.38   # If obstacle height > 38% of frame -> PI_STOP
EMERGENCY_STOP_DIST_CM = 25.0  # Ultrasonic reflex stop
CRUISE_SPEED = 165             # Forward PWM
SLOW_SPEED = 120

# ── Camera Controls & AWB Definitions ────────────────────────────────────────
try:
    from libcamera import controls
    AWB_MODES = {
        "daylight": controls.AwbModeEnum.Daylight,
        "auto": controls.AwbModeEnum.Auto,
        "indoor": controls.AwbModeEnum.Indoor,
        "cloudy": controls.AwbModeEnum.Cloudy,
        "tungsten": controls.AwbModeEnum.Tungsten,
        "fluorescent": controls.AwbModeEnum.Fluorescent,
    }
except Exception:
    AWB_MODES = {
        "auto": 0, "indoor": 4, "daylight": 5, "cloudy": 6, "tungsten": 2, "fluorescent": 3
    }

current_awb = "daylight"
swap_rb = True
cam_instance = None

def set_awb_mode(cam, mode_name: str) -> bool:
    global current_awb
    mode_name = mode_name.lower().strip()
    if mode_name not in AWB_MODES:
        return False
    val = AWB_MODES[mode_name]
    try:
        cam.set_controls({"AwbMode": val})
        current_awb = mode_name
        print(f"☀️ Camera AWB set to: {mode_name.upper()} (matches daylight.jpg)")
        return True
    except Exception as e:
        print(f"⚠️ Failed to set AwbMode: {e}")
        return False

# ── Load AI Models ───────────────────────────────────────────────────────────
print(f"📦 Loading YOLOv10n obstacle model from: {OBSTACLE_MODEL}")
obs_model = YOLO(OBSTACLE_MODEL, task="detect")

pot_model = None
if Path(POTHOLE_MODEL).exists():
    print(f"📦 Loading Pothole model from: {POTHOLE_MODEL}")
    try:
        pot_model = YOLO(POTHOLE_MODEL, task="detect")
    except Exception as e:
        print(f"⚠️ Could not load NCNN pothole model: {e}")
else:
    print(f"ℹ️ {POTHOLE_MODEL} not found. Using classical CV Laplacian pothole detector.")

def cv_detect_potholes(frame: np.ndarray) -> List[Dict[str, Any]]:
    h, w = frame.shape[:2]
    roi_y = int(h * 0.45)
    roi = frame[roi_y:, :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    lap = cv2.Laplacian(blur, cv2.CV_64F)
    lap_u = np.uint8(np.clip(np.absolute(lap), 0, 255))
    _, thresh = cv2.threshold(lap_u, 28, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    dets = []
    roi_area = roi.shape[0] * roi.shape[1]
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 900 or area > roi_area * 0.25:
            continue
        x, y, cw, ch = cv2.boundingRect(cnt)
        if not (0.25 < cw / max(ch, 1) < 5.0):
            continue
        dets.append({"bbox": [x, y + roi_y, x + cw, y + roi_y + ch], "conf": 0.42, "type": "negative"})
    return dets

# ── Parallel Arduino Serial Bridge ───────────────────────────────────────────
class ArduinoBridge:
    """Manages full-duplex communication with Arduino Uno at 115200 baud."""
    def __init__(self):
        self.serial_port = None
        self.port_name = "SIMULATED"
        self.connected = False
        self.lock = Lock()
        self.latest_sonar = {"left": 999.0, "right": 999.0}
        self.latest_state = "IDLE"
        self.last_rx = 0.0
        self.motors_enabled = False  # Safety gate for bench testing
        self.pi_latched = False

    def auto_connect(self):
        import serial
        candidates = ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyUSB1", "/dev/ttyACM1"]
        for p in candidates:
            try:
                s = serial.Serial(p, 115200, timeout=0.1)
                time.sleep(2.0)  # Allow ATmega328P bootloader to settle
                self.serial_port = s
                self.port_name = p
                self.connected = True
                print(f"🔌 Arduino connected on {p} @ 115200 baud")
                return True
            except Exception:
                continue
        print("⚠️ No physical Arduino detected. Operating in DRY-RUN mode.")
        self.connected = False
        return False

    def send(self, cmd: str) -> bool:
        if not self.connected or not self.serial_port:
            return False
        with self.lock:
            try:
                self.serial_port.write((cmd.strip() + "\n").encode("utf-8"))
                self.serial_port.flush()
                return True
            except Exception as e:
                print(f"⚠️ Serial send error: {e}")
                self.connected = False
                return False

    def read_telemetry(self):
        if not self.connected or not self.serial_port:
            return
        try:
            while self.serial_port.in_waiting:
                line = self.serial_port.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                self.last_rx = time.time()
                # Parse: [TELEM] US LEFT=x.x RIGHT=y.y
                if "US LEFT=" in line and "RIGHT=" in line:
                    parts = line.split()
                    for pt in parts:
                        if pt.startswith("LEFT="):
                            try: self.latest_sonar["left"] = float(pt.split("=")[1])
                            except: pass
                        elif pt.startswith("RIGHT="):
                            try: self.latest_sonar["right"] = float(pt.split("=")[1])
                            except: pass
                elif "STATE=" in line:
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
        self.send(f"PI_AVOID {direction}")

    def drive(self, speed: int, duration_ms: int):
        if self.motors_enabled and not self.pi_latched:
            self.send(f"DRIVE {speed} {duration_ms}")

    def stop(self):
        self.send("STOP")

arduino = ArduinoBridge()

# ── Autonomous Decision Logic (Zones & Priority) ─────────────────────────────
active_action_label = "IDLE (Safety Hold)"

def evaluate_autonomous_action(frame_w: int, frame_h: int, obs_boxes: list, pot_boxes: list):
    """
    Parallel arbitration:
      1. Obstacle directly ahead > 38% height -> PI_STOP
      2. Pothole / Road hazard -> PI_AVOID LEFT/RIGHT
      3. Ultrasonic auto-brake (<25cm) -> Arduino handles reflex
      4. Clear path -> Forward DRIVE
    """
    global active_action_label

    # Check for direct forward obstacle (Rule 2)
    obstacle_ahead = False
    for b in obs_boxes:
        x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
        height_frac = (y2 - y1) / max(frame_h, 1)
        cx = (x1 + x2) / 2
        # If in center 60% of frame and tall enough
        if 0.20 * frame_w < cx < 0.80 * frame_w and height_frac > BBOX_STOP_HEIGHT_FRAC:
            obstacle_ahead = True
            break

    # 1. PI_STOP Tier 1
    if obstacle_ahead:
        active_action_label = "🛑 PI_STOP: Obstacle Imminent"
        arduino.pi_stop()
        return

    # If was latched, release if path clear
    if arduino.pi_latched and not obstacle_ahead:
        active_action_label = "✅ PI_GO: Path Cleared"
        arduino.pi_go()

    # 2. Ultrasonic Reflex Check
    us_left = arduino.latest_sonar["left"]
    us_right = arduino.latest_sonar["right"]
    if us_left < EMERGENCY_STOP_DIST_CM or us_right < EMERGENCY_STOP_DIST_CM:
        active_action_label = f"⚠️ US AUTO-BRAKE (L:{us_left:.0f}cm R:{us_right:.0f}cm)"
        # Arduino handles reflex locally per Tier 2
        return

    # 3. Pothole / Road Obstacle Avoidance
    danger_left = 0.0
    danger_right = 0.0

    # Evaluate potholes in bottom half
    for p in pot_boxes:
        x1, y1, x2, y2 = p["bbox"]
        cx = (x1 + x2) / 2
        if cx < frame_w * 0.5:
            danger_left += 1.0
        else:
            danger_right += 1.0

    if danger_left > 0 or danger_right > 0:
        if danger_left > danger_right:
            active_action_label = "🔀 AVOIDING POTHOLE: Steering RIGHT"
            arduino.pi_avoid("RIGHT")
        else:
            active_action_label = "🔀 AVOIDING POTHOLE: Steering LEFT"
            arduino.pi_avoid("LEFT")
        return

    # 4. Clear Path: Forward Cruise Pulse
    if arduino.motors_enabled:
        active_action_label = f"🟢 CRUISING FORWARD (PWM {CRUISE_SPEED})"
        arduino.drive(CRUISE_SPEED, 250)
    else:
        active_action_label = "⏸️ MOTORS DISABLED (Bench Test Safe)"

# ── Fullscreen Web Server Components ─────────────────────────────────────────
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
      --accent: #58a6ff;
      --success: #3fb950;
      --danger: #f85149;
      --warning: #d29922;
      --text: #f0f6fc;
      --muted: #8b949e;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    html, body {
      background: var(--bg);
      color: var(--text);
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      width: 100vw;
      height: 100vh;
      overflow: hidden;
      display: flex;
      justify-content: center;
      align-items: center;
      user-select: none;
    }

    /* Video fills 100% of Chrome viewport */
    #viewport {
      position: absolute;
      top: 0; left: 0;
      width: 100vw; height: 100vh;
      display: flex;
      justify-content: center;
      align-items: center;
      background: #000;
    }
    img#stream {
      width: 100%;
      height: 100%;
      object-fit: contain;
      display: block;
    }

    /* Floating Glassmorphic Top Status Bar */
    header {
      position: absolute;
      top: 14px;
      left: 50%;
      transform: translateX(-50%);
      background: var(--card-glass);
      backdrop-filter: blur(12px);
      -webkit-backdrop-filter: blur(12px);
      border: 1px solid var(--border-glass);
      border-radius: 30px;
      padding: 6px 18px;
      display: flex;
      align-items: center;
      gap: 14px;
      box-shadow: 0 8px 32px rgba(0,0,0,0.6);
      z-index: 100;
      transition: opacity 0.3s ease;
    }
    .badge {
      padding: 3px 10px;
      border-radius: 12px;
      font-size: 0.75rem;
      font-weight: 700;
      letter-spacing: 0.5px;
    }
    .badge-daylight { background: rgba(227, 179, 65, 0.2); color: #e3b341; border: 1px solid #e3b341; }
    .badge-motors-on { background: rgba(63, 185, 80, 0.2); color: var(--success); border: 1px solid var(--success); }
    .badge-motors-off { background: rgba(248, 81, 73, 0.2); color: var(--danger); border: 1px solid var(--danger); }

    /* Floating Bottom Control Pill */
    .controls-dock {
      position: absolute;
      bottom: 16px;
      left: 50%;
      transform: translateX(-50%);
      background: var(--card-glass);
      backdrop-filter: blur(14px);
      -webkit-backdrop-filter: blur(14px);
      border: 1px solid var(--border-glass);
      border-radius: 28px;
      padding: 6px 14px;
      display: flex;
      align-items: center;
      gap: 8px;
      box-shadow: 0 8px 32px rgba(0,0,0,0.6);
      z-index: 100;
      transition: all 0.3s ease;
    }
    .controls-dock.hidden {
      opacity: 0;
      pointer-events: none;
      transform: translateX(-50%) translateY(20px);
    }
    button {
      background: rgba(255, 255, 255, 0.08);
      color: var(--text);
      border: 1px solid var(--border-glass);
      padding: 7px 14px;
      border-radius: 18px;
      font-size: 0.8rem;
      font-weight: 600;
      cursor: pointer;
      display: inline-flex;
      align-items: center;
      gap: 6px;
      transition: all 0.15s ease;
    }
    button:hover { background: rgba(255, 255, 255, 0.18); border-color: rgba(255, 255, 255, 0.3); }
    button.btn-active { background: #1f6feb; border-color: #58a6ff; color: #fff; }
    button.btn-danger { background: rgba(248, 81, 73, 0.25); border-color: var(--danger); color: #ff7b72; }
    button.btn-danger:hover { background: rgba(248, 81, 73, 0.4); }
    button.btn-success { background: rgba(63, 185, 80, 0.25); border-color: var(--success); color: #56d364; }

    /* Key shortcut hints */
    .shortcut-tip {
      position: absolute;
      bottom: 6px;
      right: 12px;
      font-size: 0.7rem;
      color: var(--muted);
      opacity: 0.6;
    }

    /* Modal for Reference Image */
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
  </header>

  <div class="controls-dock" id="dock">
    <button id="btn-motors" class="btn-danger" onclick="toggleMotors()">🚗 Enable Motors</button>
    <button class="btn-active" id="btn-daylight" onclick="setAwb('daylight')">☀️ Daylight</button>
    <button onclick="setAwb('auto')">🔄 Auto</button>
    <button onclick="setAwb('indoor')">💡 Indoor</button>
    <button onclick="toggleFullscreen()">⛶ Fullscreen (F)</button>
    <button onclick="openRef()">📷 Ref daylight.jpg</button>
    <button onclick="toggleDock()" style="padding:7px 10px">👁️</button>
  </div>

  <div class="shortcut-tip">Press <b>F</b> for Fullscreen &bull; <b>M</b> for Motors &bull; <b>H</b> to Hide Dock</div>

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
          btn.innerText = '🛑 Disable Motors';
          btn.className = 'btn-success';
          badge.innerText = 'MOTORS: ACTIVE ⚡';
          badge.className = 'badge badge-motors-on';
        } else {
          btn.innerText = '🚗 Enable Motors';
          btn.className = 'btn-danger';
          badge.innerText = 'MOTORS: OFF ⏸️';
          badge.className = 'badge badge-motors-off';
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

    // Keyboard Shortcuts for Presentations
    window.addEventListener('keydown', (e) => {
      const k = e.key.toLowerCase();
      if (k === 'f') toggleFullscreen();
      if (k === 'm') toggleMotors();
      if (k === 'h') toggleDock();
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
    for b in result.boxes:
        x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
        label = f"{prefix}{result.names[int(b.cls[0])]} {float(b.conf[0]):.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, max(18, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

# ── Main Parallel Processing & Detection Loop ────────────────────────────────
def detect_loop(cam, output):
    global current_awb, swap_rb
    n = 0
    last_pot = None
    last_cv_pot = []
    fps = 0.0
    t_prev = time.time()

    print("🚀 Parallel Detection & Control Loop Active!")
    while True:
        # 1. Drain Arduino Telemetry
        arduino.read_telemetry()

        # 2. Grab Camera Frame
        frame_raw = cam.capture_array()
        if swap_rb:
            frame = cv2.cvtColor(frame_raw, cv2.COLOR_RGB2BGR)
        else:
            frame = frame_raw.copy()

        h, w = frame.shape[:2]

        # 3. Obstacle Detection (YOLOv10n NCNN)
        r = obs_model.predict(frame, imgsz=IMGSZ, conf=CONF_OBSTACLE,
                              classes=OBSTACLE_CLASSES, verbose=False)[0]

        # 4. Pothole Detection (Deep Model or Classical CV)
        if pot_model is not None:
            if n % POTHOLE_EVERY == 0:
                last_pot = pot_model.predict(frame, imgsz=IMGSZ, conf=CONF_POTHOLE, verbose=False)[0]
        else:
            if n % POTHOLE_EVERY == 0:
                last_cv_pot = cv_detect_potholes(frame)

        # 5. Parallel Arbitration & Motor Decision
        evaluate_autonomous_action(w, h, r.boxes, last_cv_pot)

        # 6. Render Overlays
        draw_boxes(frame, r, (0, 255, 0))

        if pot_model is not None and last_pot is not None:
            draw_boxes(frame, last_pot, (0, 0, 255), prefix="pothole ")
        elif last_cv_pot:
            for d in last_cv_pot:
                x1, y1, x2, y2 = d["bbox"]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, f"pothole {d['conf']:.2f}", (x1, max(18, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 7. Presentation HUD Overlays
        now = time.time()
        fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-6)
        t_prev = now

        # Top Overlay: FPS + Action
        cv2.putText(frame, f"{fps:.1f} FPS (Pi 5 HW) | {current_awb.upper()} AWB", (14, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(frame, f"Action: {active_action_label}", (14, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 0) if "CRUISING" in active_action_label else (0, 165, 255), 2)

        # Bottom Overlay: Ultrasonic Sensors + Arduino State
        us_str = f"US: L={arduino.latest_sonar['left']:.1f}cm  R={arduino.latest_sonar['right']:.1f}cm"
        ard_str = f"Arduino: {arduino.port_name} [{arduino.latest_state}]"
        cv2.putText(frame, us_str, (14, h - 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
        cv2.putText(frame, ard_str, (14, h - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 200, 200), 1)

        # JPEG Encoding
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            output.write(jpg.tobytes())

        n += 1

# ── HTTP Request Handler ─────────────────────────────────────────────────────
class Handler(server.BaseHTTPRequestHandler):
    def do_GET(self):
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
    global output, cam_instance
    logging.basicConfig(level=logging.INFO)

    # 1. Initialize Arduino
    arduino.auto_connect()

    # 2. Initialize Picamera2
    print("🎥 Initializing Picamera2 in BGR888 mode...")
    cam = Picamera2()
    cam_instance = cam
    cam.configure(cam.create_video_configuration(main={"size": (640, 480), "format": "BGR888"}))
    cam.start()

    # Apply Daylight AWB preset
    set_awb_mode(cam, "daylight")

    output = StreamingOutput()
    worker = threading.Thread(target=detect_loop, args=(cam, output), daemon=True)
    worker.start()

    port = 8000
    print(f"🚀 Fullscreen presentation & control ready at: http://<pi-ip>:{port}")
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
