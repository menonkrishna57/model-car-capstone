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

IMGSZ = 320
OBSTACLE_CLASSES = [0, 1, 2, 3, 5, 7]  # person, bicycle, car, motorcycle, bus, truck
CONF_OBSTACLE = 0.35
CONF_POTHOLE = 0.20                     # Sensitive threshold for actual D40 potholes
POTHOLE_ONLY = True                     # STRICT FILTER: Only detect D40 potholes; ignore alligator/longitudinal cracks
POTHOLE_EVERY = 2

BBOX_STOP_HEIGHT_FRAC = 0.38
EMERGENCY_STOP_DIST_CM = 25.0
CRUISE_SPEED = 165

# ── 2. Camera Controls & Daylight White Balance ──────────────────────────────
try:
    from libcamera import controls
    AWB_MODES = {
        "daylight": controls.AwbModeEnum.Daylight,
        "auto": controls.AwbModeEnum.Auto,
        "indoor": controls.AwbModeEnum.Indoor,
        "cloudy": controls.AwbModeEnum.Cloudy,
        "tungsten": controls.AwbModeEnum.Tungsten,
    }
except Exception:
    AWB_MODES = {"auto": 0, "indoor": 4, "daylight": 5, "cloudy": 6, "tungsten": 2}

current_awb = "daylight"
swap_rb = True
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
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # Suppress tiny noise (text/carpet dots) and huge false areas
        if area < 1200 or area > roi_area * 0.30: continue
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

    def auto_connect(self):
        import serial
        for p in ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyUSB1", "/dev/ttyACM1"]:
            try:
                s = serial.Serial(p, 115200, timeout=0.1)
                time.sleep(2.0)
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
        try:
            while self.serial_port.in_waiting:
                line = self.serial_port.readline().decode("utf-8", errors="ignore").strip()
                if not line: continue
                if "US LEFT=" in line and "RIGHT=" in line:
                    for pt in line.split():
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
active_action_label = "IDLE (Safety Hold)"

def evaluate_autonomous_action(frame_w: int, frame_h: int, obs_boxes: list, pot_dets: list):
    global active_action_label

    obstacle_ahead = False
    for b in obs_boxes:
        x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
        height_frac = (y2 - y1) / max(frame_h, 1)
        cx = (x1 + x2) / 2
        if 0.20 * frame_w < cx < 0.80 * frame_w and height_frac > BBOX_STOP_HEIGHT_FRAC:
            obstacle_ahead = True
            break

    if obstacle_ahead:
        active_action_label = "🛑 PI_STOP: Obstacle Imminent"
        arduino.pi_stop()
        return

    if arduino.pi_latched and not obstacle_ahead:
        active_action_label = "✅ PI_GO: Path Cleared"
        arduino.pi_go()

    us_left = arduino.latest_sonar["left"]
    us_right = arduino.latest_sonar["right"]
    if us_left < EMERGENCY_STOP_DIST_CM or us_right < EMERGENCY_STOP_DIST_CM:
        active_action_label = f"⚠️ US AUTO-BRAKE (L:{us_left:.0f}cm R:{us_right:.0f}cm)"
        return

    # Evaluate true potholes
    danger_left = sum(1.0 for p in pot_dets if p.get("is_pothole") and (p["bbox"][0] + p["bbox"][2]) / 2 < frame_w * 0.5)
    danger_right = sum(1.0 for p in pot_dets if p.get("is_pothole") and (p["bbox"][0] + p["bbox"][2]) / 2 >= frame_w * 0.5)

    if danger_left > 0 or danger_right > 0:
        if danger_left > danger_right:
            active_action_label = "🔀 AVOIDING POTHOLE: Steering RIGHT"
            arduino.pi_avoid("RIGHT")
        else:
            active_action_label = "🔀 AVOIDING POTHOLE: Steering LEFT"
            arduino.pi_avoid("LEFT")
        return

    if arduino.motors_enabled:
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
  </header>

  <div class="controls-dock" id="dock">
    <button id="btn-motors" class="btn-danger" onclick="toggleMotors()">🚗 Enable Motors</button>
    <button class="btn-active" id="btn-daylight" onclick="setAwb('daylight')">☀️ Daylight</button>
    <button onclick="setAwb('auto')">🔄 Auto</button>
    <button onclick="setAwb('indoor')">💡 Indoor</button>
    <button id="btn-pothole-mode" class="btn-active" onclick="togglePotholeMode()">🕳️ Potholes Only</button>
    <button onclick="toggleFullscreen()">⛶ Fullscreen (F)</button>
    <button onclick="openRef()">📷 Ref daylight.jpg</button>
    <button onclick="toggleDock()" style="padding:7px 10px">👁️</button>
  </div>

  <div class="shortcut-tip">Press <b>F</b> for Fullscreen &bull; <b>M</b> for Motors &bull; <b>H</b> to Hide Dock &bull; <b>P</b> Pothole Mode</div>

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

    window.addEventListener('keydown', (e) => {
      const k = e.key.toLowerCase();
      if (k === 'f') toggleFullscreen();
      if (k === 'm') toggleMotors();
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
    for b in result.boxes:
        x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
        label = f"{prefix}{result.names[int(b.cls[0])]} {float(b.conf[0]):.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, max(18, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

def detect_loop(cam, output):
    global current_awb, swap_rb, POTHOLE_ONLY
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
        for p in last_pot_dets:
            x1, y1, x2, y2 = p["bbox"]
            color = (0, 69, 255) if p.get("is_pothole") else (255, 200, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, p["label"], (x1, max(18, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        now = time.time()
        fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-6)
        t_prev = now

        # HUD Overlays
        cv2.putText(frame, f"{fps:.1f} FPS (Pi 5 HW) | {current_awb.upper()} AWB", (14, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(frame, f"Action: {active_action_label}", (14, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 0) if "CRUISING" in active_action_label else (0, 165, 255), 2)

        us_str = f"US: L={arduino.latest_sonar['left']:.1f}cm  R={arduino.latest_sonar['right']:.1f}cm"
        ard_str = f"Arduino: {arduino.port_name} [{arduino.latest_state}]"
        cv2.putText(frame, us_str, (14, h - 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
        cv2.putText(frame, ard_str, (14, h - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 200, 200), 1)

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
    global output, cam_instance
    logging.basicConfig(level=logging.INFO)

    arduino.auto_connect()

    print("🎥 Initializing Picamera2 in BGR888 mode...")
    cam = Picamera2()
    cam_instance = cam
    cam.configure(cam.create_video_configuration(main={"size": (640, 480), "format": "BGR888"}))
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
