#!/usr/bin/env python3
"""
stream.py — Lightweight MJPEG Live Camera Streamer for Pi 5
===========================================================
Used for bench testing and camera orientation without the high latency
and CPU overhead of remote desktop / Raspberry Pi Connect screen sharing.

Features:
    - ☀️ Daylight AWB filter enabled by default (matches user's daylight.jpg)
    - 🎛️ Dynamic AWB switcher (/set_awb?mode=daylight|auto|indoor|cloudy)
    - 📷 Reference image comparison endpoint (/daylight.jpg)

Usage on Raspberry Pi 5:
    python3 tools/stream.py
    # or
    python3 stream.py

Access from laptop browser on same Wi-Fi / hotspot:
    http://<pi-ip-address>:8000
"""

import io
import json
import logging
import socketserver
import urllib.parse
from http import server
from pathlib import Path
from threading import Condition

from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput

PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Model Car Live Stream (Daylight AWB)</title>
  <style>
    :root {
      --bg: #0d1117;
      --card: #161b22;
      --border: #30363d;
      --accent: #58a6ff;
      --success: #3fb950;
      --text: #f0f6fc;
      --muted: #8b949e;
      --gold: #e3b341;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background: var(--bg);
      color: var(--text);
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 16px;
    }
    header {
      width: 100%;
      max-width: 800px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 12px;
      padding-bottom: 10px;
      border-bottom: 1px solid var(--border);
    }
    h1 { font-size: 1.2rem; }
    .badge {
      background: rgba(63, 185, 80, 0.15);
      color: var(--success);
      border: 1px solid var(--success);
      padding: 3px 8px;
      border-radius: 10px;
      font-size: 0.75rem;
      font-weight: 600;
    }
    .video-box {
      background: #000;
      border-radius: 8px;
      overflow: hidden;
      border: 1px solid var(--border);
      max-width: 100%;
    }
    img#stream {
      display: block;
      width: 640px;
      max-width: 100%;
      height: auto;
      object-fit: contain;
    }
    .controls {
      width: 100%;
      max-width: 800px;
      margin-top: 12px;
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 12px 16px;
      display: flex;
      flex-wrap: wrap;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
    }
    .btn-group {
      display: flex;
      align-items: center;
      gap: 6px;
      flex-wrap: wrap;
    }
    button {
      background: #21262d;
      color: var(--text);
      border: 1px solid var(--border);
      padding: 5px 10px;
      border-radius: 6px;
      font-size: 0.8rem;
      cursor: pointer;
    }
    button.active {
      background: #1f6feb;
      border-color: #58a6ff;
    }
    #status {
      margin-top: 8px;
      font-size: 0.8rem;
      color: var(--muted);
    }
  </style>
</head>
<body>
  <header>
    <h1>📹 Live Camera Stream</h1>
    <div class="badge" id="badge">AWB: DAYLIGHT ☀️</div>
  </header>
  <div class="video-box">
    <img id="stream" src="stream.mjpg" alt="Live Camera Feed">
  </div>
  <div class="controls">
    <div class="btn-group">
      <span style="font-size:0.8rem; color:var(--muted)">AWB Preset:</span>
      <button class="active" onclick="setAwb('daylight', this)">☀️ Daylight</button>
      <button onclick="setAwb('auto', this)">🔄 Auto</button>
      <button onclick="setAwb('indoor', this)">💡 Indoor</button>
      <button onclick="setAwb('cloudy', this)">☁️ Cloudy</button>
      <button onclick="setAwb('tungsten', this)">🏮 Tungsten</button>
    </div>
  </div>
  <div id="status">Daylight white balance active (matching daylight.jpg).</div>

  <script>
    function setAwb(mode, btn) {
      document.getElementById('status').innerText = 'Setting AWB to ' + mode + '...';
      fetch('/set_awb?mode=' + mode)
        .then(r => r.json())
        .then(data => {
          if (data.status === 'ok') {
            document.querySelectorAll('button').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            document.getElementById('badge').innerText = 'AWB: ' + data.mode.toUpperCase() + (data.mode === 'daylight' ? ' ☀️' : '');
            document.getElementById('status').innerText = '✅ Applied ' + data.mode.toUpperCase() + ' AWB.';
          }
        });
    }
  </script>
</body>
</html>
"""

try:
    from libcamera import controls
    AWB_MODES = {
        "daylight": controls.AwbModeEnum.Daylight,
        "auto": controls.AwbModeEnum.Auto,
        "indoor": controls.AwbModeEnum.Indoor,
        "cloudy": controls.AwbModeEnum.Cloudy,
        "tungsten": controls.AwbModeEnum.Tungsten,
        "fluorescent": controls.AwbModeEnum.Fluorescent,
        "incandescent": controls.AwbModeEnum.Incandescent,
    }
except Exception:
    AWB_MODES = {
        "auto": 0,
        "incandescent": 1,
        "tungsten": 2,
        "fluorescent": 3,
        "indoor": 4,
        "daylight": 5,
        "cloudy": 6,
    }

current_awb = "daylight"
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
        print(f"☀️ Camera AWB set to: {mode_name.upper()} (matches daylight.jpg profile)")
        return True
    except Exception as e:
        print(f"⚠️ Failed to set AwbMode to {mode_name}: {e}")
        return False

def find_daylight_ref() -> Path:
    candidates = [
        Path("/home/capstone/daylight.jpg"),
        Path("/home/capstone/model/daylight.jpg"),
        Path.cwd() / "daylight.jpg",
        Path.home() / "daylight.jpg",
    ]
    for p in candidates:
        if p.exists():
            return p
    return None

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

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
                self.send_error(404, "daylight.jpg not found on device")
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
                logging.warning('Client disconnected: %s', e)
        else:
            self.send_error(404)

class Server(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

def main():
    global output, cam_instance
    logging.basicConfig(level=logging.INFO)
    print("🎥 Initializing Picamera2 (640x480 @ 30 FPS)...")
    cam = Picamera2()
    cam_instance = cam
    cam.configure(cam.create_video_configuration(main={"size": (640, 480)}))
    output = StreamingOutput()
    cam.start_recording(MJPEGEncoder(), FileOutput(output))

    # Apply Daylight AWB filter matching daylight.jpg
    set_awb_mode(cam, "daylight")

    port = 8000
    print(f"🚀 MJPEG server running with Daylight AWB on port {port}.")
    print("👉 View stream in browser at: http://<pi-ip>:8000")
    try:
        Server(('', port), Handler).serve_forever()
    except KeyboardInterrupt:
        print("\n🛑 Stopping stream...")
    finally:
        cam.stop_recording()
        cam.close()
        print("Camera released.")

if __name__ == '__main__':
    main()
