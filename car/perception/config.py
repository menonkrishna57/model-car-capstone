"""
config.py — Runtime Configuration for Pi 5 Perception & Navigation
==================================================================
Owned by perception-agent.
Central place for sensor pins, model paths, thresholds, and I2C addresses.
"""

from pathlib import Path

# ── Model & Inference ────────────────────────────────────────────────────────
# Relative to repo root or car/ runtime directory
REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MODEL_PATH = str(REPO_ROOT / "benchmarking" / "models" / "yolov10n_ncnn_model")
IMGSZ = 320                            # Optimal resolution identified in benchmarks
CONFIDENCE_THRESHOLD = 0.25            # Detection threshold

# ── Serial Link (Pi 5 ↔ Arduino Uno) ─────────────────────────────────────────
# Default transport: USB-serial (/dev/ttyUSB0 or /dev/ttyACM0).
# For GPIO UART: /dev/ttyAMA0
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 9600                     # Matches arduino/PROTOCOL.md

# ── Camera (CSI — Global Shutter) ────────────────────────────────────────────
# Managed via picamera2 on Pi 5
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# ── Ultrasonic Array (3× HC-SR04P, 3.3V/5V compatible) ───────────────────────
# GPIO BCM pins: Left, Center, Right
ULTRASONIC_SENSORS = [
    {"name": "left",   "trigger": 23, "echo": 24},
    {"name": "center", "trigger": 17, "echo": 27},
    {"name": "right",  "trigger": 25, "echo": 8},
]
EMERGENCY_STOP_DISTANCE_CM = 20.0      # Stop immediately if obstacle closer than 20cm

# ── IMU (ICM20948 over I2C) ──────────────────────────────────────────────────
IMU_I2C_BUS = 1
IMU_I2C_ADDRESS = 0x68
