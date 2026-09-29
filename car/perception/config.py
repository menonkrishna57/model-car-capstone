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
SERIAL_BAUD = 115200                   # Matches arduino/PROTOCOL.md

# ── Camera (CSI — Global Shutter) ────────────────────────────────────────────
# Managed via picamera2 on Pi 5
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# ── Ultrasonic Sensors (2× HC-SR04P, both front-facing) ──────────────────────
# Polling is owned by the Arduino Uno to avoid Pi OS GPIO jitter.
# Distances are received as '[TELEM] US LEFT=x RIGHT=y' serial messages
# (parsed by MotorController.read_telemetry → SensorManager.read_ultrasonic).
EMERGENCY_STOP_DISTANCE_CM = 20.0      # Stop immediately if obstacle closer than 20cm

# ── IMU (ICM20948 over I2C) ──────────────────────────────────────────────────
IMU_I2C_BUS = 1
IMU_I2C_ADDRESS = 0x68
