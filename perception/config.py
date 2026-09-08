"""
config.py — Runtime Configuration for Pi 5 Perception
======================================================
Central place for all tunable parameters. Import this instead of
hardcoding values throughout the perception code.
"""

from pathlib import Path

# ── Model ────────────────────────────────────────────────────────────────
MODEL_PATH = "yolov10n_ncnn_model"     # Path to NCNN model directory
IMGSZ = 320                            # Inference resolution (square)
CONFIDENCE_THRESHOLD = 0.25            # Minimum detection confidence

# ── Serial (Pi ↔ Arduino Nano) ───────────────────────────────────────────
# Transport not yet decided (USB-serial vs GPIO-UART).
# Update this once the decision is made.
SERIAL_PORT = "/dev/ttyUSB0"           # USB-serial default; change to
                                       # "/dev/ttyAMA0" for GPIO UART
SERIAL_BAUD = 9600                     # Must match Arduino sketch

# ── Camera (CSI — global shutter) ────────────────────────────────────────
# Pi Camera connector not yet purchased — these are placeholder settings.
CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# ── Ultrasonic (HC-SR04P × 2) ───────────────────────────────────────────
# GPIO BCM pin numbers — update once wiring is finalised.
ULTRASONIC_SENSORS = [
    {"name": "front_left",  "trigger": 23, "echo": 24},
    {"name": "front_right", "trigger": 25, "echo": 8},
]
ULTRASONIC_STOP_DISTANCE_CM = 20.0     # Emergency stop threshold

# ── IMU (ICM20948 over I2C) ─────────────────────────────────────────────
IMU_I2C_BUS = 1
IMU_I2C_ADDRESS = 0x68
