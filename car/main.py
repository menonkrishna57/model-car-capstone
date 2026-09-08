"""
main.py — Entry Point: Camera → Detect → Decide → Drive
========================================================
Owned by integration-agent.
Wires together camera capture, YOLOv10n NCNN detection, sensor monitoring,
and motor control to drive the autonomous model car.
"""

import sys
import time
import cv2
import numpy as np

from car.perception.config import (
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FPS,
    EMERGENCY_STOP_DISTANCE_CM,
)
from car.perception.detector import Detector
from car.perception.sensors import SensorManager
from car.control.motor_controller import MotorController
from car.control.navigation import NavigationSystem


def run_pipeline():
    print("🚀 Initializing Autonomous RC Car Pipeline...")

    # 1. Initialize detector
    print("  📦 Loading YOLOv10n NCNN detector...")
    try:
        detector = Detector()
    except Exception as e:
        print(f"❌ Failed to load detector: {e}")
        return

    # 2. Initialize sensors & motor controller
    sensors = SensorManager()
    motor = MotorController()

    if not motor.connect():
        print("⚠️ Running without active Arduino serial connection (dry run / offline mode).")

    nav = NavigationSystem(controller=motor, sensors=sensors)

    # 3. Camera capture setup
    # Try Picamera2 (Pi 5 global shutter) or fallback to OpenCV VideoCapture
    cap = None
    use_picam2 = False

    try:
        from picamera2 import Picamera2
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"})
        picam2.configure(config)
        picam2.start()
        use_picam2 = True
        print("  📷 CSI Camera initialized via Picamera2.")
    except Exception:
        print("  📷 Picamera2 unavailable; falling back to cv2.VideoCapture (USB / mock).")
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

    print("✅ System Ready. Starting control loop (Ctrl+C to stop)...")

    try:
        while True:
            # Capture frame
            if use_picam2:
                frame = picam2.capture_array()
            else:
                ret, frame = cap.read() if cap else (False, None)
                if not ret or frame is None:
                    # Simulated frame for testing if no camera hardware is connected
                    frame = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)

            # Check ultrasonic obstacle array
            distances = sensors.read_ultrasonic_distances()
            if distances.get("center", 100.0) < EMERGENCY_STOP_DISTANCE_CM:
                print("🛑 Immediate Stop: Obstacle detected!")
                motor.stop()
                time.sleep(0.1)
                continue

            # Run YOLOv10n inference
            detections = detector.detect(frame)

            # Process detections (person, vehicle, stop sign, etc.)
            for d in detections:
                if d["confidence"] > 0.4:
                    print(f"  👁️ Detected: {d['class_name']} ({d['confidence']:.2f})")

            # Read any serial telemetry from Arduino
            telem = motor.read_telemetry()
            if telem:
                print(f"  📡 Telemetry: {telem}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n🛑 Shutting down autonomous car loop...")
    finally:
        motor.stop()
        motor.disconnect()
        sensors.close()
        if cap is not None:
            cap.release()
        print("👋 Shutdown complete.")


if __name__ == "__main__":
    run_pipeline()
