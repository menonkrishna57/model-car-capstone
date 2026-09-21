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
from car.perception.avoidance import compute_zone_scores, decide_avoidance
from car.perception.tracker import ObjectTracker
from car.control.pid import PID


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
    motor = MotorController()
    sensors = SensorManager(motor_controller=motor)

    if not motor.connect():
        print("⚠️ Running without active Arduino serial connection (dry run / offline mode).")

    nav = NavigationSystem(controller=motor, sensors=sensors)
    
    # 3. Initialize Brains (Tracker and PID)
    tracker = ObjectTracker()
    # PID for keeping person in center of frame
    # SetPoint is frame center. Output is steering angle.
    steering_pid = PID(P=0.3, I=0.01, D=0.05)
    steering_pid.SetPoint = CAMERA_WIDTH / 2.0
    steering_pid.setSampleTime(0.05)
    steering_pid.setWindup(50)

    # 4. Camera capture setup
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

    frame_count = 0

    try:
        while True:
            # Capture frame
            if use_picam2:
                frame = picam2.capture_array()
            else:
                ret, frame = cap.read() if cap else (False, None)
                if not ret or frame is None:
                    frame = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)

            frame_count += 1

            # -------------------------------------------------------------
            # PRIORITY 1: EMERGENCY STOP (Ultrasonics)
            # -------------------------------------------------------------
            distances = sensors.read_ultrasonic_distances()
            if distances.get("center", 100.0) < EMERGENCY_STOP_DISTANCE_CM:
                print("🛑 Immediate Stop: Obstacle detected within 20cm!")
                motor.stop()
                tracker.stop() # Lost target due to emergency
                time.sleep(0.1)
                continue

            # -------------------------------------------------------------
            # PRIORITY 2 & 3: AVOIDANCE & FOLLOW-ME
            # -------------------------------------------------------------
            
            # Run YOLO detector every 3rd frame to save CPU
            if frame_count % 3 == 0:
                detections = detector.detect(frame)
                
                # Filter low confidence
                valid_detections = [d for d in detections if d["confidence"] > 0.4]
                
                # Avoidance Engine
                scores = compute_zone_scores(valid_detections, CAMERA_WIDTH, CAMERA_HEIGHT)
                decision = decide_avoidance(scores)
                
                if decision != "CLEAR":
                    print(f"⚠️ Avoidance Maneuver: {decision}")
                    tracker.stop() # Drop tracking if avoiding
                    if decision == "STOP":
                        motor.stop()
                    elif decision == "LEFT":
                        motor.turn(-30, speed=160)
                    elif decision == "RIGHT":
                        motor.turn(30, speed=160)
                    continue
                
                # Follow-Me Engine (Only if CLEAR)
                if not tracker.is_tracking:
                    # Look for a person (COCO class 0)
                    persons = [d for d in valid_detections if d["class_name"] == "person"]
                    if persons:
                        # Pick the largest person bounding box
                        best_person = max(persons, key=lambda d: (d["bbox"][2]-d["bbox"][0]) * (d["bbox"][3]-d["bbox"][1]))
                        x1, y1, x2, y2 = map(int, best_person["bbox"])
                        w, h = x2 - x1, y2 - y1
                        tracker.init_tracker(frame, (x1, y1, w, h))
                        print(f"🎯 Locked onto person. Starting tracking.")
                        
            # If we are tracking, update PID and drive
            if tracker.is_tracking:
                success, bbox = tracker.update(frame)
                if success:
                    x, y, w, h = bbox
                    center_x = x + (w / 2)
                    
                    # PID loop for steering
                    steering_pid.update(center_x)
                    
                    # Convert PID output to angle (-45 to +45 deg)
                    # Output is positive if target is to the left of center (we need to steer left/negative)
                    # output is negative if target is right of center (we need to steer right/positive)
                    angle = -int(steering_pid.output) 
                    angle = max(-45, min(45, angle))
                    
                    # Deadband - don't steer if basically centered
                    if abs(angle) < 5:
                        motor.drive(160, 200) # Drive straight
                    else:
                        motor.turn(angle, speed=160)
                else:
                    print("🤷 Target lost.")
                    motor.stop()

            # Read any serial telemetry from Arduino
            telem = motor.read_telemetry()
            # if telem and "SONAR" not in telem: # Optional: print non-sonar telemetry
            #     print(f"  📡 Telemetry: {telem}")

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
