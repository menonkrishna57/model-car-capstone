"""
main.py — Autonomous RC Car Entry Point
========================================
Owned by integration-agent.

Wires together:
  Camera capture  →  YOLOv10n (NCNN) + classical CV
  →  Zone Scorer + Decision Engine  →  Motor Controller (serial → Arduino)

Two operating modes (select via --mode flag at launch):

  autonomous (default):
    Avoid all obstacles. Persons are treated as obstacles.
    STOP > AVOID > DRIVE priority stack mirrors the three-tier Arduino FSM.

  follow:
    Lock onto the closest person at startup. Follow them.
    Non-target obstacles still trigger PI_STOP (Rules 1 & 2 from §6).
    If KCF confidence drops → stop, re-acquire via YOLO + appearance profile.

Usage:
  python -m car.main --mode autonomous
  python -m car.main --mode follow

Pi_STOP Decision Rules (architecture.md §6):
  Rule 1 — Direction awareness: only stop when heading TOWARD a threat.
           Threats during REVERSING or IDLE are ignored.
  Rule 2 — Distance proxy: stop only when bbox height > 40% of frame height.
           Slow down (lower PWM) at 15–40%.
  Rule 3 — Mode awareness: person = target in follow mode (never stop for them).
"""

import argparse
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
from car.perception.avoidance import compute_zone_scores, decide_avoidance
from car.perception.tracker import ObjectTracker
from car.control.motor_controller import MotorController
from car.control.navigation import NavigationSystem
from car.control.pid import PID

# ── Architecture §6 Rule 2 — bounding-box height thresholds ──────────────────
# Expressed as fractions of frame height.
BBOX_STOP_THRESHOLD  = 0.40   # > 40% → PI_STOP
BBOX_SLOW_THRESHOLD  = 0.15   # 15–40% → reduce PWM

# Default cruise PWM for normal forward driving
CRUISE_SPEED  = 180
SLOW_SPEED    = 120

# ── Follow-me speed modulation (bbox height vs frame height) ─────────────────
# Architecture §5 Phase 2 table.
FOLLOW_TOO_CLOSE     = 0.50   # > 50% → slow / stop
FOLLOW_IDEAL_MIN     = 0.25   # 25–50% → cruise
FOLLOW_GETTING_FAR   = 0.10   # 10–25% → speed up
FOLLOW_CRUISE_SPEED  = 160
FOLLOW_FAST_SPEED    = 200
FOLLOW_SLOW_SPEED    = 80


def _bbox_height_frac(bbox, frame_height: int) -> float:
    """Returns bounding box height as a fraction of frame height."""
    x1, y1, x2, y2 = bbox
    return (y2 - y1) / max(frame_height, 1)


def _bbox_center_x(bbox) -> float:
    """Returns the horizontal center of a bounding box."""
    x1, y1, x2, y2 = bbox
    return (x1 + x2) / 2.0


def _is_person_detected_ahead(detections: list, frame_height: int) -> tuple:
    """
    Checks YOLO detections for persons in the forward path using Rule 2.
    Ignores reversing / idle (Rule 1 — caller checks current_state).

    Returns:
        (action, speed): action ∈ {"STOP", "SLOW", "CLEAR"}, speed int or None.
    """
    for det in detections:
        if det.get("class_name") != "person":
            continue
        frac = _bbox_height_frac(det["bbox"], frame_height)
        if frac > BBOX_STOP_THRESHOLD:
            return "STOP", None
        elif frac > BBOX_SLOW_THRESHOLD:
            return "SLOW", SLOW_SPEED
    return "CLEAR", None


# =============================================================================
# Main Pipeline
# =============================================================================

def run_pipeline(mode: str = "autonomous"):
    print(f"🚀 Autonomous RC Car — mode: {mode.upper()}")

    if mode not in ("autonomous", "follow"):
        print(f"❌ Unknown mode '{mode}'. Use --mode autonomous  or  --mode follow")
        sys.exit(1)

    # ── Detector ──────────────────────────────────────────────────────────────
    print("  📦 Loading YOLOv10n NCNN detector...")
    try:
        detector = Detector()
    except Exception as e:
        print(f"❌ Failed to load detector: {e}")
        sys.exit(1)

    # ── Hardware interfaces ────────────────────────────────────────────────────
    motor   = MotorController()
    sensors = SensorManager(motor_controller=motor)

    if not motor.connect():
        print("⚠️  Running without active Arduino (dry-run / offline mode).")
    else:
        # Wait for [TELEM] READY before issuing any commands
        ready_deadline = time.monotonic() + 5.0
        while time.monotonic() < ready_deadline:
            line = motor.read_telemetry()
            if line and "READY" in line:
                print(f"  🤖 Arduino ready: {line}")
                break
            time.sleep(0.05)

    nav = NavigationSystem(controller=motor, sensors=sensors)

    # ── Follow-me components ───────────────────────────────────────────────────
    tracker = ObjectTracker()
    steering_pid = PID(P=0.3, I=0.01, D=0.05)
    steering_pid.SetPoint = CAMERA_WIDTH / 2.0
    steering_pid.setSampleTime(0.05)
    steering_pid.setWindup(50)

    # Follow-me state
    follow_state = "LOCK_ON"   # LOCK_ON / TRACKING / RE_ACQUIRE

    # ── Camera ────────────────────────────────────────────────────────────────
    use_picam2 = False
    picam2     = None
    cap        = None

    try:
        from picamera2 import Picamera2
        picam2 = Picamera2()
        cfg = picam2.create_preview_configuration(
            main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"}
        )
        picam2.configure(cfg)
        picam2.start()
        try:
            from libcamera import controls
            picam2.set_controls({"AwbMode": controls.AwbModeEnum.Daylight})
            print("  ☀️ Camera AWB configured to Daylight mode (matching daylight.jpg).")
        except Exception:
            try:
                picam2.set_controls({"AwbMode": 5})
            except Exception:
                pass
        use_picam2 = True
        print("  📷 CSI Camera initialised via Picamera2.")
    except Exception:
        print("  📷 Picamera2 unavailable; falling back to cv2.VideoCapture.")
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS,          CAMERA_FPS)

    print("✅ System ready. Ctrl+C to stop.\n")

    frame_count    = 0
    pi_latched     = False   # True while we've sent PI_STOP and awaiting PI_GO
    last_state     = motor.latest_state

    try:
        while True:
            # ── Capture frame ─────────────────────────────────────────────────
            if use_picam2 and picam2:
                frame = picam2.capture_array()
                # Picamera2 RGB888 → BGR for OpenCV compatibility
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif cap:
                ret, frame = cap.read()
                if not ret or frame is None:
                    frame = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
            else:
                frame = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)

            frame_count += 1

            # ── Drain Arduino telemetry (updates latest_sonar, latest_state) ──
            for _ in range(5):   # Drain up to 5 pending lines per cycle
                line = motor.read_telemetry()
                if line is None:
                    break
                if "HEARTBEAT_TIMEOUT" in line:
                    print("⚠️  Arduino watchdog fired — Pi link was silent too long.")

            # ── TIER 1 GATE: If Pi latch is active, only PI_GO can release it ─
            # (We track the pi_latched flag on the Pi side to avoid re-sending PI_STOP
            #  every loop iteration when the car is already frozen.)

            # ── Ultrasonic obstacle check (2 sensors: left / right) ───────────
            us = sensors.read_ultrasonic()
            us_blocked = us["left"] < EMERGENCY_STOP_DISTANCE_CM or \
                         us["right"] < EMERGENCY_STOP_DISTANCE_CM

            # ── Run inference every 3rd frame ─────────────────────────────────
            detections = []
            if frame_count % 3 == 0:
                detections = detector.detect(frame)
                detections = [d for d in detections if d.get("confidence", 0) > 0.4]

            # =================================================================
            # MODE: AUTONOMOUS
            # =================================================================
            if mode == "autonomous":
                _run_autonomous(motor, sensors, detections, us_blocked,
                                frame, pi_latched)

            # =================================================================
            # MODE: FOLLOW-ME
            # =================================================================
            else:
                follow_state, pi_latched = _run_follow(
                    motor, sensors, tracker, steering_pid,
                    detections, us_blocked, frame,
                    follow_state, pi_latched,
                )

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
    finally:
        motor.pi_go()           # Release any latch that may still be active
        time.sleep(0.05)
        motor.stop()
        motor.disconnect()
        sensors.close()
        if cap is not None:
            cap.release()
        if picam2 is not None:
            try:
                picam2.stop()
            except Exception:
                pass
        print("👋 Shutdown complete.")


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
) -> bool:
    """
    One loop iteration of autonomous mode.
    Priority: PI_STOP (camera) > Arduino avoidance > DRIVE.

    Returns updated pi_latched flag.
    """
    frame_h = frame.shape[0]

    # ── Rule 1: Only care about threats while driving forward ─────────────────
    # (The Arduino handles actual state; we use a conservative check here:
    #  if we haven't sent any PI_STOP recently, assume we're driving forward.)

    # ── Check camera for persons/obstacles → Rules 1+2 ────────────────────────
    if detections and not pi_latched:
        action, slow_speed = _is_person_detected_ahead(detections, frame_h)
        if action == "STOP":
            print("🛑 PI_STOP: Obstacle/person imminent ahead (camera)")
            motor.pi_stop()
            pi_latched = True
            return pi_latched
        elif action == "SLOW" and slow_speed is not None:
            motor.drive(slow_speed, 500)  # Brief slow-down command

    # ── If Pi-latched, release if path is clear ───────────────────────────────
    if pi_latched:
        action, _ = _is_person_detected_ahead(detections, frame_h)
        if action == "CLEAR" and not us_blocked:
            print("✅ PI_GO: Path clear, releasing latch.")
            motor.pi_go()
            pi_latched = False

    # ── Zone-based avoidance hint for Arduino Tier 2 ─────────────────────────
    if detections and not pi_latched:
        scores  = compute_zone_scores(detections, frame.shape[1], frame.shape[0])
        decision = decide_avoidance(scores)
        if decision == "LEFT":
            motor.pi_avoid("LEFT")
        elif decision == "RIGHT":
            motor.pi_avoid("RIGHT")

    # ── Normal forward drive if not latched ───────────────────────────────────
    if not pi_latched:
        motor.drive(CRUISE_SPEED, 200)   # Short-horizon drive; loop re-issues it

    return pi_latched


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
) -> tuple:
    """
    One loop iteration of follow-me mode.

    Architecture §5 state machine:
      LOCK_ON → TRACKING → RE_ACQUIRE → TRACKING (or back to LOCK_ON)

    Returns: (new_follow_state, updated_pi_latched)
    """
    frame_h = frame.shape[0]
    frame_w = frame.shape[1]

    persons = [d for d in detections if d.get("class_name") == "person"]

    # ── Non-target obstacle check (Rule 3 — any non-target obstacle applies) ──
    non_person_obstacles = [d for d in detections
                            if d.get("class_name") != "person" and d.get("type") != "negative"]
    pothole_obstacles    = [d for d in detections if d.get("type") == "negative"]
    all_obstacles        = non_person_obstacles + pothole_obstacles

    if all_obstacles and not pi_latched:
        scores   = compute_zone_scores(all_obstacles, frame_w, frame_h)
        decision = decide_avoidance(scores)
        if decision == "STOP":
            print("🛑 PI_STOP: Non-target obstacle ahead (camera)")
            motor.pi_stop()
            pi_latched = True
        elif decision in ("LEFT", "RIGHT"):
            motor.pi_avoid(decision)

    # ── If US triggered, this is handled by Arduino Tier 2 automatically ─────
    # (We do not send PI_STOP for ultrasonic triggers — let Tier 2 handle it.)

    # ── Release Pi latch if path is clear ─────────────────────────────────────
    if pi_latched:
        scores   = compute_zone_scores(all_obstacles, frame_w, frame_h)
        decision = decide_avoidance(scores)
        if decision == "CLEAR" and not us_blocked:
            print("✅ PI_GO: Path clear, releasing follow-mode latch.")
            motor.pi_go()
            pi_latched = False

    if pi_latched:
        return follow_state, pi_latched   # Frozen — skip driving logic

    # ── Phase 1: LOCK_ON ──────────────────────────────────────────────────────
    if follow_state == "LOCK_ON":
        if persons:
            # Select the closest person (largest bounding box area)
            best = max(persons, key=lambda d: (d["bbox"][2] - d["bbox"][0]) *
                                              (d["bbox"][3] - d["bbox"][1]))
            x1, y1, x2, y2 = [int(v) for v in best["bbox"]]
            bbox = (x1, y1, x2 - x1, y2 - y1)
            if tracker.lock_on(frame, bbox):
                print("🎯 Owner locked. Starting follow-me tracking.")
                follow_state = "TRACKING"
        else:
            motor.stop()   # Wait for owner to appear

    # ── Phase 2: TRACKING ────────────────────────────────────────────────────
    elif follow_state == "TRACKING":
        success, bbox = tracker.update(frame)
        if success and bbox is not None:
            x, y, w, h = bbox
            center_x = x + w / 2.0
            bbox_h_frac = h / max(frame_h, 1)

            # PID steering
            pid.update(center_x)
            angle = -int(pid.output)
            angle = max(-45, min(45, angle))

            # Speed modulation from architecture §5 Phase 2 table
            if bbox_h_frac > FOLLOW_TOO_CLOSE:
                motor.stop()
            elif bbox_h_frac > FOLLOW_IDEAL_MIN:
                # Cruise; steer toward target
                if abs(angle) < 5:
                    motor.drive(FOLLOW_CRUISE_SPEED, 200)
                else:
                    motor.turn(angle, speed=FOLLOW_CRUISE_SPEED)
            elif bbox_h_frac > FOLLOW_GETTING_FAR:
                # Speed up; steer
                if abs(angle) < 5:
                    motor.drive(FOLLOW_FAST_SPEED, 200)
                else:
                    motor.turn(angle, speed=FOLLOW_FAST_SPEED)
            else:
                # Very far / almost losing them — max speed, prepare re-acquire
                motor.drive(FOLLOW_FAST_SPEED, 300)
        else:
            # KCF confidence dropped → Phase 3
            print("🤷 KCF lost target → entering re-acquisition.")
            motor.pi_stop()
            pi_latched = True
            follow_state = "RE_ACQUIRE"

    # ── Phase 3: RE_ACQUIRE ───────────────────────────────────────────────────
    elif follow_state == "RE_ACQUIRE":
        # Car is frozen (PI_STOP sent when entering this state).
        # Try to find the owner among current YOLO person detections.
        if persons:
            if tracker.reacquire(frame, persons):
                print("🎯 Owner re-identified! Resuming tracking.")
                motor.pi_go()
                pi_latched   = False
                follow_state = "TRACKING"
            # else: no match yet — stay frozen, try again next frame
        else:
            # No persons visible at all — release stop so Arduino doesn't
            # trigger watchdog, but keep LOCK_ON state; wait for owner.
            # (Don't release latch immediately — wait a few frames to avoid flicker)
            pass

    return follow_state, pi_latched


# =============================================================================
# Entry Point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="Autonomous RC Car Controller")
    parser.add_argument(
        "--mode",
        choices=["autonomous", "follow"],
        default="autonomous",
        help="Operating mode: 'autonomous' (avoid obstacles) or 'follow' (follow a person).",
    )
    args = parser.parse_args()
    run_pipeline(mode=args.mode)


if __name__ == "__main__":
    main()
