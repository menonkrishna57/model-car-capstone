"""
test_follow_me_scale_lighting.py — Reproduction & Regression Tests for Follow-Me
================================================================================
Tests specifically targeting real-world bugs reported by user:
  1. Lighting changes (brightness drop, shadows, AWB shift) must not drop similarity below threshold.
  2. Transition from partial body (close) to full body (moving away) must not drop similarity below threshold.
  3. Distance modulation must not falsely report 'TARGET TOO CLOSE' when user is at 2m to 3.5m.
  4. Tracked person must not be classified as a non-target obstacle when scale or IoU shifts.
"""

import numpy as np
import pytest
import cv2

from car.perception.config import (
    FOLLOW_SIMILARITY_THRESHOLD,
    FOLLOW_TRACK_VERIFY_THRESHOLD,
)
from car.perception.tracker import (
    ObjectTracker,
    _build_appearance_profile,
    _compare_profiles,
)
from car.control.motor_controller import MotorController
from car.perception.sensors import SensorManager
from car.control.pid import PID
from car.main import CommandThrottler, SystemState, _run_follow
from tests.test_follow_me import create_mock_motor


def make_realistic_person_frame(
    shirt_bgr=(180, 80, 30),
    pants_bgr=(50, 40, 30),
    skin_bgr=(130, 160, 200),
    bbox=(260, 90, 120, 300),
    frame_shape=(480, 640, 3),
    full_body=True,
    brightness_mult=1.0,
    color_tint=0,
):
    frame = np.full(frame_shape, 120, dtype=np.uint8)
    x, y, w, h = bbox
    
    if full_body:
        head_end = int(0.15 * h)
        shirt_end = int(0.55 * h)
        frame[y:y + head_end, x + int(w * 0.25):x + int(w * 0.75)] = skin_bgr
        frame[y + head_end:y + shirt_end, x + int(w * 0.15):x + int(w * 0.85)] = shirt_bgr
        frame[y + shirt_end:y + h, x + int(w * 0.20):x + int(w * 0.80)] = pants_bgr
    else:
        head_end = int(0.25 * h)
        frame[y:y + head_end, x + int(w * 0.25):x + int(w * 0.75)] = skin_bgr
        frame[y + head_end:y + h, x + int(w * 0.15):x + int(w * 0.85)] = shirt_bgr

    if brightness_mult != 1.0 or color_tint != 0:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV).astype(np.float32)
        hsv[:, :, 2] = np.clip(hsv[:, :, 2] * brightness_mult, 0, 255)
        if color_tint != 0:
            hsv[:, :, 0] = (hsv[:, :, 0] + color_tint) % 180
        frame = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

    return frame


def test_reproduce_lighting_change_failure():
    """Demonstrates failure under -20% lighting or AWB tint shift."""
    base_frame = make_realistic_person_frame(bbox=(260, 90, 120, 300))
    base_prof = _build_appearance_profile(base_frame, (260, 90, 120, 300))

    # Stepping into shadow (-20% brightness)
    dark_frame = make_realistic_person_frame(bbox=(260, 90, 120, 300), brightness_mult=0.80)
    dark_prof = _build_appearance_profile(dark_frame, (260, 90, 120, 300))
    sim_dark = _compare_profiles(base_prof, dark_prof)

    # Must stay above verify and similarity threshold!
    assert sim_dark >= FOLLOW_TRACK_VERIFY_THRESHOLD, f"Failed on dark frame: {sim_dark:.3f} < {FOLLOW_TRACK_VERIFY_THRESHOLD}"
    assert sim_dark >= FOLLOW_SIMILARITY_THRESHOLD, f"Failed on dark frame: {sim_dark:.3f} < {FOLLOW_SIMILARITY_THRESHOLD}"


def test_reproduce_partial_to_full_body_failure():
    """Demonstrates failure when calibrated close (half body) and user moves away (full body)."""
    # Calibration frame (close: user waist-up visible, clipped at top)
    calib_bbox = (250, 0, 140, 220)
    calib_frame = make_realistic_person_frame(bbox=calib_bbox, full_body=False)
    calib_prof = _build_appearance_profile(calib_frame, calib_bbox)

    # Full body frame when user walked away
    full_bbox = (260, 90, 120, 300)
    full_frame = make_realistic_person_frame(bbox=full_bbox, full_body=True)
    full_prof = _build_appearance_profile(full_frame, full_bbox)

    sim = _compare_profiles(calib_prof, full_prof)
    assert sim >= FOLLOW_TRACK_VERIFY_THRESHOLD, f"Failed on partial-to-full body: {sim:.3f} < {FOLLOW_TRACK_VERIFY_THRESHOLD}"
    assert sim >= FOLLOW_SIMILARITY_THRESHOLD, f"Failed on partial-to-full body: {sim:.3f} < {FOLLOW_SIMILARITY_THRESHOLD}"


def test_reproduce_false_target_too_close_at_normal_following_distance():
    """
    Demonstrates false reporting of TARGET TOO CLOSE when user is walking away (2.2m - 3.2m).
    Full-body at 2.2m occupies ~80% of frame height (h=384 in 480p frame).
    It should CRUISE, not trigger 'TARGET TOO CLOSE (Holding)'.
    """
    motor, mock_serial = create_mock_motor()
    sensors = SensorManager()
    pid = PID(P=0.4, I=0.0, D=0.05)
    pid.SetPoint = 320
    throttler = CommandThrottler()
    sys_state = SystemState()

    tracker = ObjectTracker()
    frame = np.full((480, 640, 3), 120, dtype=np.uint8)
    
    # User at 2.2m: bbox height 380px (frac = 380/480 = 0.792), y1=50, y2=430 (head and feet visible!)
    tgt_box = (270, 50, 100, 380)
    frame = make_realistic_person_frame(bbox=tgt_box, full_body=True)
    tracker.lock_on(frame, tgt_box)

    # Sensor reading: US distance is 220cm (clear!)
    us_data = {"left": 220.0, "right": 220.0}

    # Run follow step
    state, latched, action = _run_follow(
        motor=motor,
        sensors=sensors,
        tracker=tracker,
        pid=pid,
        detections=[{"bbox": [270, 50, 370, 430], "class_name": "person", "confidence": 0.9, "type": "positive"}],
        us_blocked=False,
        frame=frame,
        follow_state="TRACKING",
        pi_latched=False,
        motors_enabled=True,
        throttler=throttler,
        sys_state=sys_state,
    )

    # It MUST NOT report TARGET TOO CLOSE (Holding) at 2.2m!
    assert "TARGET TOO CLOSE" not in action, f"Falsely reported TARGET TOO CLOSE at 2.2m! Action: {action}"
    assert "FOLLOWING" in action or "Cruise" in action or "Fast" in action, f"Expected FOLLOWING action, got {action}"
