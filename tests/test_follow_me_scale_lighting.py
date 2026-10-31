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


def test_reproduce_follow_steering_over_amplification_and_deadband():
    """
    Demonstrates bug: 'its turning too much when it sees me'.
    1. Small offset (center_x = 360, offset = 40px ~= 3.3 deg) should fall in deadband
       and command DRIVE (straight), NOT in-place skid-turn!
    2. Moderate offset (center_x = 440, offset = 120px ~= 10.0 deg) should command
       a moderate turn burst (<= 15 deg), NOT an aggressive over-steer (36 deg - 45 deg)!
    """
    motor, mock_serial = create_mock_motor()
    sensors = SensorManager()
    pid = PID(P=0.085, I=0.0, D=0.02)
    pid.SetPoint = 320
    throttler = CommandThrottler()
    sys_state = SystemState()

    tracker = ObjectTracker()
    # Case 1: Person slightly off-center (x=310, w=100 -> center_x = 360, offset = 40px)
    tgt_box = (310, 80, 100, 300)
    frame = make_realistic_person_frame(bbox=tgt_box, full_body=True)
    tracker.lock_on(frame, tgt_box)

    state, latched, action = _run_follow(
        motor=motor,
        sensors=sensors,
        tracker=tracker,
        pid=pid,
        detections=[{"bbox": [310, 80, 410, 380], "class_name": "person", "confidence": 0.9, "type": "positive"}],
        us_blocked=False,
        frame=frame,
        follow_state="TRACKING",
        pi_latched=False,
        motors_enabled=True,
        throttler=throttler,
        sys_state=sys_state,
    )

    # 40px offset (~3.3 deg) MUST NOT trigger an in-place TURN command; it should DRIVE straight!
    turn_cmds = [cmd for cmd in mock_serial.written_commands if cmd.startswith("TURN")]
    drive_cmds = [cmd for cmd in mock_serial.written_commands if cmd.startswith("DRIVE")]
    assert len(turn_cmds) == 0, f"Small 3.3 deg offset triggered in-place TURN: {turn_cmds}"
    assert len(drive_cmds) > 0, f"Expected DRIVE command for small offset, got none. Action: {action}"

    # Case 2: Moderate offset (center_x = 440, offset = 120px ~= 10 deg)
    mock_serial.written_commands.clear()
    tgt_box_offset = (390, 80, 100, 300)
    frame_offset = make_realistic_person_frame(bbox=tgt_box_offset, full_body=True)
    tracker.lock_on(frame_offset, tgt_box_offset)

    state, latched, action = _run_follow(
        motor=motor,
        sensors=sensors,
        tracker=tracker,
        pid=pid,
        detections=[{"bbox": [390, 80, 490, 380], "class_name": "person", "confidence": 0.9, "type": "positive"}],
        us_blocked=False,
        frame=frame_offset,
        follow_state="TRACKING",
        pi_latched=False,
        motors_enabled=True,
        throttler=throttler,
        sys_state=sys_state,
    )

    turn_cmds = [cmd for cmd in mock_serial.written_commands if cmd.startswith("TURN")]
    assert len(turn_cmds) == 1, f"Expected 1 TURN command, got {turn_cmds}"
    angle_arg = int(turn_cmds[0].split()[1])
    # Turn angle must be bounded (<= 18 deg burst limit) and reasonable for a 10 deg physical error
    assert 5 <= angle_arg <= 18, f"Turn command angle {angle_arg} is over-amplified! Expected <= 18 deg."


def test_follow_turn_settling_guard_prevents_command_stacking():
    """
    When Arduino is in STATE_TURNING or within settling window,
    _run_follow must NOT issue additional turn commands.
    """
    motor, mock_serial = create_mock_motor()
    sensors = SensorManager()
    pid = PID(P=0.085, I=0.0, D=0.02)
    pid.SetPoint = 320
    throttler = CommandThrottler()
    sys_state = SystemState()

    tracker = ObjectTracker()
    tgt_box = (390, 80, 100, 300)  # offset to right
    frame = make_realistic_person_frame(bbox=tgt_box, full_body=True)
    tracker.lock_on(frame, tgt_box)

    # 1. First iteration triggers a turn
    state, latched, action = _run_follow(
        motor=motor,
        sensors=sensors,
        tracker=tracker,
        pid=pid,
        detections=[{"bbox": [390, 80, 490, 380], "class_name": "person", "confidence": 0.9, "type": "positive"}],
        us_blocked=False,
        frame=frame,
        follow_state="TRACKING",
        pi_latched=False,
        motors_enabled=True,
        throttler=throttler,
        sys_state=sys_state,
    )
    turn_cmds = [cmd for cmd in mock_serial.written_commands if cmd.startswith("TURN")]
    assert len(turn_cmds) == 1

    # 2. Arduino reports it is currently executing the turn: STATE_TURNING
    motor.latest_state = "STATE_TURNING"
    mock_serial.written_commands.clear()

    state, latched, action = _run_follow(
        motor=motor,
        sensors=sensors,
        tracker=tracker,
        pid=pid,
        detections=[{"bbox": [390, 80, 490, 380], "class_name": "person", "confidence": 0.9, "type": "positive"}],
        us_blocked=False,
        frame=frame,
        follow_state="TRACKING",
        pi_latched=False,
        motors_enabled=True,
        throttler=throttler,
        sys_state=sys_state,
    )

    # Must NOT send another command while Arduino is turning!
    assert len(mock_serial.written_commands) == 0, f"Turn queued while Arduino was TURNING: {mock_serial.written_commands}"
    assert "SETTLING" in action or "TURNING" in action

