"""
test_close_approach.py — Test Suite for Requirement 1: Human and Object Close Approach Stopping
=================================================================================================
Verification tests for:
  - Distant person/obstacle ahead (bbox height 20%, ultrasonic 80cm) yields SLOW (continue forward driving), NOT STOP.
  - Multi-object distant scene does not prematurely trigger STOP.
  - Close obstacle (bbox height > 40%) yields STOP.
  - Emergency ultrasonic (< 25.0cm) yields STOP.
  - Safe ultrasonic (> 25.0cm) with distant obstacle allows approach.
  - Boundary conditions (0.14 vs 0.15, 0.40 vs 0.41, 24.9cm vs 25.0cm).
  - Proximity-scaled danger scoring in avoidance.py (distant objects stay < 0.30).
"""

import pytest
from car.perception.approach import (
    check_close_approach,
    BBOX_STOP_THRESHOLD,
    BBOX_SLOW_THRESHOLD,
    EMERGENCY_STOP_DISTANCE_CM,
    APPROACH_SPEED,
)
from car.perception.avoidance import (
    compute_zone_scores,
    decide_avoidance,
    DANGER_THRESHOLD,
    CRITICAL_DANGER_THRESHOLD,
)


# =============================================================================
# 1. Core Requirement 1 Close Approach Tests
# =============================================================================

def test_distant_obstacle_ahead_yields_slow_not_stop():
    """
    R1.1: Verify that a distant person or obstacle ahead (e.g. bbox height 20%,
    ultrasonic 80cm) yields SLOW (PWM 180) to continue forward driving, NOT STOP.
    """
    frame_w, frame_h = 640, 480
    # Bbox in center corridor: cx = 320 (50%), height = 96 (20% of 480)
    detections = [{
        "class_name": "person",
        "confidence": 0.88,
        "bbox": [260, 100, 380, 196],
        "type": "positive",
    }]
    sonar = {"left": 80.0, "right": 85.0}

    action, speed = check_close_approach(detections, frame_w, frame_h, us_sonar=sonar)

    assert action == "SLOW", f"Expected SLOW approach for distant obstacle, got {action}"
    assert speed == 180, f"Expected approach speed 180 PWM, got {speed}"


def test_multi_object_distant_scene_does_not_prematurely_trigger_stop():
    """
    R1.2: Verify that multiple distant obstacles in a scene do not prematurely
    trigger STOP. The vehicle should continue driving forward cautiously.
    """
    frame_w, frame_h = 640, 480
    detections = [
        # Distant person in center corridor: height = 96 (20% of 480)
        {"class_name": "person", "confidence": 0.85, "bbox": [280, 120, 360, 216]},
        # Distant object on left: cx = 100, height = 80 (16.7% of 480)
        {"class_name": "obstacle", "confidence": 0.80, "bbox": [60, 100, 140, 180]},
        # Distant object on right: cx = 560, height = 70 (14.6% of 480)
        {"class_name": "obstacle", "confidence": 0.75, "bbox": [520, 110, 600, 180]},
    ]
    sonar = {"left": 75.0, "right": 70.0}

    action, speed = check_close_approach(detections, frame_w, frame_h, us_sonar=sonar)

    assert action == "SLOW", f"Multi-object distant scene must yield SLOW, got {action}"
    assert speed == 180

    # Also verify avoidance.py zone scoring does not trigger STOP on this distant scene
    scores = compute_zone_scores(detections, frame_w, frame_h, sonar=sonar)
    assert scores["center"] < DANGER_THRESHOLD, (
        f"Distant center obstacle danger score {scores['center']} must be below DANGER_THRESHOLD (0.30)"
    )
    assert decide_avoidance(scores) != "STOP", "decide_avoidance must not STOP on distant multi-object scene"


def test_close_obstacle_bbox_height_triggers_stop():
    """
    R1.3: Verify that an imminent obstacle breaching BBOX_STOP_THRESHOLD
    (bbox height > 40% of frame height) halts forward motion and triggers STOP.
    """
    frame_w, frame_h = 640, 480
    # Center corridor: cx = 320, height = 240 (50% of 480 > 40%)
    detections = [{
        "class_name": "person",
        "confidence": 0.92,
        "bbox": [220, 100, 420, 340],
        "type": "positive",
    }]
    sonar = {"left": 50.0, "right": 50.0}

    action, speed = check_close_approach(detections, frame_w, frame_h, us_sonar=sonar)

    assert action == "STOP", f"Expected STOP for close obstacle > 40% height, got {action}"
    assert speed is None


def test_emergency_ultrasonic_distance_triggers_stop():
    """
    R1.4: Verify that an obstacle breaching the emergency ultrasonic boundary
    (< 25.0 cm) triggers STOP immediately, even with no camera detections.
    """
    frame_w, frame_h = 640, 480
    sonar_left_critical = {"left": 20.0, "right": 60.0}
    sonar_right_critical = {"left": 60.0, "right": 24.5}
    sonar_both_critical = {"left": 18.0, "right": 15.0}

    action_l, _ = check_close_approach([], frame_w, frame_h, us_sonar=sonar_left_critical)
    action_r, _ = check_close_approach([], frame_w, frame_h, us_sonar=sonar_right_critical)
    action_b, _ = check_close_approach([], frame_w, frame_h, us_sonar=sonar_both_critical)

    assert action_l == "STOP", "Critical left ultrasonic (< 25cm) must trigger STOP"
    assert action_r == "STOP", "Critical right ultrasonic (< 25cm) must trigger STOP"
    assert action_b == "STOP", "Critical dual ultrasonic (< 25cm) must trigger STOP"


def test_safe_ultrasonic_allows_close_approach():
    """
    R1.5: Verify that when ultrasonic sensors report safe distance (> 25.0 cm,
    e.g. 35.0 cm), forward motion is allowed to continue approaching at SLOW speed.
    """
    frame_w, frame_h = 640, 480
    # Distant obstacle in corridor: height = 120 (25% of 480)
    detections = [{
        "class_name": "person",
        "confidence": 0.85,
        "bbox": [260, 100, 380, 220],
    }]
    sonar_safe = {"left": 35.0, "right": 38.0}

    action, speed = check_close_approach(detections, frame_w, frame_h, us_sonar=sonar_safe)

    assert action == "SLOW", f"Safe ultrasonic (> 25cm) must allow approach, got {action}"
    assert speed == 180


# =============================================================================
# 2. Boundary Condition & Edge Case Tests
# =============================================================================

def test_bbox_height_boundaries():
    """
    Verify exact bounding-box height boundary behavior:
      - height_frac < 0.15 -> CLEAR
      - height_frac == 0.15 -> SLOW (180)
      - height_frac == 0.40 -> SLOW (180)
      - height_frac > 0.40 -> STOP
    """
    frame_w, frame_h = 640, 480

    # 1. Height = 14% (< 15%) -> CLEAR
    det_14 = [{"bbox": [280, 100, 360, 167]}]  # h = 67.2 / 480 = 14%
    assert check_close_approach(det_14, frame_w, frame_h)[0] == "CLEAR"

    # 2. Height = 15% (== 15%) -> SLOW
    det_15 = [{"bbox": [280, 100, 360, 172]}]  # h = 72 / 480 = 15%
    action_15, speed_15 = check_close_approach(det_15, frame_w, frame_h)
    assert action_15 == "SLOW"
    assert speed_15 == 180

    # 3. Height = 40% (== 40%) -> SLOW
    det_40 = [{"bbox": [280, 100, 360, 292]}]  # h = 192 / 480 = 40%
    action_40, speed_40 = check_close_approach(det_40, frame_w, frame_h)
    assert action_40 == "SLOW"
    assert speed_40 == 180

    # 4. Height = 41% (> 40%) -> STOP
    det_41 = [{"bbox": [280, 100, 360, 297]}]  # h = 197 / 480 = 41%
    assert check_close_approach(det_41, frame_w, frame_h)[0] == "STOP"


def test_ultrasonic_distance_boundary():
    """
    Verify ultrasonic emergency boundary at 25.0 cm:
      - 25.0 cm -> safe (does not stop)
      - 24.9 cm -> emergency stop
    """
    frame_w, frame_h = 640, 480

    # At exactly 25.0 cm: safe boundary, does not trigger STOP
    sonar_25 = {"left": 25.0, "right": 50.0}
    assert check_close_approach([], frame_w, frame_h, us_sonar=sonar_25)[0] == "CLEAR"

    # At 24.9 cm: breaches emergency stop distance -> STOP
    sonar_24_9 = {"left": 24.9, "right": 50.0}
    assert check_close_approach([], frame_w, frame_h, us_sonar=sonar_24_9)[0] == "STOP"


def test_corridor_filtering():
    """
    Verify obstacles outside the center corridor (cx < 20% or cx > 80%)
    do not trigger SLOW or STOP on forward driving path.
    """
    frame_w, frame_h = 640, 480

    # Obstacle on far left: cx = 60 / 640 = 9.38% (< 20%) with large height (50%)
    det_far_left = [{"bbox": [10, 100, 110, 340]}]
    assert check_close_approach(det_far_left, frame_w, frame_h)[0] == "CLEAR"

    # Obstacle on far right: cx = 580 / 640 = 90.6% (> 80%) with large height (50%)
    det_far_right = [{"bbox": [530, 100, 630, 340]}]
    assert check_close_approach(det_far_right, frame_w, frame_h)[0] == "CLEAR"


def test_sensor_timeout_and_noise_rejection():
    """
    Verify invalid ultrasonic readings (999.0 timeout, 0.0 noise spikes)
    are filtered out and do not trigger false emergency stops.
    """
    frame_w, frame_h = 640, 480
    noisy_sonar = {"left": 0.0, "right": 999.0, "rear": 10.0}

    action, _ = check_close_approach([], frame_w, frame_h, us_sonar=noisy_sonar)
    assert action == "CLEAR", "Sensor noise spikes and timeouts must be ignored"


# =============================================================================
# 3. Avoidance Engine Proximity-Scaled Scoring Tests
# =============================================================================

def test_avoidance_proximity_scaled_formula():
    """
    Verify that in avoidance.py, distant detections stay below DANGER_THRESHOLD (0.30)
    and danger scales smoothly as an obstacle approaches the ground bumper.
    """
    frame_w, frame_h = 640, 480

    # 1. Distant person at horizon (y2 = 180 / 480 = 0.375)
    det_distant = [{"bbox": [260, 80, 380, 180], "confidence": 0.90, "class_name": "person"}]
    scores_distant = compute_zone_scores(det_distant, frame_w, frame_h)
    assert scores_distant["center"] < DANGER_THRESHOLD, (
        f"Distant detection danger {scores_distant['center']} must be < {DANGER_THRESHOLD}"
    )

    # 2. Mid-range approaching obstacle (y2 = 320 / 480 = 0.667)
    det_mid = [{"bbox": [260, 160, 380, 320], "confidence": 0.90, "class_name": "person"}]
    scores_mid = compute_zone_scores(det_mid, frame_w, frame_h)

    # 3. Close obstacle near bumper (y2 = 450 / 480 = 0.938)
    det_close = [{"bbox": [260, 220, 380, 450], "confidence": 0.90, "class_name": "person"}]
    scores_close = compute_zone_scores(det_close, frame_w, frame_h)

    # Monotonic scaling verification
    assert scores_distant["center"] < scores_mid["center"] < scores_close["center"], (
        f"Danger score must scale smoothly with proximity: "
        f"distant={scores_distant['center']:.3f}, mid={scores_mid['center']:.3f}, close={scores_close['center']:.3f}"
    )
    assert scores_close["center"] > DANGER_THRESHOLD, "Close obstacle must exceed danger threshold"


def test_avoidance_ultrasonic_proximity_scaling():
    """
    Verify ultrasonic proximity scaling in avoidance.py:
      - Distance < 25.0 cm produces critical danger (> 0.85).
      - Distance between 25.0 cm and 42.0 cm produces cautionary danger (<= 0.85).
    """
    frame_w, frame_h = 640, 480

    # Critical range (< 25.0 cm)
    scores_crit = compute_zone_scores([], frame_w, frame_h, sonar={"left": 20.0, "right": 80.0})
    assert scores_crit["left"] > CRITICAL_DANGER_THRESHOLD, (
        f"Critical ultrasonic must exceed {CRITICAL_DANGER_THRESHOLD}, got {scores_crit['left']}"
    )

    # Cautionary range (25.0 cm <= dist < 42.0 cm)
    scores_caut = compute_zone_scores([], frame_w, frame_h, sonar={"left": 30.0, "right": 80.0})
    assert scores_caut["left"] <= CRITICAL_DANGER_THRESHOLD, (
        f"Cautionary ultrasonic danger {scores_caut['left']} must not exceed critical threshold {CRITICAL_DANGER_THRESHOLD}"
    )
    assert scores_caut["left"] > 0.1, "Cautionary ultrasonic danger must be positive"
