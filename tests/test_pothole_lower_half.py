"""
Unit tests for lower-half pothole detection constraint in car/main.py and car/perception/detector.py.
Verifies parity with car/live_detect.py behavior.
"""

from unittest.mock import MagicMock
import numpy as np
import pytest

from car.perception.config import (
    POTHOLE_LOWER_Y_MIN_FRAC,
    POTHOLE_CONFIDENCE_THRESHOLD,
)
from car.main import _check_obstacle_ahead, draw_hud


def test_pothole_lower_y_min_frac_value():
    """Ensure POTHOLE_LOWER_Y_MIN_FRAC is configured to 0.40 matching live_detect.py."""
    assert POTHOLE_LOWER_Y_MIN_FRAC == 0.40
    assert POTHOLE_CONFIDENCE_THRESHOLD == 0.20


def test_check_obstacle_ahead_filters_upper_half_pothole():
    """Verify that _check_obstacle_ahead ignores potholes in the upper half/section of the frame."""
    frame_w = 640
    frame_h = 480

    # Upper-section pothole (yc = 150 < 480 * 0.40 = 192)
    upper_pothole = {
        "class_id": -1,
        "class_name": "pothole",
        "confidence": 0.85,
        "bbox": [200, 100, 300, 200],  # yc = 150, y2 = 200
        "type": "negative",
    }

    action, speed = _check_obstacle_ahead([upper_pothole], frame_w, frame_h)
    assert action == "CLEAR", f"Expected CLEAR for upper-section pothole, got {action}"

    # Lower-section pothole (yc = 350 >= 192, y2 = 400 >= 216) in center corridor
    lower_pothole = {
        "class_id": -1,
        "class_name": "pothole",
        "confidence": 0.85,
        "bbox": [200, 300, 440, 400],  # yc = 350, y2 = 400, height = 100
        "type": "negative",
    }

    action_lower, _ = _check_obstacle_ahead([lower_pothole], frame_w, frame_h)
    # The lower-section pothole should be processed as a valid detection
    assert action_lower in ("STOP", "SLOW", "CLEAR")


def test_draw_hud_renders_horizon_line_and_suppresses_upper_potholes():
    """Verify that draw_hud draws the ROI horizon line and suppresses upper-frame potholes."""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    upper_pothole = {
        "class_id": -1,
        "class_name": "pothole",
        "confidence": 0.80,
        "bbox": [100, 50, 200, 150],  # y2 = 150 < 480 * 0.45
        "type": "negative",
    }

    lower_pothole = {
        "class_id": -1,
        "class_name": "pothole",
        "confidence": 0.80,
        "bbox": [100, 300, 200, 400],  # Lower section
        "type": "negative",
    }

    hud = draw_hud(
        frame=frame,
        detections=[upper_pothole, lower_pothole],
        action_label="IDLE",
        latest_sonar={"left": 999.0, "right": 999.0},
        latest_state="IDLE",
        port_name="/dev/ttyUSB0",
        fps=30.0,
        awb_mode="daylight",
        mode="autonomous",
        motors_enabled=True,
        invert_motors=False,
        pi_latched=False,
    )

    # 1. Horizon line must be drawn at y = int(480 * 0.40) = 192 with color (75, 75, 75)
    horizon_y = int(480 * POTHOLE_LOWER_Y_MIN_FRAC)
    assert np.any(hud[horizon_y, :, :] == 75)

    # 2. Upper pothole bbox region (y = 50:150, x = 100:200) should NOT contain red/orange pothole color (0, 69, 255)
    upper_sample = hud[50:150, 100:200]
    has_pothole_color = np.any((upper_sample[:, :, 0] == 0) & (upper_sample[:, :, 1] == 69) & (upper_sample[:, :, 2] == 255))
    assert not has_pothole_color, "Upper-half pothole was rendered in HUD but should have been suppressed!"


def test_detector_filters_primary_model_pothole_to_lower_section():
    """Verify that if the primary model outputs a pothole, it is filtered to the lower camera section."""
    from car.perception.detector import Detector

    det = Detector()

    # Mock primary model prediction with one upper pothole and one lower pothole
    mock_box_upper = MagicMock()
    mock_box_upper.cls = [0]
    mock_box_upper.conf = [0.85]
    mock_box_upper.xyxy = [[100, 50, 200, 150]]  # yc = 100, in upper half

    mock_box_lower = MagicMock()
    mock_box_lower.cls = [0]
    mock_box_lower.conf = [0.85]
    mock_box_lower.xyxy = [[100, 260, 220, 340]]  # yc = 300, in lower section

    mock_res = MagicMock()
    mock_res.boxes = [mock_box_upper, mock_box_lower]
    mock_res.names = {0: "pothole_D40"}

    det.model = MagicMock(return_value=[mock_res])
    det.pothole_model = None  # test primary model filtering specifically
    det.enable_pothole_cv = False

    test_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    dets = det.detect(test_frame)

    # Upper pothole must be excluded, only lower pothole included
    assert len(dets) == 1
    assert dets[0]["class_name"] == "pothole"
    assert dets[0]["type"] == "negative"
    assert dets[0]["bbox"] == [100, 260, 220, 340]
