"""
test_lane_detection.py — Verification of Chart-Paper Lane Detection & Multi-Model Arbitration
=============================================================================================
Tests:
  1. Synthetic chart-paper line detection (straight & curved track).
  2. Single-line fallback recovery (SINGLE_LEFT and SINGLE_RIGHT).
  3. Pothole printout bounding-box exclusion masking so dark potholes do not pull curves.
  4. Multi-model priority stack coordination in _run_lane():
     - Clear lane -> DRIVE straight / corrective curve TURN.
     - In-lane medium hazard -> Tier 2.5 active bypass.
     - Imminent obstacle -> Tier 1 PI_STOP latch.
     - Ultrasonic blocked -> Tier 2 US AUTO-BRAKE.
     - Lost lane (> 8 frames) -> safe STOP.
  5. Color mode presets (dark, yellow, blue, white).
  6. Web UI mode switching and HUD corridor rendering.
  7. Deep edge cases and robustness.
  8. Satin ribbon lane following (orange, brown, green, pink HSV segmentation,
     morphological dilation, mode switching, fallback, and obstacle overrides).
"""

import io
import json
from typing import Tuple, List, Optional, Dict, Any
import numpy as np
import pytest
import cv2

from car.perception.config import (
    LANE_ROI_Y_MIN_FRAC,
    LANE_LOOKAHEAD_Y_FRAC,
    LANE_DEFAULT_WIDTH_FRAC,
    LANE_LOST_MAX_FRAMES,
    LANE_CRUISE_SPEED,
    LANE_CURVE_SPEED,
)
from car.perception.lane_detector import (
    ChartLaneDetector,
    LaneCurve,
    LaneDetectionResult,
    fit_poly_mad,
)
from car.main import (
    CommandThrottler,
    SystemState,
    DiagnosticHandler,
    StreamingServer,
    draw_hud,
    _run_lane,
)
from car.control.motor_controller import MotorController
from car.perception.sensors import SensorManager


class MockSerialDevice:
    """Mock serial transport recording written bytes and serving simulated telemetry."""

    def __init__(self, incoming_lines=None):
        self.is_open = True
        self.written_commands = []
        self._in_buffer = list(incoming_lines or [])

    @property
    def in_waiting(self):
        return sum(len(line.encode("utf-8") + b"\n") for line in self._in_buffer)

    def readline(self):
        if self._in_buffer:
            line = self._in_buffer.pop(0)
            return (line + "\n").encode("utf-8")
        return b""

    def write(self, data: bytes):
        text = data.decode("utf-8").strip()
        self.written_commands.append(text)
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self._in_buffer.clear()

    def reset_output_buffer(self):
        pass

    def close(self):
        self.is_open = False


def create_mock_motor(incoming_lines=None):
    """Creates a MotorController backed by a MockSerialDevice."""
    motor = MotorController(port="MOCK_PORT", baud=115200)
    mock_serial = MockSerialDevice(incoming_lines)
    motor._serial = mock_serial
    return motor, mock_serial


def create_synthetic_chart_paper_frame(
    width: int = 640,
    height: int = 480,
    left_x_frac: float = 0.25,
    right_x_frac: float = 0.75,
    curve_offset_px: float = 0.0,
    line_color: Tuple[int, int, int] = (25, 25, 25),
    bg_color: Tuple[int, int, int] = (245, 245, 245),
    draw_left: bool = True,
    draw_right: bool = True,
    line_thickness: int = 8,
) -> np.ndarray:
    """Creates a synthetic image representing lines drawn or taped on chart paper."""
    frame = np.full((height, width, 3), bg_color, dtype=np.uint8)

    ys = np.linspace(int(height * 0.40), height - 1, 60).astype(int)

    if draw_left:
        pts_left = []
        for y in ys:
            # Curve offset bends horizontally toward top
            y_norm = (height - y) / height
            x = int(width * left_x_frac + curve_offset_px * (y_norm ** 2))
            pts_left.append((x, y))
        cv2.polylines(frame, [np.array(pts_left)], False, line_color, line_thickness)

    if draw_right:
        pts_right = []
        for y in ys:
            y_norm = (height - y) / height
            x = int(width * right_x_frac + curve_offset_px * (y_norm ** 2))
            pts_right.append((x, y))
        cv2.polylines(frame, [np.array(pts_right)], False, line_color, line_thickness)

    return frame


# =============================================================================
# 1. Pure Vision: Synthetic Chart-Paper Lane Detection
# =============================================================================

def test_straight_track_dual_locked():
    """Verify ChartLaneDetector detects straight dark lines on light chart paper."""
    detector = ChartLaneDetector(color_mode="dark")
    frame = create_synthetic_chart_paper_frame(
        width=640, height=480, left_x_frac=0.25, right_x_frac=0.75, curve_offset_px=0.0
    )

    result = detector.detect(frame)

    assert result.status == "DUAL_LOCKED"
    assert result.left_line is not None
    assert result.right_line is not None
    assert result.center_coeffs is not None
    # Straight lines centered in frame: steering angle should be near 0 deg (within deadband)
    assert abs(result.steer_angle_deg) <= 3.0
    # Lane width should be ~0.50 (0.75 - 0.25)
    assert 0.45 <= result.lane_width_frac <= 0.55
    # Lookahead point X should be near 320
    assert abs(result.lookahead_point[0] - 320) <= 8


def test_curved_track_steering():
    """Verify right and left curving tracks command appropriate positive and negative steering."""
    detector = ChartLaneDetector(color_mode="dark")

    # Curve toward right (+x at lookahead)
    frame_right = create_synthetic_chart_paper_frame(
        width=640, height=480, left_x_frac=0.25, right_x_frac=0.75, curve_offset_px=80.0
    )
    res_right = detector.detect(frame_right)
    assert res_right.status == "DUAL_LOCKED"
    assert res_right.steer_angle_deg > 4.0

    detector.reset()

    # Curve toward left (-x at lookahead)
    frame_left = create_synthetic_chart_paper_frame(
        width=640, height=480, left_x_frac=0.25, right_x_frac=0.75, curve_offset_px=-80.0
    )
    res_left = detector.detect(frame_left)
    assert res_left.status == "DUAL_LOCKED"
    assert res_left.steer_angle_deg < -4.0


# =============================================================================
# 2. Single-Line Fallback Recovery
# =============================================================================

def test_single_line_fallback_left():
    """Verify that when only the left line is visible, virtual centerline is reconstructed."""
    detector = ChartLaneDetector(color_mode="dark")
    frame = create_synthetic_chart_paper_frame(
        width=640, height=480, left_x_frac=0.25, draw_right=False
    )

    result = detector.detect(frame)

    assert result.status == "SINGLE_LEFT"
    assert result.left_line is not None
    assert result.right_line is None
    assert result.center_coeffs is not None
    # Reconstructed centerline should be shifted right by half the learned width (~0.26)
    # Left base at ~0.25 -> Centerline at ~0.51
    assert abs(result.lookahead_point[0] - 326) <= 20


def test_single_line_fallback_right():
    """Verify that when only the right line is visible, virtual centerline is reconstructed."""
    detector = ChartLaneDetector(color_mode="dark")
    frame = create_synthetic_chart_paper_frame(
        width=640, height=480, right_x_frac=0.75, draw_left=False
    )

    result = detector.detect(frame)

    assert result.status == "SINGLE_RIGHT"
    assert result.left_line is None
    assert result.right_line is not None
    assert result.center_coeffs is not None
    # Right base at ~0.75 -> Centerline at ~0.49
    assert abs(result.lookahead_point[0] - 314) <= 20


# =============================================================================
# 3. Pothole Printout BBox Exclusion Masking
# =============================================================================

def test_pothole_printout_bbox_exclusion():
    """
    Verify that placing a dark pothole printout between chart paper lines is
    masked out when exclude_boxes is passed, preventing curve distortion.
    """
    detector = ChartLaneDetector(color_mode="dark")
    frame = create_synthetic_chart_paper_frame(
        width=640, height=480, left_x_frac=0.25, right_x_frac=0.75
    )

    # Place a dark pothole blob right in the center corridor (x: 280-360, y: 350-430)
    pothole_bbox = [280, 350, 360, 430]
    cv2.circle(frame, (320, 390), 38, (15, 15, 15), -1)

    # Pass the pothole detection bbox into exclude_boxes
    exclude_boxes = [{
        "bbox": pothole_bbox,
        "type": "negative",
        "class_name": "pothole",
        "confidence": 0.88,
    }]

    result = detector.detect(frame, exclude_boxes=exclude_boxes)

    # The pothole pixels should be masked out; detector locks onto the true lines
    assert result.status == "DUAL_LOCKED"
    assert result.left_line is not None
    assert result.right_line is not None
    assert abs(result.steer_angle_deg) <= 3.5

    # Check that mask inside the pothole center is zeroed out
    roi_y_start = int(480 * LANE_ROI_Y_MIN_FRAC)
    mask = result.binary_mask
    assert mask is not None
    pothole_roi_y = 390 - roi_y_start
    assert mask[pothole_roi_y, 320] == 0


# =============================================================================
# 4. Multi-Model Arbitration in _run_lane()
# =============================================================================

def test_run_lane_nominal_straight():
    """Verify clear chart paper commands DRIVE forward at LANE_CRUISE_SPEED."""
    motor, serial_dev = create_mock_motor()
    sensors = SensorManager(motor_controller=motor)
    throttler = CommandThrottler()
    detector = ChartLaneDetector(color_mode="dark")
    state = SystemState()

    frame = create_synthetic_chart_paper_frame(width=640, height=480)

    pi_latched, action, res = _run_lane(
        motor=motor,
        sensors=sensors,
        lane_detector=detector,
        detections=[],
        us_blocked=False,
        frame=frame,
        pi_latched=False,
        motors_enabled=True,
        throttler=throttler,
        sys_state=state,
    )

    assert pi_latched is False
    assert res.status == "DUAL_LOCKED"
    assert "LANE LOCKED" in action
    assert any("DRIVE" in c and str(LANE_CRUISE_SPEED) in c for c in serial_dev.written_commands)


def test_run_lane_curved_steer():
    """Verify curved chart paper commands corrective TURN at LANE_CURVE_SPEED."""
    motor, serial_dev = create_mock_motor()
    sensors = SensorManager(motor_controller=motor)
    throttler = CommandThrottler()
    detector = ChartLaneDetector(color_mode="dark")
    state = SystemState()

    frame = create_synthetic_chart_paper_frame(
        width=640, height=480, curve_offset_px=110.0
    )

    pi_latched, action, res = _run_lane(
        motor=motor,
        sensors=sensors,
        lane_detector=detector,
        detections=[],
        us_blocked=False,
        frame=frame,
        pi_latched=False,
        motors_enabled=True,
        throttler=throttler,
        sys_state=state,
    )

    assert pi_latched is False
    assert res.status == "DUAL_LOCKED"
    assert "LANE STEER RIGHT" in action
    assert any("TURN" in c and str(LANE_CURVE_SPEED) in c for c in serial_dev.written_commands)


def test_run_lane_in_lane_hazard_bypass():
    """Verify that a medium-range obstacle/pothole in the lane corridor triggers active bypass."""
    motor, serial_dev = create_mock_motor()
    sensors = SensorManager(motor_controller=motor)
    throttler = CommandThrottler()
    detector = ChartLaneDetector(color_mode="dark")
    state = SystemState()

    frame = create_synthetic_chart_paper_frame(width=640, height=480)

    # Obstacle on the left side of the corridor -> should bypass RIGHT
    detections = [{
        "class_name": "obstacle",
        "bbox": [60, 220, 160, 340],  # cx=110 (< 0.33*640 = 211) -> left zone -> bypass RIGHT
        "confidence": 0.75,
        "type": "positive",
    }]

    pi_latched, action, res = _run_lane(
        motor=motor,
        sensors=sensors,
        lane_detector=detector,
        detections=detections,
        us_blocked=False,
        frame=frame,
        pi_latched=False,
        motors_enabled=True,
        throttler=throttler,
        sys_state=state,
    )

    assert "LANE BYPASS" in action
    assert "RIGHT" in action
    assert any("PI_AVOID RIGHT" in c for c in serial_dev.written_commands)


def test_run_lane_tier1_imminent_stop():
    """Verify Tier 1 camera stop latch is engaged when an imminent obstacle appears in the lane."""
    motor, serial_dev = create_mock_motor()
    sensors = SensorManager(motor_controller=motor)
    throttler = CommandThrottler()
    detector = ChartLaneDetector(color_mode="dark")
    state = SystemState()

    frame = create_synthetic_chart_paper_frame(width=640, height=480)

    # Huge imminent obstacle in center corridor (height > 40%)
    detections = [{
        "class_name": "obstacle",
        "bbox": [200, 100, 440, 360],  # height = 260 / 480 = 54%
        "confidence": 0.85,
        "type": "positive",
    }]

    pi_latched, action, res = _run_lane(
        motor=motor,
        sensors=sensors,
        lane_detector=detector,
        detections=detections,
        us_blocked=False,
        frame=frame,
        pi_latched=False,
        motors_enabled=True,
        throttler=throttler,
        sys_state=state,
    )

    assert pi_latched is True
    assert "PI_STOP" in action
    assert "PI_STOP" in serial_dev.written_commands


def test_run_lane_lost_holding_stop():
    """Verify that when both lane boundaries are lost for > 8 frames, car holds safe stop."""
    motor, serial_dev = create_mock_motor()
    sensors = SensorManager(motor_controller=motor)
    throttler = CommandThrottler()
    detector = ChartLaneDetector(color_mode="dark")
    state = SystemState()

    # Completely blank frame (no lines on paper)
    blank_frame = np.full((480, 640, 3), 245, dtype=np.uint8)

    # Step through lost_max_frames + 1 iterations
    for _ in range(LANE_LOST_MAX_FRAMES + 1):
        pi_latched, action, res = _run_lane(
            motor=motor,
            sensors=sensors,
            lane_detector=detector,
            detections=[],
            us_blocked=False,
            frame=blank_frame,
            pi_latched=False,
            motors_enabled=True,
            throttler=throttler,
            sys_state=state,
        )

    assert res.status == "LANE_LOST"
    assert "Holding Safe Stop" in action
    assert "STOP" in serial_dev.written_commands


# =============================================================================
# 5. Color Mode Presets
# =============================================================================

def test_color_mode_presets():
    """Verify yellow and blue tape presets extract lines correctly."""
    # Yellow tape on white paper
    detector_yellow = ChartLaneDetector(color_mode="yellow")
    # BGR yellow is (0, 220, 240)
    frame_yellow = create_synthetic_chart_paper_frame(
        width=640, height=480, line_color=(0, 220, 240), bg_color=(245, 245, 245)
    )
    res_yellow = detector_yellow.detect(frame_yellow)
    assert res_yellow.status == "DUAL_LOCKED"

    # Blue tape on white paper
    detector_blue = ChartLaneDetector(color_mode="blue")
    # BGR blue is (220, 80, 20)
    frame_blue = create_synthetic_chart_paper_frame(
        width=640, height=480, line_color=(220, 80, 20), bg_color=(245, 245, 245)
    )
    res_blue = detector_blue.detect(frame_blue)
    assert res_blue.status == "DUAL_LOCKED"


# =============================================================================
# 6. HUD Corridor Rendering & Web UI
# =============================================================================

def test_draw_hud_lane_rendering():
    """Verify draw_hud renders the green corridor, boundary curves, and reticle without error."""
    detector = ChartLaneDetector(color_mode="dark")
    frame = create_synthetic_chart_paper_frame(width=640, height=480)
    res = detector.detect(frame)

    hud_img = draw_hud(
        frame=frame,
        detections=[],
        action_label="🛣️ LANE LOCKED (DUAL_LOCKED): Straight",
        latest_sonar={"left": 120.0, "right": 120.0},
        latest_state="IDLE",
        port_name="MOCK_PORT",
        fps=30.0,
        awb_mode="daylight",
        mode="lane",
        motors_enabled=True,
        invert_motors=False,
        pi_latched=False,
        lane_result=res,
    )

    assert hud_img is not None
    assert hud_img.shape == (480, 640, 3)
    # The green corridor adds green tint to the drivable region between lines (sampled at x=280)
    assert hud_img[360, 280, 1] > 100
    # The magenta centerline is drawn with B=255, R=255 at x=320
    assert hud_img[360, 320, 0] > 200 and hud_img[360, 320, 2] > 200


# =============================================================================
# 7. Deep Edge Cases & Robustness
# =============================================================================

def test_off_center_two_lines_left_half():
    """Verify that when the vehicle is displaced to the right, both lines falling in the left half are tracked as DUAL_LOCKED."""
    detector = ChartLaneDetector(color_mode="dark")
    frame = np.full((480, 640, 3), 245, dtype=np.uint8)
    # Two lines: left at x=70 (0.11*w), right at x=310 (0.48*w)
    cv2.line(frame, (70, 200), (70, 480), (25, 25, 25), 8)
    cv2.line(frame, (310, 200), (310, 480), (25, 25, 25), 8)

    res = detector.detect(frame)

    assert res.status == "DUAL_LOCKED"
    assert res.left_line is not None
    assert res.right_line is not None
    measured_w = res.right_line.x(0.68) - res.left_line.x(0.68)
    assert 0.35 <= measured_w <= 0.40
    # Centerline is at x=190 -> Vehicle center is 320 -> should command corrective steer LEFT
    assert res.steer_angle_deg < -15.0


def test_off_center_two_lines_right_half():
    """Verify that when the vehicle is displaced to the left, both lines falling in the right half are tracked as DUAL_LOCKED."""
    detector = ChartLaneDetector(color_mode="dark")
    frame = np.full((480, 640, 3), 245, dtype=np.uint8)
    # Two lines: left at x=330 (0.515*w), right at x=570 (0.89*w)
    cv2.line(frame, (330, 200), (330, 480), (25, 25, 25), 8)
    cv2.line(frame, (570, 200), (570, 480), (25, 25, 25), 8)

    res = detector.detect(frame)

    assert res.status == "DUAL_LOCKED"
    assert res.left_line is not None
    assert res.right_line is not None
    measured_w = res.right_line.x(0.68) - res.left_line.x(0.68)
    assert 0.35 <= measured_w <= 0.40
    # Centerline is at x=450 -> Vehicle center is 320 -> should command corrective steer RIGHT
    assert res.steer_angle_deg > 15.0


def test_single_line_right_off_center_disambiguation():
    """Verify that when only the right line is visible near the center (left line has exited frame), it is correctly identified as SINGLE_RIGHT."""
    detector = ChartLaneDetector(color_mode="dark")
    frame = np.full((480, 640, 3), 245, dtype=np.uint8)
    # Right line at x=310 (sloping towards center-top in perspective), left line offscreen
    cv2.line(frame, (270, 200), (310, 480), (25, 25, 25), 8)

    res = detector.detect(frame)

    assert res.status == "SINGLE_RIGHT"
    assert res.left_line is None
    assert res.right_line is not None
    # Right line at 310 means car is way too far right; steering must be LEFT (< 0)
    assert res.steer_angle_deg < -10.0


def test_coasting_trajectory_motor_execution():
    """Verify that when both lines temporarily disappear after being locked, _run_lane executes motor commands along the coasting path."""
    motor, serial_dev = create_mock_motor()
    sensors = SensorManager(motor_controller=motor)
    throttler = CommandThrottler()
    detector = ChartLaneDetector(color_mode="dark")
    state = SystemState()

    # Frame 1: Lock onto straight lane
    frame1 = create_synthetic_chart_paper_frame(width=640, height=480)
    _run_lane(
        motor=motor, sensors=sensors, lane_detector=detector,
        detections=[], us_blocked=False, frame=frame1,
        pi_latched=False, motors_enabled=True, throttler=throttler, sys_state=state,
    )
    serial_dev.written_commands.clear()

    # Frame 2: Gap in chart paper (blank frame) -> should coast and keep driving
    blank_frame = np.full((480, 640, 3), 245, dtype=np.uint8)
    pi_latched, action, res = _run_lane(
        motor=motor, sensors=sensors, lane_detector=detector,
        detections=[], us_blocked=False, frame=blank_frame,
        pi_latched=False, motors_enabled=True, throttler=throttler, sys_state=state,
    )

    assert res.status == "LANE_LOST"
    assert res.center_coeffs is not None  # Coasting memory preserved
    assert res.corridor_pts is not None    # Corridor polygon retained for HUD
    assert "COASTING" in action
    # Throttler or motor command was dispatched
    assert any("DRIVE" in c for c in serial_dev.written_commands) or (throttler.last_cmd == "DRIVE" and throttler.last_speed == LANE_CRUISE_SPEED)


def test_immediate_stop_when_uninitialized():
    """Verify that on cold start with no previous lane lock, _run_lane immediately holds a safe stop instead of coasting."""
    motor, serial_dev = create_mock_motor()
    sensors = SensorManager(motor_controller=motor)
    throttler = CommandThrottler()
    detector = ChartLaneDetector(color_mode="dark")
    state = SystemState()

    blank_frame = np.full((480, 640, 3), 245, dtype=np.uint8)
    pi_latched, action, res = _run_lane(
        motor=motor, sensors=sensors, lane_detector=detector,
        detections=[], us_blocked=False, frame=blank_frame,
        pi_latched=False, motors_enabled=True, throttler=throttler, sys_state=state,
    )

    assert res.status == "LANE_LOST"
    assert res.center_coeffs is None
    assert "Holding Safe Stop" in action
    assert "STOP" in serial_dev.written_commands


def test_bbox_exclusion_out_of_bounds_and_negative():
    """Verify that exclude_boxes with coordinates out of bounds or above ROI does not trigger slice index errors."""
    detector = ChartLaneDetector(color_mode="dark")
    frame = create_synthetic_chart_paper_frame(width=640, height=480)

    exclude_boxes = [
        {"bbox": [-50, -30, 100, 150]},        # Top-left out of bounds, above ROI
        {"bbox": [600, 400, 750, 550]},        # Bottom-right out of bounds
        {"bbox": [200, 10, 400, 50]},          # Entirely above ROI
    ]

    res = detector.detect(frame, exclude_boxes=exclude_boxes)
    assert res.status == "DUAL_LOCKED"


# =============================================================================
# 8. Satin Ribbon Lane Following (Requirements R1, R2, R3)
# =============================================================================

@pytest.mark.parametrize("color,bgr_color", [
    ("orange", (45, 98, 167)),
    ("brown", (47, 66, 95)),
    ("green", (65, 152, 55)),
    ("pink", (145, 71, 177)),
])
def test_ribbon_color_modes_dual_locked(color: str, bgr_color: Tuple[int, int, int]):
    """
    Verify that all four satin ribbon color modes ('orange', 'brown', 'green', 'pink')
    detect straight lines on synthetic chart paper frames, achieving DUAL_LOCKED status,
    a plausible lane width (~0.40 - 0.65), and reasonable steering angle (|steer| <= 3.0 deg).
    """
    detector = ChartLaneDetector(color_mode=color)
    frame = create_synthetic_chart_paper_frame(
        width=640,
        height=480,
        left_x_frac=0.25,
        right_x_frac=0.75,
        line_color=bgr_color,
        bg_color=(245, 245, 245),
        line_thickness=8,
    )

    result = detector.detect(frame)

    assert result.status == "DUAL_LOCKED"
    assert result.left_line is not None
    assert result.right_line is not None
    assert result.center_coeffs is not None
    # Lane width should be plausible (~0.40 to 0.65)
    assert 0.40 <= result.lane_width_frac <= 0.65
    # Straight lines centered in frame: steering angle should be near 0 deg
    assert abs(result.steer_angle_deg) <= 3.0
    # Center lookahead point X near image center
    assert abs(result.lookahead_point[0] - 320) <= 10


def test_ribbon_morphological_dilation_effect():
    """
    Verify that 5x5 elliptical dilation (2 iterations) applied in ribbon modes:
    1. Substantially increases active ribbon mask pixels compared to the undilated mask.
    2. Bridges discontinuous gaps in ribbon lines (e.g. caused by specular reflection / texture voids).
    """
    # Create frame with orange ribbon lines having a 4-pixel discontinuous gap at y=350:354
    frame = create_synthetic_chart_paper_frame(
        width=640,
        height=480,
        left_x_frac=0.25,
        right_x_frac=0.75,
        line_color=(45, 98, 167),
        bg_color=(245, 245, 245),
        line_thickness=4,
    )
    # Introduce gap across both lines at y = 350 to 354
    frame[350:354, :] = (245, 245, 245)

    detector = ChartLaneDetector(color_mode="orange")
    dilated_mask, roi_y = detector._extract_binary_mask(frame)

    gap_roi_y = 352 - roi_y
    left_x = int(640 * 0.25)
    right_x = int(640 * 0.75)

    # 1. Verify gap bridging: in dilated mask, the gap row contains active ribbon pixels
    assert dilated_mask[gap_roi_y, left_x] == 255
    assert dilated_mask[gap_roi_y, right_x] == 255

    # 2. Compute undilated mask (raw inRange + MORPH_OPEN without dilation) for comparison
    roi = frame[roi_y:, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower = np.array([5, 120, 80], dtype=np.uint8)
    upper = np.array([22, 255, 255], dtype=np.uint8)
    raw_mask = cv2.inRange(hsv, lower, upper)
    clean_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    raw_opened_mask = cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, clean_kernel)

    # In undilated mask, gap is completely unbridged (0)
    assert raw_opened_mask[gap_roi_y, left_x] == 0
    assert raw_opened_mask[gap_roi_y, right_x] == 0

    # Dilation with 5x5 ellipse (2 iterations) significantly increases active pixel count
    raw_active_pixels = int(np.count_nonzero(raw_opened_mask))
    dilated_active_pixels = int(np.count_nonzero(dilated_mask))
    assert dilated_active_pixels > 1.8 * raw_active_pixels

    # Full detection across the bridged gap must succeed and achieve DUAL_LOCKED
    result = detector.detect(frame)
    assert result.status == "DUAL_LOCKED"
    assert result.left_line is not None
    assert result.right_line is not None


def test_set_color_mode_validation():
    """
    Verify set_color_mode() accepts all 8 valid modes (case-insensitive, trimmed):
    {"dark", "yellow", "blue", "white", "orange", "brown", "green", "pink"},
    and ignores/rejects invalid mode strings without state corruption.
    """
    detector = ChartLaneDetector()
    valid_modes = ["dark", "yellow", "blue", "white", "orange", "brown", "green", "pink"]

    # Verify all 8 valid modes are accepted (with whitespace & case variations)
    for mode in valid_modes:
        detector.set_color_mode(f"  {mode.upper()}  ")
        assert detector.color_mode == mode.lower()

    # Anchor to 'orange' and test invalid modes
    detector.set_color_mode("orange")
    assert detector.color_mode == "orange"

    invalid_modes = ["purple", "neon", "invalid", "", "red", "cyan", "RGB", "123", "magenta"]
    for inv in invalid_modes:
        detector.set_color_mode(inv)
        assert detector.color_mode == "orange", f"Mode '{inv}' should have been rejected"


def test_ribbon_single_line_fallback():
    """
    Verify single-line fallback recovery when only one ribbon line is visible
    for both SINGLE_LEFT and SINGLE_RIGHT tracking on satin ribbons.
    """
    # 1. Left line only (orange ribbon) -> SINGLE_LEFT
    det_left = ChartLaneDetector(color_mode="orange")
    frame_left = create_synthetic_chart_paper_frame(
        width=640,
        height=480,
        left_x_frac=0.25,
        line_color=(45, 98, 167),
        draw_right=False,
    )
    res_left = det_left.detect(frame_left)

    assert res_left.status == "SINGLE_LEFT"
    assert res_left.left_line is not None
    assert res_left.right_line is None
    assert res_left.center_coeffs is not None
    # Reconstructed centerline shifted right by ~half lane width (lookahead X ~ 326)
    assert abs(res_left.lookahead_point[0] - 326) <= 20

    # 2. Right line only (green ribbon) -> SINGLE_RIGHT
    det_right = ChartLaneDetector(color_mode="green")
    frame_right = create_synthetic_chart_paper_frame(
        width=640,
        height=480,
        right_x_frac=0.75,
        line_color=(65, 152, 55),
        draw_left=False,
    )
    res_right = det_right.detect(frame_right)

    assert res_right.status == "SINGLE_RIGHT"
    assert res_right.left_line is None
    assert res_right.right_line is not None
    assert res_right.center_coeffs is not None
    # Reconstructed centerline shifted left by ~half lane width (lookahead X ~ 314)
    assert abs(res_right.lookahead_point[0] - 314) <= 20


def test_ribbon_obstacle_stop_and_exclusion():
    """
    Verify that:
    1. Obstacle and pothole bounding boxes exclude ribbon pixels from the binary mask.
    2. Obstacle overrides in multi-model priority stack (_run_lane) take precedence
       over ribbon lane following (triggering PI_STOP for imminent hazards and PI_AVOID for corridor hazards).
    """
    detector = ChartLaneDetector(color_mode="orange")
    frame = create_synthetic_chart_paper_frame(
        width=640,
        height=480,
        line_color=(45, 98, 167),
    )

    # 1. Test BBox exclusion: an obstacle over the left ribbon line (x: 140-180, y: 350-400)
    obstacle_bbox = [140, 350, 180, 400]
    exclude_boxes = [{
        "bbox": obstacle_bbox,
        "class_name": "obstacle",
        "type": "positive",
        "confidence": 0.90,
    }]

    res_excluded = detector.detect(frame, exclude_boxes=exclude_boxes)
    roi_y_start = int(480 * detector.roi_y_min_frac)
    mask = res_excluded.binary_mask
    assert mask is not None
    # Center of excluded bbox in ROI:
    bx_center = 160
    by_center_roi = 375 - roi_y_start
    assert mask[by_center_roi, bx_center] == 0

    # 2. Test Multi-Model priority stack: Tier 1 imminent obstacle override takes precedence
    motor, serial_dev = create_mock_motor()
    sensors = SensorManager(motor_controller=motor)
    throttler = CommandThrottler()
    state = SystemState()

    imminent_det = [{
        "class_name": "obstacle",
        "bbox": [200, 100, 440, 360],  # Height = 260 / 480 = 54% (> 40% threshold)
        "confidence": 0.85,
        "type": "positive",
    }]

    pi_latched, action, res_lane = _run_lane(
        motor=motor,
        sensors=sensors,
        lane_detector=detector,
        detections=imminent_det,
        us_blocked=False,
        frame=frame,
        pi_latched=False,
        motors_enabled=True,
        throttler=throttler,
        sys_state=state,
    )

    # Even with clean orange ribbons, obstacle override must engage PI_STOP
    assert pi_latched is True
    assert "PI_STOP" in action
    assert "PI_STOP" in serial_dev.written_commands

    # 3. Test corridor hazard bypass priority: left-zone obstacle commands PI_AVOID RIGHT
    serial_dev.written_commands.clear()
    state_bypass = SystemState()
    corridor_det = [{
        "class_name": "obstacle",
        "bbox": [60, 220, 160, 340],  # Left zone -> bypass right
        "confidence": 0.75,
        "type": "positive",
    }]

    pi_latched2, action2, res_lane2 = _run_lane(
        motor=motor,
        sensors=sensors,
        lane_detector=detector,
        detections=corridor_det,
        us_blocked=False,
        frame=frame,
        pi_latched=False,
        motors_enabled=True,
        throttler=throttler,
        sys_state=state_bypass,
    )

    assert "LANE BYPASS" in action2
    assert "RIGHT" in action2
    assert any("PI_AVOID RIGHT" in c for c in serial_dev.written_commands)


def test_default_lane_color_mode_is_orange():
    """
    Verify that LANE_COLOR_MODE defaults to 'orange' in car/perception/config.py,
    ChartLaneDetector() defaults to 'orange', and LANE_HSV_RANGES contains exact bounds
    for all 4 ribbon colors.
    """
    import car.perception.config as cfg

    # Requirement R2: LANE_COLOR_MODE = "orange"
    assert cfg.LANE_COLOR_MODE == "orange"

    # Detector default initialization matches config
    detector = ChartLaneDetector()
    assert detector.color_mode == "orange"

    # Verify all four ribbon colors exist in LANE_HSV_RANGES with specified bounds
    assert "orange" in cfg.LANE_HSV_RANGES
    assert cfg.LANE_HSV_RANGES["orange"]["lower"] == [5, 120, 80]
    assert cfg.LANE_HSV_RANGES["orange"]["upper"] == [22, 255, 255]

    assert "brown" in cfg.LANE_HSV_RANGES
    assert cfg.LANE_HSV_RANGES["brown"]["lower"] == [5, 60, 30]
    assert cfg.LANE_HSV_RANGES["brown"]["upper"] == [20, 200, 160]

    assert "green" in cfg.LANE_HSV_RANGES
    assert cfg.LANE_HSV_RANGES["green"]["lower"] == [38, 70, 50]
    assert cfg.LANE_HSV_RANGES["green"]["upper"] == [88, 255, 255]

    assert "pink" in cfg.LANE_HSV_RANGES
    assert cfg.LANE_HSV_RANGES["pink"]["lower"] == [140, 50, 100]
    assert cfg.LANE_HSV_RANGES["pink"]["upper"] == [179, 255, 255]


def test_web_api_set_lane_color_ribbons():
    """
    Verify DiagnosticHandler /set_lane_color endpoint accepts and applies
    all 4 ribbon colors ('orange', 'brown', 'green', 'pink'), updates both SystemState
    and ChartLaneDetector, and rejects invalid color modes with HTTP 400.
    """
    motor, _ = create_mock_motor()
    sys_state = SystemState()
    sys_state.motor = motor
    detector = ChartLaneDetector(color_mode="dark")

    class MockServer:
        def __init__(self):
            self.sys_state = sys_state
            self.lane_detector = detector

    server = MockServer()

    # Verify switching to each of the 4 ribbon colors via GET request
    for col in ["orange", "brown", "green", "pink"]:
        handler = DiagnosticHandler.__new__(DiagnosticHandler)
        handler.server = server
        handler.wfile = io.BytesIO()
        handler.send_response = lambda code: None
        handler.send_header = lambda k, v: None
        handler.end_headers = lambda: None
        handler.path = f"/set_lane_color?color={col}"
        handler.do_GET()

        assert sys_state.lane_color_mode == col
        assert detector.color_mode == col

    # Verify invalid color mode rejection
    status_codes = []
    handler = DiagnosticHandler.__new__(DiagnosticHandler)
    handler.server = server
    handler.wfile = io.BytesIO()
    handler.send_response = lambda code: status_codes.append(code)
    handler.send_header = lambda k, v: None
    handler.end_headers = lambda: None
    handler.path = "/set_lane_color?color=purple"
    handler.do_GET()

    assert 400 in status_codes
    assert detector.color_mode == "pink"  # State preserved


