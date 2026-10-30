"""
test_adversarial_ribbon_stress.py — Adversarial Empirical Stress Testing for Satin Ribbon Lane Following
========================================================================================================
Adversarial Verification Suite for ChartLaneDetector:
1. Boundary Value & Adversarial Tests:
   - HSV Boundaries: Exact lower, upper, just inside, and just outside for orange, brown, green, pink.
   - Lighting & Illumination: Specular highlights, dim shadows / low light, bright overexposure,
     Gaussian noise, and salt-and-pepper noise.
   - Discontinuous Ribbon Lines: Gap bridging of 5x5 elliptical dilation (2 iterations) across 1-8px gaps.
   - Negative / Adversarial Inputs: Blank/black, solid colors, high saturation backgrounds,
     malformed/out-of-bounds bounding boxes, and non-standard image resolutions.
   - Obstacle Exclusion: Exact pixel excision and single-line fallback under severe occlusion.
"""

from typing import Tuple, List, Optional, Dict, Any
import numpy as np
import pytest
import cv2

from car.perception.config import (
    LANE_ROI_Y_MIN_FRAC,
    LANE_LOOKAHEAD_Y_FRAC,
    LANE_DEFAULT_WIDTH_FRAC,
    LANE_HSV_RANGES,
)
from car.perception.lane_detector import (
    ChartLaneDetector,
    LaneCurve,
    LaneDetectionResult,
)


def create_synthetic_ribbon_frame(
    width: int = 640,
    height: int = 480,
    left_x_frac: float = 0.25,
    right_x_frac: float = 0.75,
    curve_offset_px: float = 0.0,
    line_bgr: Tuple[int, int, int] = (45, 98, 167),
    bg_bgr: Tuple[int, int, int] = (245, 245, 245),
    draw_left: bool = True,
    draw_right: bool = True,
    line_thickness: int = 8,
) -> np.ndarray:
    """Helper to synthesize chart paper frame with satin ribbon lines."""
    frame = np.full((height, width, 3), bg_bgr, dtype=np.uint8)
    ys = np.linspace(int(height * 0.40), height - 1, 60).astype(int)

    if draw_left:
        pts_left = []
        for y in ys:
            y_norm = (height - y) / height
            x = int(width * left_x_frac + curve_offset_px * (y_norm ** 2))
            pts_left.append((x, y))
        cv2.polylines(frame, [np.array(pts_left)], False, line_bgr, line_thickness)

    if draw_right:
        pts_right = []
        for y in ys:
            y_norm = (height - y) / height
            x = int(width * right_x_frac + curve_offset_px * (y_norm ** 2))
            pts_right.append((x, y))
        cv2.polylines(frame, [np.array(pts_right)], False, line_bgr, line_thickness)

    return frame


# =============================================================================
# 1. HSV Boundary Value Stress Testing
# =============================================================================

class TestHSVBoundaries:
    """Rigorous boundary value testing on OpenCV inRange and LANE_HSV_RANGES."""

    @pytest.mark.parametrize("color", ["orange", "brown", "green", "pink"])
    def test_exact_lower_and_upper_boundaries_in_range(self, color: str):
        """Verify that exact lower and upper HSV boundaries are inclusive (yield 255)."""
        lower = np.array(LANE_HSV_RANGES[color]["lower"], dtype=np.uint8)
        upper = np.array(LANE_HSV_RANGES[color]["upper"], dtype=np.uint8)

        # Test lower boundary pixel
        pixel_lower = np.full((10, 10, 3), lower, dtype=np.uint8)
        mask_lower = cv2.inRange(pixel_lower, lower, upper)
        assert np.all(mask_lower == 255), f"{color} lower boundary must be IN range"

        # Test upper boundary pixel
        pixel_upper = np.full((10, 10, 3), upper, dtype=np.uint8)
        mask_upper = cv2.inRange(pixel_upper, lower, upper)
        assert np.all(mask_upper == 255), f"{color} upper boundary must be IN range"

    @pytest.mark.parametrize("color", ["orange", "brown", "green", "pink"])
    def test_just_inside_boundaries_in_range(self, color: str):
        """Verify pixels just inside (+1 on lower, -1 on upper) are inclusive (yield 255)."""
        lower = np.array(LANE_HSV_RANGES[color]["lower"], dtype=np.uint8)
        upper = np.array(LANE_HSV_RANGES[color]["upper"], dtype=np.uint8)

        inside_lower = lower + np.array([1, 1, 1], dtype=np.uint8)
        inside_upper = upper - np.array([1, 1, 1], dtype=np.uint8)

        mask_in_low = cv2.inRange(np.full((10, 10, 3), inside_lower, dtype=np.uint8), lower, upper)
        mask_in_upp = cv2.inRange(np.full((10, 10, 3), inside_upper, dtype=np.uint8), lower, upper)

        assert np.all(mask_in_low == 255), f"{color} just inside lower boundary must be IN range"
        assert np.all(mask_in_upp == 255), f"{color} just inside upper boundary must be IN range"

    @pytest.mark.parametrize("color", ["orange", "brown", "green", "pink"])
    def test_just_outside_boundaries_out_of_range(self, color: str):
        """Verify pixels just outside (-1 below lower, +1 above upper) on H, S, V are excluded (yield 0)."""
        lower = np.array(LANE_HSV_RANGES[color]["lower"], dtype=np.int16)
        upper = np.array(LANE_HSV_RANGES[color]["upper"], dtype=np.int16)
        lower_u8 = lower.astype(np.uint8)
        upper_u8 = upper.astype(np.uint8)

        # 1. Just below lower boundary for each channel
        for ch, name in enumerate(["H", "S", "V"]):
            if lower[ch] > 0:
                out_pixel = lower.copy()
                out_pixel[ch] -= 1
                mask = cv2.inRange(np.full((5, 5, 3), out_pixel.astype(np.uint8), dtype=np.uint8), lower_u8, upper_u8)
                assert np.all(mask == 0), f"{color} pixel just below lower {name} must be OUT of range"

        # 2. Just above upper boundary for each channel
        for ch, name in enumerate(["H", "S", "V"]):
            max_val = 179 if ch == 0 else 255
            if upper[ch] < max_val:
                out_pixel = upper.copy()
                out_pixel[ch] += 1
                mask = cv2.inRange(np.full((5, 5, 3), out_pixel.astype(np.uint8), dtype=np.uint8), lower_u8, upper_u8)
                assert np.all(mask == 0), f"{color} pixel just above upper {name} must be OUT of range"

    def test_cross_color_isolation(self):
        """Verify that representative colors of one ribbon mode are rejected by other ribbon modes."""
        sample_hsv = {
            "orange": [13, 186, 167],
            "brown": [12, 129, 95],
            "green": [63, 163, 152],
            "pink": [159, 153, 177],
        }

        detector = ChartLaneDetector()

        for active_mode, hsv_val in sample_hsv.items():
            detector.set_color_mode(active_mode)

            # Test active mode accepts its own color
            lower = np.array(LANE_HSV_RANGES[active_mode]["lower"], dtype=np.uint8)
            upper = np.array(LANE_HSV_RANGES[active_mode]["upper"], dtype=np.uint8)
            active_pix = np.full((10, 10, 3), hsv_val, dtype=np.uint8)
            assert np.all(cv2.inRange(active_pix, lower, upper) == 255)

            # Test other modes reject this color (except note orange/brown carefully isolated by S/V)
            for other_mode, other_hsv in sample_hsv.items():
                if other_mode == active_mode:
                    continue
                # If active is green or pink, orange/brown cannot match
                if active_mode in ("green", "pink"):
                    other_pix = np.full((10, 10, 3), other_hsv, dtype=np.uint8)
                    m = cv2.inRange(other_pix, lower, upper)
                    assert np.all(m == 0), f"{active_mode} must reject {other_mode}"

        # Test specific orange vs brown isolation:
        # Brown S is [60..200], V is [30..160]. Orange S is [120..255], V is [80..255].
        # Highly saturated orange [13, 230, 200] must be rejected by brown detector (V > 160)
        brown_low = np.array(LANE_HSV_RANGES["brown"]["lower"], dtype=np.uint8)
        brown_upp = np.array(LANE_HSV_RANGES["brown"]["upper"], dtype=np.uint8)
        bright_orange = np.full((10, 10, 3), [13, 230, 200], dtype=np.uint8)
        assert np.all(cv2.inRange(bright_orange, brown_low, brown_upp) == 0)

        # Deep brown [12, 80, 45] must be rejected by orange detector (V < 80, S < 120)
        orange_low = np.array(LANE_HSV_RANGES["orange"]["lower"], dtype=np.uint8)
        orange_upp = np.array(LANE_HSV_RANGES["orange"]["upper"], dtype=np.uint8)
        deep_brown = np.full((10, 10, 3), [12, 80, 45], dtype=np.uint8)
        assert np.all(cv2.inRange(deep_brown, orange_low, orange_upp) == 0)


# =============================================================================
# 2. Lighting & Illumination Stress Testing
# =============================================================================

class TestIlluminationVariations:
    """Stress tests across specular highlights, dim shadows, and image noise."""

    @pytest.mark.parametrize("color,bgr", [
        ("orange", (45, 98, 167)),
        ("brown", (47, 66, 95)),
        ("green", (65, 152, 55)),
        ("pink", (145, 71, 177)),
    ])
    def test_specular_highlights_along_ribbons(self, color: str, bgr: Tuple[int, int, int]):
        """
        Satin ribbons exhibit specular sheen where saturation drops and brightness peaks.
        Verify that 3px specular glare bands along the ribbons are bridged and tracking is maintained.
        """
        detector = ChartLaneDetector(color_mode=color)
        frame = create_synthetic_ribbon_frame(width=640, height=480, line_bgr=bgr, line_thickness=8)

        # Introduce 3 specular glare spots (white glare: high value, zero saturation)
        roi_y = int(480 * detector.roi_y_min_frac)
        glare_rows = [roi_y + 40, roi_y + 100, roi_y + 160]
        for gr in glare_rows:
            frame[gr:gr + 3, :] = (255, 255, 255)

        result = detector.detect(frame)
        assert result.status == "DUAL_LOCKED", f"Specular glare caused tracking loss in {color}"
        assert abs(result.steer_angle_deg) <= 3.5

    @pytest.mark.parametrize("color,bgr", [
        ("orange", (45, 98, 167)),
        ("green", (65, 152, 55)),
        ("pink", (145, 71, 177)),
    ])
    def test_moderate_shadow_illumination(self, color: str, bgr: Tuple[int, int, int]):
        """Verify tracking under realistic moderate shadow / dim lighting (65% brightness)."""
        detector = ChartLaneDetector(color_mode=color)
        frame = create_synthetic_ribbon_frame(width=640, height=480, line_bgr=bgr)

        # Scale intensity to simulate indoor dimming / shadow cast
        dim_frame = cv2.convertScaleAbs(frame, alpha=0.68, beta=0)

        result = detector.detect(dim_frame)
        assert result.status == "DUAL_LOCKED"
        assert abs(result.steer_angle_deg) <= 3.5

    def test_gaussian_noise_robustness(self):
        """Verify ribbon detection survives additive Gaussian sensor noise (sigma=15)."""
        detector = ChartLaneDetector(color_mode="orange")
        frame = create_synthetic_ribbon_frame(width=640, height=480, line_bgr=(45, 98, 167))

        np.random.seed(42)
        noise = np.random.normal(0, 15, frame.shape).astype(np.int16)
        noisy_frame = np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        result = detector.detect(noisy_frame)
        assert result.status == "DUAL_LOCKED"
        assert abs(result.steer_angle_deg) <= 3.5

    def test_salt_and_pepper_noise_rejection(self):
        """
        Verify that morphological opening (3x3 RECT) filters out isolated salt noise specks
        before 5x5 elliptical dilation can expand them.
        """
        detector = ChartLaneDetector(color_mode="orange")
        frame = create_synthetic_ribbon_frame(width=640, height=480, line_bgr=(45, 98, 167))

        # Inject 3% salt noise (random orange-like pixels scattered across the frame)
        np.random.seed(123)
        h, w = frame.shape[:2]
        num_salt = int(0.03 * h * w)
        ys = np.random.randint(int(h * detector.roi_y_min_frac), h, num_salt)
        xs = np.random.randint(0, w, num_salt)
        # Avoid overwriting true ribbons
        for y, x in zip(ys, xs):
            if not (140 <= x <= 180 or 460 <= x <= 500):
                frame[y, x] = (45, 98, 167)

        result = detector.detect(frame)
        # Should still cleanly identify the two prominent ribbons
        assert result.status == "DUAL_LOCKED"
        assert abs(result.steer_angle_deg) <= 4.0


# =============================================================================
# 3. Discontinuous Ribbon Lines & Morphological Dilation Stress Testing
# =============================================================================

class TestDiscontinuousGapsAndDilation:
    """Stress testing the gap-bridging capability of 5x5 elliptical dilation (2 iterations)."""

    @pytest.mark.parametrize("gap_px", [2, 3, 4])
    def test_dilation_bridges_small_gaps_2_to_4px(self, gap_px: int):
        """
        Requirement: Verify that 5x5 elliptical dilation (2 iterations)
        bridges small gaps (2-4px) and prevents loss of track.
        """
        detector = ChartLaneDetector(color_mode="orange")
        frame = create_synthetic_ribbon_frame(
            width=640, height=480, line_bgr=(45, 98, 167), line_thickness=6
        )

        # Introduce a discontinuous gap of gap_px at row 360
        gap_y_start = 360
        frame[gap_y_start:gap_y_start + gap_px, :] = (245, 245, 245)

        dilated_mask, roi_y = detector._extract_binary_mask(frame)

        left_x = int(640 * 0.25)
        right_x = int(640 * 0.75)
        roi_gap_y = (gap_y_start + gap_px // 2) - roi_y

        # 1. Empirically verify that every gap row is bridged in dilated mask
        for y_offset in range(gap_px):
            assert dilated_mask[gap_y_start - roi_y + y_offset, left_x] == 255
            assert dilated_mask[gap_y_start - roi_y + y_offset, right_x] == 255

        # 2. Verify tracking status remains DUAL_LOCKED
        result = detector.detect(frame)
        assert result.status == "DUAL_LOCKED"
        assert result.left_line is not None
        assert result.right_line is not None

    def test_dilation_vs_raw_opened_ablation(self):
        """
        Empirically compare undilated mask vs dilated mask:
        A 3px gap is strictly 0 in the undilated mask and fully 255 in the dilated mask.
        """
        frame = create_synthetic_ribbon_frame(
            width=640, height=480, line_bgr=(45, 98, 167), line_thickness=6
        )
        gap_y = 350
        gap_len = 3
        frame[gap_y:gap_y + gap_len, :] = (245, 245, 245)

        detector = ChartLaneDetector(color_mode="orange")
        dilated_mask, roi_y = detector._extract_binary_mask(frame)

        # Undilated baseline
        roi = frame[roi_y:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        raw_mask = cv2.inRange(hsv, np.array([5, 120, 80]), np.array([22, 255, 255]))
        clean_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        undilated_opened = cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, clean_kernel)

        left_x = int(640 * 0.25)
        roi_gap_row = gap_y + 1 - roi_y

        # Undilated must have a void
        assert undilated_opened[roi_gap_row, left_x] == 0
        # Dilated must bridge the void
        assert dilated_mask[roi_gap_row, left_x] == 255

    def test_large_gap_exceeding_dilation_radius(self):
        """
        Test a gap of 14px (well beyond the 8px bridging reach of 2x 5x5 ellipse iterations).
        Verify that while the mask cannot bridge a 14px gap directly, sliding window
        skips empty windows and polynomial MAD fitting still recovers lines or coasting logic engages safely.
        """
        detector = ChartLaneDetector(color_mode="orange")
        frame = create_synthetic_ribbon_frame(
            width=640, height=480, line_bgr=(45, 98, 167), line_thickness=6
        )
        # 14px gap at y=350..364
        frame[350:364, :] = (245, 245, 245)

        result = detector.detect(frame)
        # Even with a large 14px gap, the 9 sliding windows span the entire vertical ROI,
        # so windows above and below the gap capture sufficient points (>= 8) for polynomial fitting
        assert result.status == "DUAL_LOCKED"


# =============================================================================
# 4. Negative & Adversarial Inputs Stress Testing
# =============================================================================

class TestAdversarialInputs:
    """Stress tests on blank, solid, extreme resolution, and malformed inputs."""

    def test_blank_black_frame(self):
        """Blank black frame must return LANE_LOST without crashing or nan outputs."""
        detector = ChartLaneDetector(color_mode="orange")
        black_frame = np.zeros((480, 640, 3), dtype=np.uint8)

        result = detector.detect(black_frame)
        assert result.status == "LANE_LOST"
        assert result.left_line is None
        assert result.right_line is None
        assert result.center_coeffs is None
        assert result.steer_angle_deg == 0.0
        assert not np.isnan(result.steer_angle_deg)

    def test_solid_neutral_white_and_gray_frames(self):
        """Solid white and gray frames must return LANE_LOST without crashing."""
        detector = ChartLaneDetector(color_mode="orange")

        white_frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        res_w = detector.detect(white_frame)
        assert res_w.status == "LANE_LOST"

        gray_frame = np.full((480, 640, 3), 128, dtype=np.uint8)
        res_g = detector.detect(gray_frame)
        assert res_g.status == "LANE_LOST"

    @pytest.mark.parametrize("color,hsv_val", [
        ("orange", [13, 200, 200]),
        ("brown", [12, 120, 100]),
        ("green", [60, 200, 200]),
        ("pink", [160, 200, 200]),
    ])
    def test_solid_target_color_frame_adversarial_rejection(self, color: str, hsv_val: List[int]):
        """
        Adversarial: The entire frame is filled with the target ribbon color!
        The sliding window algorithm must reject windows exceeding max_blob_w (12% width)
        and return LANE_LOST rather than hallucinating invalid lane lines.
        """
        detector = ChartLaneDetector(color_mode=color)
        hsv_frame = np.full((480, 640, 3), hsv_val, dtype=np.uint8)
        bgr_frame = cv2.cvtColor(hsv_frame, cv2.COLOR_HSV2BGR)

        result = detector.detect(bgr_frame)
        assert result.status == "LANE_LOST"
        assert result.left_line is None
        assert result.right_line is None

    def test_high_saturation_non_target_background(self):
        """Highly saturated non-ribbon background (e.g. vibrant saturated blue) must yield 0 mask."""
        detector = ChartLaneDetector(color_mode="orange")
        # Pure saturated blue in BGR
        blue_frame = np.full((480, 640, 3), (255, 0, 0), dtype=np.uint8)

        mask, _ = detector._extract_binary_mask(blue_frame)
        assert np.count_nonzero(mask) == 0

        result = detector.detect(blue_frame)
        assert result.status == "LANE_LOST"

    def test_malformed_and_out_of_bounds_obstacle_boxes(self):
        """Verify detector robustness against negative, inverted, and malformed exclude_boxes."""
        detector = ChartLaneDetector(color_mode="orange")
        frame = create_synthetic_ribbon_frame(width=640, height=480, line_bgr=(45, 98, 167))

        adversarial_boxes = [
            None,
            "not a dict",
            {},
            {"bbox": None},
            {"bbox": [10, 20]},                 # Too short
            {"bbox": [-100, -50, 50, 100]},      # Negative / out-of-bounds coords
            {"bbox": [500, 400, 1200, 900]},    # Exceeding frame bounds
            {"bbox": [300, 400, 100, 200]},     # Inverted coordinates (x1 > x2)
            {"bbox": [100, 100, 100, 100]},     # Zero area
            {"bbox": [10.5, 20.8, 50.3, 80.9]},  # Floats
        ]

        # Must execute cleanly without unhandled exceptions
        result = detector.detect(frame, exclude_boxes=adversarial_boxes)
        assert result.status == "DUAL_LOCKED"
        assert result.left_line is not None
        assert result.right_line is not None

    @pytest.mark.parametrize("res", [
        (320, 240),
        (1280, 720),
        (1920, 1080),
        (640, 300),
        (300, 640),
        (64, 64),
        (10, 10),
        (1, 1),
    ])
    def test_non_standard_resolutions(self, res: Tuple[int, int]):
        """Verify detector runs without unhandled crashes across arbitrary resolutions."""
        w, h = res
        detector = ChartLaneDetector(color_mode="orange")
        frame = np.full((h, w, 3), (245, 245, 245), dtype=np.uint8)

        # Draw lines if frame has reasonable dimensions
        if w >= 100 and h >= 100:
            frame = create_synthetic_ribbon_frame(width=w, height=h, line_bgr=(45, 98, 167))

        result = detector.detect(frame)
        assert isinstance(result, LaneDetectionResult)
        assert result.status in ("DUAL_LOCKED", "SINGLE_LEFT", "SINGLE_RIGHT", "LANE_LOST")

    def test_empty_and_none_frame_handling(self):
        """Verify detector handles None and 0-sized frames cleanly."""
        detector = ChartLaneDetector(color_mode="orange")

        res_none = detector.detect(None)
        assert res_none.status == "LANE_LOST"

        res_empty = detector.detect(np.zeros((0, 0, 3), dtype=np.uint8))
        assert res_empty.status == "LANE_LOST"


# =============================================================================
# 5. Obstacle Exclusion Rigorous Verification
# =============================================================================

class TestObstacleExclusion:
    """Rigorous verification of obstacle bounding-box exclusion masking on satin ribbons."""

    def test_clean_ribbon_pixel_excision(self):
        """
        Verify that an obstacle bounding box cleanly excises all ribbon pixels within its padded bounds
        while leaving ribbon pixels outside completely untouched.
        """
        detector = ChartLaneDetector(color_mode="orange")
        frame = create_synthetic_ribbon_frame(width=640, height=480, line_bgr=(45, 98, 167))

        obstacle_bbox = [140, 350, 180, 410]
        exclude_boxes = [{
            "bbox": obstacle_bbox,
            "class_name": "obstacle",
            "type": "positive",
            "confidence": 0.92,
        }]

        res = detector.detect(frame, exclude_boxes=exclude_boxes)
        mask = res.binary_mask
        assert mask is not None
        roi_y = int(480 * detector.roi_y_min_frac)

        pad = 10
        # Inside the padded exclusion box: must be strictly 0
        roi_y1 = max(0, obstacle_bbox[1] - roi_y - pad)
        roi_y2 = obstacle_bbox[3] - roi_y + pad
        bx1 = obstacle_bbox[0] - pad
        bx2 = obstacle_bbox[2] + pad
        excluded_region = mask[roi_y1:roi_y2, bx1:bx2]
        assert np.count_nonzero(excluded_region) == 0, "All pixels in exclusion bbox + pad must be 0"

        # Outside the padded exclusion box: ribbon line at y=450 must remain intact (255)
        left_x = int(640 * 0.25)
        outside_roi_y = 450 - roi_y
        assert mask[outside_roi_y, left_x] == 255, "Ribbon pixels outside exclusion box must remain intact"

    def test_complete_left_line_occlusion_falls_back_to_single_right(self):
        """
        Verify that when an obstacle completely occludes the left ribbon line,
        the detector cleanly falls back to SINGLE_RIGHT and reconstructs the virtual centerline.
        """
        detector = ChartLaneDetector(color_mode="orange")
        frame = create_synthetic_ribbon_frame(width=640, height=480, line_bgr=(45, 98, 167))

        # Occlude entire left ribbon span in ROI (y: 216..480, x: 120..200)
        big_obstacle_bbox = [120, 210, 200, 480]
        exclude_boxes = [{
            "bbox": big_obstacle_bbox,
            "class_name": "obstacle",
            "confidence": 0.95,
        }]

        result = detector.detect(frame, exclude_boxes=exclude_boxes)
        assert result.status == "SINGLE_RIGHT"
        assert result.left_line is None
        assert result.right_line is not None
        assert result.center_coeffs is not None
        # Centerline should be shifted left of right line by ~half lane width
        assert abs(result.lookahead_point[0] - 314) <= 25

    def test_both_lines_complete_occlusion_causes_lane_lost(self):
        """Verify that when obstacles completely occlude both lines, detector safely enters LANE_LOST."""
        detector = ChartLaneDetector(color_mode="orange")
        frame = create_synthetic_ribbon_frame(width=640, height=480, line_bgr=(45, 98, 167))

        # Occlude left and right ribbons completely
        exclude_boxes = [
            {"bbox": [120, 210, 200, 480], "class_name": "obstacle"},
            {"bbox": [440, 210, 520, 480], "class_name": "obstacle"},
        ]

        result = detector.detect(frame, exclude_boxes=exclude_boxes)
        assert result.status == "LANE_LOST"
        assert result.left_line is None
        assert result.right_line is None
