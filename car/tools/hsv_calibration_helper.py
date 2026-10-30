"""
hsv_calibration_helper.py — HSV Range Calibration & Ribbon Mask Analyzer
========================================================================
Owned by perception-agent.

Utility for tuning and verifying HSV color segmentation ranges for satin ribbons
(orange, brown, green, pink) and legacy tape colors (yellow, blue).

Features:
- Headless quantitative analysis: Computes pixel counts, area coverage %,
  and p5..p95 percentile distributions for H, S, V channels of masked ribbon pixels.
- Interactive GUI trackbar mode: Allows real-time tuning of H_min, H_max, S_min,
  S_max, V_min, V_max with live composite display (original, mask, dilated).
- Synthetic test frame synthesis: Generates realistic test frames when no input
  image is provided.
- Graceful GUI fallback: Automatically drops to headless quantitative mode if
  display server / OpenCV GUI backend is not available.

Usage:
  python model/car/tools/hsv_calibration_helper.py --headless --mode orange
  python model/car/tools/hsv_calibration_helper.py --image path/to/frame.jpg --mode pink
  python model/car/tools/hsv_calibration_helper.py --lower 5,120,80 --upper 22,255,255 --headless
"""

import argparse
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
import cv2
import numpy as np

# Ensure project root and car package are on sys.path
_current_dir = Path(__file__).resolve().parent
_model_dir = _current_dir.parent.parent
_repo_root = _model_dir.parent

for _p in [str(_model_dir), str(_repo_root), str(_current_dir)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from car.perception.config import (
        LANE_HSV_RANGES,
        LANE_COLOR_MODE,
        LANE_ROI_Y_MIN_FRAC,
    )
except ImportError:
    LANE_COLOR_MODE = "orange"
    LANE_ROI_Y_MIN_FRAC = 0.45
    LANE_HSV_RANGES = {
        "orange": {"lower": [5, 120, 80], "upper": [22, 255, 255]},
        "brown": {"lower": [5, 60, 30], "upper": [20, 200, 160]},
        "green": {"lower": [38, 70, 50], "upper": [88, 255, 255]},
        "pink": {"lower": [140, 50, 100], "upper": [179, 255, 255]},
    }

# Complete presets table including legacy modes
COLOR_PRESETS: Dict[str, Dict[str, List[int]]] = {
    "orange": LANE_HSV_RANGES.get("orange", {"lower": [5, 120, 80], "upper": [22, 255, 255]}),
    "brown": LANE_HSV_RANGES.get("brown", {"lower": [5, 60, 30], "upper": [20, 200, 160]}),
    "green": LANE_HSV_RANGES.get("green", {"lower": [38, 70, 50], "upper": [88, 255, 255]}),
    "pink": LANE_HSV_RANGES.get("pink", {"lower": [140, 50, 100], "upper": [179, 255, 255]}),
    "yellow": {"lower": [16, 65, 65], "upper": [38, 255, 255]},
    "blue": {"lower": [92, 70, 60], "upper": [130, 255, 255]},
    "white": {"lower": [0, 0, 195], "upper": [179, 40, 255]},
    "dark": {"lower": [0, 0, 0], "upper": [179, 255, 90]},
}


def parse_hsv_triplet(val: Any) -> Optional[List[int]]:
    """Parses a string or iterable into a list of 3 integers [H, S, V]."""
    if val is None:
        return None
    if isinstance(val, (list, tuple)) and len(val) == 3:
        return [int(v) for v in val]
    if isinstance(val, str):
        cleaned = val.strip().strip("[]()").replace(" ", ",")
        parts = [p for p in cleaned.split(",") if p]
        if len(parts) == 3:
            return [int(p) for p in parts]
    raise ValueError(f"Cannot parse HSV bound: {val}. Expected format: 'H,S,V' or [H, S, V]")


def generate_synthetic_ribbon_frame(
    mode: str = "orange",
    lower: Optional[List[int]] = None,
    upper: Optional[List[int]] = None,
    width: int = 640,
    height: int = 480,
) -> np.ndarray:
    """
    Generates a synthetic camera frame with two satin ribbon lane boundaries on neutral floor.
    Ensures the drawn ribbon colors fall squarely within the specified or preset HSV bounds.
    """
    # Neutral indoor lab floor / chart paper (light gray with subtle texture)
    frame = np.full((height, width, 3), (240, 240, 240), dtype=np.uint8)

    # If bounds not explicitly provided, look up from color presets
    if lower is None or upper is None:
        if mode.lower() in COLOR_PRESETS:
            preset = COLOR_PRESETS[mode.lower()]
            lower = preset["lower"]
            upper = preset["upper"]

    if lower and upper:
        # Synthesize mid-point color from bounds to ensure exact inRange capture
        mid_h = int((lower[0] + upper[0]) / 2)
        mid_s = int((lower[1] + upper[1]) / 2)
        mid_v = int((lower[2] + upper[2]) / 2)
        hsv_px = np.uint8([[[mid_h, mid_s, mid_v]]])
        bgr_px = cv2.cvtColor(hsv_px, cv2.COLOR_HSV2BGR)[0][0]
        ribbon_color = (int(bgr_px[0]), int(bgr_px[1]), int(bgr_px[2]))
    else:
        mode_bgr_map = {
            "orange": (45, 98, 167),
            "brown": (47, 66, 95),
            "green": (65, 152, 55),
            "pink": (142, 71, 177),
            "yellow": (0, 220, 240),
            "blue": (220, 80, 20),
            "white": (255, 255, 255),
            "dark": (25, 25, 25),
        }
        ribbon_color = mode_bgr_map.get(mode.lower(), (45, 98, 167))

    # Draw left and right perspective lane boundary ribbons
    ys = np.linspace(int(height * 0.40), height - 1, 60).astype(int)

    pts_left = []
    pts_right = []
    for y in ys:
        y_norm = (height - y) / height
        # Standard lane perspective: left line at ~25% base width, right line at ~75% base width
        xl = int(width * 0.25 - 40 * y_norm)
        xr = int(width * 0.75 + 40 * y_norm)
        pts_left.append((xl, y))
        pts_right.append((xr, y))

    # Draw ribbons with realistic width (~10 px)
    cv2.polylines(frame, [np.array(pts_left)], False, ribbon_color, thickness=10)
    cv2.polylines(frame, [np.array(pts_right)], False, ribbon_color, thickness=10)

    # Add a slight ambient lighting gradient to simulate room conditions
    gradient = np.tile(np.linspace(0.95, 1.05, height)[:, None, None], (1, width, 3))
    frame = np.clip(frame.astype(np.float32) * gradient, 0, 255).astype(np.uint8)

    return frame


def analyze_hsv_mask(
    roi: np.ndarray,
    lower: List[int],
    upper: List[int],
    is_ribbon: bool = True,
) -> Dict[str, Any]:
    """
    Computes HSV binary mask, applies opening and satin ribbon dilation,
    and calculates pixel counts, coverage %, and HSV percentile distributions.
    """
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_arr = np.array(lower, dtype=np.uint8)
    upper_arr = np.array(upper, dtype=np.uint8)

    # 1. Raw HSV range segmentation
    raw_mask = cv2.inRange(hsv, lower_arr, upper_arr)

    # 2. Morphological opening (removes small isolated salt specks)
    clean_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    opened_mask = cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, clean_kernel)

    # 3. Morphological dilation for satin ribbons (bridges specular voids)
    if is_ribbon:
        dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        dilated_mask = cv2.dilate(opened_mask, dilate_kernel, iterations=2)
    else:
        dilated_mask = opened_mask.copy()

    total_pixels = roi.shape[0] * roi.shape[1]
    raw_count = int(np.count_nonzero(raw_mask))
    opened_count = int(np.count_nonzero(opened_mask))
    dilated_count = int(np.count_nonzero(dilated_mask))

    stats: Dict[str, Any] = {
        "total_pixels": total_pixels,
        "raw_count": raw_count,
        "raw_coverage_pct": (raw_count / total_pixels) * 100.0 if total_pixels > 0 else 0.0,
        "opened_count": opened_count,
        "opened_coverage_pct": (opened_count / total_pixels) * 100.0 if total_pixels > 0 else 0.0,
        "dilated_count": dilated_count,
        "dilated_coverage_pct": (dilated_count / total_pixels) * 100.0 if total_pixels > 0 else 0.0,
        "percentiles": {},
        "raw_mask": raw_mask,
        "opened_mask": opened_mask,
        "dilated_mask": dilated_mask,
        "hsv": hsv,
    }

    # Percentile distribution across active dilated pixels
    active_mask = dilated_mask > 0
    if np.any(active_mask):
        active_hsv = hsv[active_mask]
        for name, idx in [("Hue", 0), ("Saturation", 1), ("Value", 2)]:
            vals = active_hsv[:, idx]
            p5, p25, p50, p75, p95 = np.percentile(vals, [5, 25, 50, 75, 95])
            stats["percentiles"][name] = {
                "min": int(np.min(vals)),
                "p5": float(p5),
                "p25": float(p25),
                "p50": float(p50),
                "p75": float(p75),
                "p95": float(p95),
                "max": int(np.max(vals)),
                "mean": float(np.mean(vals)),
                "std": float(np.std(vals)),
            }

    return stats


def print_quantitative_summary(
    mode: str,
    lower: List[int],
    upper: List[int],
    source_desc: str,
    roi_shape: Tuple[int, int],
    stats: Dict[str, Any],
) -> None:
    """Prints a clear, formatted quantitative table to stdout."""
    roi_h, roi_w = roi_shape
    total_px = stats["total_pixels"]

    print("=" * 80)
    print("                HSV CALIBRATION QUANTITATIVE SUMMARY                ")
    print("=" * 80)
    print(f"Mode:               {mode}")
    print(f"Lower HSV Bound:    {lower}  (H: {lower[0]}, S: {lower[1]}, V: {lower[2]})")
    print(f"Upper HSV Bound:    {upper}  (H: {upper[0]}, S: {upper[1]}, V: {upper[2]})")
    print(f"Image Source:       {source_desc}")
    print(f"Analyzed Area:      ROI shape ({roi_h}x{roi_w}), Total Pixels: {total_px:,}")
    print("-" * 80)
    print("MASK EXTRACTION STATISTICS:")
    print(f"  * Raw inRange Mask:              {stats['raw_count']:>8,} px  ({stats['raw_coverage_pct']:>6.2f}% coverage)")
    print(f"  * After 3x3 Morph Open:          {stats['opened_count']:>8,} px  ({stats['opened_coverage_pct']:>6.2f}% coverage)")
    print(f"  * After 5x5 Ellipse Dilate (2x): {stats['dilated_count']:>8,} px  ({stats['dilated_coverage_pct']:>6.2f}% coverage)")
    print("-" * 80)

    if stats["percentiles"]:
        print("HSV CHANNEL DISTRIBUTION (Active Ribbon Pixels):")
        print(f"  {'Channel':<12} {'Min':>5} {'p5':>6} {'p25':>6} {'p50(Med)':>9} {'p75':>6} {'p95':>6} {'Max':>5}   {'Mean +/- Std':>14}")
        print("  " + "-" * 74)
        for name in ["Hue", "Saturation", "Value"]:
            p = stats["percentiles"][name]
            unit = " (0-179)" if name == "Hue" else " (0-255)"
            chan_label = f"{name}{unit}"
            print(
                f"  {chan_label:<14} "
                f"{p['min']:>5} "
                f"{p['p5']:>6.1f} "
                f"{p['p25']:>6.1f} "
                f"{p['p50']:>9.1f} "
                f"{p['p75']:>6.1f} "
                f"{p['p95']:>6.1f} "
                f"{p['max']:>5}   "
                f"{p['mean']:>6.1f} +/- {p['std']:<5.1f}"
            )
    else:
        print("[!] No pixels matched the specified HSV boundaries.")
    print("=" * 80)


def create_composite_image(
    roi: np.ndarray,
    raw_mask: np.ndarray,
    dilated_mask: np.ndarray,
) -> np.ndarray:
    """Creates a side-by-side composite comparison image."""
    h, w = roi.shape[:2]
    raw_bgr = cv2.cvtColor(raw_mask, cv2.COLOR_GRAY2BGR)
    dilated_bgr = cv2.cvtColor(dilated_mask, cv2.COLOR_GRAY2BGR)

    # Color overlay (ribbon pixels highlighted on darkened background)
    masked_roi = cv2.bitwise_and(roi, roi, mask=dilated_mask)
    dim_background = cv2.addWeighted(roi, 0.25, np.zeros_like(roi), 0.75, 0)
    inverse_mask = cv2.bitwise_not(dilated_mask)
    dim_bg_masked = cv2.bitwise_and(dim_background, dim_background, mask=inverse_mask)
    overlay = cv2.add(masked_roi, dim_bg_masked)

    # Put labels
    def put_panel_label(img: np.ndarray, label: str):
        cv2.rectangle(img, (5, 5), (180, 28), (0, 0, 0), -1)
        cv2.putText(img, label, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    panel1 = roi.copy()
    put_panel_label(panel1, "Original ROI")

    panel2 = raw_bgr.copy()
    put_panel_label(panel2, "Raw Mask")

    panel3 = dilated_bgr.copy()
    put_panel_label(panel3, "Dilated (5x5 2x)")

    panel4 = overlay.copy()
    put_panel_label(panel4, "Ribbon Isolation")

    return np.hstack([panel1, panel2, panel3, panel4])


def run_gui_mode(
    frame: np.ndarray,
    roi_y_start: int,
    init_mode: str,
    init_lower: List[int],
    init_upper: List[int],
    output_path: Optional[str] = None,
) -> None:
    """Runs interactive OpenCV trackbar tuning loop."""
    window_name = "HSV Calibration Helper [Q/ESC=Exit, S=Save, P=Print Summary]"

    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    except Exception as e:
        print(f"[WARN] GUI initialization failed: {e}. Falling back to headless summary.")
        roi = frame[roi_y_start:, :]
        stats = analyze_hsv_mask(roi, init_lower, init_upper, is_ribbon=True)
        print_quantitative_summary(init_mode, init_lower, init_upper, "Interactive Fallback", roi.shape[:2], stats)
        return

    def nothing(x):
        pass

    # Create 6 trackbars
    cv2.createTrackbar("H_min", window_name, init_lower[0], 179, nothing)
    cv2.createTrackbar("H_max", window_name, init_upper[0], 179, nothing)
    cv2.createTrackbar("S_min", window_name, init_lower[1], 255, nothing)
    cv2.createTrackbar("S_max", window_name, init_upper[1], 255, nothing)
    cv2.createTrackbar("V_min", window_name, init_lower[2], 255, nothing)
    cv2.createTrackbar("V_max", window_name, init_upper[2], 255, nothing)

    print("\n[INFO] Starting interactive calibration GUI.")
    print("       • Adjust trackbars to tune HSV range.")
    print("       • Press 'p' to print current quantitative summary.")
    print("       • Press 's' to save current view and print config snippet.")
    print("       • Press 'q' or ESC to exit.\n")

    roi = frame[roi_y_start:, :]

    try:
        while True:
            h_min = cv2.getTrackbarPos("H_min", window_name)
            h_max = cv2.getTrackbarPos("H_max", window_name)
            s_min = cv2.getTrackbarPos("S_min", window_name)
            s_max = cv2.getTrackbarPos("S_max", window_name)
            v_min = cv2.getTrackbarPos("V_min", window_name)
            v_max = cv2.getTrackbarPos("V_max", window_name)

            lower = [h_min, s_min, v_min]
            upper = [h_max, s_max, v_max]

            stats = analyze_hsv_mask(roi, lower, upper, is_ribbon=True)
            composite = create_composite_image(roi, stats["raw_mask"], stats["dilated_mask"])

            cv2.imshow(window_name, composite)
            key = cv2.waitKey(30) & 0xFF

            if key == 27 or key == ord("q"):
                break
            elif key == ord("p"):
                print_quantitative_summary("custom", lower, upper, "Interactive GUI Snapshot", roi.shape[:2], stats)
            elif key == ord("s"):
                print(f"\n[SAVE] Current HSV Configuration:")
                print(f"       Lower: {lower}")
                print(f"       Upper: {upper}")
                save_file = output_path or "hsv_calibration_snapshot.png"
                cv2.imwrite(save_file, composite)
                print(f"       Saved screenshot to: {save_file}\n")

    except Exception as e:
        print(f"[WARN] Error during GUI loop: {e}")
    finally:
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="HSV Calibration Helper for Satin Ribbon & Lane Detection",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--image",
        type=str,
        default=None,
        help="Path to an input image file. If omitted or not found, a synthetic frame is generated.",
    )
    parser.add_argument(
        "--mode",
        type=str,
        default=LANE_COLOR_MODE,
        choices=list(COLOR_PRESETS.keys()) + ["custom"],
        help="Color preset name (orange, brown, green, pink, yellow, blue, white, dark, custom).",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run quantitative analysis without displaying GUI trackbars.",
    )
    parser.add_argument(
        "--lower",
        type=str,
        default=None,
        help="Custom lower HSV bounds triplet (e.g. '5,120,80' or '[5, 120, 80]'). Overrides preset.",
    )
    parser.add_argument(
        "--upper",
        type=str,
        default=None,
        help="Custom upper HSV bounds triplet (e.g. '22,255,255' or '[22, 255, 255]'). Overrides preset.",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Path to save the resulting composite visualization image.",
    )
    parser.add_argument(
        "--full-frame",
        action="store_true",
        help="Analyze entire frame instead of cropping the lower lane ROI.",
    )

    args = parser.parse_args()

    # Determine lower and upper bounds
    preset = COLOR_PRESETS.get(args.mode.lower(), COLOR_PRESETS["orange"])
    lower = parse_hsv_triplet(args.lower) if args.lower else preset["lower"]
    upper = parse_hsv_triplet(args.upper) if args.upper else preset["upper"]

    # Load or generate frame
    source_desc = ""
    frame = None
    if args.image and os.path.isfile(args.image):
        frame = cv2.imread(args.image)
        if frame is not None:
            source_desc = f"File: {args.image} ({frame.shape[1]}x{frame.shape[0]})"
        else:
            print(f"[WARN] Failed to read image '{args.image}'. Generating synthetic frame instead.")

    if frame is None:
        frame = generate_synthetic_ribbon_frame(mode=args.mode, lower=lower, upper=upper)
        source_desc = f"Synthetic Frame ({frame.shape[1]}x{frame.shape[0]}) for mode '{args.mode}'"

    # Compute ROI
    h, w = frame.shape[:2]
    if args.full_frame:
        roi_y_start = 0
    else:
        roi_y_start = int(h * LANE_ROI_Y_MIN_FRAC)

    roi = frame[roi_y_start:, :]
    is_ribbon = args.mode.lower() in ("orange", "brown", "green", "pink")

    # Run analysis
    stats = analyze_hsv_mask(roi, lower, upper, is_ribbon=is_ribbon)

    # In headless mode or if forced by CLI:
    if args.headless:
        print_quantitative_summary(args.mode, lower, upper, source_desc, roi.shape[:2], stats)
        if args.output:
            composite = create_composite_image(roi, stats["raw_mask"], stats["dilated_mask"])
            cv2.imwrite(args.output, composite)
            print(f"[INFO] Saved composite visualization to: {args.output}")
        return 0

    # Otherwise, attempt interactive GUI mode
    # Check if GUI is supported by testing imshow in a try/except block
    gui_available = False
    try:
        # Check for headless environment flags
        if os.name != "nt" and not os.environ.get("DISPLAY"):
            gui_available = False
        else:
            gui_available = True
    except Exception:
        gui_available = False

    if gui_available:
        try:
            run_gui_mode(frame, roi_y_start, args.mode, lower, upper, output_path=args.output)
            return 0
        except Exception as e:
            print(f"[WARN] Failed to run GUI mode ({e}). Falling back to headless summary.")

    # Fallback to headless summary
    print_quantitative_summary(args.mode, lower, upper, source_desc, roi.shape[:2], stats)
    if args.output:
        composite = create_composite_image(roi, stats["raw_mask"], stats["dilated_mask"])
        cv2.imwrite(args.output, composite)
        print(f"[INFO] Saved composite visualization to: {args.output}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
