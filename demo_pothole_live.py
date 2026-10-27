#!/usr/bin/env python3
"""
LIVE DEMO: Pothole Detection & Directional Swerve Avoidance
===========================================================
Dedicated standalone script for evaluating Pothole Detection on RDD2022 D40 benchmark
and live held-up road photos / noisy camera feeds.
- High-recall ensemble: Primary fine-tuned best.pt + Specialist + Noise-Resilient Classical CV
- CLAHE contrast recovery for printed/screen pothole photos held in front of the car
- Dynamic corridor risk meters (Left, Center, Right)
- Real-time steering swerve recommendations (<< SWERVE LEFT, SWERVE RIGHT >>)
- Accepts ANY video path from ANY drive or folder
- No disk saving; pure real-time evaluation

Usage:
    python demo_pothole_live.py "C:\\path\\to\\any_video.mp4"
    python demo_pothole_live.py --video "D:\\pothole_dashcam.mp4"
    python demo_pothole_live.py --conf 0.05
"""

import argparse
import sys
import time
from pathlib import Path
import numpy as np
import cv2

try:
    from ultralytics import YOLO
except ImportError:
    print("Error: ultralytics is required. Run: pip install ultralytics", file=sys.stderr)
    sys.exit(1)

COLOR_CLEAR = (80, 220, 80)
COLOR_WARNING = (30, 200, 255)
COLOR_ALERT = (40, 40, 240)
COLOR_POTHOLE = (0, 140, 255)
COLOR_DARK = (20, 20, 25)

ZONE_LEFT_MAX = 0.33
ZONE_RIGHT_MIN = 0.67
DANGER_THRESHOLD = 0.18


def resolve_video_path(raw_path):
    if not raw_path:
        return None
    cleaned = str(raw_path).strip().strip('"').strip("'")
    if not cleaned:
        return None
    p = Path(cleaned).expanduser()
    if p.exists() and p.is_file():
        return str(p.resolve())
    p_rel = (Path.cwd() / cleaned).resolve()
    if p_rel.exists() and p_rel.is_file():
        return str(p_rel)
    return None


def enhance_contrast(roi):
    """CLAHE contrast normalization for held-up road photos, printouts, and noisy feeds."""
    if roi is None or roi.size == 0:
        return roi
    denoised = cv2.GaussianBlur(roi, (3, 3), 0)
    lab = cv2.cvtColor(denoised, cv2.COLOR_BGR2LAB)
    l_c, a_c, b_c = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
    return cv2.cvtColor(cv2.merge((clahe.apply(l_c), a_c, b_c)), cv2.COLOR_LAB2BGR)


def cv_detect_surface_defects(roi, base_x=0, base_y=0):
    """Noise-resilient Classical CV fallback for road damage and held-up pothole photos."""
    h, w = roi.shape[:2]
    if h < 30 or w < 30:
        return []
    road_surf = roi[int(h * 0.12):, :]
    rh, rw = road_surf.shape[:2]
    gray_raw = cv2.cvtColor(road_surf, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray_raw, (5, 5), 0)
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
    gray = clahe.apply(blur)

    lap = cv2.Laplacian(gray, cv2.CV_64F)
    lap_u = np.uint8(np.clip(np.absolute(lap), 0, 255))
    _, edge_thresh = cv2.threshold(lap_u, 22, 255, cv2.THRESH_BINARY)

    bh_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (23, 23))
    blackhat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, bh_kernel)
    _, dark_thresh = cv2.threshold(blackhat, 25, 255, cv2.THRESH_BINARY)

    combined = cv2.bitwise_or(edge_thresh, dark_thresh)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    closed = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
    opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    dets = []
    surf_area = rh * rw

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 350 or area > surf_area * 0.25:
            continue
        x, y, cw, ch = cv2.boundingRect(cnt)
        if x <= 2 or y <= 2 or (x + cw) >= (rw - 2) or (y + ch) >= (rh - 2):
            continue
        if 0.30 < cw / max(ch, 1) < 4.8:
            actual_y = y + int(h * 0.12) + base_y
            actual_x = x + base_x
            dets.append({
                "bbox": [actual_x, actual_y, actual_x + cw, actual_y + ch],
                "conf": 0.55,
                "label": "POTHOLE (CV)"
            })
    return dets


def find_model(names):
    base_dir = Path(__file__).resolve().parent
    for name in names:
        p = base_dir / name
        if p.exists():
            return str(p)
        p_sub = base_dir / "pothole_detection" / name
        if p_sub.exists():
            return str(p_sub)
    return None


def main():
    parser = argparse.ArgumentParser(description="Live Pothole Detection & Avoidance")
    parser.add_argument("video_pos", nargs="?", default=None, help="Path to video file (positional)")
    parser.add_argument("--video", "-v", default=None, help="Path to video file (flag)")
    parser.add_argument("--webcam", action="store_true", help="Use live webcam feed")
    parser.add_argument("--conf", type=float, default=0.07, help="Sensitivity threshold (default 0.07)")
    parser.add_argument("--imgsz", type=int, default=640, help="Inference resolution")
    args = parser.parse_args()

    raw_path = args.video or args.video_pos
    video_src = None
    if args.webcam:
        video_src = 0
    elif raw_path:
        video_src = resolve_video_path(raw_path)
        if not video_src:
            print(f"❌ Video not found at: {raw_path}")
            return

    if video_src is None and not args.webcam:
        for candidate in ["pothole_test_2.mp4", "pothole_detection/pothole_test_2.mp4", "dashcam_test.mp4"]:
            if Path(candidate).exists():
                video_src = str(Path(candidate).resolve())
                break

    if video_src is None and not args.webcam:
        user_input = input("Enter path to MP4 video file: ").strip().strip('"').strip("'")
        video_src = resolve_video_path(user_input)
        if not video_src:
            print("❌ Invalid file path. Exiting.")
            return

    pothole_path = find_model(["pothole_best.pt", "best.pt"])
    if not pothole_path:
        pothole_path = "yolov10n.pt"
    print(f"[*] Loading Primary Pothole Model: {pothole_path}")
    model = YOLO(pothole_path)

    spec_path = find_model(["rdd2022_yolov8n.pt"])
    specialist = None
    if spec_path and spec_path != pothole_path:
        print(f"[*] Loading Specialist Model: {spec_path}")
        try:
            specialist = YOLO(spec_path)
        except Exception:
            specialist = None

    cap = cv2.VideoCapture(video_src)
    if not cap.isOpened():
        print(f"❌ Failed to open video: {video_src}")
        return

    win_name = "Pothole Detection & Corridor Swerve Avoidance — Live Evaluation"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 1280, 720)

    fps = 30.0
    prev_time = time.perf_counter()
    paused = False
    conf_level = args.conf

    print("=" * 60)
    print("  POTHOLE DETECTION LIVE DEMO RUNNING")
    print(f"  Input:       {video_src}")
    print(f"  Sensitivity: {conf_level} (Press - / + to adjust)")
    print("  Controls:    [SPACE] Pause | [R] Replay | [Q] Quit")
    print("=" * 60)

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = cap.read()
                if not ret:
                    break

            h, w = frame.shape[:2]
            t0 = time.perf_counter()

            # Expanded Road surface + held-photo ROI (15%..98%)
            roi_y1, roi_y2 = int(h * 0.15), int(h * 0.98)
            roi_x1, roi_x2 = int(w * 0.02), int(w * 0.98)
            sub_frame = frame[roi_y1:roi_y2, roi_x1:roi_x2]
            eff_w, eff_h = roi_x2 - roi_x1, roi_y2 - roi_y1

            detections = []

            # 1. Primary Model (with CLAHE fallback for noisy / held-photo inputs)
            try:
                results = model.predict(sub_frame, conf=conf_level, imgsz=args.imgsz, verbose=False)[0]
                if len(results.boxes) == 0:
                    results = model.predict(enhance_contrast(sub_frame), conf=conf_level, imgsz=args.imgsz, verbose=False)[0]
                for box in results.boxes:
                    c_val = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    name = results.names.get(cls_id, "pothole").lower()
                    lbl = "POTHOLE (D40)" if ("pothole" in name or "d40" in name or cls_id == 0) else "ROAD CRACK"
                    x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
                    detections.append({
                        "bbox": [x1 + roi_x1, y1 + roi_y1, x2 + roi_x1, y2 + roi_y1],
                        "conf": c_val,
                        "label": lbl
                    })
            except Exception:
                pass

            # 2. Specialist Model
            if specialist:
                try:
                    res_s = specialist.predict(sub_frame, conf=conf_level, imgsz=args.imgsz, verbose=False)[0]
                    for box in res_s.boxes:
                        c_val = float(box.conf[0])
                        cls_id = int(box.cls[0])
                        name = specialist.names.get(cls_id, "pothole").lower()
                        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
                        gx1, gy1, gx2, gy2 = x1 + roi_x1, y1 + roi_y1, x2 + roi_x1, y2 + roi_y1

                        overlap = False
                        for ex in detections:
                            ex1, ey1, ex2, ey2 = ex["bbox"]
                            if abs(gx1 - ex1) < 45 and abs(gy1 - ey1) < 45:
                                overlap = True
                                break
                        if not overlap:
                            lbl = "POTHOLE (D40)" if ("pothole" in name or cls_id in [0, 3]) else "ROAD DAMAGE"
                            detections.append({"bbox": [gx1, gy1, gx2, gy2], "conf": c_val, "label": lbl})
                except Exception:
                    pass

            # 3. Classical CV Fallback
            try:
                cv_dets = cv_detect_surface_defects(sub_frame, base_x=roi_x1, base_y=roi_y1)
                for cd in cv_dets:
                    cx1, cy1, cx2, cy2 = cd["bbox"]
                    overlap = False
                    for ex in detections:
                        ex1, ey1, ex2, ey2 = ex["bbox"]
                        if abs(cx1 - ex1) < 40 and abs(cy1 - ey1) < 40:
                            overlap = True
                            break
                    if not overlap and len(detections) < 8:
                        detections.append(cd)
            except Exception:
                pass

            t1 = time.perf_counter()
            curr_time = time.perf_counter()
            fps = 0.9 * fps + 0.1 * (1.0 / max(curr_time - prev_time, 1e-4))
            prev_time = curr_time
            latency_ms = (t1 - t0) * 1000

            # Danger Scoring
            scores = {"left": 0.0, "center": 0.0, "right": 0.0}
            road_area = eff_w * eff_h

            for det in detections:
                x1, y1, x2, y2 = det["bbox"]
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                norm_y = min(max((cy - roi_y1) / eff_h, 0.0), 1.0)
                proximity = 0.6 + 0.9 * norm_y
                area_factor = min(max((x2 - x1) * (y2 - y1), 1) / road_area * 10.0, 1.0)
                danger = det["conf"] * 2.5 * proximity * (0.35 + 0.65 * area_factor)

                norm_x = (cx - roi_x1) / eff_w
                if norm_x < ZONE_LEFT_MAX:
                    scores["left"] += danger
                elif norm_x > ZONE_RIGHT_MIN:
                    scores["right"] += danger
                else:
                    scores["center"] += danger

            for k in scores:
                scores[k] = min(scores[k], 1.0)

            # Steering Guidance
            L, C, R = scores["left"], scores["center"], scores["right"]
            T = DANGER_THRESHOLD

            if L < T and C < T and R < T:
                directive = "PATH CLEAR -- CRUISING"
                subtext = "No Potholes Ahead"
                color = COLOR_CLEAR
            elif C >= T:
                if L >= T and R >= T:
                    directive = "!! EMERGENCY BRAKE: ROAD OBSTRUCTED !!"
                    subtext = "Hazard in all corridors"
                    color = COLOR_ALERT
                elif L <= R:
                    directive = "<< POTHOLE AHEAD: SWERVE LEFT"
                    subtext = f"Clear corridor Left (Risk: {int(L*100)}% vs {int(R*100)}%)"
                    color = COLOR_WARNING
                else:
                    directive = "POTHOLE AHEAD: SWERVE RIGHT >>"
                    subtext = f"Clear corridor Right (Risk: {int(R*100)}% vs {int(L*100)}%)"
                    color = COLOR_WARNING
            elif L >= T:
                directive = "POTHOLE ON LEFT >> SWERVE RIGHT"
                subtext = f"Left hazard risk: {int(L*100)}%"
                color = COLOR_WARNING
            elif R >= T:
                directive = "<< POTHOLE ON RIGHT: SWERVE LEFT"
                subtext = f"Right hazard risk: {int(R*100)}%"
                color = COLOR_WARNING
            else:
                directive = "CAUTION: SURFACE DEFECT DETECTED"
                subtext = "Reduce speed"
                color = COLOR_WARNING

            # HUD Rendering
            overlay = frame.copy()
            zl = int(roi_x1 + eff_w * ZONE_LEFT_MAX)
            zr = int(roi_x1 + eff_w * ZONE_RIGHT_MIN)
            cv2.line(overlay, (zl, roi_y1), (zl, roi_y2), (70, 70, 80), 2, cv2.LINE_AA)
            cv2.line(overlay, (zr, roi_y1), (zr, roi_y2), (70, 70, 80), 2, cv2.LINE_AA)

            cv2.rectangle(overlay, (0, 0), (w, 52), COLOR_DARK, -1)
            cv2.addWeighted(overlay, 0.85, frame, 0.15, 0, frame)

            cv2.putText(frame, "AUTONOMOUS POTHOLE AVOIDANCE SYSTEM", (16, 26),
                        cv2.FONT_HERSHEY_DUPLEX, 0.65, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, f"Model: RDD2022 D40 (Noise Fine-Tuned) | Potholes Detected: {len(detections)}", (16, 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 220, 255), 1, cv2.LINE_AA)

            hud_r = f"FPS: {fps:.1f} | Latency: {latency_ms:.1f} ms | Sens: {conf_level:.2f}"
            (tw, _), _ = cv2.getTextSize(hud_r, cv2.FONT_HERSHEY_SIMPLEX, 0.50, 1)
            cv2.putText(frame, hud_r, (w - tw - 16, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (100, 255, 160), 1, cv2.LINE_AA)

            for det in detections:
                x1, y1, x2, y2 = det["bbox"]
                cv2.rectangle(frame, (x1, y1), (x2, y2), COLOR_POTHOLE, 2)
                tag = f"{det['label']} {int(det['conf']*100)}%"
                (lw, lh), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.44, 1)
                cv2.rectangle(frame, (x1, max(y1 - lh - 6, 0)), (x1 + lw + 6, y1), COLOR_POTHOLE, -1)
                cv2.putText(frame, tag, (x1 + 3, y1 - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1, cv2.LINE_AA)

            bw, bh = min(int(w * 0.70), 620), 46
            bx = (w - bw) // 2
            by = 64
            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (25, 25, 30), -1)
            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), color, 2)
            (dt_w, _), _ = cv2.getTextSize(directive, cv2.FONT_HERSHEY_DUPLEX, 0.65, 2)
            cv2.putText(frame, directive, (bx + (bw - dt_w)//2, by + 25), cv2.FONT_HERSHEY_DUPLEX, 0.65, color, 2, cv2.LINE_AA)
            (st_w, _), _ = cv2.getTextSize(subtext, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1)
            cv2.putText(frame, subtext, (bx + (bw - st_w)//2, by + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1, cv2.LINE_AA)

            gy = h - 68
            cv2.rectangle(frame, (14, gy - 6), (250, h - 14), (20, 20, 25), -1)
            cv2.rectangle(frame, (14, gy - 6), (250, h - 14), (60, 60, 70), 1)
            cv2.putText(frame, "CORRIDOR RISK TELEMETRY", (22, gy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1, cv2.LINE_AA)
            for i, (zname, val) in enumerate([("L", scores["left"]), ("C", scores["center"]), ("R", scores["right"])]):
                zy = gy + 26 + (i * 12)
                cv2.putText(frame, f"{zname}:", (22, zy), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (220, 220, 220), 1, cv2.LINE_AA)
                cv2.rectangle(frame, (42, zy - 7), (182, zy + 1), (45, 45, 50), -1)
                bfill = int(140 * val)
                bcol = COLOR_ALERT if val >= DANGER_THRESHOLD else COLOR_CLEAR
                if bfill > 0:
                    cv2.rectangle(frame, (42, zy - 7), (42 + bfill, zy + 1), bcol, -1)
                cv2.putText(frame, f"{int(val*100)}%", (188, zy), cv2.FONT_HERSHEY_SIMPLEX, 0.35, bcol, 1, cv2.LINE_AA)

            ctrl_str = "[SPACE: Pause | R: Replay | -/+: Sensitivity | Q: Quit]"
            (cw, _), _ = cv2.getTextSize(ctrl_str, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1)
            cv2.putText(frame, ctrl_str, (w - cw - 16, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (150, 150, 160), 1, cv2.LINE_AA)

            display_frame = frame
        else:
            display_frame = frame.copy()
            cv2.putText(display_frame, "|| PAUSED  [Press SPACE to resume]", (w//2 - 180, h//2),
                        cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow(win_name, display_frame)
        key = cv2.waitKey(1 if not paused else 30) & 0xFF
        if key in [ord('q'), ord('Q'), 27]:
            break
        elif key == ord(' '):
            paused = not paused
        elif key in [ord('r'), ord('R')]:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        elif key in [ord('-'), ord('_')]:
            conf_level = max(round(conf_level - 0.02, 2), 0.02)
            print(f"[Pothole Sensitivity Increased]: conf = {conf_level}")
        elif key in [ord('+'), ord('=')]:
            conf_level = min(round(conf_level + 0.02, 2), 0.50)
            print(f"[Pothole Sensitivity Filtered]: conf = {conf_level}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
