#!/usr/bin/env python3
"""
run_detection.py
=================
Pothole Detection & Directional Swerve Avoidance System
Trained on RDD2022 India (D40 Pothole Benchmark)

Usage:
  python run_detection.py               # Annotates test_video.mp4 -> annotated_video.mp4
  python run_detection.py --show        # Also shows live preview window
"""

import sys
import time
import argparse
from pathlib import Path
import numpy as np
import cv2

try:
    from ultralytics import YOLO
except ImportError:
    print("Error: ultralytics not installed. Run: pip install ultralytics", file=sys.stderr)
    sys.exit(1)

ROOT_DIR = Path(__file__).resolve().parent
SUB_DIR = ROOT_DIR / "negative_obstacle_detection"

# Locate weights (Prioritize newly trained custom best.pt)
MODEL_PATHS = [
    ROOT_DIR / "best.pt",
    ROOT_DIR / "rdd2022_yolov8n.pt",
    SUB_DIR / "best.pt",
    SUB_DIR / "rdd2022_yolov8n.pt",
]
MODEL_PATH = None
for p in MODEL_PATHS:
    if p.exists():
        MODEL_PATH = p
        break

COCO_PATH = ROOT_DIR / "yolov10n.pt"
if not COCO_PATH.exists() and (SUB_DIR / "yolov10n.pt").exists():
    COCO_PATH = SUB_DIR / "yolov10n.pt"

# Locate test video
VIDEO_PATHS = [
    ROOT_DIR / "test_video.mp4",
    SUB_DIR / "test_video.mp4",
]
DEFAULT_VIDEO = None
for p in VIDEO_PATHS:
    if p.exists():
        DEFAULT_VIDEO = p
        break

# Colors (BGR)
COLOR_CLEAR = (80, 220, 80)       # Green
COLOR_WARNING = (30, 200, 255)     # Amber / Yellow
COLOR_ALERT = (40, 40, 240)        # Red
COLOR_POTHOLE = (0, 140, 255)      # Orange
COLOR_OBSTACLE = (230, 160, 40)    # Blue/Cyan
COLOR_TEXT = (245, 245, 245)
COLOR_BG = (20, 20, 25)

# Zones
ZONE_LEFT_MAX = 0.33
ZONE_RIGHT_MIN = 0.67
DANGER_THRESHOLD = 0.20


def cv_detect_surface_defects(roi, base_x=0, base_y=0):
    """Classical CV fallback for road damage."""
    h, w = roi.shape[:2]
    road_surf = roi[int(h * 0.4):, :]
    gray = cv2.cvtColor(road_surf, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    lap = cv2.Laplacian(blur, cv2.CV_64F)
    lap_u = np.uint8(np.clip(np.absolute(lap), 0, 255))

    _, thresh = cv2.threshold(lap_u, 26, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    dets = []
    surf_area = road_surf.shape[0] * road_surf.shape[1]

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 500 or area > surf_area * 0.35:
            continue
        x, y, cw, ch = cv2.boundingRect(cnt)
        if 0.3 < cw / max(ch, 1) < 5.0:
            actual_y = y + int(h * 0.4) + base_y
            actual_x = x + base_x
            dets.append({
                "bbox": [actual_x, actual_y, actual_x + cw, actual_y + ch],
                "conf": 0.45,
                "label": "ROAD DAMAGE (CV)",
                "is_pothole": True
            })
    return dets


class PotholeDetector:
    def __init__(self, conf_thresh=0.07, mode="auto"):
        self.conf = conf_thresh
        self.mode = mode
        print(f"[+] Loading Primary Pothole Model: {MODEL_PATH.name}")
        self.pothole_model = YOLO(str(MODEL_PATH))
        
        # Load secondary pothole specialist for enhanced recall
        self.specialist_model = None
        spec_path = SUB_DIR / "rdd2022_yolov8n.pt"
        if spec_path.exists() and spec_path != MODEL_PATH:
            print(f"[+] Loading Secondary Model: {spec_path.name}")
            self.specialist_model = YOLO(str(spec_path))

        self.coco_model = YOLO(str(COCO_PATH)) if COCO_PATH.exists() else None

    def process_frame(self, frame):
        h, w = frame.shape[:2]
        
        # Determine ROI based on camera mount perspective
        if self.mode == "cabin":
            # For cabin POV video with interior steering wheel & A-pillars
            roi_y1, roi_y2 = int(h * 0.22), int(h * 0.49)
            roi_x1, roi_x2 = int(w * 0.28), int(w * 0.67)
        else:
            # Standard Dashcam / RC Car camera (e.g. pothole_test_2.mp4, bumper cam)
            # The drivable road surface occupies the lower ~50% of the frame
            roi_y1, roi_y2 = int(h * 0.44), int(h * 0.92)
            roi_x1, roi_x2 = int(w * 0.08), int(w * 0.92)

        sub_frame = frame[roi_y1:roi_y2, roi_x1:roi_x2]
        eff_w, eff_h = roi_x2 - roi_x1, roi_y2 - roi_y1

        detections = []

        # 1. Primary Model Inference (best.pt)
        try:
            res = self.pothole_model.predict(sub_frame, conf=self.conf, imgsz=640, verbose=False)[0]
            for box in res.boxes:
                cls_id = int(box.cls[0])
                name = res.names.get(cls_id, "pothole")
                conf = float(box.conf[0])
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
                
                is_pothole = "pothole" in name.lower() or "crack" in name.lower() or cls_id in [0, 1, 2, 3] or name == "0"
                if "pothole" in name.lower() or cls_id == 0:
                    lbl = "POTHOLE (D40)"
                elif "crack" in name.lower():
                    lbl = "ROAD CRACK"
                else:
                    lbl = name.replace("_", " ").upper()
                    
                detections.append({
                    "bbox": [x1 + roi_x1, y1 + roi_y1, x2 + roi_x1, y2 + roi_y1],
                    "conf": conf,
                    "label": lbl,
                    "is_pothole": is_pothole
                })
        except Exception:
            pass

        # 2. Secondary Model for Enhanced Recall
        if self.specialist_model:
            try:
                res2 = self.specialist_model.predict(sub_frame, conf=self.conf, imgsz=640, verbose=False)[0]
                for box in res2.boxes:
                    cls_id = int(box.cls[0])
                    name = self.specialist_model.names.get(cls_id, "road_damage")
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
                    
                    # Avoid duplicate overlapping detections
                    overlap = False
                    for existing in detections:
                        ex1, ey1, ex2, ey2 = existing["bbox"]
                        if abs((x1 + roi_x1) - ex1) < 50 and abs((y1 + roi_y1) - ey1) < 50:
                            overlap = True
                            break
                    if not overlap:
                        is_pothole = "pothole" in name.lower() or "crack" in name.lower() or cls_id in [0, 1, 2, 3]
                        lbl = "POTHOLE (D40)" if "pothole" in name.lower() else "ROAD DAMAGE"
                        detections.append({
                            "bbox": [x1 + roi_x1, y1 + roi_y1, x2 + roi_x1, y2 + roi_y1],
                            "conf": conf,
                            "label": lbl,
                            "is_pothole": is_pothole
                        })
            except Exception:
                pass

        # 3. Obstacles (Vehicles / People)
        if self.coco_model:
            try:
                res_c = self.coco_model.predict(sub_frame, conf=0.30, imgsz=416, verbose=False)[0]
                for box in res_c.boxes:
                    cls_id = int(box.cls[0])
                    if cls_id in [0, 1, 2, 3, 5, 7]:
                        name = res_c.names.get(cls_id, "obstacle")
                        conf = float(box.conf[0])
                        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
                        detections.append({
                            "bbox": [x1 + roi_x1, y1 + roi_y1, x2 + roi_x1, y2 + roi_y1],
                            "conf": conf,
                            "label": name.upper(),
                            "is_pothole": False
                        })
            except Exception:
                pass

        # 4. Danger Scoring
        scores = {"left": 0.0, "center": 0.0, "right": 0.0}
        road_area = eff_w * eff_h

        for det in detections:
            x1, y1, x2, y2 = det["bbox"]
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            conf = det["conf"]
            weight = 2.2 if det["is_pothole"] else 1.0

            norm_y = min(max((cy - roi_y1) / eff_h, 0.0), 1.0)
            proximity = 0.5 + 0.8 * norm_y
            area_factor = min(max((x2 - x1) * (y2 - y1), 1) / road_area * 10.0, 1.0)
            danger = conf * weight * proximity * (0.3 + 0.7 * area_factor)

            norm_x = (cx - roi_x1) / eff_w
            if norm_x < ZONE_LEFT_MAX:
                scores["left"] += danger
            elif norm_x > ZONE_RIGHT_MIN:
                scores["right"] += danger
            else:
                scores["center"] += danger

        for k in scores:
            scores[k] = min(scores[k], 1.0)

        # 5. Steering Swerve Guidance
        L, C, R = scores["left"], scores["center"], scores["right"]
        T = DANGER_THRESHOLD

        if L < T and C < T and R < T:
            swerve_text = "PATH CLEAR -- CRUISING"
            subtext = "No Hazard Detected"
            color = COLOR_CLEAR
        elif C >= T:
            if L >= T and R >= T:
                swerve_text = "!! HAZARD AHEAD: STOP !!"
                subtext = "All Corridors Obstructed"
                color = COLOR_ALERT
            elif L <= R:
                swerve_text = "<< POTHOLE AHEAD: SWERVE LEFT"
                subtext = f"Left corridor clearer (Risk: {L:.2f} vs {R:.2f})"
                color = COLOR_WARNING
            else:
                swerve_text = "POTHOLE AHEAD: SWERVE RIGHT >>"
                subtext = f"Right corridor clearer (Risk: {R:.2f} vs {L:.2f})"
                color = COLOR_WARNING
        elif L >= T:
            swerve_text = "POTHOLE ON LEFT >> SWERVE RIGHT"
            subtext = f"Left hazard detected (Risk: {L:.2f})"
            color = COLOR_WARNING
        elif R >= T:
            swerve_text = "<< POTHOLE ON RIGHT: SWERVE LEFT"
            subtext = f"Right hazard detected (Risk: {R:.2f})"
            color = COLOR_WARNING
        else:
            swerve_text = "!! STOP: ROAD DAMAGE !!"
            subtext = "Multiple Hazards"
            color = COLOR_ALERT

        return detections, scores, swerve_text, subtext, color, (roi_x1, roi_y1, eff_w, eff_h)


def draw_annotations(frame, detections, scores, swerve_text, subtext, color, roi, fps, frame_idx, total_frames):
    h, w = frame.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_bold = cv2.FONT_HERSHEY_DUPLEX
    rx, ry, rw, rh = roi

    # 1. Zone Dividing Lines
    z_left_x = int(rx + rw * ZONE_LEFT_MAX)
    z_right_x = int(rx + rw * ZONE_RIGHT_MIN)
    c_left = COLOR_ALERT if scores["left"] >= DANGER_THRESHOLD else (70, 70, 70)
    c_right = COLOR_ALERT if scores["right"] >= DANGER_THRESHOLD else (70, 70, 70)

    for x_pos, col in [(z_left_x, c_left), (z_right_x, c_right)]:
        y = ry
        while y < ry + rh:
            cv2.line(frame, (x_pos, y), (x_pos, min(y + 12, ry + rh)), col, 2, cv2.LINE_AA)
            y += 20

    # 2. Pothole / Obstacle Bounding Boxes
    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        box_col = COLOR_POTHOLE if det["is_pothole"] else COLOR_OBSTACLE
        cv2.rectangle(frame, (x1, y1), (x2, y2), box_col, 2, cv2.LINE_AA)

        badge = f"{det['label']} {int(det['conf']*100)}%"
        (tw, th), _ = cv2.getTextSize(badge, font, 0.42, 1)
        tag_y1 = max(y1 - th - 6, 0)
        cv2.rectangle(frame, (x1, tag_y1), (x1 + tw + 8, y1), box_col, -1)
        cv2.putText(frame, badge, (x1 + 4, y1 - 4), font, 0.42, (10, 10, 10), 1, cv2.LINE_AA)

    # 3. Top Banner: Swerve Indicator & Direction
    banner_h = 56
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, banner_h), COLOR_BG, -1)
    cv2.addWeighted(overlay, 0.78, frame, 0.22, 0, frame)
    cv2.line(frame, (0, 0), (w, 0), color, 4, cv2.LINE_AA)
    cv2.line(frame, (0, banner_h), (w, banner_h), (50, 50, 55), 1, cv2.LINE_AA)

    (tw, th), _ = cv2.getTextSize(swerve_text, font_bold, 0.72, 2)
    cv2.putText(frame, swerve_text, ((w - tw) // 2, 28), font_bold, 0.72, color, 2, cv2.LINE_AA)

    (stw, _), _ = cv2.getTextSize(subtext, font, 0.40, 1)
    cv2.putText(frame, subtext, ((w - stw) // 2, 48), font, 0.40, (170, 170, 170), 1, cv2.LINE_AA)

    cv2.putText(frame, f"FPS: {fps:.1f}", (14, 24), font, 0.45, COLOR_TEXT, 1, cv2.LINE_AA)
    cv2.putText(frame, "YOLOv10n (D40)", (14, 44), font, 0.38, (0, 190, 255), 1, cv2.LINE_AA)

    prog = f"{frame_idx}/{total_frames}"
    (ptw, _), _ = cv2.getTextSize(prog, font, 0.42, 1)
    cv2.putText(frame, prog, (w - ptw - 14, 25), font, 0.42, (170, 170, 170), 1, cv2.LINE_AA)

    # 4. Bottom Dock: Zone Risk Meters
    dock_h = 42
    dock_y = h - dock_h
    overlay_dock = frame.copy()
    cv2.rectangle(overlay_dock, (0, dock_y), (w, h), COLOR_BG, -1)
    cv2.addWeighted(overlay_dock, 0.75, frame, 0.25, 0, frame)
    cv2.line(frame, (0, dock_y), (w, dock_y), (50, 50, 55), 1, cv2.LINE_AA)

    zones = [
        ("LEFT CORRIDOR", scores["left"], w * 0.16),
        ("CENTER PATH", scores["center"], w * 0.50),
        ("RIGHT CORRIDOR", scores["right"], w * 0.84),
    ]
    for label, score, center_x in zones:
        meter_w = int(w * 0.22)
        meter_x = int(center_x - meter_w / 2)
        meter_y = dock_y + 22
        meter_h = 8
        col = COLOR_ALERT if score >= 0.55 else (COLOR_WARNING if score >= DANGER_THRESHOLD else COLOR_CLEAR)
        lbl_str = f"{label}: {int(score * 100)}%"
        (ltw, _), _ = cv2.getTextSize(lbl_str, font, 0.38, 1)
        cv2.putText(frame, lbl_str, (int(center_x - ltw / 2), dock_y + 15), font, 0.38, col, 1, cv2.LINE_AA)
        cv2.rectangle(frame, (meter_x, meter_y), (meter_x + meter_w, meter_y + meter_h), (50, 50, 50), -1)
        fill_w = int(meter_w * min(score, 1.0))
        if fill_w > 0:
            cv2.rectangle(frame, (meter_x, meter_y), (meter_x + fill_w, meter_y + meter_h), col, -1)

    return frame


def main():
    parser = argparse.ArgumentParser(description="Pothole Detection & Swerve Indicator")
    parser.add_argument("--video", default=None, help="Input video path (default: pothole_test_2.mp4 or test_video.mp4)")
    parser.add_argument("--output", default=None, help="Output video path (default: annotated_<name>.mp4)")
    parser.add_argument("--duration", type=float, default=None, help="Process first N seconds")
    parser.add_argument("--mode", choices=["auto", "dashcam", "cabin"], default="auto", help="Camera perspective (auto, dashcam, cabin)")
    parser.add_argument("--conf", type=float, default=0.07, help="Detection confidence threshold")
    parser.add_argument("--show", action="store_true", help="Display live window")
    args = parser.parse_args()

    # Auto-select input video if not provided
    if args.video:
        video_path = Path(args.video)
    elif (ROOT_DIR / "pothole_test_2.mp4").exists():
        video_path = ROOT_DIR / "pothole_test_2.mp4"
    elif DEFAULT_VIDEO:
        video_path = Path(DEFAULT_VIDEO)
    else:
        print("Error: No video found in directory.", file=sys.stderr)
        sys.exit(1)

    if not video_path.exists():
        print(f"Error: Video file not found: {video_path}", file=sys.stderr)
        sys.exit(1)

    # Auto-detect camera mode
    if args.mode == "auto":
        mode = "cabin" if "test_video" in video_path.name.lower() else "dashcam"
    else:
        mode = args.mode

    # Output video path
    if args.output:
        output_path = Path(args.output)
    else:
        output_path = ROOT_DIR / f"annotated_{video_path.stem}.mp4"

    print(f"[+] Camera Perspective : {mode.upper()} mode")
    print(f"[+] Detection Conf Thresh: {args.conf}")

    detector = PotholeDetector(conf_thresh=args.conf, mode=mode)
    cap = cv2.VideoCapture(str(video_path))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frames_to_run = min(int(args.duration * fps), total_frames) if args.duration else total_frames

    out = cv2.VideoWriter(str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (fw, fh))

    print(f"\n[+] Input Video  : {video_path} ({frames_to_run} frames, {frames_to_run/fps:.1f}s)")
    print(f"[+] Output Video : {output_path}\n")

    t_start = time.time()
    for i in range(frames_to_run):
        ret, frame = cap.read()
        if not ret:
            break

        t0 = time.perf_counter()
        dets, scores, swerve_text, subtext, color, roi = detector.process_frame(frame)
        cur_fps = 1.0 / max(time.perf_counter() - t0, 0.001)

        annotated = draw_annotations(frame, dets, scores, swerve_text, subtext, color, roi, cur_fps, i + 1, frames_to_run)
        out.write(annotated)

        if (i + 1) % 30 == 0 or i == 0 or (i + 1) == frames_to_run:
            print(f"  Frame {i+1:4d}/{frames_to_run} | {swerve_text}")

        if args.show:
            cv2.imshow("Pothole Avoidance", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    cap.release()
    out.release()
    if args.show:
        cv2.destroyAllWindows()

    print(f"\n[+] Complete in {time.time() - t_start:.1f}s! Saved to: {output_path}\n")


if __name__ == "__main__":
    main()
