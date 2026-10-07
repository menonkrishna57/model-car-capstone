#!/usr/bin/env python3
"""
LIVE DEMO: YOLOv10n Real-Time Obstacle Detection
================================================
Dedicated standalone script for evaluating YOLOv10n obstacle detection.
- Fast, non-blocking real-time OpenCV window display
- Targets pedestrians, cars, bicycles, motorcycles, trucks, hazards on roads
- Proximity estimation (CLOSE / MED / FAR)
- Live FPS and Latency HUD
- Accepts ANY video path from ANY folder or drive
- No disk saving; pure real-time evaluation

Usage:
    python demo_yolov10_live.py "C:\\path\\to\\any_video.mp4"
    python demo_yolov10_live.py --video "D:\\traffic_video.mp4"
    python demo_yolov10_live.py --webcam
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


def find_model():
    base_dir = Path(__file__).resolve().parent
    for name in ["yolov10n.pt", "yolov10n_ncnn_model"]:
        p = base_dir / name
        if p.exists():
            return str(p)
    return "yolov10n.pt"


def main():
    parser = argparse.ArgumentParser(description="Live YOLOv10n Obstacle Detection")
    parser.add_argument("video_pos", nargs="?", default=None, help="Path to video file (positional)")
    parser.add_argument("--video", "-v", default=None, help="Path to video file (flag)")
    parser.add_argument("--webcam", action="store_true", help="Use live webcam feed")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold (default 0.25)")
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
        for candidate in ["pothole_test_2.mp4", "dashcam_test.mp4", "pothole_detection/pothole_test_2.mp4"]:
            if Path(candidate).exists():
                video_src = str(Path(candidate).resolve())
                break

    if video_src is None and not args.webcam:
        user_input = input("Enter path to MP4 video file: ").strip().strip('"').strip("'")
        video_src = resolve_video_path(user_input)
        if not video_src:
            print("❌ Invalid file path. Exiting.")
            return

    model_path = find_model()
    print(f"[*] Loading YOLOv10n: {model_path}")
    model = YOLO(model_path)

    cap = cv2.VideoCapture(video_src)
    if not cap.isOpened():
        print(f"❌ Failed to open video: {video_src}")
        return

    win_name = "YOLOv10n Real-Time Obstacle Detection — Live Evaluation"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 1280, 720)

    fps = 30.0
    prev_time = time.perf_counter()
    paused = False

    print("=" * 60)
    print("  YOLOv10n LIVE OBSTACLE DEMO RUNNING")
    print(f"  Input:    {video_src}")
    print("  Controls: [SPACE] Pause | [R] Replay | [Q] Quit")
    print("=" * 60)

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = cap.read()
                if not ret:
                    break

            t0 = time.perf_counter()
            results = model.predict(frame, conf=args.conf, imgsz=args.imgsz, verbose=False)[0]
            t1 = time.perf_counter()

            curr_time = time.perf_counter()
            fps = 0.9 * fps + 0.1 * (1.0 / max(curr_time - prev_time, 1e-4))
            prev_time = curr_time
            latency_ms = (t1 - t0) * 1000

            h, w = frame.shape[:2]
            overlay = frame.copy()

            # Top Header Bar
            cv2.rectangle(overlay, (0, 0), (w, 52), (20, 20, 25), -1)
            cv2.addWeighted(overlay, 0.85, frame, 0.15, 0, frame)

            cv2.putText(frame, "YOLOv10n REAL-TIME OBSTACLE DETECTION", (16, 25),
                        cv2.FONT_HERSHEY_DUPLEX, 0.65, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, f"Detections: {len(results.boxes)} | Conf: {args.conf}", (16, 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 220, 255), 1, cv2.LINE_AA)

            hud_right = f"FPS: {fps:.1f} | Latency: {latency_ms:.1f} ms"
            (tw, _), _ = cv2.getTextSize(hud_right, cv2.FONT_HERSHEY_SIMPLEX, 0.50, 1)
            cv2.putText(frame, hud_right, (w - tw - 16, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (100, 255, 160), 1, cv2.LINE_AA)

            # Draw Detections
            for box in results.boxes:
                cls_id = int(box.cls[0])
                c_val = float(box.conf[0])
                label = results.names.get(cls_id, "OBJ").upper()
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]

                # Estimate proximity based on box height relative to frame height
                box_h_ratio = (y2 - y1) / float(h)
                if box_h_ratio > 0.35:
                    prox = "CLOSE"
                    color = (40, 40, 240)  # Red
                elif box_h_ratio > 0.16:
                    prox = "MED"
                    color = (30, 200, 255)  # Amber
                else:
                    prox = "FAR"
                    color = (235, 160, 40)  # Blue/Cyan

                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                tag = f"{label} {int(c_val*100)}% [{prox}]"
                (lw, lh), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                cv2.rectangle(frame, (x1, max(y1 - lh - 6, 0)), (x1 + lw + 6, y1), color, -1)
                cv2.putText(frame, tag, (x1 + 3, y1 - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

            ctrl = "[SPACE: Pause | R: Replay | Q: Quit]"
            cv2.putText(frame, ctrl, (w - 240, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 160, 160), 1, cv2.LINE_AA)
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

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
