"""
YOLOv10n Video & Webcam Live Inference Demo
===========================================
Capstone Project: Autonomous RC Model Car
Authors: Krishna Menon, Aditya Rana, Parth Surve
Target: Raspberry Pi 5 + Arduino Uno

Usage Examples:
    # 1. Run on the bundled dashcam test video:
    python test_yolov10_video.py --video dashcam_test.mp4

    # 2. Run on any custom user video:
    python test_yolov10_video.py --video path/to/any_video.mp4

    # 3. Run live on your laptop or USB webcam:
    python test_yolov10_video.py --webcam 0

    # 4. Save an annotated output video for presentations:
    python test_yolov10_video.py --video dashcam_test.mp4 --save output_annotated.mp4

Keyboard Shortcuts during playback:
    'q' : Quit demo
    'p' : Pause / Resume
    's' : Save current frame as screenshot
"""

import argparse
import sys
import time
from pathlib import Path
import cv2
import numpy as np
from ultralytics import YOLO

def parse_args():
    parser = argparse.ArgumentParser(description="Live YOLOv10n Video/Webcam Inference Demo")
    parser.add_argument("--video", type=str, default="pothole_test_2.mp4", help="Path to input video file (default: pothole_test_2.mp4)")
    parser.add_argument("--webcam", type=int, default=None, help="Webcam device index (e.g. 0)")
    parser.add_argument("--model", type=str, default="yolov10n.pt", help="Path to YOLOv10 weights (.pt or .onnx or ncnn directory)")
    parser.add_argument("--imgsz", type=int, default=320, help="Inference resolution: 320, 480, or 640 (default: 320 for speed)")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold (default: 0.25)")
    parser.add_argument("--save", type=str, default=None, help="Optional output path to save annotated .mp4 video")
    parser.add_argument("--headless", action="store_true", help="Run without opening GUI window (batch processing mode)")
    parser.add_argument("--max-frames", type=int, default=None, help="Optional maximum number of frames to process before stopping")
    return parser.parse_args()

def draw_hud(frame, fps, latency_ms, det_count, counts_by_class, imgsz):
    """Draws a clean, professional semi-transparent heads-up display overlay."""
    h, w = frame.shape[:2]
    
    # HUD background card (Top-Left)
    card_w = min(420, w - 20)
    card_h = 110 + (25 if counts_by_class else 0)
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (10 + card_w, 10 + card_h), (20, 25, 35), -1)
    cv2.rectangle(overlay, (10, 10), (10 + card_w, 10 + card_h), (0, 180, 255), 2)
    cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)

    # Text styling
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, "YOLOv10n Real-Time Perception", (22, 35), font, 0.65, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, f"Architecture: End-to-End NMS-Free | {imgsz}x{imgsz}", (22, 58), font, 0.42, (200, 200, 200), 1, cv2.LINE_AA)
    
    # Latency & FPS indicators
    fps_color = (0, 255, 100) if fps >= 25 else (0, 200, 255) if fps >= 15 else (0, 100, 255)
    cv2.putText(frame, f"Inference: {latency_ms:.1f} ms", (22, 82), font, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, f"FPS: {fps:.1f}", (240, 82), font, 0.65, fps_color, 2, cv2.LINE_AA)

    # Detection breakdown
    summary_txt = f"Objects: {det_count}"
    if counts_by_class:
        class_details = ", ".join([f"{k}:{v}" for k, v in list(counts_by_class.items())[:4]])
        summary_txt += f" ({class_details})"
    cv2.putText(frame, summary_txt, (22, 108), font, 0.45, (100, 230, 255), 1, cv2.LINE_AA)

def main():
    args = parse_args()

    print("=" * 65)
    print(" YOLOv10n Real-Time Inference Demo — Model Car Capstone ")
    print("=" * 65)
    print(f" • Model:       {args.model}")
    print(f" • Input:       {'Webcam ' + str(args.webcam) if args.webcam is not None else args.video}")
    print(f" • Resolution:  {args.imgsz}x{args.imgsz}")
    print(f" • Confidence:  {args.conf}")
    if args.save:
        print(f" • Save Output: {args.save}")
    print("=" * 65)

    # 1. Load YOLOv10 Model
    if not Path(args.model).exists():
        print(f"❌ Error: Model weights '{args.model}' not found in current folder.")
        sys.exit(1)

    print(f"Loading weights '{args.model}'...")
    model = YOLO(args.model)
    print(f"✅ Model loaded successfully! (Detects {len(model.names)} classes)")

    # 2. Open Video Capture Source
    source = args.webcam if args.webcam is not None else args.video
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"❌ Error: Could not open video source '{source}'")
        sys.exit(1)

    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    print(f"✅ Video source opened: {src_w}x{src_h} @ {src_fps:.1f} FPS")

    # 3. Setup Optional Video Writer
    writer = None
    if args.save:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.save, fourcc, src_fps, (src_w, src_h))
        print(f"Recording annotated video to: {args.save}")

    window_name = "YOLOv10n Live Inference Demo (Press 'q' to Quit, 'p' to Pause)"
    use_gui = not args.headless

    frame_count = 0
    fps_history = []
    paused = False

    print("\n▶️ Starting live inference loop... (Press 'q' in window to exit)\n")

    try:
        while True:
            if not paused:
                ret, frame = cap.read()
                if not ret:
                    # Loop back to beginning if it's a file
                    if args.webcam is None:
                        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        ret, frame = cap.read()
                        if not ret:
                            break
                    else:
                        break

                frame_count += 1
                if args.max_frames and frame_count > args.max_frames:
                    print(f"Reached maximum frame limit ({args.max_frames}). Stopping.")
                    break

                # Measure inference latency
                t0 = time.perf_counter()
                results = model.predict(
                    frame,
                    imgsz=args.imgsz,
                    conf=args.conf,
                    verbose=False
                )
                t1 = time.perf_counter()
                latency_ms = (t1 - t0) * 1000.0
                curr_fps = 1000.0 / max(latency_ms, 1.0)
                fps_history.append(curr_fps)
                if len(fps_history) > 30:
                    fps_history.pop(0)
                avg_fps = sum(fps_history) / len(fps_history)

                # Process detections
                det_count = 0
                counts_by_class = {}
                annotated_frame = frame.copy()

                if len(results) > 0 and results[0].boxes is not None:
                    boxes = results[0].boxes
                    det_count = len(boxes)
                    for box in boxes:
                        cls_id = int(box.cls[0].item())
                        cls_name = model.names.get(cls_id, f"ID_{cls_id}")
                        conf_val = float(box.conf[0].item())
                        counts_by_class[cls_name] = counts_by_class.get(cls_name, 0) + 1

                        # Box coordinates
                        xyxy = box.xyxy[0].cpu().numpy().astype(int)
                        x1, y1, x2, y2 = xyxy

                        # Draw bounding box
                        color = (0, 255, 120) if cls_id == 0 else (255, 180, 0) if cls_id in [2, 3, 5, 7] else (0, 200, 255)
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                        
                        # Label badge
                        label_str = f"{cls_name} {conf_val:.2f}"
                        (tw, th), _ = cv2.getTextSize(label_str, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                        cv2.rectangle(annotated_frame, (x1, max(0, y1 - 22)), (x1 + tw + 6, max(22, y1)), color, -1)
                        cv2.putText(annotated_frame, label_str, (x1 + 3, max(16, y1 - 6)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (10, 10, 10), 1, cv2.LINE_AA)

                # Overlay HUD
                draw_hud(annotated_frame, avg_fps, latency_ms, det_count, counts_by_class, args.imgsz)

                # Write to output video if enabled
                if writer:
                    writer.write(annotated_frame)

                # Periodic terminal heartbeat every 30 frames
                if frame_count % 30 == 0:
                    print(f"[{frame_count:04d} frames] Latency: {latency_ms:5.1f} ms | FPS: {avg_fps:4.1f} | Objects: {det_count}")

            # GUI display handling
            if use_gui:
                try:
                    cv2.imshow(window_name, annotated_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q') or key == 27:  # 'q' or ESC
                        print("\nExiting demo...")
                        break
                    elif key == ord('p'):
                        paused = not paused
                        print("⏸️ Paused" if paused else "▶️ Resumed")
                    elif key == ord('s'):
                        shot_path = f"screenshot_frame_{frame_count}.jpg"
                        cv2.imwrite(shot_path, annotated_frame)
                        print(f"📸 Screenshot saved to {shot_path}")
                except cv2.error as e:
                    # In case of headless environment without X11 or Win32 window support
                    print(f"\n⚠️ GUI window not available: {e}. Switching to headless mode.")
                    use_gui = False

    except KeyboardInterrupt:
        print("\nSession interrupted by user.")

    finally:
        cap.release()
        if writer:
            writer.release()
            print(f"✅ Saved full annotated video to: {args.save}")
        if use_gui:
            cv2.destroyAllWindows()
        print("Demo completed successfully.\n")

if __name__ == "__main__":
    main()
