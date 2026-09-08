"""
YOLOv10n Benchmark + Inference Script for Oracle Ampere A1 (ARM)
================================================================
This script does everything from the notebooks in one go:
1. Downloads YOLOv10n
2. Exports to NCNN FP16 + OpenVINO FP16
3. Benchmarks all formats on your dashcam video
4. Saves annotated output video with detections drawn

Usage:
    python benchmark.py                          # uses dashcam_test.mp4
    python benchmark.py --video my_video.mp4     # custom video
    python benchmark.py --imgsz 480              # higher resolution
"""

import argparse
import time
import os
import numpy as np

def install_deps():
    """Install required packages if missing."""
    try:
        import ultralytics
        import cv2
    except ImportError:
        print("Installing dependencies...")
        os.system("pip install ultralytics opencv-python-headless")
        print("Done. Re-run the script.\n")
        exit(0)

install_deps()

import cv2
from pathlib import Path
from ultralytics import YOLO


def export_models(imgsz):
    """Export YOLOv10n to NCNN and OpenVINO formats."""
    print("\n" + "=" * 60)
    print("STEP 1: EXPORTING MODELS")
    print("=" * 60)

    model = YOLO("yolov10n.pt")

    # --- NCNN FP16 ---
    ncnn_dir = Path("yolov10n_ncnn_model")
    if ncnn_dir.exists():
        print(f"\n✅ NCNN FP16 already exported: {ncnn_dir}")
    else:
        print("\n📦 Exporting to NCNN FP16...")
        model.export(format="ncnn", imgsz=imgsz, half=True)
        print(f"✅ NCNN FP16 exported")

    # --- OpenVINO FP16 ---
    ov_dir = Path("yolov10n_openvino_model")
    if ov_dir.exists():
        print(f"✅ OpenVINO FP16 already exported: {ov_dir}")
    else:
        print("\n📦 Exporting to OpenVINO FP16...")
        model.export(format="openvino", imgsz=imgsz, half=True)
        print(f"✅ OpenVINO FP16 exported")

    # --- ONNX ---
    onnx_file = Path("yolov10n.onnx")
    if onnx_file.exists():
        print(f"✅ ONNX already exported: {onnx_file}")
    else:
        print("\n📦 Exporting to ONNX...")
        model.export(format="onnx", imgsz=imgsz)
        print(f"✅ ONNX exported")


def benchmark_model(model_path, video_path, imgsz=320, num_frames=100, label=""):
    """Benchmark a model on video frames. Returns results dict."""
    print(f"\n{'─' * 50}")
    print(f"Benchmarking: {label} ({model_path})")
    print(f"{'─' * 50}")

    model = YOLO(str(model_path))
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print(f"❌ Cannot open video: {video_path}")
        return None

    # Warmup
    print("  Warming up (5 frames)...")
    for _ in range(5):
        ret, frame = cap.read()
        if ret:
            model.predict(frame, imgsz=imgsz, conf=0.25, verbose=False)

    # Reset video
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    # Benchmark
    latencies = []
    det_total = 0
    print(f"  Running {num_frames} frames...")

    for i in range(num_frames):
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = cap.read()

        t0 = time.perf_counter()
        results = model.predict(frame, imgsz=imgsz, conf=0.25, verbose=False)
        t1 = time.perf_counter()

        latencies.append((t1 - t0) * 1000)
        det_total += len(results[0].boxes)

    cap.release()
    latencies = np.array(latencies)
    avg_fps = 1000.0 / latencies.mean()

    print(f"\n  📊 {label}")
    print(f"     Avg latency:  {latencies.mean():.1f} ms")
    print(f"     Median:       {np.median(latencies):.1f} ms")
    print(f"     Min/Max:      {latencies.min():.1f} / {latencies.max():.1f} ms")
    print(f"     Avg FPS:      {avg_fps:.1f}")
    print(f"     Detections:   {det_total / num_frames:.1f}/frame")

    return {
        "label": label,
        "avg_fps": avg_fps,
        "avg_latency_ms": latencies.mean(),
        "median_latency_ms": np.median(latencies),
        "detections_per_frame": det_total / num_frames,
    }


def run_benchmarks(video_path, imgsz):
    """Benchmark all available models."""
    print("\n" + "=" * 60)
    print("STEP 2: BENCHMARKING ALL FORMATS")
    print(f"  Video: {video_path}")
    print(f"  Resolution: {imgsz}x{imgsz}")
    print("=" * 60)

    models = [
        ("yolov10n.pt", "PyTorch FP32 (baseline)"),
        ("yolov10n.onnx", "ONNX FP32"),
        ("yolov10n_ncnn_model", "NCNN FP16"),
        ("yolov10n_openvino_model", "OpenVINO FP16"),
    ]

    results = []
    for path, label in models:
        if Path(path).exists():
            r = benchmark_model(path, video_path, imgsz=imgsz, num_frames=100, label=label)
            if r:
                results.append(r)
        else:
            print(f"\n  ⚠️  Skipping {label} — {path} not found")

    # Print comparison table
    if results:
        print(f"\n\n{'=' * 60}")
        print(f"{'BENCHMARK COMPARISON (ARM — Ampere A1)':^60}")
        print(f"{'=' * 60}")
        print(f"{'Model':<25} {'Avg FPS':>10} {'Latency (ms)':>15} {'Det/frame':>12}")
        print(f"{'-' * 25} {'-' * 10} {'-' * 15} {'-' * 12}")
        for r in results:
            print(f"{r['label']:<25} {r['avg_fps']:>10.1f} {r['avg_latency_ms']:>15.1f} {r['detections_per_frame']:>10.1f}")

        # Find winner
        best = max(results, key=lambda x: x["avg_fps"])
        print(f"\n  🏆 Fastest: {best['label']} at {best['avg_fps']:.1f} FPS")

    return results


def resolution_comparison(video_path):
    """Test different resolutions."""
    print(f"\n\n{'=' * 60}")
    print("STEP 3: RESOLUTION COMPARISON (NCNN FP16)")
    print("=" * 60)

    model = YOLO("yolov10n_ncnn_model")
    cap = cv2.VideoCapture(video_path)

    frames = []
    for _ in range(50):
        ret, frame = cap.read()
        if ret:
            frames.append(frame)
    cap.release()

    if not frames:
        print("❌ Could not capture frames.")
        return

    resolutions = [640, 480, 320]  # 256 excluded: NCNN segfaults below export resolution (320)
    print(f"\n{'Resolution':<15} {'Avg Latency':>12} {'Avg FPS':>10} {'Detections':>12}")
    print(f"{'-' * 15} {'-' * 12} {'-' * 10} {'-' * 12}")

    for res in resolutions:
        # Warmup
        for f in frames[:3]:
            model.predict(f, imgsz=res, verbose=False)

        latencies = []
        det_count = 0
        for f in frames:
            t0 = time.perf_counter()
            r = model.predict(f, imgsz=res, conf=0.25, verbose=False)
            t1 = time.perf_counter()
            latencies.append((t1 - t0) * 1000)
            det_count += len(r[0].boxes)

        lat = np.array(latencies)
        print(f"{res}x{res:<10} {lat.mean():>10.1f}ms {1000 / lat.mean():>10.1f} {det_count / len(frames):>10.1f}/frame")

    print(f"\n💡 320x320 is the sweet spot for real-time robotics.")


def save_annotated_video(video_path, imgsz):
    """Save full annotated video with detection boxes."""
    print(f"\n\n{'=' * 60}")
    print("STEP 4: SAVING ANNOTATED VIDEO")
    print("=" * 60)

    model = YOLO("yolov10n_ncnn_model")
    cap = cv2.VideoCapture(video_path)

    fps = int(cap.get(cv2.CAP_PROP_FPS)) or 30
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    output_path = "dashcam_annotated.mp4"
    out = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))

    print(f"  Input:  {video_path} ({total} frames, {w}x{h}, {fps} FPS)")
    print(f"  Output: {output_path}")
    print(f"  Processing...")

    for i in range(total):
        ret, frame = cap.read()
        if not ret:
            break

        results = model.predict(frame, imgsz=imgsz, conf=0.25, verbose=False)
        annotated = results[0].plot()
        out.write(annotated)

        if (i + 1) % 100 == 0:
            print(f"    {i + 1}/{total} frames...")

    cap.release()
    out.release()

    size_mb = os.path.getsize(output_path) / 1024 / 1024
    print(f"\n  ✅ Saved: {output_path} ({size_mb:.1f} MB)")


def main():
    parser = argparse.ArgumentParser(description="YOLOv10n Benchmark on ARM")
    parser.add_argument("--video", default="dashcam_test.mp4", help="Path to test video")
    parser.add_argument("--imgsz", type=int, default=320, help="Inference resolution (default: 320)")
    parser.add_argument("--skip-export", action="store_true", help="Skip model export step")
    parser.add_argument("--skip-video", action="store_true", help="Skip annotated video output")
    args = parser.parse_args()

    if not os.path.exists(args.video):
        print(f"❌ Video not found: {args.video}")
        print(f"   Upload your dashcam video and name it '{args.video}'")
        print(f"   Or specify: python benchmark.py --video your_video.mp4")
        return

    print("=" * 60)
    print("  YOLOv10n ARM Benchmark — Oracle Ampere A1")
    print("=" * 60)
    print(f"  Video:      {args.video}")
    print(f"  Resolution: {args.imgsz}x{args.imgsz}")
    print(f"  Platform:   ARM (simulating Pi 5)")

    # Step 1: Export models
    if not args.skip_export:
        export_models(args.imgsz)

    # Step 2: Benchmark all formats
    run_benchmarks(args.video, args.imgsz)

    # Step 3: Resolution comparison
    resolution_comparison(args.video)

    # Step 4: Save annotated video
    if not args.skip_video:
        save_annotated_video(args.video, args.imgsz)

    print(f"\n\n{'=' * 60}")
    print("  ALL DONE!")
    print("=" * 60)
    print("  📥 Download 'dashcam_annotated.mp4' to review detections")
    print("  📊 Compare the FPS numbers above with your Colab results")
    print("     (These ARM numbers ≈ what you'll get on real Pi 5)")
    print("=" * 60)


if __name__ == "__main__":
    main()
