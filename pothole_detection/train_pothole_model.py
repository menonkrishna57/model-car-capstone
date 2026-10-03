#!/usr/bin/env python3
"""
train_pothole_model.py
======================
Trains YOLOv10n on RDD2022 India (D40 Pothole Benchmark) using your NVIDIA RTX 3050 GPU.
Estimated runtime: ~20 to 25 minutes for 50 epochs.
"""

import sys
import shutil
import argparse
from pathlib import Path
import torch
from ultralytics import YOLO

ROOT_DIR = Path(__file__).resolve().parent
DATASET_YAML = ROOT_DIR / "negative_obstacle_detection" / "rdd_yolo_dataset" / "rdd_dataset.yaml"
if not DATASET_YAML.exists():
    DATASET_YAML = ROOT_DIR / "rdd_yolo_dataset" / "rdd_dataset.yaml"


def main():
    parser = argparse.ArgumentParser(description="Train YOLOv10n on Indian Potholes (RTX 3050 GPU)")
    parser.add_argument("--epochs", type=int, default=25, help="Number of epochs (default: 25, ~8-10 mins)")
    parser.add_argument("--batch", type=int, default=8, help="Batch size (default: 8, optimal for 4GB VRAM)")
    parser.add_argument("--imgsz", type=int, default=640, help="Image resolution (default: 640)")
    args = parser.parse_args()

    if not DATASET_YAML.exists():
        print(f"❌ Error: Dataset config not found at {DATASET_YAML}", file=sys.stderr)
        sys.exit(1)

    # Detect GPU
    device = 0 if torch.cuda.is_available() else "cpu"
    device_name = torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU"

    print("=" * 65)
    print("  AUTONOMOUS VEHICLE PERCEPTION - POTHOLE MODEL TRAINING")
    print("=" * 65)
    print(f"  Target Device : {device_name} (ID: {device})")
    print(f"  Dataset Config: {DATASET_YAML}")
    print(f"  Total Epochs  : {args.epochs}")
    print(f"  Batch Size    : {args.batch} (Safe for 4GB RTX 3050)")
    print(f"  Resolution    : {args.imgsz}x{args.imgsz}")
    print("=" * 65 + "\n")

    # Load base YOLOv10n
    base_weights = ROOT_DIR / "yolov10n.pt"
    model = YOLO(str(base_weights) if base_weights.exists() else "yolov10n.pt")

    # Launch Training
    results = model.train(
        data=str(DATASET_YAML),
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        device=device,
        workers=2,
        project=str(ROOT_DIR / "runs_rdd"),
        name="custom_pothole_model",
        exist_ok=True,
        verbose=True
    )

    # Copy best weights to root for immediate use
    best_weights_src = ROOT_DIR / "runs_rdd" / "custom_pothole_model" / "weights" / "best.pt"
    best_weights_dst = ROOT_DIR / "best.pt"
    if best_weights_src.exists():
        shutil.copy(best_weights_src, best_weights_dst)
        # Also copy to subfolder
        sub_dst = ROOT_DIR / "negative_obstacle_detection" / "best.pt"
        shutil.copy(best_weights_src, sub_dst)
        print("\n" + "=" * 65)
        print("  TRAINING COMPLETED SUCCESSFULLY!")
        print("=" * 65)
        print(f"  [+] Saved Best Model to : {best_weights_dst}")
        print(f"  [+] Results & Metrics in : {ROOT_DIR / 'runs_rdd' / 'custom_pothole_model'}")
        print("  You can now run: python run_detection.py to test your trained model!")
        print("=" * 65 + "\n")


if __name__ == "__main__":
    main()
