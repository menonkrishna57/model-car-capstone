#!/usr/bin/env python3
"""
train_pothole_model.py
======================
Trains / fine-tunes YOLOv10n on RDD2022 India (D40 Pothole Benchmark) with
synthetic sensor noise, blur, lighting glare, and "held-photo in front of camera"
augmentations.
"""

import os
import sys
import shutil
import argparse
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent
os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT_DIR / ".ultralytics_cfg"))
(ROOT_DIR / ".ultralytics_cfg").mkdir(parents=True, exist_ok=True)

import torch
from ultralytics import YOLO

DATASET_YAML = ROOT_DIR / "noisy_pothole_dataset" / "noisy_rdd_dataset.yaml"
if not DATASET_YAML.exists():
    DATASET_YAML = ROOT_DIR / "negative_obstacle_detection" / "rdd_yolo_dataset" / "rdd_dataset.yaml"
if not DATASET_YAML.exists():
    DATASET_YAML = ROOT_DIR / "rdd_yolo_dataset" / "rdd_dataset.yaml"


def main():
    parser = argparse.ArgumentParser(description="Train/Fine-Tune YOLOv10n on Noisy + Held-Photo Pothole Dataset")
    parser.add_argument("--epochs", type=int, default=25, help="Number of epochs (default: 25)")
    parser.add_argument("--batch", type=int, default=8, help="Batch size (default: 8, optimal for 4GB VRAM)")
    parser.add_argument("--imgsz", type=int, default=640, help="Image resolution (default: 640)")
    parser.add_argument("--build-noisy-data", action="store_true", help="Regenerate noisy + held-photo dataset before training")
    args = parser.parse_args()

    if args.build_noisy_data or not DATASET_YAML.exists():
        print("[*] Generating noise-augmented + held-photo pothole dataset...")
        from augment_and_finetune import extract_base_frames_and_labels, build_noisy_yolo_dataset
        samples = extract_base_frames_and_labels()
        yaml_path = build_noisy_yolo_dataset(samples)
    else:
        yaml_path = DATASET_YAML

    if not yaml_path.exists():
        print(f"❌ Error: Dataset config not found at {yaml_path}", file=sys.stderr)
        sys.exit(1)

    device = 0 if torch.cuda.is_available() else "cpu"
    device_name = torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU"

    print("=" * 65)
    print("  AUTONOMOUS VEHICLE PERCEPTION - NOISY POTHOLE MODEL FINE-TUNING")
    print("=" * 65)
    print(f"  Target Device : {device_name} (ID: {device})")
    print(f"  Dataset Config: {yaml_path}")
    print(f"  Total Epochs  : {args.epochs}")
    print(f"  Batch Size    : {args.batch}")
    print(f"  Resolution    : {args.imgsz}x{args.imgsz}")
    print("=" * 65 + "\n")

    # Start from existing best.pt for fine-tuning if available, else yolov10n.pt
    base_weights = ROOT_DIR / "best.pt"
    if not base_weights.exists():
        base_weights = ROOT_DIR / "yolov10n.pt"
    model = YOLO(str(base_weights) if base_weights.exists() else "yolov10n.pt")

    # Launch Training with strong noise, perspective, scale, and lighting augmentations
    model.train(
        data=str(yaml_path),
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        device=device,
        workers=2 if torch.cuda.is_available() else 0,
        lr0=0.002,
        lrf=0.05,
        hsv_h=0.025,
        hsv_s=0.75,
        hsv_v=0.55,
        degrees=12.0,
        translate=0.18,
        scale=0.60,
        shear=4.0,
        perspective=0.0005,
        mosaic=1.0,
        mixup=0.15,
        erasing=0.35,
        project=str(ROOT_DIR / "runs_rdd"),
        name="custom_pothole_model",
        exist_ok=True,
        verbose=True,
    )

    best_weights_src = ROOT_DIR / "runs_rdd" / "custom_pothole_model" / "weights" / "best.pt"
    best_weights_dst = ROOT_DIR / "best.pt"
    if best_weights_src.exists():
        shutil.copy(best_weights_src, best_weights_dst)
        shutil.copy(best_weights_src, ROOT_DIR.parent / "pothole_best.pt")
        sub_dst = ROOT_DIR / "negative_obstacle_detection" / "best.pt"
        if sub_dst.parent.exists():
            shutil.copy(best_weights_src, sub_dst)
        print("\n" + "=" * 65)
        print("  FINE-TUNING COMPLETED SUCCESSFULLY!")
        print("=" * 65)
        print(f"  [+] Saved Best Model to : {best_weights_dst}")
        print(f"  [+] Results & Metrics in : {ROOT_DIR / 'runs_rdd' / 'custom_pothole_model'}")
        print("=" * 65 + "\n")


if __name__ == "__main__":
    main()
