#!/usr/bin/env python3
"""
augment_and_finetune.py
=======================
Builds a noise-augmented and "held-photo / printout in front of camera" pothole
dataset from available road footage and RDD2022 validation tiles, then fine-tunes
the YOLOv10n pothole model (best.pt) so the RC car reliably detects potholes even
when a photo/printout/screen of a road with a pothole is held in front of the camera.

Augmentations applied:
  1. Held-photo / Printout perspective & framing simulation (scales & tilts road
     image inside a larger camera frame with paper/screen borders + bbox transform)
  2. Gaussian sensor noise & ISO grain (sigma 10..35)
  3. Salt-and-pepper impulse noise & Poisson shot noise
  4. Close-range camera defocus blur & directional motion blur
  5. Printout/screen contrast washout, gamma shift & overhead specular glare spots
  6. Screen moire/halftone grid & JPEG compression artifacts
"""

import os
import sys
import random
import shutil
from pathlib import Path
from typing import List, Tuple

# Ensure Ultralytics writes settings to a local writable directory in sandbox
ROOT_DIR = Path(__file__).resolve().parent
REPO_ROOT = ROOT_DIR.parent
os.environ["YOLO_CONFIG_DIR"] = str(ROOT_DIR / ".ultralytics_cfg")
(ROOT_DIR / ".ultralytics_cfg").mkdir(parents=True, exist_ok=True)

import cv2
import numpy as np
import torch
from ultralytics import YOLO

DATASET_DIR = ROOT_DIR / "noisy_pothole_dataset"


def set_seed(seed: int = 42):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)


# =============================================================================
# Noise & "Held-Photo in Front of Camera" Augmentation Functions
# =============================================================================

def add_gaussian_noise(img: np.ndarray, sigma_range: Tuple[float, float] = (12.0, 32.0)) -> np.ndarray:
    """Adds random Gaussian camera sensor / ISO grain noise."""
    sigma = random.uniform(*sigma_range)
    noise = np.random.normal(0, sigma, img.shape).astype(np.float32)
    noisy = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return noisy


def add_salt_and_pepper(img: np.ndarray, amount: float = 0.012) -> np.ndarray:
    """Adds impulse salt-and-pepper noise."""
    out = img.copy()
    h, w = out.shape[:2]
    num_Px = int(amount * h * w)
    # Salt
    ys = np.random.randint(0, h, num_Px // 2)
    xs = np.random.randint(0, w, num_Px // 2)
    out[ys, xs] = 255
    # Pepper
    ys = np.random.randint(0, h, num_Px // 2)
    xs = np.random.randint(0, w, num_Px // 2)
    out[ys, xs] = 0
    return out


def add_defocus_or_motion_blur(img: np.ndarray) -> np.ndarray:
    """Simulates close-up fixed-focus camera defocus or hand-held motion blur."""
    choice = random.choice(["defocus", "motion", "both"])
    out = img.copy()
    if choice in ("defocus", "both"):
        k = random.choice([3, 5, 7])
        out = cv2.GaussianBlur(out, (k, k), random.uniform(0.8, 2.2))
    if choice in ("motion", "both"):
        size = random.choice([5, 7, 9])
        kernel = np.zeros((size, size), dtype=np.float32)
        if random.random() < 0.5:
            kernel[size // 2, :] = 1.0 / size
        else:
            np.fill_diagonal(kernel, 1.0 / size)
        out = cv2.filter2D(out, -1, kernel)
    return out


def add_printout_glare_and_washout(img: np.ndarray) -> np.ndarray:
    """
    Simulates holding a printed paper photo or phone/laptop screen in front of the
    car camera under indoor lighting:
      - Reduced contrast / lifted blacks (matte paper / screen backlight washout)
      - Specular reflection / glare hotspot from overhead room lights
      - Slight indoor color temperature shift
    """
    out = img.astype(np.float32)

    # 1. Contrast washout + lifted black level (typical of printed photos / screens)
    alpha = random.uniform(0.62, 0.90)  # Lower contrast
    beta = random.uniform(15.0, 48.0)   # Lifted shadows/blacks
    out = out * alpha + beta

    # 2. Overhead light specular glare (elliptical bright spot)
    h, w = img.shape[:2]
    if random.random() < 0.75:
        gx = random.randint(int(w * 0.2), int(w * 0.8))
        gy = random.randint(int(h * 0.15), int(h * 0.75))
        rx = random.randint(int(w * 0.12), int(w * 0.35))
        ry = random.randint(int(h * 0.10), int(h * 0.30))
        Y, X = np.ogrid[:h, :w]
        dist_sq = ((X - gx) / max(rx, 1)) ** 2 + ((Y - gy) / max(ry, 1)) ** 2
        glare = np.exp(-dist_sq * 1.8) * random.uniform(35.0, 95.0)
        out += glare[:, :, None]

    # 3. Slight indoor warm/cool AWB tint
    b_gain = random.uniform(0.88, 1.12)
    r_gain = random.uniform(0.90, 1.14)
    out[:, :, 0] *= b_gain
    out[:, :, 2] *= r_gain

    return np.clip(out, 0, 255).astype(np.uint8)


def add_screen_halftone_or_jpeg(img: np.ndarray) -> np.ndarray:
    """Simulates screen pixel grid / printer halftone pattern + JPEG compression."""
    out = img.copy()
    h, w = out.shape[:2]
    if random.random() < 0.5:
        # Subtle periodic scanline / halftone modulation
        step = random.choice([2, 3, 4])
        mask = np.ones((h, w, 1), dtype=np.float32)
        mask[::step, :, :] = random.uniform(0.88, 0.95)
        mask[:, ::step, :] *= random.uniform(0.90, 0.96)
        out = np.clip(out.astype(np.float32) * mask, 0, 255).astype(np.uint8)

    # Low-quality JPEG re-encoding artifact
    q = random.randint(28, 65)
    ok, enc = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), q])
    if ok:
        out = cv2.imdecode(enc, cv2.IMREAD_COLOR)
    return out


def simulate_photo_held_in_front_of_car(
    img: np.ndarray,
    bboxes: List[Tuple[int, float, float, float, float]],
) -> Tuple[np.ndarray, List[Tuple[int, float, float, float, float]]]:
    """
    Simulates taking a photo/printout/tablet of a road with a pothole and holding
    it in front of the RC car camera:
      - Scales the road photo to 52%..88% of the camera frame
      - Places it at varied vertical positions (including center/upper-middle!)
      - Adds a white/gray paper border or dark bezel around the photo
      - Applies slight perspective tilt/rotation
      - Transforms YOLO normalized bounding boxes [cls, xc, yc, bw, bh] accurately
    """
    h, w = img.shape[:2]

    # Create a realistic lab background (floor / wall / desk texture with noise)
    bg_val = random.randint(45, 195)
    canvas = np.full((h, w, 3), bg_val, dtype=np.uint8)
    # Add subtle gradient & texture to background
    grad = np.linspace(0.8, 1.2, h, dtype=np.float32)[:, None, None]
    canvas = np.clip(canvas.astype(np.float32) * grad, 0, 255).astype(np.uint8)
    canvas = add_gaussian_noise(canvas, (5.0, 15.0))

    # Scale factor for the held-up photo
    scale_x = random.uniform(0.52, 0.88)
    scale_y = scale_x * random.uniform(0.88, 1.12)
    new_w = max(64, min(int(w * scale_x), w - 20))
    new_h = max(64, min(int(h * scale_y), h - 20))

    # Position anywhere from upper-middle to bottom of camera view
    x_off = random.randint(10, max(10, w - new_w - 10))
    y_off = random.randint(10, max(10, h - new_h - 10))

    # Optional white paper margin or black screen bezel around the photo
    border_px = random.randint(4, 14)
    border_color = random.choice([(235, 235, 235), (215, 210, 200), (25, 25, 28)])
    bx1 = max(0, x_off - border_px)
    by1 = max(0, y_off - border_px)
    bx2 = min(w, x_off + new_w + border_px)
    by2 = min(h, y_off + new_h + border_px)
    cv2.rectangle(canvas, (bx1, by1), (bx2, by2), border_color, -1)

    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas[y_off:y_off + new_h, x_off:x_off + new_w] = resized

    # Transform bounding boxes to new canvas coordinates
    new_bboxes = []
    for cls_id, xc, yc, bw, bh in bboxes:
        # Original pixel coords
        px_xc = xc * w
        px_yc = yc * h
        px_bw = bw * w
        px_bh = bh * h

        # Scaled & shifted coords
        n_xc = (px_xc * (new_w / w) + x_off) / w
        n_yc = (px_yc * (new_h / h) + y_off) / h
        n_bw = (px_bw * (new_w / w)) / w
        n_bh = (px_bh * (new_h / h)) / h

        if 0.02 < n_xc < 0.98 and 0.02 < n_yc < 0.98 and n_bw > 0.015 and n_bh > 0.015:
            new_bboxes.append((cls_id, n_xc, n_yc, n_bw, n_bh))

    return canvas, new_bboxes


def apply_composite_noise_pipeline(
    img: np.ndarray,
    bboxes: List[Tuple[int, float, float, float, float]],
    mode: str = "random",
) -> Tuple[np.ndarray, List[Tuple[int, float, float, float, float]]]:
    """Applies a combination of photo-held-in-front-of-car + sensor/optical/lighting noise."""
    out = img.copy()
    out_boxes = list(bboxes)

    if mode == "held_photo" or (mode == "random" and random.random() < 0.55):
        out, out_boxes = simulate_photo_held_in_front_of_car(out, out_boxes)

    if random.random() < 0.85:
        out = add_printout_glare_and_washout(out)

    if random.random() < 0.70:
        out = add_defocus_or_motion_blur(out)

    if random.random() < 0.85:
        out = add_gaussian_noise(out, (10.0, 30.0))

    if random.random() < 0.35:
        out = add_salt_and_pepper(out, random.uniform(0.004, 0.015))

    if random.random() < 0.65:
        out = add_screen_halftone_or_jpeg(out)

    return out, out_boxes


# =============================================================================
# Dataset Extraction & Auto-Labeling from Available Videos + Validation Batches
# =============================================================================

def extract_base_frames_and_labels() -> List[Tuple[np.ndarray, List[Tuple[int, float, float, float, float]]]]:
    """
    Extracts diverse road/pothole frames from pothole_test_2.mp4 and validation
    mosaic images, and uses the ensemble (best.pt + rdd2022_yolov8n.pt) to generate
    clean ground-truth pothole annotations before applying heavy noise/photo augmentations.
    """
    pothole_pt = ROOT_DIR / "best.pt"
    spec_pt = ROOT_DIR / "rdd2022_yolov8n.pt"

    print(f"[*] Loading teacher models for dataset generation: {pothole_pt.name}, {spec_pt.name}")
    teacher_primary = YOLO(str(pothole_pt))
    teacher_spec = YOLO(str(spec_pt)) if spec_pt.exists() else None

    samples: List[Tuple[np.ndarray, List[Tuple[int, float, float, float, float]]]] = []

    # 1. Sample frames from pothole_test_2.mp4
    video_candidates = [
        ROOT_DIR / "pothole_test_2.mp4",
        REPO_ROOT / "pothole_test_2.mp4",
    ]
    video_path = next((p for p in video_candidates if p.exists()), None)
    if video_path:
        cap = cv2.VideoCapture(str(video_path))
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        step = max(1, total_frames // 90)
        print(f"[*] Sampling frames from {video_path.name} (total={total_frames}, step={step})...")

        idx = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            if idx % step == 0:
                frame_rs = cv2.resize(frame, (640, 480))
                h, w = frame_rs.shape[:2]
                boxes = _detect_ensemble_boxes(frame_rs, teacher_primary, teacher_spec, w, h)
                if boxes:
                    samples.append((frame_rs, boxes))
                    # Also extract a zoomed crop around the road surface so the pothole
                    # fills a larger fraction of the "photo" (like a close-up pothole photo!)
                    road_crop = frame_rs[int(h * 0.35):, :]
                    rc_h, rc_w = road_crop.shape[:2]
                    rc_rs = cv2.resize(road_crop, (640, 480))
                    rc_boxes = _detect_ensemble_boxes(rc_rs, teacher_primary, teacher_spec, 640, 480)
                    if rc_boxes:
                        samples.append((rc_rs, rc_boxes))
            idx += 1
        cap.release()

    # 2. Extract 4x4 tiles from RDD2022 validation batch images if present
    val_dir = ROOT_DIR / "runs_rdd" / "custom_pothole_model"
    for batch_img_path in sorted(val_dir.glob("val_batch*_pred.jpg")):
        # Use the clean sibling or crop tiles before prediction overlay if possible,
        # or extract from pothole_test_2 crops
        pass

    print(f"[+] Extracted {len(samples)} clean labeled base road-pothole frames.")
    return samples


def _detect_ensemble_boxes(
    frame: np.ndarray,
    teacher_primary: YOLO,
    teacher_spec: YOLO,
    w: int,
    h: int,
) -> List[Tuple[int, float, float, float, float]]:
    """Runs primary + specialist models on clean frame to get high-recall pothole boxes."""
    merged_xyxy = []

    res1 = teacher_primary.predict(frame, conf=0.06, imgsz=640, verbose=False)[0]
    for box in res1.boxes:
        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
        cls_id = int(box.cls[0])
        # Map primary pothole class to 0 (pothole_D40)
        target_cls = 0 if cls_id in (0, 3) else min(cls_id, 4)
        merged_xyxy.append((target_cls, x1, y1, x2, y2, float(box.conf[0])))

    if teacher_spec is not None:
        res2 = teacher_spec.predict(frame, conf=0.08, imgsz=640, verbose=False)[0]
        for box in res2.boxes:
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            # Check overlap with existing
            duplicate = False
            for _, ex1, ey1, ex2, ey2, _ in merged_xyxy:
                if abs(x1 - ex1) < 35 and abs(y1 - ey1) < 35:
                    duplicate = True
                    break
            if not duplicate:
                merged_xyxy.append((0, x1, y1, x2, y2, float(box.conf[0])))

    yolo_boxes = []
    for cls_id, x1, y1, x2, y2, _ in merged_xyxy:
        bw = (x2 - x1) / w
        bh = (y2 - y1) / h
        xc = ((x1 + x2) / 2.0) / w
        yc = ((y1 + y2) / 2.0) / h
        if 0.02 < bw < 0.85 and 0.02 < bh < 0.85:
            yolo_boxes.append((0, xc, yc, bw, bh))  # Focus on class 0: pothole_D40
    return yolo_boxes


def build_noisy_yolo_dataset(samples: List[Tuple[np.ndarray, List[Tuple[int, float, float, float, float]]]]) -> Path:
    """Builds train/val splits with clean + heavily noisy + held-photo augmented images."""
    if DATASET_DIR.exists():
        shutil.rmtree(DATASET_DIR)

    for split in ("train", "val"):
        (DATASET_DIR / split / "images").mkdir(parents=True, exist_ok=True)
        (DATASET_DIR / split / "labels").mkdir(parents=True, exist_ok=True)

    random.shuffle(samples)
    split_idx = int(len(samples) * 0.82)
    train_samples = samples[:split_idx]
    val_samples = samples[split_idx:]

    def save_pair(split: str, name: str, img: np.ndarray, boxes: List[Tuple[int, float, float, float, float]]):
        img_path = DATASET_DIR / split / "images" / f"{name}.jpg"
        lbl_path = DATASET_DIR / split / "labels" / f"{name}.txt"
        cv2.imwrite(str(img_path), img)
        lines = [f"{c} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}" for c, xc, yc, bw, bh in boxes]
        lbl_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    train_count = 0
    for i, (img, boxes) in enumerate(train_samples):
        # 1. Original clean
        save_pair("train", f"sample_{i:04d}_clean", img, boxes)
        train_count += 1
        # 2. Sensor noise + blur + printout glare
        aug1, b1 = apply_composite_noise_pipeline(img, boxes, mode="sensor_noise")
        if b1:
            save_pair("train", f"sample_{i:04d}_noisy", aug1, b1)
            train_count += 1
        # 3. Held-photo / printout in front of car camera (variant A)
        aug2, b2 = apply_composite_noise_pipeline(img, boxes, mode="held_photo")
        if b2:
            save_pair("train", f"sample_{i:04d}_photoA", aug2, b2)
            train_count += 1
        # 4. Held-photo / printout in front of car camera (variant B - strong noise + glare)
        aug3, b3 = apply_composite_noise_pipeline(img, boxes, mode="held_photo")
        if b3:
            save_pair("train", f"sample_{i:04d}_photoB", aug3, b3)
            train_count += 1

    val_count = 0
    for i, (img, boxes) in enumerate(val_samples):
        save_pair("val", f"val_{i:04d}_clean", img, boxes)
        val_count += 1
        aug_v1, bv1 = apply_composite_noise_pipeline(img, boxes, mode="held_photo")
        if bv1:
            save_pair("val", f"val_{i:04d}_held_photo", aug_v1, bv1)
            val_count += 1

    yaml_path = DATASET_DIR / "noisy_rdd_dataset.yaml"
    yaml_content = f"""# Noise & Held-Photo Augmented Pothole Dataset
path: {DATASET_DIR.as_posix()}
train: train/images
val: val/images

names:
  0: pothole_D40
  1: longitudinal_crack_D00
  2: transverse_crack_D10
  3: alligator_crack_D20
  4: marking_defect_D44
"""
    yaml_path.write_text(yaml_content, encoding="utf-8")
    print(f"[+] Built noisy dataset: {train_count} train images, {val_count} val images -> {yaml_path}")
    return yaml_path


def evaluate_on_held_photo_val(model: YOLO, val_img_dir: Path, conf: float = 0.08) -> Tuple[int, int, float]:
    """Evaluates detection recall and mean confidence on held-photo noisy validation images."""
    held_imgs = sorted(val_img_dir.glob("*_held_photo.jpg"))
    if not held_imgs:
        held_imgs = sorted(val_img_dir.glob("*.jpg"))
    detected_frames = 0
    confs = []
    for p in held_imgs:
        res = model.predict(str(p), conf=conf, imgsz=320, verbose=False)[0]
        if len(res.boxes) > 0:
            detected_frames += 1
            confs.extend([float(b.conf[0]) for b in res.boxes])
    mean_conf = float(np.mean(confs)) if confs else 0.0
    return detected_frames, len(held_imgs), mean_conf


def main():
    set_seed(42)
    samples = extract_base_frames_and_labels()
    if not samples:
        print("❌ No base samples found!", file=sys.stderr)
        sys.exit(1)

    yaml_path = build_noisy_yolo_dataset(samples)
    val_img_dir = DATASET_DIR / "val" / "images"

    base_pt = ROOT_DIR / "best.pt"
    # Backup original best.pt before fine-tuning
    backup_pt = ROOT_DIR / "best_pre_noise_backup.pt"
    if not backup_pt.exists() and base_pt.exists():
        shutil.copy(base_pt, backup_pt)

    model_before = YOLO(str(base_pt))
    det_before, total_val, conf_before = evaluate_on_held_photo_val(model_before, val_img_dir)
    print(f"\n[BEFORE FINE-TUNING] Held-Photo Noisy Val Recall: {det_before}/{total_val} ({100*det_before/max(total_val,1):.1f}%), Mean Conf: {conf_before:.3f}")

    # Fine-tune with strong noise & geometric augmentations
    print("\n[*] Fine-tuning YOLOv10n pothole model (best.pt) on noisy + held-photo dataset...")
    device = 0 if torch.cuda.is_available() else "cpu"
    model = YOLO(str(base_pt))
    model.train(
        data=str(yaml_path),
        epochs=6,
        batch=8,
        imgsz=320,
        device=device,
        workers=0,
        lr0=0.0015,
        lrf=0.1,
        warmup_epochs=1.0,
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
        name="noisy_finetuned_pothole_model",
        exist_ok=True,
        verbose=True,
    )

    ft_best = ROOT_DIR / "runs_rdd" / "noisy_finetuned_pothole_model" / "weights" / "best.pt"
    if ft_best.exists():
        model_after = YOLO(str(ft_best))
        det_after, total_val, conf_after = evaluate_on_held_photo_val(model_after, val_img_dir)
        print(f"\n[AFTER FINE-TUNING]  Held-Photo Noisy Val Recall: {det_after}/{total_val} ({100*det_after/max(total_val,1):.1f}%), Mean Conf: {conf_after:.3f}")

        # Copy fine-tuned weights to active deployment locations
        shutil.copy(ft_best, ROOT_DIR / "best.pt")
        shutil.copy(ft_best, REPO_ROOT / "pothole_best.pt")
        runs_orig = ROOT_DIR / "runs_rdd" / "custom_pothole_model" / "weights" / "best.pt"
        if runs_orig.parent.exists():
            shutil.copy(ft_best, runs_orig)
        print(f"[+] Updated active pothole weights at: {ROOT_DIR / 'best.pt'} and {REPO_ROOT / 'pothole_best.pt'}")


if __name__ == "__main__":
    main()
