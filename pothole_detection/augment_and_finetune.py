#!/usr/bin/env python3
"""
augment_and_finetune.py
=======================
High-Precision GPU Fine-Tuning Pipeline for Held Printout Potholes (Lower Camera Section)
NMIMS MPSTME Capstone — Autonomous RC Car

Fixes the previous false-positive issue by:
1. Starting strictly from the clean 50-epoch RDD2022 checkpoint (best_pre_noise_backup.pt).
2. Extracting ONLY high-confidence D40 potholes located in the lower section of the road.
3. Simulating a printed road photo held in front of the camera with potholes positioned
   in the lower section of the camera frame (y_center in [0.45, 0.92]), realistic paper
   dynamic-range compression, slight tilt/perspective, and mild camera noise.
4. Including ~35% HARD-NEGATIVE images with empty labels (blank paper printouts with sharp
   rectangular borders, clean asphalt printouts without potholes, floor tiles, shadows,
   table edges, and non-pothole dashcam frames) so the model never fires on paper borders,
   floors, or room backgrounds.
5. Freezing the YOLOv10n backbone (freeze=10) and training with a conservative learning
   rate (lr0=0.0005) on the local NVIDIA GeForce RTX 3050 GPU (device=0).
"""

import os
import sys
import shutil
import random
from pathlib import Path

# Ensure ultralytics from user site-packages is importable when running inside Conda PyTorch_env
USER_SITE = r"C:\Users\menon\AppData\Roaming\Python\Python313\site-packages"
if os.path.exists(USER_SITE) and USER_SITE not in sys.path:
    sys.path.append(USER_SITE)

import cv2
import numpy as np
import torch
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
REPO_ROOT = ROOT.parent

CLEAN_BASE_WEIGHTS = ROOT / "best_pre_noise_backup.pt"
if not CLEAN_BASE_WEIGHTS.exists():
    CLEAN_BASE_WEIGHTS = ROOT / "best.pt"

VIDEO_PATH = ROOT / "pothole_test_2.mp4"
NEG_VIDEO_PATH = REPO_ROOT / "benchmarking" / "test_videos" / "dashcam_test.mp4"
SAMPLE_IMG = ROOT / "sample_road.jpg"

DATASET_DIR = ROOT / "datasets" / "printout_lower_pothole_yolo"
RUNS_DIR = ROOT / "runs_rdd"
RUN_NAME = "printout_gpu_pothole_model"


def create_indoor_background(h: int, w: int, rng: random.Random) -> np.ndarray:
    """Generates realistic indoor lab floor / table / wall background with tiles, wood, or shadows."""
    bg_type = rng.choice(["floor_tiles", "matte_desk", "wall_floor_split", "shadow_gradient"])
    base_val = rng.randint(95, 195)
    bg = np.full((h, w, 3), base_val, dtype=np.uint8)

    # Slight warm/cool tint
    b_shift = rng.randint(-12, 12)
    r_shift = rng.randint(-12, 12)
    bg[:, :, 0] = np.clip(bg[:, :, 0].astype(np.int16) + b_shift, 0, 255).astype(np.uint8)
    bg[:, :, 2] = np.clip(bg[:, :, 2].astype(np.int16) + r_shift, 0, 255).astype(np.uint8)

    if bg_type == "floor_tiles":
        # Draw grid lines resembling lab floor tiles (teaches model tile seams != potholes)
        step_x = rng.randint(45, 95)
        step_y = rng.randint(35, 75)
        line_col = max(30, base_val - rng.randint(20, 45))
        for x in range(0, w, step_x):
            cv2.line(bg, (x, 0), (x + rng.randint(-15, 15), h), (line_col, line_col, line_col), 1)
        for y in range(int(h * 0.35), h, step_y):
            cv2.line(bg, (0, y), (w, y), (line_col, line_col, line_col), 1)
    elif bg_type == "wall_floor_split":
        horizon_y = rng.randint(int(h * 0.25), int(h * 0.55))
        floor_val = max(40, base_val - rng.randint(25, 55))
        bg[horizon_y:, :] = floor_val
        cv2.line(bg, (0, horizon_y), (w, horizon_y), (35, 35, 35), rng.randint(1, 3))
    elif bg_type == "shadow_gradient":
        # Cast an elliptical or diagonal room shadow (teaches model shadows != potholes)
        yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
        cx = rng.uniform(0.2 * w, 0.8 * w)
        cy = rng.uniform(0.45 * h, 0.90 * h)
        rx = rng.uniform(0.18 * w, 0.45 * w)
        ry = rng.uniform(0.12 * h, 0.35 * h)
        dist = ((xx - cx) / rx) ** 2 + ((yy - cy) / ry) ** 2
        shadow_mask = np.exp(-dist * 1.8) * rng.uniform(35, 75)
        bg = np.clip(bg.astype(np.float32) - shadow_mask[..., None], 15, 250).astype(np.uint8)

    # Mild sensor grain
    noise = np.random.normal(0, rng.uniform(2.0, 6.0), (h, w, 3)).astype(np.float32)
    bg = np.clip(bg.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return bg


def apply_printout_appearance(img: np.ndarray, rng: random.Random) -> np.ndarray:
    """
    Simulates how an inkjet/laser printout of a road looks to the Pi camera:
    - Matte paper dynamic range compression (darks aren't pitch black, whites are paper-toned)
    - Slight warm/cool indoor lighting cast and soft glare
    - Light optical defocus + mild camera noise
    """
    out = img.astype(np.float32)
    h, w = out.shape[:2]

    # 1. Matte paper black-point lift & contrast compression
    alpha = rng.uniform(0.72, 0.95)
    black_lift = rng.uniform(14.0, 34.0)
    out = out * alpha + black_lift

    # 2. Slight indoor color temperature shift
    out[:, :, 0] *= rng.uniform(0.92, 1.08)
    out[:, :, 1] *= rng.uniform(0.95, 1.05)
    out[:, :, 2] *= rng.uniform(0.92, 1.08)

    # 3. Soft directional paper sheen / lighting gradient
    if rng.random() < 0.70:
        yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
        gx = rng.uniform(-0.4, 0.4) * (xx / max(w, 1) - 0.5)
        gy = rng.uniform(-0.4, 0.4) * (yy / max(h, 1) - 0.5)
        grad = 1.0 + (gx + gy)
        out = out * grad[..., None]

    # 4. Mild Gaussian blur (printout held close to fixed-focus lens)
    out_u8 = np.clip(out, 0, 255).astype(np.uint8)
    if rng.random() < 0.55:
        out_u8 = cv2.GaussianBlur(out_u8, (3, 3), rng.uniform(0.4, 1.0))

    # 5. Moderate realistic sensor noise (never destructive)
    sigma = rng.uniform(3.0, 10.0)
    noise = np.random.normal(0, sigma, out_u8.shape).astype(np.float32)
    out_u8 = np.clip(out_u8.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return out_u8


def embed_printout_in_lower_camera(
    road_img: np.ndarray,
    boxes: list,
    rng: random.Random,
) -> tuple[np.ndarray, list]:
    """
    Places a printed road photo in the lower section of the camera frame (or full lower view)
    with realistic white paper margins and indoor background above/around it, transforming
    bounding boxes and keeping ONLY potholes located in the lower section (yc >= 0.40).
    """
    h, w = road_img.shape[:2]
    printed = apply_printout_appearance(road_img, rng)

    mode = rng.choice(["lower_sheet_with_border", "close_printout_lower", "tilted_sheet_lower"])

    if mode == "close_printout_lower":
        # Printout held close to camera: shift road vertically if needed so potholes sit in lower section
        canvas = create_indoor_background(h, w, rng)
        scale_x = rng.uniform(0.82, 0.98)
        scale_y = rng.uniform(0.75, 0.92)
        nh, nw = int(h * scale_y), int(w * scale_x)
        resized = cv2.resize(printed, (nw, nh), interpolation=cv2.INTER_LINEAR)

        # Position printout toward the bottom of the camera frame
        ox = rng.randint(max(0, (w - nw) // 4), max(0, w - nw))
        oy = rng.randint(max(0, h - nh - int(0.06 * h)), max(0, h - nh))

        # Draw white paper margin around the printed road
        pad = rng.randint(5, 14)
        px1, py1 = max(0, ox - pad), max(0, oy - pad)
        px2, py2 = min(w, ox + nw + pad), min(h, oy + nh + pad)
        paper_val = rng.randint(215, 245)
        cv2.rectangle(canvas, (px1, py1), (px2, py2), (paper_val, paper_val, paper_val), -1)
        cv2.rectangle(canvas, (px1, py1), (px2, py2), (70, 70, 70), 1)
        canvas[oy:oy + nh, ox:ox + nw] = resized

        new_boxes = []
        for cls_id, xc, yc, bw, bh in boxes:
            nxc = (xc * nw + ox) / w
            nyc = (yc * nh + oy) / h
            nbw = (bw * nw) / w
            nbh = (bh * nh) / h
            if 0.40 <= nyc <= 0.96 and 0.04 <= nxc <= 0.96 and nbw >= 0.025 and nbh >= 0.02:
                new_boxes.append((cls_id, nxc, nyc, nbw, nbh))
        return canvas, new_boxes

    # Lower sheet with border or slight tilt
    canvas = create_indoor_background(h, w, rng)
    # Crop road from y=0.20..1.0 so the printout focuses on the road surface
    crop_y1 = int(h * rng.uniform(0.15, 0.30))
    road_crop = printed[crop_y1:, :]
    ch, cw = road_crop.shape[:2]

    scale_x = rng.uniform(0.68, 0.92)
    scale_y = rng.uniform(0.55, 0.78)
    nw = int(w * scale_x)
    nh = int(h * scale_y)
    resized = cv2.resize(road_crop, (nw, nh), interpolation=cv2.INTER_LINEAR)

    ox = rng.randint(max(4, (w - nw) // 2 - int(0.08 * w)), min(w - nw - 4, (w - nw) // 2 + int(0.08 * w)))
    oy = rng.randint(int(h * 0.20), max(int(h * 0.21), h - nh - 6))

    # White paper border around the road image
    pad = rng.randint(6, 16)
    px1, py1 = max(0, ox - pad), max(0, oy - pad)
    px2, py2 = min(w, ox + nw + pad), min(h, oy + nh + pad)
    paper_val = rng.randint(210, 245)
    cv2.rectangle(canvas, (px1, py1), (px2, py2), (paper_val, paper_val, paper_val), -1)
    cv2.rectangle(canvas, (px1, py1), (px2, py2), (55, 55, 55), rng.randint(1, 2))
    canvas[oy:oy + nh, ox:ox + nw] = resized

    new_boxes = []
    crop_frac = crop_y1 / h
    rem_frac = 1.0 - crop_frac
    for cls_id, xc, yc, bw, bh in boxes:
        if yc <= crop_frac:
            continue
        rel_yc = (yc - crop_frac) / rem_frac
        rel_bh = bh / rem_frac
        nxc = (xc * nw + ox) / w
        nyc = (rel_yc * nh + oy) / h
        nbw = (bw * nw) / w
        nbh = (rel_bh * nh) / h
        # Enforce lower-section camera constraint (nyc >= 0.40)
        if 0.40 <= nyc <= 0.96 and 0.04 <= nxc <= 0.96 and 0.025 <= nbw <= 0.55 and 0.02 <= nbh <= 0.45:
            new_boxes.append((cls_id, nxc, nyc, min(nbw, 0.95), min(nbh, 0.95)))

    return canvas, new_boxes


def generate_hard_negative_frame(h: int, w: int, clean_road_crops: list, rng: random.Random) -> np.ndarray:
    """
    Generates hard-negative frames that contain ZERO potholes:
    1. Held paper printout with clean road asphalt / lane markings (no potholes)
    2. Blank or text/line paper sheet with sharp borders on indoor floor
    3. Pure indoor floor tiles / shadows / walls
    """
    neg_kind = rng.choice(["clean_road_printout", "paper_sheet_border", "indoor_floor_shadows"])
    canvas = create_indoor_background(h, w, rng)

    if neg_kind == "clean_road_printout" and clean_road_crops:
        clean_patch = rng.choice(clean_road_crops).copy()
        clean_patch = apply_printout_appearance(clean_patch, rng)
        nw = int(w * rng.uniform(0.65, 0.92))
        nh = int(h * rng.uniform(0.50, 0.78))
        resized = cv2.resize(clean_patch, (nw, nh), interpolation=cv2.INTER_LINEAR)
        ox = rng.randint(max(4, (w - nw) // 2 - 20), max(5, min(w - nw - 4, (w - nw) // 2 + 20)))
        oy = rng.randint(int(h * 0.22), max(int(h * 0.23), h - nh - 6))
        pad = rng.randint(6, 15)
        paper_val = rng.randint(215, 245)
        cv2.rectangle(canvas, (max(0, ox - pad), max(0, oy - pad)),
                      (min(w, ox + nw + pad), min(h, oy + nh + pad)),
                      (paper_val, paper_val, paper_val), -1)
        cv2.rectangle(canvas, (max(0, ox - pad), max(0, oy - pad)),
                      (min(w, ox + nw + pad), min(h, oy + nh + pad)),
                      (50, 50, 50), rng.randint(1, 2))
        canvas[oy:oy + nh, ox:ox + nw] = resized

    elif neg_kind == "paper_sheet_border":
        # Blank or ruled/asphalt-colored rectangular sheet held in lower camera section
        nw = int(w * rng.uniform(0.55, 0.88))
        nh = int(h * rng.uniform(0.45, 0.75))
        ox = rng.randint(10, max(12, w - nw - 10))
        oy = rng.randint(int(h * 0.25), max(int(h * 0.26), h - nh - 8))
        paper_val = rng.randint(205, 245)
        cv2.rectangle(canvas, (ox, oy), (ox + nw, oy + nh), (paper_val, paper_val, paper_val), -1)
        cv2.rectangle(canvas, (ox, oy), (ox + nw, oy + nh), (40, 40, 40), rng.randint(1, 3))
        # Inner gray road rectangle (smooth asphalt with lane lines, no potholes)
        pad = rng.randint(8, 18)
        if nw > 2 * pad + 20 and nh > 2 * pad + 20:
            asphalt_val = rng.randint(75, 135)
            cv2.rectangle(canvas, (ox + pad, oy + pad), (ox + nw - pad, oy + nh - pad),
                          (asphalt_val, asphalt_val, asphalt_val), -1)
            # Optional straight white/yellow road stripe
            if rng.random() < 0.6:
                lx1 = ox + pad + rng.randint(10, nw - 2 * pad - 10)
                lx2 = ox + pad + rng.randint(10, nw - 2 * pad - 10)
                cv2.line(canvas, (lx1, oy + nh - pad), (lx2, oy + pad), (215, 215, 215), rng.randint(2, 5))

    return canvas


def write_sample(split_dir: Path, stem: str, img: np.ndarray, boxes: list):
    img_path = split_dir / "images" / f"{stem}.jpg"
    lbl_path = split_dir / "labels" / f"{stem}.txt"
    cv2.imwrite(str(img_path), img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    with open(lbl_path, "w", encoding="utf-8") as f:
        for cls_id, xc, yc, bw, bh in boxes:
            f.write(f"{cls_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}\n")


def build_dataset() -> Path:
    print(f"🔍 Building High-Precision Lower-Section Printout Dataset from: {CLEAN_BASE_WEIGHTS}")
    if DATASET_DIR.exists():
        shutil.rmtree(DATASET_DIR)

    for split in ["train", "val"]:
        (DATASET_DIR / split / "images").mkdir(parents=True, exist_ok=True)
        (DATASET_DIR / split / "labels").mkdir(parents=True, exist_ok=True)

    device = 0 if torch.cuda.is_available() else "cpu"
    base_model = YOLO(str(CLEAN_BASE_WEIGHTS))

    positive_samples = []
    clean_road_crops = []

    # 1. Extract high-confidence lower-section potholes from pothole_test_2.mp4
    if VIDEO_PATH.exists():
        cap = cv2.VideoCapture(str(VIDEO_PATH))
        frame_idx = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            frame_idx += 1
            if frame_idx % 2 != 0:
                continue

            fh, fw = frame.shape[:2]
            res = base_model.predict(frame, conf=0.25, imgsz=640, device=device, verbose=False)[0]
            boxes = []
            for b in res.boxes:
                cls_id = int(b.cls[0])
                if cls_id != 0:
                    continue
                xc, yc, bw, bh = [float(v) for v in b.xywhn[0]]
                # STRICT filter: only genuine potholes in the lower section (yc >= 0.42)
                if 0.42 <= yc <= 0.94 and 0.03 <= bw <= 0.48 and 0.025 <= bh <= 0.38:
                    boxes.append((0, xc, yc, bw, bh))

            if boxes:
                positive_samples.append((f"vid_{frame_idx:04d}", frame.copy(), boxes))
            elif len(res.boxes) == 0 and len(clean_road_crops) < 40:
                # Completely clean road surface with zero detections
                clean_road_crops.append(frame[int(fh * 0.35):, :].copy())
        cap.release()

    # 2. Extract clean road/background frames from dashcam_test.mp4 for hard negatives
    if NEG_VIDEO_PATH.exists():
        cap = cv2.VideoCapture(str(NEG_VIDEO_PATH))
        f_idx = 0
        while len(clean_road_crops) < 70:
            ret, frame = cap.read()
            if not ret:
                break
            f_idx += 1
            if f_idx % 12 != 0:
                continue
            res = base_model.predict(frame, conf=0.18, imgsz=640, device=device, verbose=False)[0]
            if len(res.boxes) == 0:
                fh, fw = frame.shape[:2]
                clean_road_crops.append(frame[int(fh * 0.35):, :].copy())
        cap.release()

    # 3. Also include sample_road.jpg if it has high-confidence lower-section potholes
    if SAMPLE_IMG.exists():
        s_img = cv2.imread(str(SAMPLE_IMG))
        if s_img is not None:
            res = base_model.predict(s_img, conf=0.25, imgsz=640, device=device, verbose=False)[0]
            boxes = []
            for b in res.boxes:
                if int(b.cls[0]) == 0:
                    xc, yc, bw, bh = [float(v) for v in b.xywhn[0]]
                    if 0.40 <= yc <= 0.95 and 0.03 <= bw <= 0.50 and 0.025 <= bh <= 0.40:
                        boxes.append((0, xc, yc, bw, bh))
            if boxes:
                for k in range(8):
                    positive_samples.append((f"sample_road_{k}", s_img.copy(), boxes))

    print(f"✅ Extracted {len(positive_samples)} high-confidence lower-section pothole base frames "
          f"and {len(clean_road_crops)} verified pothole-free road crops.")

    rng = random.Random(42)
    rng.shuffle(positive_samples)

    split_idx = int(len(positive_samples) * 0.82)
    train_pos = positive_samples[:split_idx]
    val_pos = positive_samples[split_idx:]

    train_count = 0
    val_count = 0
    neg_train_count = 0
    neg_val_count = 0

    # Generate Training Set (Positive + Printout-in-Lower-Camera + Hard Negatives)
    for stem, frame, boxes in train_pos:
        h, w = frame.shape[:2]
        # 1. Clean printout-style full frame
        p_img = apply_printout_appearance(frame, rng)
        write_sample(DATASET_DIR / "train", f"{stem}_print", p_img, boxes)
        train_count += 1

        # 2. Horizontal flip printout
        flip_img = cv2.flip(apply_printout_appearance(frame, rng), 1)
        flip_boxes = [(c, 1.0 - xc, yc, bw, bh) for (c, xc, yc, bw, bh) in boxes]
        write_sample(DATASET_DIR / "train", f"{stem}_flip", flip_img, flip_boxes)
        train_count += 1

        # 3. Two variations of held printout positioned in the lower camera section
        for v in range(2):
            emb_img, emb_boxes = embed_printout_in_lower_camera(frame, boxes, rng)
            if emb_boxes:
                write_sample(DATASET_DIR / "train", f"{stem}_lower_{v}", emb_img, emb_boxes)
                train_count += 1

        # 4. Hard-negative samples (paper borders, floors, clean road printouts -> EMPTY labels)
        for n_i in range(2):
            neg_img = generate_hard_negative_frame(h, w, clean_road_crops, rng)
            write_sample(DATASET_DIR / "train", f"{stem}_neg_{n_i}", neg_img, [])
            neg_train_count += 1

    # Generate Validation Set (Positive Held Printouts + Hard Negatives)
    for stem, frame, boxes in val_pos:
        h, w = frame.shape[:2]
        p_img = apply_printout_appearance(frame, rng)
        write_sample(DATASET_DIR / "val", f"{stem}_print", p_img, boxes)
        val_count += 1

        emb_img, emb_boxes = embed_printout_in_lower_camera(frame, boxes, rng)
        if emb_boxes:
            write_sample(DATASET_DIR / "val", f"{stem}_lower", emb_img, emb_boxes)
            val_count += 1

        neg_img = generate_hard_negative_frame(h, w, clean_road_crops, rng)
        write_sample(DATASET_DIR / "val", f"{stem}_neg", neg_img, [])
        neg_val_count += 1

    print(f"📊 Dataset ready at {DATASET_DIR}:")
    print(f"   Train: {train_count} positive + {neg_train_count} hard-negative = {train_count + neg_train_count} images")
    print(f"   Val  : {val_count} positive + {neg_val_count} hard-negative = {val_count + neg_val_count} images")

    # Preserve exact 5-class RDD2022 schema from best_pre_noise_backup.pt
    yaml_path = DATASET_DIR / "data.yaml"
    yaml_content = (
        f"path: {DATASET_DIR.as_posix()}\n"
        f"train: train/images\n"
        f"val: val/images\n"
        f"nc: 5\n"
        f"names:\n"
        f"  0: pothole_D40\n"
        f"  1: longitudinal_crack_D00\n"
        f"  2: transverse_crack_D10\n"
        f"  3: alligator_crack_D20\n"
        f"  4: marking_defect_D44\n"
    )
    yaml_path.write_text(yaml_content, encoding="utf-8")
    return yaml_path


def train_on_gpu(yaml_path: Path):
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA GPU is required for training! Run with E:\\Conda\\envs\\PyTorch_env\\python.exe")

    gpu_name = torch.cuda.get_device_name(0)
    print(f"🚀 Starting GPU Fine-Tuning on: {gpu_name} (CUDA device=0)")
    print(f"   Base checkpoint: {CLEAN_BASE_WEIGHTS}")

    model = YOLO(str(CLEAN_BASE_WEIGHTS))
    model.train(
        data=str(yaml_path),
        epochs=18,
        imgsz=416,
        batch=16,
        device=0,
        workers=0,
        project=str(RUNS_DIR),
        name=RUN_NAME,
        exist_ok=True,
        pretrained=True,
        optimizer="AdamW",
        lr0=0.0004,
        lrf=0.05,
        weight_decay=0.001,
        warmup_epochs=2.0,
        freeze=10,        # Freeze YOLOv10n backbone so pretrained features aren't corrupted
        mosaic=0.25,      # Low mosaic to preserve lower-section spatial prior
        mixup=0.0,
        degrees=5.0,
        translate=0.06,
        scale=0.20,
        fliplr=0.5,
        hsv_h=0.01,
        hsv_s=0.35,
        hsv_v=0.30,
        erasing=0.05,
        verbose=True,
    )

    best_out = RUNS_DIR / RUN_NAME / "weights" / "best.pt"
    if not best_out.exists():
        best_out = RUNS_DIR / RUN_NAME / "weights" / "last.pt"

    if best_out.exists():
        targets = [
            ROOT / "best.pt",
            REPO_ROOT / "pothole_best.pt",
            RUNS_DIR / "custom_pothole_model" / "weights" / "best.pt",
        ]
        for t in targets:
            t.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(best_out, t)
            print(f"✅ Copied GPU fine-tuned weights -> {t}")

        # Verify false-positive suppression and printout recall immediately
        verify_model(best_out)
    else:
        raise FileNotFoundError(f"Training finished but weights not found at {best_out}")


def verify_model(weights_path: Path):
    print("\n🧪 Running Post-Training Verification (Printout Recall vs. Hard-Negative False Positives)...")
    m = YOLO(str(weights_path))
    rng = random.Random(999)

    # 1. Test 20 hard-negative synthetic frames (floors, shadows, paper borders)
    fp_count = 0
    for i in range(20):
        neg = generate_hard_negative_frame(240, 320, [], rng)
        res = m.predict(neg, imgsz=320, conf=0.25, device=0, verbose=False)[0]
        potholes = [b for b in res.boxes if int(b.cls[0]) == 0]
        if potholes:
            fp_count += len(potholes)

    # 2. Test held-printout lower-section positive frames
    val_imgs = sorted((DATASET_DIR / "val" / "images").glob("*_lower*.jpg"))
    tp_frames = 0
    for vp in val_imgs:
        im = cv2.imread(str(vp))
        res = m.predict(im, imgsz=320, conf=0.25, device=0, verbose=False)[0]
        potholes = [b for b in res.boxes if int(b.cls[0]) == 0]
        if potholes:
            tp_frames += 1

    print(f"   Hard-Negative False Positives (20 room/border frames @ conf=0.25): {fp_count} (Target: 0)")
    print(f"   Held-Printout Lower-Section Recall ({len(val_imgs)} val frames @ conf=0.25): {tp_frames}/{len(val_imgs)}")


if __name__ == "__main__":
    yaml_file = build_dataset()
    train_on_gpu(yaml_file)
