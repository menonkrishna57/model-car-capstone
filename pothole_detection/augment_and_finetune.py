#!/usr/bin/env python3
"""
augment_and_finetune.py
=======================
GPU Fine-Tuning Pipeline for Held Printout Potholes (Water-Filled & Dry Potholes in Lower Camera Section)
NMIMS MPSTME Capstone — Autonomous RC Car (YOLOv10n on NVIDIA GeForce RTX 3050 GPU)

Key Improvements:
1. Starts from clean RDD2022 YOLOv10n checkpoint (best_pre_noise_backup.pt).
2. Incorporates both water-filled/monsoon puddle potholes (matching the held road printout)
   and dry/dark asphalt craters in the lower section of the camera frame (yc >= 0.40).
3. Simulates 320x240 Pi camera capture across diverse distances (full-frame close-up printout
   and mid-distance sheet with paper borders), angles, lighting, and AWB modes.
4. Uses realistic hard negatives (clean dashcam roads, floor tile grids, walls, blank paper
   margins) WITHOUT suppressing bright water-puddle reflections.
5. Unfreezes YOLOv10n mid/high-level backbone layers (freeze=3) and trains for 25 epochs on
   CUDA device=0 (RTX 3050 GPU) at imgsz=416.
"""

import os
import sys
import shutil
import random
from pathlib import Path

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
NEG_VIDEO_PATH_2 = REPO_ROOT / "benchmarking" / "test_videos" / "testvideovid1.mp4"
SAMPLE_IMG = ROOT / "sample_road.jpg"
PRINTOUT_CLEAN_PATH = ROOT / "printout_road_clean.jpg"
UPLOADED_SCREENSHOT = Path(
    r"C:\Users\menon\.gemini\antigravity\brain\56af8221-1a61-4984-8913-4ad94ebcd5f2\.user_uploaded\media_1793160568176_9e7bcb4c.png"
)

DATASET_DIR = Path(r"C:\Users\menon\AppData\Local\Temp") / "printout_lower_pothole_yolo_v3"
RUNS_DIR = ROOT / "runs_rdd"
RUN_NAME = "printout_gpu_pothole_model"

# Verified ground-truth normalized xyxy boxes (x1, y1, x2, y2) for the held road printout
PRINTOUT_GT_XYXY = [
    (0.09, 0.70, 0.31, 0.99),  # P1: Bottom-left large water pothole
    (0.17, 0.55, 0.37, 0.67),  # P2: Mid-left water pothole
    (0.36, 0.64, 0.47, 0.71),  # P3: Mid-center-left water pothole
    (0.43, 0.51, 0.57, 0.64),  # P4: Center water potholes
    (0.48, 0.76, 0.64, 0.94),  # P5: Bottom-center water pothole
    (0.63, 0.59, 0.75, 0.68),  # P6: Mid-right dark/muddy pothole
    (0.73, 0.74, 0.84, 0.84),  # P7: Lower-right water pothole
]


def prepare_clean_printout_reference() -> np.ndarray | None:
    """Extracts and cleans the 4:3 camera stream from the held printout capture (removing all UI overlays)."""
    if UPLOADED_SCREENSHOT.exists():
        im = cv2.imread(str(UPLOADED_SCREENSHOT))
        if im is not None and im.shape[0] >= 606 and im.shape[1] >= 955:
            crop = im[14:606, 49:955].copy()
            # 1. Remove thin horizontal ROI line around y=268..274
            for y in range(266, 275):
                crop[y, :] = (crop[263, :].astype(np.int16) + crop[277, :].astype(np.int16)) // 2
            # 2. Replace top HTML pill bar region (y=0..86, x=130..800) with natural background texture
            patch_src = cv2.GaussianBlur(crop[88:174, 130:800], (15, 15), 0)
            crop[0:86, 130:800] = cv2.flip(patch_src, 0)
            # 3. Inpaint bright green/red/yellow HUD boxes & text in top 250px
            top = crop[:250, :]
            hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
            mask_g = cv2.inRange(hsv, (35, 105, 105), (88, 255, 255))
            mask_r1 = cv2.inRange(hsv, (0, 115, 105), (12, 255, 255))
            mask_r2 = cv2.inRange(hsv, (168, 115, 105), (180, 255, 255))
            mask_y = cv2.inRange(hsv, (18, 115, 135), (35, 255, 255))
            mask = cv2.dilate(mask_g | mask_r1 | mask_r2 | mask_y, np.ones((3, 3), np.uint8), iterations=2)
            crop[:250, :] = cv2.inpaint(top, mask, 4, cv2.INPAINT_TELEA)
            cv2.imwrite(str(PRINTOUT_CLEAN_PATH), crop)
            return crop

    if PRINTOUT_CLEAN_PATH.exists():
        return cv2.imread(str(PRINTOUT_CLEAN_PATH))
    return None


def xyxy_to_yolo(boxes_xyxy: list) -> list:
    out = []
    for x1, y1, x2, y2 in boxes_xyxy:
        xc = (x1 + x2) / 2.0
        yc = (y1 + y2) / 2.0
        bw = x2 - x1
        bh = y2 - y1
        if 0.36 <= yc <= 0.98 and 0.02 <= bw <= 0.60 and 0.02 <= bh <= 0.45:
            out.append((0, xc, yc, bw, bh))
    return out


def create_indoor_background(h: int, w: int, rng: random.Random) -> np.ndarray:
    """Generates indoor lab floor / wall / desk background without puddle-like ovals."""
    bg_type = rng.choice(["floor_tiles", "matte_desk", "wall_floor_split"])
    base_val = rng.randint(105, 190)
    bg = np.full((h, w, 3), base_val, dtype=np.uint8)

    b_shift = rng.randint(-10, 10)
    r_shift = rng.randint(-10, 10)
    bg[:, :, 0] = np.clip(bg[:, :, 0].astype(np.int16) + b_shift, 0, 255).astype(np.uint8)
    bg[:, :, 2] = np.clip(bg[:, :, 2].astype(np.int16) + r_shift, 0, 255).astype(np.uint8)

    if bg_type == "floor_tiles":
        step_x = rng.randint(55, 95)
        step_y = rng.randint(45, 80)
        line_col = max(40, base_val - rng.randint(18, 35))
        for x in range(0, w, step_x):
            cv2.line(bg, (x, 0), (x, h), (line_col, line_col, line_col), 1)
        for y in range(int(h * 0.35), h, step_y):
            cv2.line(bg, (0, y), (w, y), (line_col, line_col, line_col), 1)
    elif bg_type == "wall_floor_split":
        horizon_y = rng.randint(int(h * 0.25), int(h * 0.50))
        floor_val = max(55, base_val - rng.randint(20, 45))
        bg[horizon_y:, :] = floor_val
        cv2.line(bg, (0, horizon_y), (w, horizon_y), (45, 45, 45), rng.randint(1, 2))

    noise = np.random.normal(0, rng.uniform(2.0, 5.0), (h, w, 3)).astype(np.float32)
    return np.clip(bg.astype(np.float32) + noise, 0, 255).astype(np.uint8)


def apply_camera_printout_variation(img: np.ndarray, rng: random.Random) -> np.ndarray:
    """Simulates Pi camera viewing conditions (AWB shifts, brightness/contrast, paper sheen, 320x240 capture)."""
    out = img.astype(np.float32)
    h, w = out.shape[:2]

    # Contrast & exposure variation
    alpha = rng.uniform(0.80, 1.18)
    beta = rng.uniform(-16.0, 20.0)
    out = out * alpha + beta

    # AWB color balance variation (Daylight greenish/warm vs Auto vs Indoor)
    out[:, :, 0] *= rng.uniform(0.88, 1.12)
    out[:, :, 1] *= rng.uniform(0.92, 1.10)
    out[:, :, 2] *= rng.uniform(0.88, 1.12)

    # Soft directional lighting gradient across paper
    if rng.random() < 0.65:
        yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
        gx = rng.uniform(-0.28, 0.28) * (xx / max(w, 1) - 0.5)
        gy = rng.uniform(-0.28, 0.28) * (yy / max(h, 1) - 0.5)
        out = out * (1.0 + gx + gy)[..., None]

    out_u8 = np.clip(out, 0, 255).astype(np.uint8)

    # Simulate 320x240 camera capture resolution + mild optical blur
    if rng.random() < 0.70:
        small = cv2.resize(out_u8, (320, 240), interpolation=cv2.INTER_AREA)
        if rng.random() < 0.45:
            small = cv2.GaussianBlur(small, (3, 3), rng.uniform(0.4, 0.9))
        out_u8 = cv2.resize(small, (w, h), interpolation=cv2.INTER_LINEAR)

    sigma = rng.uniform(2.0, 8.0)
    noise = np.random.normal(0, sigma, out_u8.shape).astype(np.float32)
    return np.clip(out_u8.astype(np.float32) + noise, 0, 255).astype(np.uint8)


def augment_printout_view(
    base_img: np.ndarray,
    boxes_yolo: list,
    rng: random.Random,
    out_w: int = 416,
    out_h: int = 312,
) -> tuple[np.ndarray, list]:
    """
    Creates diverse camera perspectives of the held printout:
    - Full-frame close-ups (with random crop/pan/zoom/tilt like when held right in front of the Pi camera)
    - Medium-distance views where the paper margin is visible at the bottom/sides
    """
    img = apply_camera_printout_variation(base_img, rng)
    if rng.random() < 0.5:
        img = cv2.flip(img, 1)
        boxes_yolo = [(c, 1.0 - xc, yc, bw, bh) for (c, xc, yc, bw, bh) in boxes_yolo]

    view_type = rng.choice(["closeup_crop", "closeup_crop", "sheet_with_margin"])

    if view_type == "closeup_crop":
        # Random crop / pan / zoom around the frame while keeping lower potholes in view
        rx1 = rng.uniform(0.0, 0.12)
        rx2 = rng.uniform(0.88, 1.0)
        ry1 = rng.uniform(0.0, 0.18)
        ry2 = rng.uniform(0.90, 1.0)

        ih, iw = img.shape[:2]
        cx1, cy1 = int(rx1 * iw), int(ry1 * ih)
        cx2, cy2 = int(rx2 * iw), int(ry2 * ih)
        cropped = img[cy1:cy2, cx1:cx2]
        canvas = cv2.resize(cropped, (out_w, out_h), interpolation=cv2.INTER_LINEAR)

        new_boxes = []
        sw = rx2 - rx1
        sh = ry2 - ry1
        for c, xc, yc, bw, bh in boxes_yolo:
            x1, y1 = xc - bw / 2.0, yc - bh / 2.0
            x2, y2 = xc + bw / 2.0, yc + bh / 2.0
            nx1 = np.clip((x1 - rx1) / sw, 0.0, 1.0)
            ny1 = np.clip((y1 - ry1) / sh, 0.0, 1.0)
            nx2 = np.clip((x2 - rx1) / sw, 0.0, 1.0)
            ny2 = np.clip((y2 - ry1) / sh, 0.0, 1.0)
            nbw = nx2 - nx1
            nbh = ny2 - ny1
            nxc = (nx1 + nx2) / 2.0
            nyc = (ny1 + ny2) / 2.0
            if nbw >= 0.04 and nbh >= 0.035 and 0.36 <= nyc <= 0.96:
                new_boxes.append((c, nxc, nyc, nbw, nbh))
        return canvas, new_boxes

    # Sheet with visible paper border on indoor background
    canvas = create_indoor_background(out_h, out_w, rng)
    scale_x = rng.uniform(0.76, 0.96)
    scale_y = rng.uniform(0.74, 0.94)
    nw, nh = int(out_w * scale_x), int(out_h * scale_y)
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)

    ox = rng.randint(max(2, (out_w - nw) // 2 - 15), max(3, min(out_w - nw - 2, (out_w - nw) // 2 + 15)))
    oy = rng.randint(max(2, out_h - nh - 16), max(3, out_h - nh - 2))

    pad = rng.randint(4, 10)
    paper_val = rng.randint(210, 242)
    cv2.rectangle(canvas, (max(0, ox - pad), max(0, oy - pad)),
                  (min(out_w, ox + nw + pad), min(out_h, oy + nh + pad)),
                  (paper_val, paper_val, paper_val), -1)
    canvas[oy:oy + nh, ox:ox + nw] = resized

    new_boxes = []
    for c, xc, yc, bw, bh in boxes_yolo:
        nxc = (xc * nw + ox) / out_w
        nyc = (yc * nh + oy) / out_h
        nbw = (bw * nw) / out_w
        nbh = (bh * nh) / out_h
        if 0.36 <= nyc <= 0.96 and nbw >= 0.035 and nbh >= 0.03:
            new_boxes.append((c, nxc, nyc, nbw, nbh))
    return canvas, new_boxes


def add_water_puddle_simulation(frame: np.ndarray, boxes: list, rng: random.Random) -> np.ndarray:
    """
    On a subset of dry road pothole frames, simulates water-filled puddle reflection inside the
    ground-truth pothole bounding box (light sky reflection center + dark wet mud rim).
    """
    out = frame.copy()
    h, w = out.shape[:2]
    for _, xc, yc, bw, bh in boxes:
        if rng.random() < 0.55:
            cx, cy = int(xc * w), int(yc * h)
            rx, ry = max(6, int(bw * w * 0.42)), max(4, int(bh * h * 0.38))
            angle = rng.randint(-15, 15)
            # Dark wet rim
            rim_col = (rng.randint(35, 65), rng.randint(45, 75), rng.randint(35, 65))
            cv2.ellipse(out, (cx, cy), (int(rx * 1.15), int(ry * 1.15)), angle, 0, 360, rim_col, -1)
            # Bright water reflection inside
            water_col = (rng.randint(175, 220), rng.randint(190, 230), rng.randint(170, 215))
            cv2.ellipse(out, (cx, cy), (int(rx * 0.82), int(ry * 0.80)), angle, 0, 360, water_col, -1)
            # Blur slightly to blend naturally into road texture
            x1, y1 = max(0, cx - int(rx * 1.4)), max(0, cy - int(ry * 1.4))
            x2, y2 = min(w, cx + int(rx * 1.4)), min(h, cy + int(ry * 1.4))
            if x2 > x1 + 6 and y2 > y1 + 6:
                out[y1:y2, x1:x2] = cv2.GaussianBlur(out[y1:y2, x1:x2], (5, 5), 0)
    return out


def generate_hard_negative_frame(h: int, w: int, clean_road_frames: list, rng: random.Random) -> np.ndarray:
    """
    Generates realistic non-pothole background images (empty labels):
    1. Real clean road dashcam frames (printed appearance) with ZERO potholes
    2. Indoor floor tile grids / walls / wooden desk surfaces
    """
    if clean_road_frames and rng.random() < 0.65:
        road = rng.choice(clean_road_frames).copy()
        road = cv2.resize(road, (w, h), interpolation=cv2.INTER_LINEAR)
        return apply_camera_printout_variation(road, rng)
    return create_indoor_background(h, w, rng)


def write_sample(split_dir: Path, stem: str, img: np.ndarray, boxes: list):
    img_path = split_dir / "images" / f"{stem}.jpg"
    lbl_path = split_dir / "labels" / f"{stem}.txt"
    cv2.imwrite(str(img_path), img, [int(cv2.IMWRITE_JPEG_QUALITY), 93])
    with open(lbl_path, "w", encoding="utf-8") as f:
        for cls_id, xc, yc, bw, bh in boxes:
            f.write(f"{cls_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}\n")


def build_dataset() -> Path:
    print(f"🔍 Building Printout + Water/Dry Pothole GPU Dataset from: {CLEAN_BASE_WEIGHTS}")
    if DATASET_DIR.exists():
        shutil.rmtree(DATASET_DIR)

    for split in ["train", "val"]:
        (DATASET_DIR / split / "images").mkdir(parents=True, exist_ok=True)
        (DATASET_DIR / split / "labels").mkdir(parents=True, exist_ok=True)

    device = 0 if torch.cuda.is_available() else "cpu"
    base_model = YOLO(str(CLEAN_BASE_WEIGHTS))

    printout_ref = prepare_clean_printout_reference()
    printout_boxes_yolo = xyxy_to_yolo(PRINTOUT_GT_XYXY)

    video_positives = []
    clean_road_frames = []

    if VIDEO_PATH.exists():
        cap = cv2.VideoCapture(str(VIDEO_PATH))
        f_idx = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            f_idx += 1
            if f_idx % 3 != 0:
                continue
            res = base_model.predict(frame, conf=0.25, imgsz=640, device=device, verbose=False)[0]
            boxes = []
            for b in res.boxes:
                if int(b.cls[0]) == 0:
                    xc, yc, bw, bh = [float(v) for v in b.xywhn[0]]
                    if 0.40 <= yc <= 0.94 and 0.03 <= bw <= 0.48 and 0.025 <= bh <= 0.38:
                        boxes.append((0, xc, yc, bw, bh))
            if boxes:
                video_positives.append((f"vid_{f_idx:04d}", frame.copy(), boxes))
            elif len(res.boxes) == 0 and len(clean_road_frames) < 35:
                clean_road_frames.append(frame.copy())
        cap.release()

    if NEG_VIDEO_PATH.exists():
        cap = cv2.VideoCapture(str(NEG_VIDEO_PATH))
        f_idx = 0
        while len(clean_road_frames) < 85:
            ret, frame = cap.read()
            if not ret:
                break
            f_idx += 1
            if f_idx % 20 != 0:
                continue
            res = base_model.predict(frame, conf=0.18, imgsz=640, device=device, verbose=False)[0]
            if len(res.boxes) == 0:
                clean_road_frames.append(frame.copy())
        cap.release()

    if NEG_VIDEO_PATH_2.exists():
        cap = cv2.VideoCapture(str(NEG_VIDEO_PATH_2))
        f_idx = 0
        while len(clean_road_frames) < 120:
            ret, frame = cap.read()
            if not ret:
                break
            f_idx += 1
            if f_idx % 60 != 0:
                continue
            res = base_model.predict(frame, conf=0.18, imgsz=640, device=device, verbose=False)[0]
            if len(res.boxes) == 0:
                clean_road_frames.append(frame.copy())
        cap.release()

    rng = random.Random(42)
    rng.shuffle(video_positives)

    train_count = 0
    val_count = 0
    neg_train_count = 0
    neg_val_count = 0

    # 1. Held Printout Road Variations (Water-Filled & Muddy Potholes in Lower Camera Section)
    if printout_ref is not None and printout_boxes_yolo:
        print(f"✅ Loaded held road printout reference ({printout_ref.shape[1]}x{printout_ref.shape[0]}) with {len(printout_boxes_yolo)} lower-section potholes.")
        for i in range(72):
            aug_img, aug_boxes = augment_printout_view(printout_ref, printout_boxes_yolo, rng, out_w=416, out_h=312)
            if aug_boxes:
                if i < 60:
                    write_sample(DATASET_DIR / "train", f"printout_ref_{i:03d}", aug_img, aug_boxes)
                    train_count += 1
                else:
                    write_sample(DATASET_DIR / "val", f"printout_ref_{i:03d}", aug_img, aug_boxes)
                    val_count += 1

    # 2. General Road Video Potholes (Real textures only — no synthetic ellipses!)
    split_idx = int(len(video_positives) * 0.82)
    for stem, frame, boxes in video_positives[:split_idx]:
        resized = cv2.resize(frame, (416, 312), interpolation=cv2.INTER_LINEAR)
        p_img = apply_camera_printout_variation(resized, rng)
        write_sample(DATASET_DIR / "train", f"{stem}_dry", p_img, boxes)
        train_count += 1

        flip_img = cv2.flip(apply_camera_printout_variation(resized, rng), 1)
        flip_boxes = [(c, 1.0 - xc, yc, bw, bh) for (c, xc, yc, bw, bh) in boxes]
        write_sample(DATASET_DIR / "train", f"{stem}_flip", flip_img, flip_boxes)
        train_count += 1

    for stem, frame, boxes in video_positives[split_idx:]:
        resized = cv2.resize(frame, (416, 312), interpolation=cv2.INTER_LINEAR)
        p_img = apply_camera_printout_variation(resized, rng)
        write_sample(DATASET_DIR / "val", f"{stem}_dry", p_img, boxes)
        val_count += 1

    # 3. Hard-Negative Samples (~35% of dataset -> strong false-positive suppression on roads & rooms)
    for i in range(75):
        neg_img = generate_hard_negative_frame(312, 416, clean_road_frames, rng)
        write_sample(DATASET_DIR / "train", f"hard_neg_{i:03d}", neg_img, [])
        neg_train_count += 1

    for i in range(14):
        neg_img = generate_hard_negative_frame(312, 416, clean_road_frames, rng)
        write_sample(DATASET_DIR / "val", f"hard_neg_{i:03d}", neg_img, [])
        neg_val_count += 1

    print(f"📊 Dataset ready at {DATASET_DIR}:")
    print(f"   Train: {train_count} positive + {neg_train_count} hard-negative = {train_count + neg_train_count} images")
    print(f"   Val  : {val_count} positive + {neg_val_count} hard-negative = {val_count + neg_val_count} images")

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
    print(f"🚀 Starting YOLOv10n GPU Fine-Tuning on: {gpu_name} (CUDA device=0)")
    print(f"   Base checkpoint: {CLEAN_BASE_WEIGHTS}")

    model = YOLO(str(CLEAN_BASE_WEIGHTS))
    model.train(
        data=str(yaml_path),
        epochs=25,
        imgsz=416,
        batch=16,
        device=0,
        workers=0,
        project=str(RUNS_DIR),
        name=RUN_NAME,
        exist_ok=True,
        pretrained=True,
        optimizer="AdamW",
        lr0=0.0008,
        lrf=0.05,
        weight_decay=0.0008,
        warmup_epochs=2.0,
        freeze=3,         # Freeze only initial stem so C2f blocks adapt to water-filled printout potholes
        mosaic=0.30,
        mixup=0.0,
        degrees=6.0,
        translate=0.08,
        scale=0.25,
        fliplr=0.5,
        hsv_h=0.015,
        hsv_s=0.40,
        hsv_v=0.35,
        erasing=0.0,
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

        verify_model(best_out)
    else:
        raise FileNotFoundError(f"Training finished but weights not found at {best_out}")


def verify_model(weights_path: Path):
    print("\n🧪 Running Post-Training Verification on User's Printout & Hard Negatives...")
    m = YOLO(str(weights_path))
    rng = random.Random(999)

    # 1. Test on the exact 320x240 Pi camera frame from the user's screenshot
    if PRINTOUT_CLEAN_PATH.exists():
        clean_im = cv2.imread(str(PRINTOUT_CLEAN_PATH))
        sim_320 = cv2.resize(clean_im, (320, 240), interpolation=cv2.INTER_AREA)
        res = m.predict(sim_320, imgsz=416, conf=0.20, device=0, verbose=False)[0]
        lower_potholes = []
        upper_fps = []
        for b in res.boxes:
            if int(b.cls[0]) == 0:
                x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
                yc = (y1 + y2) / 2.0
                if yc >= 240 * 0.40:
                    lower_potholes.append((round(float(b.conf[0]), 3), [x1, y1, x2, y2]))
                else:
                    upper_fps.append((round(float(b.conf[0]), 3), [x1, y1, x2, y2]))
        print(f"   [User Printout @ 320x240]: Detected {len(lower_potholes)} lower-section potholes: {lower_potholes}")
        print(f"   [User Printout Upper Frame FPs]: {len(upper_fps)}")

    # 2. Test 20 hard-negative room/floor frames
    fp_count = 0
    for _ in range(20):
        neg = create_indoor_background(240, 320, rng)
        res = m.predict(neg, imgsz=416, conf=0.20, device=0, verbose=False)[0]
        potholes = [b for b in res.boxes if int(b.cls[0]) == 0]
        fp_count += len(potholes)
    print(f"   [Hard-Negative Floor/Wall Frames (20 frames @ conf=0.20)]: {fp_count} False Positives (Target: 0)")


if __name__ == "__main__":
    yaml_file = build_dataset()
    train_on_gpu(yaml_file)
