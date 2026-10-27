"""
detector.py — NCNN Object Detection Wrapper (YOLOv10n + Fine-Tuned Pothole Model)
=================================================================================
Owned by perception-agent.
Loads model exports from benchmarking/models/ for positive obstacles and the
noise-augmented fine-tuned pothole model (pothole_detection/best.pt) + CLAHE
noise-resilient classical CV detector for negative obstacles (potholes, road
damage, including printed/screen road photos held in front of the car).
"""

import sys
import cv2
import numpy as np
from pathlib import Path
from typing import List, Dict, Any, Optional
from ultralytics import YOLO

from car.perception.config import (
    REPO_ROOT,
    MODEL_PATH,
    POTHOLE_MODEL_PATH,
    IMGSZ,
    CONFIDENCE_THRESHOLD,
    POTHOLE_CONFIDENCE_THRESHOLD,
)


def enhance_road_contrast(bgr_roi: np.ndarray) -> np.ndarray:
    """
    Applies light denoising + LAB L-channel CLAHE contrast enhancement.
    Counteracts contrast washout, indoor lighting glare, and sensor noise when
    detecting potholes on real roads or on printed/screen photos held in front
    of the camera.
    """
    if bgr_roi is None or bgr_roi.size == 0:
        return bgr_roi
    denoised = cv2.GaussianBlur(bgr_roi, (3, 3), 0)
    lab = cv2.cvtColor(denoised, cv2.COLOR_BGR2LAB)
    l_chan, a_chan, b_chan = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.6, tileGridSize=(8, 8))
    l_eq = clahe.apply(l_chan)
    merged = cv2.merge((l_eq, a_chan, b_chan))
    return cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)


def cv_detect_potholes(frame: np.ndarray) -> List[Dict[str, Any]]:
    """Noise-resilient classical CV pothole/road-damage detector.

    Scans from 20% to 96% of frame height (so potholes on road surfaces OR on
    printed/screen photos held in front of the car camera are within view) using
    CLAHE contrast normalization, morphological black-hat / dark-cavity analysis,
    and Laplacian edge fusion. Rejects paper/screen outer borders by verifying
    that candidate cavities are darker than their surrounding local neighborhood.

    Args:
        frame: BGR numpy image array (H, W, 3).

    Returns:
        List of detection dicts compatible with the avoidance engine:
        [{"class_id": -1, "class_name": "pothole", "confidence": 0.62,
          "bbox": [x1, y1, x2, y2], "type": "negative"}, ...]
    """
    if frame is None or getattr(frame, "size", 0) == 0:
        return []

    h, w = frame.shape[:2]
    if h < 40 or w < 40:
        return []

    # Scan lower 80% of frame (y: 18%..96%, x: 4%..96%) so a photo of a road with
    # a pothole held in front of the car is captured even if held near center.
    roi_y1, roi_y2 = int(h * 0.18), int(h * 0.96)
    roi_x1, roi_x2 = int(w * 0.04), int(w * 0.96)
    roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]
    rh, rw = roi.shape[:2]
    if rh < 24 or rw < 24:
        return []

    # 1. Denoise and normalize local contrast with CLAHE
    gray_raw = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray_raw, (5, 5), 0)
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
    gray = clahe.apply(blur)

    # 2. Combine Laplacian edge energy + Black-Hat dark depression extraction
    lap = cv2.Laplacian(gray, cv2.CV_64F)
    lap_u = np.uint8(np.clip(np.absolute(lap), 0, 255))
    _, edge_thresh = cv2.threshold(lap_u, 22, 255, cv2.THRESH_BINARY)

    bh_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25))
    blackhat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, bh_kernel)
    _, dark_thresh = cv2.threshold(blackhat, 26, 255, cv2.THRESH_BINARY)

    # Fuse edge texture and dark cavity mask
    combined = cv2.bitwise_or(edge_thresh, dark_thresh)
    morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
    closed = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, morph_kernel)
    opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, morph_kernel)

    contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    dets = []
    roi_area = float(rh * rw)
    min_area = max(350.0, roi_area * 0.0025)
    max_area = roi_area * 0.22

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area or area > max_area:
            continue

        x, y, cw, ch = cv2.boundingRect(cnt)

        # Reject contours touching the extreme ROI boundary (e.g. outer edge of held paper/screen)
        if x <= 2 or y <= 2 or (x + cw) >= (rw - 2) or (y + ch) >= (rh - 2):
            continue

        aspect = cw / max(ch, 1)
        if not (0.28 < aspect < 4.8):
            continue

        # Solidity / fill check to ignore thin stray lines
        rect_area = float(cw * ch)
        if area / max(rect_area, 1.0) < 0.28:
            continue

        # Verify dark-depression property: interior of pothole candidate should be
        # darker or more textured than the surrounding road ring
        pad = 12
        ox1, oy1 = max(0, x - pad), max(0, y - pad)
        ox2, oy2 = min(rw, x + cw + pad), min(rh, y + ch + pad)
        inner_patch = blur[y:y + ch, x:x + cw]
        outer_patch = blur[oy1:oy2, ox1:ox2]
        inner_mean = float(np.mean(inner_patch)) if inner_patch.size else 128.0
        outer_mean = float(np.mean(outer_patch)) if outer_patch.size else 128.0

        # Accept if interior is darker than surroundings OR has strong Laplacian edge energy
        edge_density = float(np.mean(lap_u[y:y + ch, x:x + cw])) if inner_patch.size else 0.0
        if inner_mean > outer_mean + 6.0 and edge_density < 28.0:
            continue

        dets.append({
            "class_id": -1,
            "class_name": "pothole",
            "confidence": 0.62,
            "bbox": [x + roi_x1, y + roi_y1, x + roi_x1 + cw, y + roi_y1 + ch],
            "type": "negative",
        })

    return dets


class Detector:
    """Wrapper around YOLOv10n positive obstacle + fine-tuned pothole detection pipeline."""

    def __init__(
        self,
        model_path: str = MODEL_PATH,
        pothole_model_path: Optional[str] = POTHOLE_MODEL_PATH,
        imgsz: int = IMGSZ,
        conf: float = CONFIDENCE_THRESHOLD,
        pothole_conf: float = POTHOLE_CONFIDENCE_THRESHOLD,
        enable_pothole_cv: bool = True,
    ):
        self.imgsz = imgsz
        self.conf = conf
        self.pothole_conf = pothole_conf
        self.model_path = str(model_path)
        self.pothole_model_path = str(pothole_model_path) if pothole_model_path else None
        self.model = None
        self.pothole_model = None
        self.enable_pothole_cv = enable_pothole_cv
        self._load_model()

    def _load_model(self):
        """Loads the primary obstacle model (with PyTorch fallback if NCNN runtime is absent) and fine-tuned pothole model."""
        model_to_load = self.model_path
        if "ncnn" in model_to_load.lower():
            try:
                import ncnn  # noqa: F401
            except ImportError:
                pt_fallback = REPO_ROOT / "benchmarking" / "models" / "yolov10n.pt"
                if not pt_fallback.exists():
                    pt_fallback = REPO_ROOT / "yolov10n.pt"
                if pt_fallback.exists():
                    model_to_load = str(pt_fallback)

        try:
            self.model = YOLO(model_to_load, task="detect")
        except Exception as e:
            print(f"⚠️ Failed to load model from {model_to_load}: {e}", file=sys.stderr)
            raise

        # Load dedicated fine-tuned pothole model (best.pt / pothole_best.pt)
        candidates = []
        if self.pothole_model_path:
            candidates.append(Path(self.pothole_model_path))
        candidates.extend([
            REPO_ROOT / "pothole_best.pt",
            REPO_ROOT / "pothole_detection" / "best.pt",
            REPO_ROOT / "pothole_detection" / "rdd2022_yolov8n.pt",
        ])
        for cand in candidates:
            if cand.exists():
                try:
                    self.pothole_model = YOLO(str(cand), task="detect")
                    break
                except Exception:
                    continue

    def _detect_yolo_potholes(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        """Runs the fine-tuned pothole YOLO model with CLAHE fallback for noisy/held-photo inputs."""
        if self.pothole_model is None:
            return []

        h, w = frame.shape[:2]
        # Expanded vertical window (12% to 98%) so a road photo held in front of
        # the car camera is fully covered even if held in the upper/center view.
        roi_y1, roi_y2 = int(h * 0.12), int(h * 0.98)
        roi_x1, roi_x2 = int(w * 0.02), int(w * 0.98)
        sub_frame = frame[roi_y1:roi_y2, roi_x1:roi_x2]
        if sub_frame.size == 0:
            return []

        potholes: List[Dict[str, Any]] = []

        # Pass 1: Direct inference on expanded ROI
        passes = [sub_frame, enhance_road_contrast(sub_frame)]
        for pass_idx, img_in in enumerate(passes):
            if pass_idx == 1 and len(potholes) > 0:
                # Only run CLAHE pass if Pass 1 found nothing (saves latency on Pi 5)
                break
            try:
                res = self.pothole_model.predict(
                    img_in,
                    imgsz=max(self.imgsz, 320),
                    conf=self.pothole_conf,
                    verbose=False,
                )[0]
                for box in res.boxes:
                    cls_id = int(box.cls[0])
                    raw_conf = float(box.conf[0])
                    name = str(res.names.get(cls_id, "pothole")).lower()
                    # Accept pothole (D40) and major road surface damage classes
                    if "pothole" in name or "d40" in name or "crack" in name or cls_id in (0, 1, 2, 3):
                        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                        gx1, gy1, gx2, gy2 = x1 + roi_x1, y1 + roi_y1, x2 + roi_x1, y2 + roi_y1

                        # Calibrate confidence so downstream avoidance engine (>0.30 danger) responds decisively
                        calibrated_conf = min(0.95, max(0.58, raw_conf * 3.0))
                        potholes.append({
                            "class_id": -1,
                            "class_name": "pothole",
                            "confidence": calibrated_conf,
                            "bbox": [gx1, gy1, gx2, gy2],
                            "type": "negative",
                        })
            except Exception:
                continue

        return potholes

    def detect(self, frame) -> List[Dict[str, Any]]:
        """Run object detection on a BGR numpy frame.

        Runs YOLOv10n for positive obstacles (cars, people, etc.), runs the
        fine-tuned pothole neural network for negative obstacles (potholes,
        road damage), and fuses the noise-resilient classical CV pothole detector.

        Args:
            frame: BGR numpy image array (H, W, 3).

        Returns:
            List of detection dicts:
            [{"class_id": int, "class_name": str, "confidence": float,
              "bbox": [x1, y1, x2, y2], "type": "positive" | "negative"}, ...]
        """
        if self.model is None or frame is None:
            return []

        results = self.model(frame, imgsz=self.imgsz, conf=self.conf, verbose=False)
        detections = []

        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            name = results[0].names.get(cls_id, str(cls_id))
            conf = float(box.conf[0])
            coords = box.xyxy[0].tolist()

            detections.append({
                "class_id": cls_id,
                "class_name": name,
                "confidence": conf,
                "bbox": coords,
                "type": "positive",
            })

        # 2. Fine-tuned YOLO pothole model detections
        yolo_potholes = self._detect_yolo_potholes(frame)
        detections.extend(yolo_potholes)

        # 3. Append noise-resilient classical CV pothole detections (deduplicated)
        if self.enable_pothole_cv:
            cv_dets = cv_detect_potholes(frame)
            for cd in cv_dets:
                cx1, cy1, cx2, cy2 = cd["bbox"]
                overlap = False
                for existing in yolo_potholes:
                    ex1, ey1, ex2, ey2 = existing["bbox"]
                    if abs(cx1 - ex1) < 45 and abs(cy1 - ey1) < 45:
                        overlap = True
                        break
                if not overlap:
                    detections.append(cd)

        return detections
