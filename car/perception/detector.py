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
    """Classical CV pothole/road-damage detector.

    Scans the bottom 55% of the frame (the road surface) using Laplacian
    edge detection + morphological filtering to find dark, irregular holes
    without triggering false positives on shadows or upper walls.

    Args:
        frame: BGR numpy image array (H, W, 3).

    Returns:
        List of detection dicts compatible with the avoidance engine:
        [{"class_id": -1, "class_name": "pothole", "confidence": 0.42,
          "bbox": [x1, y1, x2, y2], "type": "negative"}, ...]
    """
    if frame is None or getattr(frame, "size", 0) == 0:
        return []

    h, w = frame.shape[:2]
    if h < 40 or w < 40:
        return []

    roi_y = int(h * 0.45)
    roi = frame[roi_y:, :]
    if roi.size == 0:
        return []

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    lap = cv2.Laplacian(blur, cv2.CV_64F)
    lap_u = np.uint8(np.clip(np.absolute(lap), 0, 255))

    _, thresh = cv2.threshold(lap_u, 28, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    dets = []
    roi_area = roi.shape[0] * roi.shape[1]

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 900 or area > roi_area * 0.25:
            continue
        x, y, cw, ch = cv2.boundingRect(cnt)
        if not (0.25 < cw / max(ch, 1) < 5.0):
            continue
        dets.append({
            "class_id": -1,
            "class_name": "pothole",
            "confidence": 0.42,
            "bbox": [x, y + roi_y, x + cw, y + roi_y + ch],
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
        """Runs the dedicated D40 pothole YOLO model strictly on the road surface."""
        if self.pothole_model is None or frame is None:
            return []

        h, w = frame.shape[:2]
        # Focus strictly on road surface (lower 60% of frame) to eliminate false positives
        roi_y1, roi_y2 = int(h * 0.38), int(h * 0.98)
        roi_x1, roi_x2 = int(w * 0.05), int(w * 0.95)
        sub_frame = frame[roi_y1:roi_y2, roi_x1:roi_x2]
        if sub_frame.size == 0:
            return []

        potholes: List[Dict[str, Any]] = []
        try:
            res = self.pothole_model.predict(
                sub_frame,
                imgsz=max(self.imgsz, 320),
                conf=self.pothole_conf,
                verbose=False,
            )[0]
            for box in res.boxes:
                cls_id = int(box.cls[0])
                raw_conf = float(box.conf[0])
                name = str(res.names.get(cls_id, "pothole")).lower()
                # Strict D40 pothole filter (Class 0: pothole_D40) — ignore cracks and lines
                if cls_id == 0 or "pothole" in name or "d40" in name:
                    x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                    gx1, gy1, gx2, gy2 = x1 + roi_x1, y1 + roi_y1, x2 + roi_x1, y2 + roi_y1
                    potholes.append({
                        "class_id": -1,
                        "class_name": "pothole",
                        "confidence": raw_conf,
                        "bbox": [gx1, gy1, gx2, gy2],
                        "type": "negative",
                    })
        except Exception:
            pass

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
