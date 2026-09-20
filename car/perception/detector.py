"""
detector.py — NCNN Object Detection Wrapper (YOLOv10n)
=======================================================
Owned by perception-agent.
Loads model exports from benchmarking/models/ and runs NCNN inference.
Also includes a classical CV pothole detector (no ML model needed) to catch
negative obstacles (potholes, road damage) that YOLO can't see.
"""

import sys
import cv2
import numpy as np
from pathlib import Path
from typing import List, Dict, Any
from ultralytics import YOLO

from car.perception.config import MODEL_PATH, IMGSZ, CONFIDENCE_THRESHOLD


def cv_detect_potholes(frame) -> List[Dict[str, Any]]:
    """Classical CV pothole/road-damage detector (no ML model needed).

    Scans the bottom 55% of the frame (the road surface) using Laplacian
    edge detection + morphological filtering to find dark, irregular holes.

    Args:
        frame: BGR numpy image array (H, W, 3).

    Returns:
        List of detection dicts compatible with the avoidance engine:
        [{"class_id": -1, "class_name": "pothole", "confidence": 0.42,
          "bbox": [x1, y1, x2, y2], "type": "negative"}, ...]
    """
    h, w = frame.shape[:2]

    # Only look at bottom 55% of frame (where the road actually is)
    roi_y = int(h * 0.45)
    roi = frame[roi_y:, :]

    # Convert to grayscale and blur to reduce noise
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Laplacian edge detection highlights sharp intensity changes (hole edges)
    lap = cv2.Laplacian(blur, cv2.CV_64F)
    lap_u = np.uint8(np.clip(np.absolute(lap), 0, 255))

    # Threshold to binary, then clean up with morphological ops
    _, thresh = cv2.threshold(lap_u, 28, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel)

    # Find contours (potential potholes)
    contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    dets = []
    roi_area = roi.shape[0] * roi.shape[1]

    for cnt in contours:
        area = cv2.contourArea(cnt)

        # Filter: too small = noise, too big = false positive
        if area < 900 or area > roi_area * 0.25:
            continue

        x, y, cw, ch = cv2.boundingRect(cnt)

        # Filter: aspect ratio sanity (potholes are wider than tall, not line-shaped)
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
    """Thin wrapper around YOLOv10n NCNN detection pipeline."""

    def __init__(
        self,
        model_path: str = MODEL_PATH,
        imgsz: int = IMGSZ,
        conf: float = CONFIDENCE_THRESHOLD,
        enable_pothole_cv: bool = True,
    ):
        self.imgsz = imgsz
        self.conf = conf
        self.model_path = str(model_path)
        self.model = None
        self.enable_pothole_cv = enable_pothole_cv
        self._load_model()

    def _load_model(self):
        """Loads the NCNN model via Ultralytics."""
        try:
            self.model = YOLO(self.model_path, task="detect")
        except Exception as e:
            print(f"⚠️ Failed to load model from {self.model_path}: {e}", file=sys.stderr)
            raise

    def detect(self, frame) -> List[Dict[str, Any]]:
        """Run object detection on a BGR numpy frame.

        Runs YOLOv10n for positive obstacles (cars, people, etc.) and
        optionally runs the classical CV pothole detector for negative
        obstacles (potholes, road damage).

        Args:
            frame: BGR numpy image array (H, W, 3).

        Returns:
            List of detection dicts:
            [{"class_id": int, "class_name": str, "confidence": float,
              "bbox": [x1, y1, x2, y2], "type": "positive" | "negative"}, ...]
        """
        if self.model is None:
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

        # Append classical CV pothole detections (no extra GPU/model cost)
        if self.enable_pothole_cv:
            pothole_dets = cv_detect_potholes(frame)
            detections.extend(pothole_dets)

        return detections

