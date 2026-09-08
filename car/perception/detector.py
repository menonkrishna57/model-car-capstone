"""
detector.py — NCNN Object Detection Wrapper (YOLOv10n)
=======================================================
Owned by perception-agent.
Loads model exports from benchmarking/models/ and runs NCNN inference.
Keeps inference behind a clean interface so the same detection code runs
on both the real Pi 5 and offline in benchmarking/.
"""

import sys
from pathlib import Path
from typing import List, Dict, Any
from ultralytics import YOLO

from car.perception.config import MODEL_PATH, IMGSZ, CONFIDENCE_THRESHOLD


class Detector:
    """Thin wrapper around YOLOv10n NCNN detection pipeline."""

    def __init__(
        self,
        model_path: str = MODEL_PATH,
        imgsz: int = IMGSZ,
        conf: float = CONFIDENCE_THRESHOLD,
    ):
        self.imgsz = imgsz
        self.conf = conf
        self.model_path = str(model_path)
        self.model = None
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

        Args:
            frame: BGR numpy image array (H, W, 3).

        Returns:
            List of detection dicts:
            [{
                "class_id": int,
                "class_name": str,
                "confidence": float,
                "bbox": [x1, y1, x2, y2]
            }, ...]
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
            })

        return detections
