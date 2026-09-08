"""
detector.py — YOLOv10n NCNN Object Detection Wrapper
=====================================================
Wraps ultralytics YOLO inference behind a clean interface so the same
detection code runs on both the Oracle Cloud A1 proxy and the real Pi 5.

Hardware-specific calls (GPIO, CSI camera, I2C) are NOT in this file —
they live in their own modules and feed frames into detect().

NOTE: Pi Camera connector not yet available — this is skeleton code.
      Marked "unverified / simulated" per project conventions.
"""

from pathlib import Path
from ultralytics import YOLO


class Detector:
    """Thin wrapper around YOLO NCNN inference."""

    def __init__(self, model_path: str = "yolov10n_ncnn_model", imgsz: int = 320, conf: float = 0.25):
        self.imgsz = imgsz
        self.conf = conf
        self.model = YOLO(model_path, task="detect")

    def detect(self, frame):
        """Run detection on a single BGR frame.

        Args:
            frame: numpy array (H, W, 3) in BGR format.

        Returns:
            List of dicts with keys: class_id, class_name, confidence, bbox (x1, y1, x2, y2).
        """
        results = self.model(frame, imgsz=self.imgsz, conf=self.conf, verbose=False)
        detections = []
        for box in results[0].boxes:
            detections.append({
                "class_id": int(box.cls[0]),
                "class_name": results[0].names[int(box.cls[0])],
                "confidence": float(box.conf[0]),
                "bbox": box.xyxy[0].tolist(),
            })
        return detections
