"""
tracker.py — Object Tracking (Follow-Me)
=========================================
Owned by perception-agent.
Wraps OpenCV's built-in trackers (KCF/CSRT) to maintain a persistent ID 
on a detected person across frames. 
Note: Can be swapped to NanoTrack later by providing the ONNX weights.
"""

import cv2
from typing import Tuple, Optional

class ObjectTracker:
    def __init__(self):
        # KCF is fast and built-in. CSRT is more accurate but slower.
        self.tracker = None
        self.is_tracking = False
        self.bbox = None

    def init_tracker(self, frame, bbox: Tuple[int, int, int, int]):
        """Initialize the tracker with a new bounding box.
        bbox format: (x, y, width, height)
        """
        # Recreate tracker instance for fresh initialization
        try:
            self.tracker = cv2.TrackerKCF_create()
        except AttributeError:
            # Fallback for some OpenCV versions
            self.tracker = cv2.TrackerMIL_create()
            
        self.is_tracking = self.tracker.init(frame, bbox)
        self.bbox = bbox
        return self.is_tracking

    def update(self, frame) -> Tuple[bool, Optional[Tuple[int, int, int, int]]]:
        """Update tracker with a new frame.
        Returns: (success, (x, y, w, h))
        """
        if not self.is_tracking or self.tracker is None:
            return False, None

        success, bbox = self.tracker.update(frame)
        if success:
            self.bbox = bbox
            return True, bbox
        else:
            self.is_tracking = False
            self.bbox = None
            return False, None

    def stop(self):
        """Stop tracking."""
        self.is_tracking = False
        self.tracker = None
        self.bbox = None
