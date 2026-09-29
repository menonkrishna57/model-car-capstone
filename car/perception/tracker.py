"""
tracker.py — KCF Person Tracker with Owner Re-Identification
=============================================================
Owned by perception-agent.

Implements the Follow-Me tracker described in architecture.md §5:

  Phase 1 — Lock-On:
    User calls lock_on(frame, bbox) to select the closest person.
    Appearance features are extracted and stored as the owner profile.
    KCF tracker is initialised on that bbox.

  Phase 2 — Tracking (steady state):
    update(frame) returns (True, bbox) while KCF is confident.
    Caller reads bbox center_x and feeds it into the PID controller.

  Phase 3 — Re-Acquisition:
    When update() returns (False, None), KCF has lost confidence.
    Caller should:
      1. Send PI_STOP to freeze the car.
      2. Run full-frame YOLO to get candidate person detections.
      3. Call reacquire(frame, candidates) to find the best match.
      4. If reacquire() returns True: send PI_GO and resume tracking.
         If it returns False (timeout): call stop() and wait for lock-on.

Owner Profile (lightweight, no neural re-ID):
  - Upper-body HSV histogram (top 40% of bbox)
  - Lower-body HSV histogram (bottom 40% of bbox)
  - Bbox aspect ratio (height / width) — body build proxy
  - Last-known bbox area — distance sanity check

Comparison: OpenCV cv2.compareHist with HISTCMP_CORREL.
Similarity threshold: 0.55 (tunable via REACQUIRE_THRESHOLD).

Limitations (from architecture.md §5):
  - Changing clothes mid-session breaks re-ID (appearance-based only).
  - Designed for 2–4 people in frame; degrades in dense crowds.
  - HSV hue channel partially mitigates lighting changes.
"""

import cv2
import numpy as np
from typing import Tuple, Optional, Dict, Any

# Minimum histogram correlation score to accept a candidate as the owner.
# Range: 0.0 (no match) to 1.0 (perfect). Tune on real footage.
REACQUIRE_THRESHOLD = 0.55

# Number of HSV histogram bins per channel for the appearance profile.
_HIST_BINS = [18, 32, 32]  # Hue / Saturation / Value
_HIST_RANGES = [0, 180, 0, 256, 0, 256]


def _extract_hsv_hist(crop: np.ndarray) -> Optional[np.ndarray]:
    """
    Computes a flattened, L1-normalised HSV histogram for a BGR crop.
    Returns None if the crop is empty or too small.
    """
    if crop is None or crop.size == 0 or crop.shape[0] < 4 or crop.shape[1] < 4:
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1, 2], None, _HIST_BINS, _HIST_RANGES)
    cv2.normalize(hist, hist, norm_type=cv2.NORM_L1)
    return hist.flatten()


def _build_appearance_profile(frame: np.ndarray, bbox: Tuple[int, int, int, int]) -> Dict[str, Any]:
    """
    Extracts lightweight appearance features from a bounding box region.

    Args:
        frame: Full BGR frame.
        bbox: (x, y, w, h) bounding box of the person.

    Returns:
        Dict with keys: upper_hist, lower_hist, aspect_ratio, bbox_area.
    """
    x, y, w, h = [int(v) for v in bbox]
    fh, fw = frame.shape[:2]

    # Clamp to frame bounds
    x1 = max(0, x)
    y1 = max(0, y)
    x2 = min(fw, x + w)
    y2 = min(fh, y + h)
    w_c = x2 - x1
    h_c = y2 - y1

    if w_c < 4 or h_c < 4:
        return {}

    # Upper 40% of bbox → shirt / jacket colour
    upper_h = max(4, int(h_c * 0.40))
    upper_crop = frame[y1 : y1 + upper_h, x1:x2]

    # Lower 40% of bbox → pants / shorts colour
    lower_start = max(y1, y2 - int(h_c * 0.40))
    lower_crop = frame[lower_start:y2, x1:x2]

    return {
        "upper_hist": _extract_hsv_hist(upper_crop),
        "lower_hist":  _extract_hsv_hist(lower_crop),
        "aspect_ratio": h_c / max(w_c, 1),   # height / width (body build)
        "bbox_area":    w_c * h_c,
    }


def _compare_profiles(profile: Dict[str, Any], candidate_profile: Dict[str, Any]) -> float:
    """
    Computes a similarity score in [0, 1] between two appearance profiles.
    Uses average of upper and lower histogram correlations.
    """
    scores = []

    for key in ("upper_hist", "lower_hist"):
        h1 = profile.get(key)
        h2 = candidate_profile.get(key)
        if h1 is not None and h2 is not None:
            # cv2.HISTCMP_CORREL returns 1.0 for identical, 0.0 for orthogonal
            scores.append(float(cv2.compareHist(
                h1.reshape(-1, 1).astype(np.float32),
                h2.reshape(-1, 1).astype(np.float32),
                cv2.HISTCMP_CORREL
            )))

    if not scores:
        return 0.0
    return float(np.mean(scores))


class ObjectTracker:
    """
    KCF-based person tracker with lightweight appearance re-identification.
    """

    def __init__(self):
        self.tracker = None
        self.is_tracking: bool = False
        self.bbox: Optional[Tuple[int, int, int, int]] = None
        self._owner_profile: Dict[str, Any] = {}

    # ── Phase 1: Lock-On ─────────────────────────────────────────────────────

    def lock_on(self, frame: np.ndarray, bbox: Tuple[int, int, int, int]) -> bool:
        """
        Initialises the KCF tracker on bbox and captures the owner appearance profile.

        Args:
            frame: Current BGR frame (full resolution).
            bbox: (x, y, w, h) bounding box around the target person.

        Returns:
            True if the tracker was initialised successfully.
        """
        # Build and store owner appearance features
        self._owner_profile = _build_appearance_profile(frame, bbox)

        # Initialise KCF tracker (fast, ~1ms/frame on Pi 5)
        try:
            self.tracker = cv2.TrackerKCF_create()
        except AttributeError:
            # Fallback: OpenCV version without KCF (rare on Pi OS)
            self.tracker = cv2.TrackerMIL_create()

        self.is_tracking = self.tracker.init(frame, bbox)
        self.bbox = bbox if self.is_tracking else None
        return self.is_tracking

    # ── Phase 2: Tracking ─────────────────────────────────────────────────────

    def update(self, frame: np.ndarray) -> Tuple[bool, Optional[Tuple[int, int, int, int]]]:
        """
        Advances the KCF tracker by one frame.

        Returns:
            (True, (x, y, w, h)) when tracking is confident.
            (False, None) when KCF has lost the target → enter Phase 3.
        """
        if not self.is_tracking or self.tracker is None:
            return False, None

        success, bbox = self.tracker.update(frame)
        if success:
            self.bbox = tuple(int(v) for v in bbox)
            return True, self.bbox
        else:
            self.is_tracking = False
            self.bbox = None
            return False, None

    # ── Phase 3: Re-Acquisition ───────────────────────────────────────────────

    def reacquire(self, frame: np.ndarray, candidates: list) -> bool:
        """
        Attempts to re-identify the owner among a list of candidate detections
        produced by full-frame YOLO. Uses histogram correlation against the
        stored owner appearance profile.

        Args:
            frame:      Current BGR frame.
            candidates: List of detection dicts with "bbox" [x1, y1, x2, y2]
                        and optionally "confidence". Only 'person' class dicts
                        should be passed (filter before calling).

        Returns:
            True if the owner was re-identified and KCF re-initialised.
            False if no candidate exceeded REACQUIRE_THRESHOLD.
        """
        if not self._owner_profile:
            # No stored profile — cannot re-identify (e.g. tracker was never locked on)
            return False

        best_score = -1.0
        best_bbox  = None

        for det in candidates:
            x1, y1, x2, y2 = [int(v) for v in det["bbox"]]
            w = x2 - x1
            h = y2 - y1
            if w < 4 or h < 4:
                continue

            candidate_bbox    = (x1, y1, w, h)
            candidate_profile = _build_appearance_profile(frame, candidate_bbox)
            score = _compare_profiles(self._owner_profile, candidate_profile)

            if score > best_score:
                best_score = score
                best_bbox  = candidate_bbox

        if best_score >= REACQUIRE_THRESHOLD and best_bbox is not None:
            # Re-lock KCF on the best-matching candidate
            return self.lock_on(frame, best_bbox)

        return False

    def get_owner_bbox_area(self) -> int:
        """Returns the last-known owner bbox area, or 0 if not tracking."""
        if self.bbox is None:
            return 0
        _, _, w, h = self.bbox
        return w * h

    # ── Stop / Reset ─────────────────────────────────────────────────────────

    def stop(self):
        """Stops tracking and clears state. Does NOT clear the owner profile."""
        self.is_tracking = False
        self.tracker = None
        self.bbox = None

    def reset(self):
        """Full reset including owner profile (use at session start)."""
        self.stop()
        self._owner_profile = {}
