"""
approach.py — Human and Object Close Approach Stopping Engine
==============================================================
Owned by perception-agent / Worker 1 (Requirement 1).

Implements close approach evaluation for forward navigation:
  - Continues forward motion at approach speed (PWM 180) upon initial distant
    detection of obstacles/humans.
  - Halts forward motion when an obstacle breaches the bounding-box stop
    threshold (> 40% frame height in center corridor) or the ultrasonic
    emergency stopping boundary (< 25.0 cm).
  - Returns:
      ("STOP", None)      — emergency stop required
      ("SLOW", 180)       — cautious forward driving during approach
      ("CLEAR", None)     — path clear / obstacles beyond caution range
"""

from typing import List, Dict, Any, Optional, Tuple

# Architecture & Configuration Thresholds
BBOX_STOP_THRESHOLD = 0.40          # > 40% of frame height -> STOP
BBOX_SLOW_THRESHOLD = 0.15          # 15% to 40% frame height -> SLOW approach
EMERGENCY_STOP_DISTANCE_CM = 25.0  # Ultrasonic critical boundary
APPROACH_SPEED = 180                # Cautious forward approach PWM (>= 180 floor)

# Forward driving center corridor boundaries (20% to 80% horizontal frame)
CENTER_CORRIDOR_X_MIN = 0.20
CENTER_CORRIDOR_X_MAX = 0.80


def is_valid_ultrasonic_distance(dist: Any) -> bool:
    """
    Validates an ultrasonic sensor reading:
    Rejects 0.0 noise spikes, negative values, and 999.0 sensor timeouts.
    """
    return isinstance(dist, (int, float)) and 1.0 < dist < 850.0


def check_close_approach(
    detections: Optional[List[Dict[str, Any]]],
    frame_w: int,
    frame_h: int,
    us_sonar: Optional[Dict[str, float]] = None,
    **kwargs: Any,
) -> Tuple[str, Optional[int]]:
    """
    Evaluates vision detections and ultrasonic distance readings to govern
    close approach stopping behavior.

    Rules:
      1. Ultrasonic < 25.0 cm on any valid sensor -> ("STOP", None).
      2. BBOX height > 0.40 (40%) in center corridor (0.20 to 0.80) -> ("STOP", None).
      3. BBOX height between 0.15 and 0.40 in center corridor and ultrasonic >= 25.0 cm
         -> ("SLOW", 180).
      4. Otherwise -> ("CLEAR", None).

    Args:
        detections: List of detection dictionaries containing 'bbox': [x1, y1, x2, y2].
        frame_w: Frame width in pixels.
        frame_h: Frame height in pixels.
        us_sonar: Dict of ultrasonic distances in cm, e.g. {'left': 80.0, 'right': 85.0}.

    Returns:
        Tuple[str, Optional[int]]: (action, speed) where action is 'STOP', 'SLOW', or 'CLEAR'.
    """
    # Accept 'sonar' as alternative kwarg for flexible caller integration
    sonar = us_sonar if us_sonar is not None else kwargs.get("sonar")

    # ── 1. Check Ultrasonic Emergency Stopping Boundary (< 25.0 cm) ──────────
    if sonar and isinstance(sonar, dict):
        for sensor_name, dist in sonar.items():
            # Only consider forward/side sensors; ignore rear sensor for forward approach
            if str(sensor_name).lower() == "rear":
                continue
            if is_valid_ultrasonic_distance(dist) and dist < EMERGENCY_STOP_DISTANCE_CM:
                return "STOP", None

    # Guard against invalid or uninitialized frame dimensions
    if frame_w <= 0 or frame_h <= 0:
        return "CLEAR", None

    # ── 2. Check Vision Bounding-Box Heights in Center Corridor ─────────────
    has_slow_approach = False

    for det in (detections or []):
        if not isinstance(det, dict):
            continue
        bbox = det.get("bbox")
        if not bbox or len(bbox) != 4:
            continue
        try:
            x1, y1, x2, y2 = [float(coord) for coord in bbox]
        except (ValueError, TypeError):
            continue

        bbox_h = y2 - y1
        if bbox_h <= 0:
            continue

        height_frac = bbox_h / float(frame_h)
        cx = (x1 + x2) / 2.0

        # Center corridor check: forward travel path spans 20% to 80% frame width
        in_center_corridor = (
            CENTER_CORRIDOR_X_MIN * frame_w <= cx <= CENTER_CORRIDOR_X_MAX * frame_w
        )

        if in_center_corridor:
            # Imminent collision threshold: bbox height > 40% of frame
            if height_frac > BBOX_STOP_THRESHOLD:
                return "STOP", None
            # Cautious approach threshold: bbox height between 15% and 40%
            elif height_frac >= BBOX_SLOW_THRESHOLD:
                has_slow_approach = True

    # ── 3. Decision Resolution ───────────────────────────────────────────────
    if has_slow_approach:
        return "SLOW", APPROACH_SPEED

    return "CLEAR", None
