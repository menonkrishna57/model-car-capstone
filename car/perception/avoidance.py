"""
avoidance.py — Obstacle Avoidance Decision Engine
===================================================
Owned by perception-agent.
Ported from Parth's original logic. Calculates danger scores for left, center,
and right zones based on bounding box size and confidence, then returns a decision.
"""

from typing import List, Dict, Any

# Constants adapted from Parth's code
ZONE_LEFT_END = 0.33
ZONE_RIGHT_START = 0.67
OBSTACLE_WEIGHT = 1.0
POTHOLE_WEIGHT = 1.2
DANGER_THRESHOLD = 0.3

def compute_zone_scores(detections: List[Dict[str, Any]], frame_w: int, frame_h: int) -> Dict[str, float]:
    """Computes danger scores for Left, Center, and Right zones."""
    scores = {"left": 0.0, "center": 0.0, "right": 0.0}
    frame_area = frame_w * frame_h
    if frame_area == 0:
        return scores

    for det in detections:
        # Expected format from detector.py: [x1, y1, x2, y2]
        x1, y1, x2, y2 = det["bbox"]
        cx = (x1 + x2) / 2
        conf = det.get("confidence", 0.5)
        
        # Apply higher weight to potholes (negative obstacles are harder to see)
        wt = POTHOLE_WEIGHT if det.get("type") == "negative" else OBSTACLE_WEIGHT
        
        bbox_area = max((x2 - x1) * (y2 - y1), 1)
        area_score = min(bbox_area / frame_area, 0.5)
        danger = conf * wt * (0.4 + area_score)

        cx_frac = cx / frame_w
        if cx_frac < ZONE_LEFT_END:
            scores["left"] += danger
            det["zone"] = "left"
        elif cx_frac > ZONE_RIGHT_START:
            scores["right"] += danger
            det["zone"] = "right"
        else:
            scores["center"] += danger
            det["zone"] = "center"

    return scores

def decide_avoidance(scores: Dict[str, float]) -> str:
    """Returns CLEAR, LEFT, RIGHT, or STOP based on zone scores."""
    L, C, R = scores["left"], scores["center"], scores["right"]
    T = DANGER_THRESHOLD

    if L < T and C < T and R < T:
        return "CLEAR"

    if C >= T:
        if L >= T and R >= T:
            return "STOP"
        return "LEFT" if L <= R else "RIGHT"

    if L >= T and R < T:
        return "RIGHT"
    if R >= T and L < T:
        return "LEFT"
    if L >= T and R >= T:
        return "STOP"

    return "CLEAR"
