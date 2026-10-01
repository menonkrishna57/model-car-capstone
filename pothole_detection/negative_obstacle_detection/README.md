# Negative Obstacle Detection & Pothole Avoidance

This directory aggregates all modules, scripts, test logs, and scenario documentation related to **negative obstacle detection** (potholes, road damage, surface cavities) in the capstone project.

---

## Files in this Directory

| File | Source Location | Description |
|---|---|---|
| [detector.py](file:///e:/Capstone/MentorPi/negative_obstacle_detection/detector.py) | `car/perception/detector.py` | Vision detector combining YOLOv10n positive detection with `cv_detect_potholes()`, a classical CV algorithm (Laplacian edge detection + morphological filtering + contour analysis) targeting the road surface (bottom 55% ROI). |
| [avoidance.py](file:///e:/Capstone/MentorPi/negative_obstacle_detection/avoidance.py) | `car/perception/avoidance.py` | Avoidance decision engine. Factors in negative obstacles with higher danger weighting (`POTHOLE_WEIGHT = 1.2` vs `OBSTACLE_WEIGHT = 1.0`) and calculates Left/Center/Right zone risk. |
| [avoidance_friend.py](file:///e:/Capstone/MentorPi/negative_obstacle_detection/avoidance_friend.py) | `Our Work/avoidance_friend.py` | Standalone test script supporting HuggingFace fine-tuned YOLOv8 pothole weights with automated classical CV fallback, visual bounding box overlays, and live/video inference. |
| [avoidance_log.csv](file:///e:/Capstone/MentorPi/negative_obstacle_detection/avoidance_log.csv) | `Our Work/avoidance_log.csv` | Sample telemetry log recording per-frame decisions, zone danger values, inference latencies, and separate counts for positive vs. negative obstacle detections. |
| [05_pothole_ahead.md](file:///e:/Capstone/MentorPi/negative_obstacle_detection/05_pothole_ahead.md) | `docs/scenarios/05_pothole_ahead.md` | Full system scenario and serial protocol sequence explaining why horizontal ultrasonic sensors fail on negative obstacles and how the camera + `PI_STOP` / bypass commands navigate around the hole. |
