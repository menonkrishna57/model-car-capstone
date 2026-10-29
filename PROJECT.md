# Project: Autonomous RC Car UI & Control Stack Overhaul

## Architecture
Two-layer autonomous vehicle architecture:
- **Arduino Uno (Firmware)**: Real-time motor control, L298N skid-steer PWM, HC-SR04P ultrasonic polling, turning and cliff avoidance state machine. Implements `TURN <angle> <speed>` and `DRIVE <speed> <duration>` commands per `arduino/PROTOCOL.md`.
- **Raspberry Pi 5 (High-Level Perception & Runtime)**: `car/main.py` entry point orchestrating YOLOv10n/NCNN obstacle detection, KCF person tracker, HTTP MJPEG live stream, diagnostic HUD, and web control cockpit.
- **Frontend Web UI**: Embedded single-page cockpit in `car/main.py` (`PAGE`), serving live video stream, telemetry badges, D-pad controls, manual speed selection, and camera calibration toggles.

## Code Layout
- `car/main.py`: Central runtime containing `SystemState`, `DiagnosticHandler`, `draw_hud`, `PAGE` (HTML/CSS/JS cockpit), camera capture loop, and autonomous coordination loops.
- `car/live_detect.py`: Reference implementation for daylight AWB and `swap_rb` BGR color conversion.
- `car/perception/tracker.py`: KCF tracker and HSV color histogram Re-ID (skin tone H: 0–25).
- `arduino/point_to_point_control/point_to_point_control.ino`: Firmware state machine supporting `TURN <angle> <speed>`.
- `tests/test_main_coordination.py`: Unit and coordination test suite.
- `tests/test_adversarial_coordination.py`: Adversarial and edge-case coordination tests.
- `tests/test_follow_me.py`: Person follower test suite.
- `tests/test_lane_detection.py`: Lane perception test suite.

## Feature Inventory
| # | Feature | Description | Milestone | Source |
|---|---------|-------------|-----------|--------|
| 1 | Dynamic Turning Speed | Use user-selected manual speed (`spd`: 130, 185, 235 PWM) in `manual_drive` turning commands instead of hardcoded 160 | M1 | Survey R1 |
| 2 | Client Key-State Tracking | Maintain `keysDown` Set in Web UI to resolve held keys so releasing one key does not prematurely stop another held direction | M1 | Survey R1 |
| 3 | Repeat Event Guard | Ignore browser `e.repeat` events to prevent 30–50Hz keydown spamming from resetting turn maneuvers | M1 | Survey R1 |
| 4 | Resolve Key Shortcut Collision | Disarm or rebind global 'D' key shortcut (Daylight AWB) so pressing 'D' to turn right does not toggle camera settings | M1 | Survey R1 |
| 5 | Camera Channel Byte-Order Swap | Align Picamera2 capture in `car/main.py` with `cv2.cvtColor(frame_raw, cv2.COLOR_RGB2BGR)` to prevent Red/Blue inversion on Pi 5 | M2 | Survey R2 |
| 6 | Skin Tone & HSV Re-ID Restoration | Restore natural skin tone perception (H: 0–25) and clothing color fidelity for person re-identification in `tracker.py` | M2 | Survey R2 |
| 7 | Web UI Color Calibration Toggle | Add `/toggle_swap_rb` and `/set_swap_rb` endpoints, telemetry exposure, and UI control for runtime field calibration | M2 | Survey R2 |
| 8 | Structured Cockpit Layout | Deconflict top glass header bar from camera HUD overlay across 1080p, 720p, and mobile viewports | M3 | Survey R3 |
| 9 | In-Stream ASCII HUD Sanitization | Replace Unicode emojis and symbols in `draw_hud` with clean ASCII to eliminate OpenCV Hershey font '?' corruptions | M3 | Survey R3 |
| 10 | High-Agency UI Modernization | Modernize Web UI per `/design-taste-frontend`: dark theme `#080b11`, Geist + JetBrains Mono typography, inline SVGs (anti-emoji) | M3 | Survey R3 |
| 11 | Tactile D-Pad & Controls Dock | Ergonomic controls dock with tactile active states (`:active`, `.active`), keyboard mirroring, and clear telemetry badges | M3 | Survey R3 |
| 12 | Comprehensive Test Suite Validation | Pass 100% of tests in `test_main_coordination.py`, `test_adversarial_coordination.py`, `test_follow_me.py`, `test_lane_detection.py` | M4 | Acceptance Criteria |
| 13 | Forensic Integrity Verification | Binary audit verification ensuring authentic implementations without hardcoding or facades | M4 | Project Rules |

## Milestones
| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| M1 | Manual Skid-Steer Turning Reliability & Responsive Control | `car/main.py`: dynamic turn speed, `keysDown` tracking, repeat guard, key conflict resolution | none | DONE |
| M2 | Camera Color Filter & Skin Tone Perception Restoration | `car/main.py`: RGB↔BGR swap, `swap_rb` state, `/toggle_swap_rb` endpoint, Re-ID verification | M1 | DONE |
| M3 | High-Agency Frontend UI Overhaul & HUD Deconfliction | `car/main.py`: Structured Cockpit layout, ASCII HUD sanitization, `/design-taste-frontend` UI overhaul | M2 | DONE |
| M4 | E2E Coordination & Full Test Suites Verification | Comprehensive execution of all test suites, adversarial stress tests, and forensic audit | M3 | DONE |

## Interface Contracts
### Web UI ↔ car/main.py (DiagnosticHandler)
- `GET /manual_drive?cmd=<forward|backward|left|right|stop>&speed=<130|185|235>`: Emits motor control commands to Arduino at selected speed.
- `GET /toggle_swap_rb`: Toggles `swap_rb` boolean in `SystemState`. Returns JSON `{"status": "ok", "swap_rb": true/false}`.
- `GET /telemetry`: Returns vehicle telemetry JSON including `{"swap_rb": bool, "speed": int, "state": str, ...}`.

### car/main.py ↔ Arduino Uno (PROTOCOL.md)
- `TURN <angle> <speed>\n`: e.g., `TURN 45 185`, `TURN -45 185`. Returns `ACK` or telemetry lines.
- `DRIVE <speed> <duration_ms>\n`: e.g., `DRIVE 185 220`.
- `STOP\n`: Immediate motor halt.
