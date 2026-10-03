# System Architecture — Autonomous Follow-Me and Obstacle-Avoidant RC Vehicle

**Status:** APPROVED / PRODUCTION BASELINE  
**Last Updated:** 2026-10-02  
**Target Hardware:** Raspberry Pi 5 (8GB) + Arduino Uno R3 (ATmega328P)  
**Primary Repositories:** `model-car-capstone` / `MentorPi`  
**Active Branch:** `pothole-detection` / `main`

---

## Executive Summary

This document specifies the canonical system architecture for the autonomous, two-layer robotic model car developed for the B.Tech Capstone Project. The vehicle operates in two distinct operational regimes:
1. **Autonomous Navigation Mode:** Cruising, dynamic obstacle avoidance, and camera-based negative obstacle (pothole/cavity) evasion.
2. **Follow-Me Mode:** Target-locking, real-time person tracking via visual servoing, HSV appearance-based re-identification, and reflexive collision prevention.

To overcome the fundamental failure modes of monolithic, single-board robotics systems—namely Linux kernel preemption, software GPIO PWM jitter, and brownout vulnerability under heavy deep learning workloads—the architecture strictly partitions tasks into a **Cognitive High-Level Brain** (Raspberry Pi 5) and a **Reflexive Hard Real-Time Spine** (Arduino Uno ATmega328P).

```
+---------------------------------------------------------------------------------------+
|                                    SYSTEM TOPOLOGY                                    |
+---------------------------------------------------------------------------------------+
|                                                                                       |
|   +----------------------------------+             +------------------------------+   |
|   |   Raspberry Pi 5 (High-Level)    |             |  Arduino Uno (Real-Time)     |   |
|   |----------------------------------|             |------------------------------|   |
|   | • CSI Global-Shutter Camera      |  USB-Serial | • Hardware Timer0/1/2 PWM    |   |
|   | • YOLOv10n (NCNN NMS-Free)       |  115200 8N1 | • 3-Tier Priority FSM        |   |
|   | • RDD2022 Pothole YOLO / CV      |<----------->| • 2x HC-SR04P Sonar Polling  |   |
|   | • HSV Appearance Re-ID Tracker   |             | • L298N Dual H-Bridge Driver |   |
|   | • Discrete PID Steering Engine   |             | • Autonomous Emergency Reflex|   |
|   +----------------------------------+             +------------------------------+   |
|                    ^                                               ^                  |
|                    | Isolated 5.1V/5A                              | 7.4V -> 5V/5A    |
|   +----------------------------------+             +------------------------------+   |
|   | boAt EnergyShroom PB400 Battery  |             | 2S 18650 Li-Ion Pack + Buck  |   |
|   +----------------------------------+             +------------------------------+   |
|                                                                                       |
+---------------------------------------------------------------------------------------+
```

---

## 1. Architectural Philosophy & The Two-Layer Split

### 1.1 The Single-Board Failure Mode (The Linux OS Problem)
Conventional edge AI mobile robots (e.g., NVIDIA JetBot, DonkeyCar) run both high-level neural inference and low-level motor pulsing on a single Linux system-on-chip. This design suffers from three critical failure modes:
1. **Software PWM Timing Jitter:** The Linux Completely Fair Scheduler (CFS) provides no deterministic real-time execution guarantees. Under heavy neural inference (such as YOLOv10 or MobileNet), CPU interrupt latency surges from $<50\,\mu\text{s}$ to $>30\,\text{ms}$. Pulses generated across software GPIO pins jitter violently, causing motor shudder, differential wheel slip, and erratic skid-steer heading errors.
2. **Safety Reflex Preemption:** When an obstacle appears within $15\,\text{cm}$, a single-board architecture must process the image frame, run bounding box prediction, score threat zones, and issue GPIO signals. This pipeline introduces $60\text{–}120\,\text{ms}$ of latency. At a cruising speed of $1.5\,\text{m/s}$, the chassis moves $18\,\text{cm}$ before braking initiates, guaranteeing a physical collision.
3. **OS Crash / Hang Vulnerability:** In the event of a Linux kernel panic, out-of-memory (OOM) freeze, or Python process crash, GPIO pins latch in their last active logic state. A car driving forward will continue driving at full speed into walls, stairs, or human bystanders.

### 1.2 The Two-Layer Solution
Our architecture enforces strict physical and functional isolation:

| Engineering Parameter | Single-Board Architecture (JetBot / DonkeyCar) | Our Two-Layer Architecture (Pi 5 + Arduino Uno) |
|---|---|---|
| **Motor PWM Generation** | Software GPIO (jittery under load, $\pm 25\,\text{ms}$) | Hardware 8-bit Timer0/Timer1/Timer2 (rock-solid, $< 4\,\mu\text{s}$) |
| **Ultrasonic Echo Timing** | User-space polling (unreliable, pulse truncation) | Dedicated microcontroller `pulseIn()` timing ($1\,\mu\text{s}$ precision) |
| **Emergency Reflex Loop** | Soft real-time (60–120 ms round trip) | Hard real-time Tier 2 reflex ($< 10\,\text{ms}$ autonomous trigger) |
| **Crash Protection** | Motors latch at previous state on crash | Hardware Watchdog halts all motors if Pi heartbeat lost ($> 500\,\text{ms}$) |
| **Electrical Stability** | Motor back-EMF couples directly into SBC power rail | Dual-rail isolated supplies eliminate brownout resets |
| **BOM Cost** | $\$250\text{–}\$350$ (expensive single boards / carrier kits) | $\sim \$120\text{–}\$150$ (accessible, modular academic components) |

---

## 2. Hardware Architecture & Electrical Design

### 2.1 Complete Pin Mapping Matrix
> **Hardware Status: PWM MIGRATION COMPLETE (Verified 2026-09-30).**  
> ENA and ENB are wired directly to Arduino PWM pins D5 and D6. Jumper clips to 5V have been removed. Variable speed control, PID modulation, and differential turning are fully operational.

Authoritative pin definitions are maintained in [`arduino/lib/pins.h`](../arduino/lib/pins.h):

| Arduino Uno Pin | Direction | Assigned Function | Connected Hardware Subsystem | Hardware Timer / Characteristics |
|:---:|:---:|---|---|---|
| **D2** | INPUT | IR Proximity Sensor 1 | Front Left IR Sensor | Digital logic input (HIGH/LOW) |
| **D3** | INPUT | IR Proximity Sensor 2 | Front Right IR Sensor | Interrupt-capable pin (`INT1`) |
| **D4** | INPUT | Ultrasonic 1 Echo | HC-SR04P Front-Left Sensor | 3.3V/5V compatible echo timing |
| **D5** | OUTPUT | **ENA (Left Motor PWM)** | L298N Dual H-Bridge Channel A | **Timer0 (OC0B)** — 8-bit PWM (0–255, 980 Hz) |
| **D6** | OUTPUT | **ENB (Right Motor PWM)** | L298N Dual H-Bridge Channel B | **Timer0 (OC0A)** — 8-bit PWM (0–255, 980 Hz) |
| **D7** | OUTPUT | Ultrasonic 2 Trigger | HC-SR04P Front-Right Sensor | $10\,\mu\text{s}$ digital trigger pulse |
| **D8** | OUTPUT | Motor IN1 (Left Dir 1) | L298N Dual H-Bridge Channel A | Direction bit 1 (HIGH = Forward) |
| **D9** | OUTPUT | Motor IN2 (Left Dir 2) | L298N Dual H-Bridge Channel A | Direction bit 2 (HIGH = Reverse) |
| **D10** | OUTPUT | Motor IN3 (Right Dir 1) | L298N Dual H-Bridge Channel B | Direction bit 1 (HIGH = Forward) |
| **D11** | OUTPUT | Motor IN4 (Right Dir 2) | L298N Dual H-Bridge Channel B | Direction bit 2 (HIGH = Reverse) |
| **D12** | OUTPUT | **Ultrasonic 1 Trigger** | HC-SR04P Front-Left Sensor | Moved from D5 during PWM migration |
| **D13** | INPUT | **Ultrasonic 2 Echo** | HC-SR04P Front-Right Sensor | Moved from D6 during PWM migration |

### 2.2 Dual-Rail Isolated Power Distribution
To prevent inductive noise and voltage drops from triggering Raspberry Pi brownouts, the power system is partitioned into two physically isolated rails:

```
  MOTOR RAIL (Shared Ground)                         PI 5 RAIL (Isolated)
  ==========================                         ====================

  +-------------------------------+                 +-----------------------------------+
  | 2S 18650 Li-Ion Battery Pack  |                 | boAt EnergyShroom PB400 Powerbank |
  |   7.4V Nominal (1500-2500mAh) |                 |   USB-C 5.1V / 5.0A Output        |
  +---------------+---------------+                 +-----------------+-----------------+
                  |                                                   |
                  v                                                   v
  +---------------+---------------+                 +-----------------+-----------------+
  | LM2596 / XL4015 Buck Converter|                 | Raspberry Pi 5 (8GB)              |
  |   Step-Down 7.4V -> 5.0V / 5A |                 | Broadcom BCM2712 Quad-Core A76    |
  +-------+---------------+-------+                 +-----------------+-----------------+
          |               |                                           |
          v               v                                           | USB-A to USB-B
     +----+----+    +-----+---------+                                 | Serial Cable
     |  L298N  |    |  Arduino Uno  |                                 | (Data + Common GND)
     | Motor   |    |  ATmega328P   |<--------------------------------+
     | Driver  |    +-------+-------+
     +----+----+            |
          |                 v
          |         +-------+-------+
          |         | 16-Pin Berg   |
          |         | Strip Header  |
          |         +-------+-------+
          |                 |
          |       +---------+---------+
          |       |         |         |
          v       v         v         v
     4x TT DC   2x HC-    2x IR     L298N
      Motors    SR04P    Sensors    Logic
```

* **Common Ground Principle:** The two power systems share a common ground reference exclusively through the USB-Serial cable shield to preserve logic integrity while preventing ground loops.
* **16-Pin Berg Strip Header:** Provides organized distribution of the regulated 5V bus:
  * Pins 1–8: Common +5V Rail (Arduino 5V, 2x Sonar VCC, 2x IR VCC, L298N Logic VCC, 2x Spares).
  * Pins 9–16: Common GND Rail (Arduino GND, 2x Sonar GND, 2x IR GND, L298N GND, 2x Spares).

### 2.3 Sensor Array & Spatial Field-of-View (FOV)
The chassis mounts an overlapping multi-sensor array:

```
                          FRONT OF CHASSIS
               ______________________________________
              |                                      |
              |   [US1: Left]          [US2: Right]  |
              |      \                    /          |   US1: 15° Left Divergence
              |       \   Forward Cone   /           |   US2: 15° Right Divergence
              |        \   Overlapping  /            |   Combined Cone: ~60°
              |         \              /             |   Threshold: < 20.0 cm
              |          \            /              |
              |           \          /               |
              |            \        /                |
              |             \      /                 |
              |  [IR1]        [CAM]        [IR2]     |   CAM: CSI Camera (120° Wide FOV)
              |               (CSI)                  |   IR1, IR2: Ground perimeter
              |                                      |
              |  [FL Wheel]          [FR Wheel]      |   Skid-Steer TT DC Motors
              |  [RL Wheel]          [RR Wheel]      |   Left Pair: ENA / IN1 / IN2
              |______________________________________|   Right Pair: ENB / IN3 / IN4

                          REAR OF CHASSIS
```

* **HC-SR04P Ultrasonic Transceivers (x2):** Splayed laterally by $15^\circ$ to cover corner obstacles. Effective ranging: $2\,\text{cm}$ to $300\,\text{cm}$. The reflex threshold is parameterized at $20.0\,\text{cm}$ (`OBSTACLE_THRESHOLD_CM` in `protocol.h`).
* **CSI Monocular Camera:** Wide-angle $120^\circ$ FOV camera mounted $12\,\text{cm}$ above the ground plane, angled downwards at $15^\circ$ pitch to cover both horizon targets and close road surface ($0.3\text{–}4.0\,\text{m}$).
* **Sensor Blind Spot Coverage:** Horizontal ultrasonic sensors cannot detect road depressions or potholes. Conversely, monocular vision struggles with transparent glass or untextured surfaces at $<10\,\text{cm}$. The multi-modal sensor fusion completely eliminates mutual blind spots.

---

## 3. Firmware Architecture & 3-Tier Reflex State Machine

The Arduino Uno firmware (`arduino/point_to_point_control/point_to_point_control.ino`) executes a deterministic, non-blocking finite state machine at a fixed $100\,\text{Hz}$ loop rate ($10\,\text{ms}$ cycle).

```mermaid
stateDiagram-v2
    [*] --> STATE_IDLE : Power On / [TELEM] READY

    STATE_IDLE --> STATE_DRIVING : DRIVE cmd
    STATE_IDLE --> STATE_TURNING : TURN cmd
    STATE_IDLE --> STATE_REVERSING : REV cmd

    STATE_DRIVING --> STATE_IDLE : Timer Expires / [TELEM] DONE DRIVE
    STATE_TURNING --> STATE_IDLE : Timer Expires / [TELEM] DONE TURN
    STATE_REVERSING --> STATE_IDLE : Timer Expires / [TELEM] DONE REV

    STATE_DRIVING --> STATE_AVOIDING : US reading < 20cm (Tier 2 Reflex)
    STATE_TURNING --> STATE_AVOIDING : US reading < 20cm (Tier 2 Reflex)
    STATE_REVERSING --> STATE_AVOIDING : US reading < 20cm (Tier 2 Reflex)

    STATE_AVOIDING --> STATE_IDLE : Avoidance Sequence Done / [TELEM] DONE AVOID

    STATE_DRIVING --> STATE_PI_STOPPED : PI_STOP (Tier 1 Latch)
    STATE_TURNING --> STATE_PI_STOPPED : PI_STOP (Tier 1 Latch)
    STATE_REVERSING --> STATE_PI_STOPPED : PI_STOP (Tier 1 Latch)
    STATE_AVOIDING --> STATE_PI_STOPPED : PI_STOP (Tier 1 Latch)
    STATE_IDLE --> STATE_PI_STOPPED : PI_STOP (Tier 1 Latch)

    STATE_PI_STOPPED --> STATE_IDLE : PI_GO (Latch Released)
```

### 3.1 Priority Tiers

#### Tier 1: Emergency Latches & Watchdog (Absolute Highest Priority)
* **`PI_STOP`:** Instantaneous latch. Forces `ENA=0`, `ENB=0`, and transitions to `STATE_PI_STOPPED`. All ongoing actions (including mid-maneuver Tier 2 avoidance) are aborted. The vehicle remains locked until a valid `PI_GO` command is received.
* **Watchdog Failsafe:** The firmware monitors incoming serial timestamps. If no valid packet is received within $5000\,\text{ms}$ (`WATCHDOG_TIMEOUT_MS`), the car halts all motors and emits `[TELEM] HEARTBEAT_TIMEOUT`.
* **Command Blocking:** Any Tier 3 movement command received during `STATE_PI_STOPPED` is rejected with `[ACK] BLOCKED (PI_STOPPED)`.

#### Tier 2: Hard Real-Time Local Avoidance Reflex (Autonomous Microcontroller Loop)
Triggered whenever either front ultrasonic reads $< 20.0\,\text{cm}$ during forward motion, turning, or reversing. Operates independently of the Raspberry Pi:
1. **Phase 1 — Reverse (`AVOID_REVERSE_MS = 400ms`):** Reverses both motors at PWM 150 to clear the collision zone.
2. **Phase 2 — Skid-Turn (`AVOID_TURN_MS = 500ms`):** Turns away from the detected obstacle at PWM 150:
   * Left sensor triggered $\rightarrow$ Turn **Right** (Left FWD, Right REV).
   * Right sensor triggered $\rightarrow$ Turn **Left** (Left REV, Right FWD).
   * Both sensors triggered $\rightarrow$ Check stored `PI_AVOID` hint. If none, default Right.
   * `PI_AVOID` Hint Override: If the Pi previously sent `PI_AVOID LEFT` or `PI_AVOID RIGHT`, the hint strictly overrides the reflex direction and is consumed.
3. **Phase 3 — Forward Resume (`AVOID_RESUME_MS = 2000ms`):** Resumes forward travel at `savedDriveSpeed`.
4. **Cooldown (`AVOID_COOLDOWN_MS = 500ms`):** Suppresses re-triggering immediately after avoidance to prevent hunting oscillations.

#### Tier 3: High-Level Navigation (Supervised by Raspberry Pi)
Executes timed movement primitives:
* `DRIVE <speed> <duration_ms>`
* `REV <speed> <duration_ms>`
* `TURN <angle_deg> <speed>` (Negative angle = Left, Positive angle = Right)
* `STOP` (Non-latched immediate halt to `STATE_IDLE`)

---

## 4. Perception & Computer Vision Architecture

The perception pipeline running on the Raspberry Pi 5 combines deep neural detection, a classical computer vision fallback for road cavities, and an appearance-based person tracker.

```mermaid
graph TD
    CAM["CSI Camera Ingest<br/>(640x480 @ 30 FPS)"] --> PRE["Preprocessing & ROI Extraction<br/>(Bottom 55% Road Plane)"]

    PRE --> YOLO["YOLOv10n (Tencent NCNN)<br/>Positive Obstacles (Cars, People)"]
    PRE --> POTHOLE["RDD2022 Custom YOLO / Classical CV<br/>Negative Obstacles (Potholes, Cracks)"]

    YOLO --> MERGE["Detection Fusion & Zone Spatial Corridors"]
    POTHOLE --> MERGE

    MERGE --> ZONE["Zone Hazard Scorer<br/>(Left, Center, Right)"]
    ZONE --> DEC["Decision Arbiter Matrix"]

    DEC -->|"EMERGENCY DANGER"| PISTOP["PI_STOP (Latch Car)"]
    DEC -->|"DIRECTIONAL SWERVE"| SWERVE["PI_AVOID + Timed TURN"]
    DEC -->|"CLEAR PATH"| FOLLOW["Follow-Me Visual Servoing"]

    FOLLOW --> KCF["KCF Spatial Tracker"]
    FOLLOW --> HSV["HSV Torso Appearance Re-ID"]
    KCF --> PID["Discrete PID Controller"]
    PID --> NAV["Serial Command Stream<br/>(DRIVE / TURN to Arduino)"]
```

### 4.1 Positive Obstacle Detection: YOLOv10n under Tencent NCNN
* **Architecture:** YOLOv10n eliminates the Non-Maximum Suppression (NMS) post-processing bottleneck by incorporating dual-label assignments during training.
* **Inference Engine:** Tencent NCNN framework with ARM Cortex-A76 NEON FP16 vectorization.
* **Target Classes:** Person (Class 0), Bicycle (1), Car (2), Motorcycle (3), Bus (5), Truck (7).
* **Latency Profile:** $\sim 22\text{–}26\,\text{ms}$ on physical Pi 5 hardware (enabling sustained 30+ FPS pipelines).

### 4.2 Negative Obstacle & Pothole Detection: Dual-Path Engine
Negative obstacles are depressions in the road plane that cannot reflect horizontal acoustic waves. Our system provides two complementary detection layers:

#### Path A: Custom RDD2022 YOLOv8n Deep Model (`pothole_detection/`)
* **Training Dataset:** Road Damage Dataset 2022 (RDD2022 India Sub-dataset, Class D40 Pothole).
* **Weights:** [`pothole_detection/best.pt`](../pothole_detection/best.pt) and [`pothole_detection/rdd2022_yolov8n.pt`](../pothole_detection/rdd2022_yolov8n.pt).
* **Dynamic Corridor & Directional Swerve Module:** Implemented in [`pothole_detection/run_detection.py`](../pothole_detection/run_detection.py):
  * Trapezoidal Perspective ROI isolates the active driving corridor.
  * Zone partitioning: Left Zone ($< 33\%$), Center Danger Corridor ($33\%\text{–}67\%$), Right Zone ($> 67\%$).
  * Automated swerve trajectory selection (`SWERVE_LEFT`, `SWERVE_RIGHT`, `EMERGENCY_STOP`).

#### Path B: Classical Computer Vision Fallback (`cv_detect_potholes`)
Implemented in [`car/perception/detector.py`](../car/perception/detector.py) and [`pothole_detection/negative_obstacle_detection/detector.py`](../pothole_detection/negative_obstacle_detection/detector.py). Runs in $<4\,\text{ms}$ with zero GPU/NPU dependency:
1. **ROI Cropping:** Extracts bottom 55% of the frame (`roi_y = int(h * 0.45)`).
2. **Gaussian Pre-Filtering:** $5\times 5$ Gaussian kernel suppresses high-frequency asphalt noise.
3. **Laplacian Edge Extraction:** Computes second-order spatial derivatives (`cv2.Laplacian(blur, cv2.CV_64F)`). Sharp intensity changes at cavity boundaries yield high response magnitudes.
4. **Morphological Elliptical Structuring:** Applies morphological closing followed by opening with a $13\times 13$ elliptical structuring element (`cv2.MORPH_ELLIPSE`) to solidify cavity regions while rejecting linear surface cracks.
5. **Contour Geometry Screening:**
   * Reject if $\text{Area} < 900\,\text{px}$ (small surface debris).
   * Reject if $\text{Area} > 0.25 \times \text{ROI Area}$ (broad shadows or lighting gradients).
   * Reject if aspect ratio $\text{Width}/\text{Height} \notin [0.25, 5.0]$.

### 4.3 Hazard Zone Danger Scoring
In [`car/perception/avoidance.py`](../car/perception/avoidance.py), bounding boxes are transformed into localized danger scores:

$$\text{Danger} = \text{Confidence} \times W \times \left(0.4 + \min\left(\frac{\text{Area}_{\text{box}}}{\text{Area}_{\text{frame}}}, 0.5\right)\right)$$

* $W = 1.0$ for standard positive obstacles.
* $W = 1.2\text{ to }2.0$ (`POTHOLE_WEIGHT`) for negative obstacles, reflecting the severe hazard of chassis drop-off and high-centering.

### 4.4 Follow-Me Person Tracking & Appearance Re-Identification
Implemented in [`car/perception/tracker.py`](../car/perception/tracker.py):
* **Tracking Subsystem:** OpenCV Kernelized Correlation Filter (KCF) for high-speed frame-to-frame bounding box tracking ($> 60\,\text{FPS}$).
* **HSV Torso Appearance Model:** On initial target lock, a normalized 2D HSV color histogram ($H \times S$, $30 \times 32$ bins) is computed from the upper-torso region of the target bounding box.
* **Re-Acquisition Engine:** If KCF tracking confidence drops below threshold (e.g., target occluded behind a pillar or walked out of frame during sharp pivot):
  1. Car halts motion (`PI_STOP`) to stabilize the camera feed.
  2. YOLOv10n scans all candidates in frame.
  3. The re-ID module evaluates candidate histograms using Bhattacharyya distance:
     $$d(H_1, H_2) = \sqrt{1 - \sum_i \frac{\sqrt{H_1(i) \cdot H_2(i)}}{\sqrt{\sum H_1 \cdot \sum H_2}}}$$
  4. The candidate with the highest similarity exceeding the re-ID threshold is re-locked as the target owner.
* **Discrete PID Visual Servoing:** [`car/control/pid.py`](../car/control/pid.py) modulates steering angle and motor PWM to maintain a $1.2\,\text{m}$ standoff distance:
  * Steering error: Horizontal offset of bounding box center from frame centerline.
  * Distance error: Deviation of bounding box height from calibrated $30\%$ frame height baseline.

---

## 5. Serial Telemetry & Command Protocol

**Authoritative Specification:** [`arduino/PROTOCOL.md`](../arduino/PROTOCOL.md)  
* **Transport:** USB-Serial CDC (`/dev/ttyUSB0` or `/dev/ttyACM0`)  
* **Baud Rate:** `115200 bps`, 8 data bits, no parity, 1 stop bit (`8-N-1`)  
* **Framing:** Plaintext ASCII terminated with newline (`\n`)  
* **Round-Trip Latency:** $\sim 1.2\,\text{ms}$ transaction time

### 5.1 Command Set (Raspberry Pi $\rightarrow$ Arduino)

| Command | Syntax | Parameters | Tier | Functional Description |
|---|---|---|:---:|---|
| **PI_STOP** | `PI_STOP` | None | 1 | Latches vehicle in `STATE_PI_STOPPED`. Forces $PWM=0$. Overrides all states. |
| **PI_GO** | `PI_GO` | None | 1 | Releases `PI_STOP` latch. Returns car to `STATE_IDLE`. |
| **PI_AVOID** | `PI_AVOID <dir>` | `LEFT` or `RIGHT` | 1 | Stores one-shot direction hint for the next Tier 2 reflex avoidance. |
| **DRIVE** | `DRIVE <spd> <ms>` | `spd`: 0–255, `ms`: Duration | 3 | Drives forward at PWM speed for specified duration, then halts to `IDLE`. |
| **REV** | `REV <spd> <ms>` | `spd`: 0–255, `ms`: Duration | 3 | Reverses at PWM speed for specified duration, then halts to `IDLE`. |
| **TURN** | `TURN <deg> <spd>` | `deg`: -180 to +180, `spd`: PWM | 3 | Skid-steer turn by angle ($<0$ Left, $>0$ Right) at specified PWM speed. |
| **STOP** | `STOP` | None | 3 | Immediate non-latched halt to `STATE_IDLE`. |
| **PING** | `PING` | None | 3 | Liveness health check. Returns `[ACK] PONG`. |

### 5.2 Telemetry Packet Set (Arduino $\rightarrow$ Raspberry Pi)

| Message Header | Format Example | Frequency | Functional Description |
|---|---|---|---|
| **[TELEM] READY** | `[TELEM] READY Uno-ATmega328P` | Boot | Emitted upon microcontroller reset and setup completion. |
| **[TELEM] STATE** | `[TELEM] STATE=DRIVING SPEED=180` | On change | Signals FSM state transition and active drive speed. |
| **[TELEM] US** | `[TELEM] US LEFT=24.5 RIGHT=88.2` | Every $100\,\text{ms}$ | Calibrated distance readings from front ultrasonic sensors. |
| **[TELEM] AVOID** | `[TELEM] AVOID RIGHT` | Event | Emitted when Tier 2 reflex initiates an evasive maneuver. |
| **[TELEM] DONE** | `[TELEM] DONE DRIVE` | Event | Confirms completion of a timed movement primitive. |
| **[ACK]** | `[ACK] DRIVE 180 2000` | Immediate | Confirms syntax validity and acceptance of command. |
| **[ACK] BLOCKED** | `[ACK] BLOCKED (PI_STOPPED)` | Immediate | Emitted when command rejected due to active Tier 1 latch. |

---

## 6. System Scenarios & Behavioral Sequence

The system architecture handles complex real-world conditions through 10 formal behavioral scenarios documented in [`docs/scenarios/`](../docs/scenarios/):

### Scenario 05: Negative Obstacle / Pothole Ahead (Camera Detection & Bypass)
* **Situation:** Vehicle is cruising forward. Both ultrasonic sensors read clear ($>90\,\text{cm}$ because horizontal sound waves fly across ground depressions). The Pi camera detects a pothole $1.5\,\text{m}$ ahead.
* **Message Sequence:**
  1. Pi vision detects road cavity via custom RDD2022 YOLO or classical CV.
  2. Pi sends `PI_STOP` $\rightarrow$ Arduino immediately halts motors ($PWM=0$) and acknowledges `[ACK] PI_STOP`.
  3. Pi analyzes Left vs. Right clearance in frame, determining Left is clear.
  4. Pi sends `PI_AVOID LEFT` hint (stored in Arduino memory).
  5. Pi sends `PI_GO` $\rightarrow$ Arduino releases emergency latch to `STATE_IDLE`.
  6. Pi executes bypass sequence: `TURN -60 150` (steer left), waits for `[TELEM] DONE TURN`, sends `DRIVE 150 2000` (clear hole), sends `TURN 60 150` (straighten path), and resumes cruising with `DRIVE 180 5000`.

### Scenario 06: PI_STOP Preemption During Tier 2 Reflex Avoidance
* **Situation:** The car is in the middle of an autonomous Tier 2 ultrasonic evasive maneuver (skid-turning away from a wall) when the Pi camera detects an approaching human or dynamic vehicle in the new heading.
* **Resolution:** `PI_STOP` has absolute priority over Tier 2. The moment `PI_STOP` arrives over serial, the Arduino immediately kills motor PWM, aborts the remaining avoidance phases, and latches into `STATE_PI_STOPPED`. When `PI_GO` is subsequently issued, the car does not resume the aborted avoidance; it enters `STATE_IDLE` with a clean slate.

### Scenario 02: Sudden Physical Obstacle (Tier 2 Ultrasonic Override)
* **Situation:** While executing a high-level `DRIVE` command from the Pi, a low obstacle or reflective surface appears $18\,\text{cm}$ in front of the left sensor.
* **Resolution:** The Arduino firmware interrupts Tier 3 navigation locally within $10\,\text{ms}$, transitions to `STATE_AVOIDING`, reverses $400\,\text{ms}$, turns right $500\,\text{ms}$, and notifies the Pi with `[TELEM] AVOID RIGHT`.

---

## 7. Comparative Benchmarking Summary

Our architecture has been benchmarked against major open-source mobile robotics frameworks:

| Metric | Our Vehicle (Model-Car) | NVIDIA JetBot | DonkeyCar | OpenBot (Stanford/Intel) |
|---|---|---|---|---|
| **Compute Type** | 2-Layer (Pi 5 + Arduino Uno) | Single-Board (Jetson Nano) | Single-Board (Pi 4 + PCA9685) | 2-Layer (Smartphone + Arduino) |
| **BOM Cost** | **$120–$150 (College Sourced)** | $250–$300 | $250–$300 | $50 Body (Excludes $400+ Phone) |
| **Motor PWM Stability** | Hardware Timer (Zero Jitter) | Linux I2C / Kernel Jitter | I2C PWM Servo Driver | Hardware Timer on Arduino |
| **Detection Stack** | YOLOv10n (NMS-Free) + RDD2022 | MobileNet-SSD / ResNet | Behavioral Cloning CNN | MobileNet-SSD (TFLite) |
| **Pothole / Negative Obstacle** | **Native Deep Model & Laplacian CV** | None (Blind to Potholes) | None | None |
| **Follow-Me Capability** | **KCF + HSV Appearance Re-ID** | None | None | MobileNet Person Tracking |
| **Reflex Fallback** | **Autonomous Hard Real-Time FSM** | None (Vision Only) | None | Sonar on Arduino Body |

---

## 8. Directory & Module Map

```
model-car-capstone/
├── AGENTS.md                                    # Multi-agent roles, constraints, and contracts
├── README.md                                    # Repository overview and getting started
├── arduino/
│   ├── PROTOCOL.md                              # Authoritative serial command/telemetry contract
│   ├── lib/
│   │   ├── pins.h                               # Hardware pin assignments (single source of truth)
│   │   └── protocol.h                           # Protocol constants, baud, timing thresholds
│   ├── motor_test/
│   │   ├── motor_test.ino                       # Legacy single-motor test (guarded)
│   │   └── dual_motor_speed_test.ino            # Full dual-motor & PWM ramp diagnostic suite
│   ├── dual_motor_speed_test/                   # Dedicated standalone Arduino IDE sketch folder
│   │   └── dual_motor_speed_test.ino
│   └── point_to_point_control/
│       ├── point_to_point_control.ino           # Production 3-tier FSM firmware
│       ├── pins.h                               # Local copy for Arduino IDE build compatibility
│       └── protocol.h                           # Local copy for Arduino IDE build compatibility
├── car/
│   ├── main.py                                  # Core perception-decision-actuation loop
│   ├── requirements.txt                         # Runtime dependencies (ultralytics, opencv, pyserial)
│   ├── control/
│   │   ├── motor_controller.py                  # Pi-side serial client and command sender
│   │   ├── navigation.py                        # Dead-reckoning and waypoint driving
│   │   └── pid.py                               # Proportional-Integral-Derivative steering controller
│   └── perception/
│       ├── config.py                            # Model paths, camera resolutions, serial port configs
│       ├── detector.py                          # YOLOv10n NCNN wrapper + classical CV pothole detector
│       ├── avoidance.py                         # Danger zone scoring engine (Left/Center/Right)
│       ├── tracker.py                           # KCF tracker + HSV color appearance re-ID
│       └── sensors.py                           # Serial telemetry parser and ultrasonic tracker
├── pothole_detection/                           # Dedicated Pothole & Road Defect Module
│   ├── CONTEXT.md                               # Pothole detection theory, architecture, and HUD specs
│   ├── run_detection.py                         # Directional swerve & perspective corridor inference engine
│   ├── train_pothole_model.py                   # YOLOv8n fine-tuning pipeline on RDD2022
│   ├── prepare_rdd_yolo.py                      # RDD2022 dataset conversion and augmentation script
│   ├── best.pt / rdd2022_yolov8n.pt             # Fine-tuned model weights (India D40 Pothole class)
│   ├── yolov10n.pt                              # Positive obstacle model weights
│   └── runs_rdd/custom_pothole_model/           # Empirical training metrics (PR/F1 curves, confusion matrix)
├── docs/
│   ├── architecture.md                          # THIS DOCUMENT (Canonical System Architecture)
│   ├── concerns_and_future_todos.md             # Edge-case analysis, calibration roadmap
│   ├── decisions/                               # Architecture Decision Records (ADRs)
│   │   └── 0001-usb-serial-vs-gpio-uart.md
│   ├── reports/                                 # Formal Capstone Benchmarking Reports (DOCX + MD)
│   └── scenarios/                               # 10 formal operational behavioral specifications
│       ├── 01_clear_path.md ... 10_rapid_pi_commands.md
│       └── README.md
└── infra/
    ├── SETUP.md                                 # Headless Pi 5 bringup, interfaces (CSI, I2C, Serial)
    └── pi-image/                                # cloud-init Wi-Fi and headless provisioning configs
```

---

## 9. Architecture Decision Records (ADRs)

### ADR-0001: Transport Selection — USB-Serial vs. GPIO-UART
* **Decision:** Adopt **USB-Serial** (`/dev/ttyUSB0` or `/dev/ttyACM0`) over Raspberry Pi 40-pin GPIO UART (`/dev/ttyAMA0`).
* **Rationale:**
  1. GPIO UART requires disabling the Linux serial login shell and Bluetooth overlay (`dtoverlay=disable-bt`), introducing setup fragility across OS updates.
  2. USB-Serial provides native opto-isolated buffering through the ATmega16U2/CH340 chip, protecting the Pi 5's delicate $3.3\,\text{V}$ unbuffered SoC GPIO pins from $5\,\text{V}$ Arduino logic levels and inductive transients.
  3. Standard USB cables eliminate jumper wire disconnections during chassis road vibration.

### ADR-0002: Perception Inference Engine — Tencent NCNN over ONNX Runtime / PyTorch
* **Decision:** Export YOLO models to **Tencent NCNN** for production Pi 5 execution.
* **Rationale:**
  1. NCNN is explicitly hand-optimized for ARM architectures with assembly-level NEON SIMD vectorization.
  2. NCNN requires zero heavy Python framework dependencies (eliminating PyTorch and TorchScript overhead), drastically reducing memory footprint and boot times.
  3. Eliminates memory fragmentation and thread contention with the camera ingest loop.

### ADR-0003: Microcontroller Evolution — Future Nano Migration
* **Decision:** Design all pin assignments and firmware for compatibility with the **Arduino Nano** (ATmega328P).
* **Rationale:** The Arduino Uno R3 and Arduino Nano share the identical ATmega328P microcontroller core, pin count, register mappings, and hardware timers. Migrating to the Nano in future revisions will reduce chassis footprint by $70\%$ with zero firmware modifications.
