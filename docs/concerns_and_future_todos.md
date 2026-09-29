# Architectural Concerns & Future Roadmap (TODOs)

**Document Status**: ACTIVE / AUTHORITATIVE  
**Date**: 2026-09-29  
**Source**: Architecture Brainstorming Review (`docs/architecture.md` Stress Test)  
**Owners**: Krishna (Lead / Hardware / Firmware), Aditya (Hardware / Chassis), Parth (Software / Perception)

---

## 1. Executive Summary

During the systematic brainstorming and stress-testing of [docs/architecture.md](file:///e:/OneDrive%20-%20Shri%20Vile%20Parle%20Kelavani%20Mandal/Capstone/model/docs/architecture.md), nine key architectural concerns, edge cases, and design gaps were identified. Each concern was analyzed with engineering trade-offs, and clear resolutions or future hardware/integration TODOs were established.

This document serves as the permanent record of these design concerns, their mitigations, and the master TODO list for bench and track testing.

---

## 2. Architectural Design Concerns & Decisions

### Concern 1: Avoidance Re-Trigger Loop during Resume Phase
- **Context**: In Tier 2 ultrasonic avoidance, the state machine executes three phases: Phase 1 (Reverse 400ms) → Phase 2 (Skid-turn 500ms away from obstacle) → Phase 3 (Resume forward 2000ms at saved drive speed) → `IDLE`.
- **The Problem**: Ultrasonics poll continuously every 100ms. If the 500ms skid-turn was insufficient to fully clear a wide barrier or wall corner, the ultrasonic sensors will immediately register `< 20cm` during Phase 3, triggering avoidance anew. This creates an infinite `Reverse → Turn → Resume → Detect → Reverse...` loop.
- **Resolution**:
  - Introduce an `AVOID_COOLDOWN_MS` constant (default **500ms**) in firmware.
  - After Phase 2 (Turn) completes and Phase 3 (Resume) initiates, ultrasonic obstacle re-triggering is suppressed for `AVOID_COOLDOWN_MS`.
  - This guarantees the car drives clear of the obstacle boundary before reflex avoidance can re-arm.

---

### Concern 2: TURN Angle-to-Duration Linear Mapping is Uncalibrated
- **Context**: The serial protocol specifies `TURN <angle> <speed>`. In the initial Arduino code, turn duration was computed via an arbitrary linear formula: `duration_ms = abs(angle) * 10`.
- **The Problem**: On a 4WD acrylic skid-steer chassis with DC TT gearmotors, physical rotational degrees are non-linear and heavily influenced by ground friction, tire grip, chassis weight distribution, and 2S battery discharge voltage. `abs(angle) * 10` is an unverified placeholder.
- **Resolution & Future TODO**:
  - Keep `TURN <angle> <speed>` command format in `PROTOCOL.md` (no API breakage).
  - Add a named calibration constant `TURN_MS_PER_DEGREE` in [`arduino/lib/protocol.h`](file:///e:/OneDrive%20-%20Shri%20Vile%20Parle%20Kelavani%20Mandal/Capstone/model/arduino/lib/protocol.h).
  - **TODO**: Calibrate this parameter empirically on real hardware across carpet and tile surfaces once the 4-wire PWM migration is complete.

---

### Concern 3: Follow-Me Steering Disruption During Tier 2 Avoidance & IMU Heading Recovery
- **Context**: In Follow-Me mode, the Pi 5 vision loop sends continuous steering corrections (~30Hz) toward the tracked person. When an obstacle enters the 20cm zone, Arduino reflex Tier 2 avoidance overrides Tier 3 motor commands for ~2.9 seconds.
- **The Problem**:
  1. The Arduino silently drops Tier 3 commands while in `STATE_AVOIDING` (current design does not queue commands).
  2. After completing avoidance, the car points in a completely new heading, and the Pi's PID loop would suddenly be facing the wrong direction without awareness of how much the car rotated.
- **Resolution & Future TODO**:
  - Tier 3 commands sent during avoidance will continue to be dropped (clean replace semantics; no queue).
  - Pi 5 must parse `[TELEM] AVOID <DIR>` and pause its PID tracker while avoidance is in progress.
  - **TODO (IMU Heading Compensation)**: The car hardware bill of materials includes an **ICM20948 9-DOF IMU** over I2C to the Pi. When avoidance finishes (`[TELEM] DONE AVOID`), the Pi reads the IMU heading delta accumulated during the maneuver and applies a heading correction to immediately re-orient toward the target before resuming visual KCF tracking.

---

### Concern 4: Pi 5 Crash / Serial Disconnect & Watchdog Fail-Safe
- **Context**: The two-layer architecture separates real-time motor control (Uno) from heavy neural inference (Pi 5).
- **The Problem**: If the Pi 5 freezes under heavy YOLOv10n load, crashes, or the USB serial cable disconnects, what prevents the car from running away or getting stuck in an undefined state?
- **Resolution**:
  - All Tier 3 motion commands (`DRIVE`, `REV`, `TURN`) are strictly **bounded-duration**; the car naturally halts when the command duration expires.
  - Add a dedicated **Serial Watchdog** in the Arduino firmware:
    - Variable `lastPiCommandTime` updated on any valid command or `PING`.
    - If `millis() - lastPiCommandTime > WATCHDOG_TIMEOUT_MS` (default **5000ms**):
      1. Immediate transition to `STATE_IDLE` (all PWM set to 0).
      2. Periodic broadcast of telemetry alert: `[TELEM] HEARTBEAT_TIMEOUT`.
    - When the Pi recovers, it detects `HEARTBEAT_TIMEOUT` in the telemetry stream and resets its communication handshake cleanly.

---

### Concern 5: Continuous 30Hz PID Steering vs. Discrete Timed `TURN` Commands
- **Context**: The PID controller in `car/control/pid.py` computes an angular error at the camera frame rate (~30Hz, every ~33ms).
- **The Problem**: Sending `TURN <angle> <speed>` with a duration of 200ms every 33ms means every incoming command resets the Arduino's turning timer before it ever completes. This can cause jerky, stuttered motion where motor states restart continuously.
- **Resolution & Future TODO**:
  - Acknowledge this impedance mismatch between continuous control and discrete timed commands.
  - **TODO**: Evaluate three candidates during on-chassis integration testing:
    1. *Option A (Differential Drive)*: Add `DRIVE_DIFF <left_pwm> <right_pwm> <duration_ms>` so PID directly modulates wheel speeds.
    2. *Option B (Micro-Pulses)*: PID issues short sub-cycle turns (e.g., 30–40ms duration) that naturally match the frame interval.
    3. *Option C (Continuous Steer Command)*: Introduce a dedicated `STEER <left_speed> <right_speed>` command with short watchdog timeouts designed specifically for streaming PID setpoints.

---

### Concern 6: Protocol Header Sync & Missing Constants
- **Context**: The architecture spec guarantees that all avoidance parameters are configurable via named constants in `protocol.h`.
- **The Problem**: `AVOID_RESUME_MS` and `AVOID_COOLDOWN_MS` were specified in documentation but missing from [`arduino/lib/protocol.h`](file:///e:/OneDrive%20-%20Shri%20Vile%20Parle%20Kelavani%20Mandal/Capstone/model/arduino/lib/protocol.h).
- **Resolution**:
  - Update `protocol.h` to declare:
    - `#define AVOID_REVERSE_MS 400`
    - `#define AVOID_TURN_MS 500`
    - `#define AVOID_RESUME_MS 2000`
    - `#define AVOID_COOLDOWN_MS 500`
    - `#define WATCHDOG_TIMEOUT_MS 5000`

---

### Concern 7: Follow-Me Re-ID False Match Risks with Similar Clothing
- **Context**: The owner profile uses HSV color histograms (top 40% upper body, bottom 40% lower body, aspect ratio) for fast re-identification when KCF loses lock.
- **The Problem**: If a bystander wears clothing of similar color hue, histogram correlation (`cv2.compareHist`) may score above threshold, causing the car to follow the wrong individual.
- **Resolution**:
  - This is an accepted design constraint to keep latency within the Pi 5's 30 FPS budget (neural Re-ID models like OSNet are too computationally heavy alongside YOLOv10n).
  - Configurable similarity threshold `REID_SIMILARITY_THRESHOLD` (baseline **0.80**).
  - If no detected bounding box exceeds the correlation threshold, the car refuses to lock on, halts in `IDLE`, and waits for the genuine owner to re-enter view.

---

### Concern 8: Autonomous Mode Waypoint Definition
- **Context**: The dual-mode architecture specifies Autonomous Navigation as Mode 1 and Follow-Me as Mode 2.
- **The Problem**: While Follow-Me has a full pipeline (lock-on → KCF → PID → re-ID), Autonomous Navigation is currently defined as an open abstraction ("drive forward / waypoints and avoid obstacles").
- **Resolution & Future TODO**:
  - Follow-Me mode is the primary deliverable for the capstone demonstration.
  - Autonomous navigation with coordinate waypoints, dead-reckoning integration, and path planning is formally classified as **Future Scope (Phase 2)**.
  - For baseline autonomous testing, the car will operate in "reactive cruise" mode (drive forward at set speed, obstacle avoidance reflexes enabled).

---

### Concern 9: Serial Buffer Overflow Risk Under Blocking Ultrasonic Reads
- **Context**: The Arduino Uno has a 64-byte hardware UART receive buffer. At 115200 baud, a 16-byte command arrives in ~1.4ms.
- **The Problem**: Arduino `readUltrasonicCm()` uses `pulseIn()` with a 30,000µs (30ms) timeout. If both left and right sensors timeout (no echo received), the Arduino blocks for up to ~60ms. During this interval, multiple rapid PID commands from the Pi could overflow the 64-byte UART buffer.
- **Resolution**:
  - Noted design observation.
  - If serial buffer overflow is observed in testing:
    1. Reduce `US_TIMEOUT_US` from 30000µs to 12000µs (sufficient for ~200cm range; beyond obstacle threshold).
    2. Interleave ultrasonic polling (ping left sensor on odd ticks, right sensor on even ticks).

---

## 3. Master Future TODO List

| Category | Item | Description | Priority | Owner |
|---|---|---|:---:|---|
| **Hardware** | **4-Wire PWM Migration** | Move ENA/ENB jumpers to D5/D6; move Echo Left/Right to A0/A1. | 🔴 Blocker | Krishna |
| **Hardware** | **Scrap IR Sensors** | Remove 2× IR sensors to reduce Berg bus clutter and free D2/D3 digital pins. | 🟡 High | Aditya |
| **Hardware** | **Rear Ultrasonic Evaluation** | Mount 1× HC-SR04P facing backward using freed D2/D3 pins to eliminate reverse blind spot. | 🟢 Medium | Aditya / Krishna |
| **Hardware** | **Nano Migration** | Swap Uno for Arduino Nano (identical ATmega328P pinout, smaller footprint). | 🔵 Future | Krishna |
| **Firmware** | **Resolve Merge Conflict** | Fix `<<<<<<< HEAD` merge conflict markers at line 398 in `point_to_point_control.ino`. | 🔴 Blocker | firmware-agent |
| **Firmware** | **Add Watchdog & Constants** | Implement `WATCHDOG_TIMEOUT_MS`, `AVOID_COOLDOWN_MS`, and `AVOID_RESUME_MS` in `protocol.h` and `.ino`. | 🟡 High | firmware-agent |
| **Firmware** | **Calibrate Turn Constant** | Empirically calibrate `TURN_MS_PER_DEGREE` on physical chassis across battery voltage levels. | 🟡 High | firmware-agent |
| **Perception** | **Align Telemetry Parser** | Update `sensors.py` and `config.py` from stale 3-sonar center format to 2-sensor `US LEFT=xx RIGHT=yy`. | 🔴 Blocker | perception-agent |
| **Perception** | **Calibrate Distance Bboxes** | Test camera bounding box height % (15% far, 40% close) against physical meter distances. | 🟡 High | perception-agent |
| **Perception** | **Implement Owner Profile** | Build HSV upper/lower body feature extraction and correlation matcher in `tracker.py`. | 🟡 High | perception-agent |
| **Integration** | **PID Steering vs TURN Model** | Benchmark continuous PID setpoints on real chassis; decide between Differential DRIVE vs Micro-TURNs. | 🟡 High | integration-agent |
| **Integration** | **IMU Post-Avoidance Recovery** | Integrate ICM20948 I2C telemetry on Pi to compute heading delta after Tier 2 avoidance. | 🟢 Medium | integration-agent |
| **Integration** | **Autonomous Waypoint FSM** | Define waypoint data structure and dead-reckoning trajectory planner for autonomous mode. | 🔵 Future | integration-agent |

---

## 4. Cross-References

- **System Architecture**: [architecture.md](architecture.md)
- **Serial Protocol Definition**: [../arduino/PROTOCOL.md](../arduino/PROTOCOL.md)
- **Hardware Wiring Baseline**: [../arduino_rf_robot_pin_configuration_wiring_guide.md](../arduino_rf_robot_pin_configuration_wiring_guide.md)
- **Behavioral Scenarios**: [scenarios/README.md](scenarios/README.md)
