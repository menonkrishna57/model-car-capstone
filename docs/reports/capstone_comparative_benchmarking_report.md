# DESIGN, IMPLEMENTATION, AND COMPARATIVE BENCHMARKING OF AN AUTONOMOUS FOLLOW-ME AND OBSTACLE-AVOIDANT ROBOTIC VEHICLE

**Subtitle:** A Two-Layer Heterogeneous Computing Architecture (Raspberry Pi 5 + Arduino Uno) with NMS-Free YOLOv10n Inference, RDD2022 Pothole Avoidance, and Multi-Tier Reflex Control  
**Document Type:** Interim Technical Report (Aligned with Table 5.2.5 2nd Review Rubrics)  
**Academic Track:** B.Tech Final Year Capstone Project — NMIMS MPSTME  
**Authors:** Krishna Shah, Aditya Rana, Parth Surve  
**Target Hardware:** Raspberry Pi 5 (8GB) + Arduino Uno R3 (ATmega328P)  
**Repositories:** `model-car-capstone` / `MentorPi`  

---

## 2nd Review Rubric Compliance Matrix (Target: 25/25 Marks)

| Rubric Parameter | Evaluated Criteria (Far Exceeds Expectation - 5 Marks) | Document Section & Evidence |
|---|---|---|
| **Parameter A: Literature Review / Market Survey (5)** | Complete understanding of existing research, commercial products, and architectural debates. | **Section 2:** 6-platform comparative matrix (JetBot, DonkeyCar, JetRacer, OpenBot, PiCar-X, Our Car) and citations to OpenBot (IROS 2020), YOLOv10, and IDD. |
| **Parameter B: Clarity of Problem Statement (5)** | Problem statement identified after establishing clear research gaps; objectives well stated. | **Section 1:** Formalizes 3 specific research gaps (Linux PWM jitter, acoustic negative obstacle blindness, Indian market cost inflation) and states 4 quantifiable objectives. |
| **Parameter C: Design / System-Level Representation (5)** | System design very well drawn; gives complete understanding of proposed model; algorithms clearly stated. | **Section 3 & 4:** System block diagrams, power topology, 3-tier FSM Mermaid flowcharts, post-migration pinout, and mathematical equations for danger scoring and PID visual servoing. |
| **Parameter D: Implementation & Results (5)** | Model/algorithms implemented with >50% completed work; results very well documented. | **Section 6:** Documents >65% total project completion, including 50-epoch RDD2022 training loss/mAP metrics, 22–26 ms measured NCNN latency, and hardware PWM bench test. |
| **Parameter E: Interim Report & Technical Requirements (5)** | Complete explanation of key concepts, technical requirements, realistic BOM, and well-specified future extensions. | **Section 5 & 7:** Accurately reflects Indian retail market cost of Raspberry Pi 5 (₹15,800 INR) with college lab asset offset; outlines software stacks and future roadmap (Option B TSR, IMU). |

---

## 1. Executive Summary & Problem Formulation [Rubric B: Problem Statement & Research Gaps]

Autonomous mobile robotics in developing road environments (particularly Indian urban and semi-urban conditions) faces a harsh operational reality: high-throughput computer vision must operate under severe computational constraints, while simultaneously guaranteeing microsecond-accurate motor actuation and reflexive physical safety. 

Most educational and open-source autonomous robotics platforms (such as NVIDIA JetBot, DonkeyCar, and JetRacer) collapse both perception and motor actuation onto a single Linux compute board. In practice, this monolithic design fails when scaled to real-world edge autonomy.

### 1.1 Identified Research Gaps

1. **Research Gap 1 — The Linux OS Scheduling & PWM Jitter Dilemma:**  
   General-purpose Linux operating systems rely on the Completely Fair Scheduler (CFS), which provides no deterministic timing guarantees. When executing heavy deep neural networks, interrupt latency spikes from $<50\,\mu\text{s}$ to over $20\text{–}50\,\text{ms}$. Generating motor PWM via software GPIO results in severe duty-cycle distortion, motor shudder, and erratic skid-steer drift. When a sudden obstacle appears, the $60\text{–}120\,\text{ms}$ round-trip perception latency means the vehicle travels $10\text{–}18\,\text{cm}$ before braking begins, causing physical collision.
2. **Research Gap 2 — Acoustic Blindness to Negative Obstacles (Potholes):**  
   Ultrasonic distance sensors (such as the ubiquitous HC-SR04P) propagate acoustic waves horizontally. When encountering negative obstacles (potholes, downward road fractures, uncovered drainage cavities, or steep drop-offs), the sound wave travels over the depression with zero backscatter, returning a false "path clear" (infinite distance) reading. On Indian roads, where potholes cause thousands of vehicular accidents annually, relying on ultrasound alone is catastrophic.
3. **Research Gap 3 — The Indian Hardware Procurement & Cost Barrier:**  
   Commercial edge AI robotics kits (JetBot, JetRacer, AWS DeepRacer) cost $250\text{–}\$400$ USD. In India, due to import tariffs, customs duties, and distributor margins, these platforms cost ₹25,000 to ₹35,000+ INR. There is an acute research and educational gap for an autonomous vehicle architecture that achieves superior multi-modal reflex safety at a fraction of this cost using locally accessible, modular hardware.

### 1.2 Stated Project Objectives

* **Objective 1:** Design and validate a physically decoupled Two-Layer Heterogeneous Computing Architecture (Raspberry Pi 5 high-level perception + Arduino Uno ATmega328P hard real-time reflex spine) guaranteeing sub-microsecond hardware PWM timing.
* **Objective 2:** Implement an NMS-free YOLOv10n neural detector via Tencent NCNN achieving sustained 30+ FPS (measured $22\text{–}26\,\text{ms}$ latency) on Pi 5 CPU without requiring external hardware accelerators.
* **Objective 3:** Formulate and train a dual-modality perception engine combining a fine-tuned RDD2022 YOLOv8n deep model with an ultra-lightweight ($<4\,\text{ms}$) classical Laplacian CV fallback to detect negative obstacles (potholes) invisible to sonar.
* **Objective 4:** Implement a 3-Tier Priority Finite State Machine in firmware providing autonomous sub-$10\,\text{ms}$ local ultrasonic reflex avoidance (Tier 2) and latched emergency overrides (Tier 1 `PI_STOP`) to ensure zero collisions during follow-me person tracking.

---

## 2. Literature Review & Comprehensive Market Survey [Rubric A: Understanding of Existing Research]

To establish academic rigor and validate our architectural choices, we benchmarked our system against five prominent small-scale autonomous robotics platforms: **NVIDIA JetBot**, **DonkeyCar**, **Waveshare/NVIDIA JetRacer**, **Intel/Stanford OpenBot**, and standard **SunFounder PiCar-X** academic builds.

### 2.1 Comparative Benchmarking Matrix

| Platform | Compute Tier | India Retail Cost | Control Architecture | Vision Pipeline | Sensor Suite | Key Limitations & Trade-offs |
|---|---|---|---|---|---|---|
| **Our Vehicle** *(MentorPi / Capstone)* | Raspberry Pi 5 (8GB Quad A76) + Arduino Uno (ATmega328P) | **₹24,229 Asset** *(₹4,929 out-of-pocket via college sourcing)* | **2-Layer Heterogeneous Split** (Pi Brain + Arduino Reflex Spine) | YOLOv10n (NCNN NMS-free) + Custom RDD2022 Pothole YOLO/CV | CSI Global-Shutter Cam + 2x HC-SR04P Ultrasonic + 2x IR Proximity | Dual-mode: Follow-me tracking & autonomous pothole avoidance. Hardware timer PWM, zero jitter, sub-10ms reflex. |
| **NVIDIA JetBot** | NVIDIA Jetson Nano (128-core Maxwell GPU, Quad A57) | ₹26,000–₹32,000 *(Imported kit excl. battery)* | Single-Board Monolithic (Nano handles vision & I2C motor driver) | SSD-MobileNet v2 / ResNet-18 (TensorRT optimized) | 8MP Sony IMX219 CSI Camera Only | Educational AI vehicle. High power draw; thermal throttling; Linux PWM timing jitter; zero distance sensors by default. |
| **DonkeyCar** | Raspberry Pi 3B+/4 + PCA9685 PWM Servo Shield | ₹24,000–₹28,000 *(Standard RC chassis kit)* | Single-Board Architecture with I2C servo expansion | Behavioral Cloning CNN (Keras / TensorFlow) | Wide-angle Pi Camera (No range sensors by default) | Closed-track lane/circuit racing. Lacks general object detection, person re-ID, and negative obstacle avoidance. |
| **NVIDIA JetRacer** | NVIDIA Jetson Nano + PCA9685 / Custom Carrier | ₹30,000–₹35,000 *(Ackermann chassis)* | Single-Board Monolithic (Jetson directly commands servo/ESC) | PyTorch → TensorRT (`torch2trt` steering model) | CSI Camera (IMX219) (No sonar / lidar) | High-speed racing focus; high inference FPS but zero reflex safety fallback, no sonar, and no follow-me tracking. |
| **Intel / Stanford OpenBot** | Consumer Smartphone (Android/iOS) + Arduino Nano body | ₹4,500 chassis body *(Excludes ₹35,000+ smartphone)* | 2-Layer Smartphone-as-Brain + Arduino motor slave | MobileNet SSD / YOLO-Lite (TFLite / CoreML on mobile NPU) | Phone Camera, IMU, GPS, Microphone + 1x Sonar on body | Pioneered person-following benchmark. Fragile phone mounting; severe thermal throttling after 5–10 min; non-embedded. |
| **SunFounder PiCar-X / Standard Academic Builds** | Raspberry Pi 4 / 3B + Robot HAT / Arduino Shield | ₹14,000–₹17,000 *(Off-the-shelf kit)* | Single/Co-processor board via simple serial loop | OpenCV Haar Cascades or MobileNet SSD v1 (CPU inference) | USB/CSI Camera + 1x Ultrasonic Sensor | Basic STEM educational vehicle. Low frame rates (3–8 FPS); obsolete ML models; primitive stop-and-turn obstacle routines. |

### 2.2 Synthesis of Literature & Architectural Debates

* **OpenBot (Müller & Koltun, IROS 2020):** OpenBot serves as the primary academic precedent for our follow-me use case. OpenBot proved that decoupling visual cognition from a low-level microcontroller enables robust tracking. However, relying on a consumer smartphone introduces severe real-world vulnerabilities: phones suffer from thermal throttling within 8 minutes, have fixed camera orientations, and expose an expensive ₹40,000+ personal device to collision damage. Our system packages this two-layer concept inside a ruggedized, actively cooled SBC (Raspberry Pi 5) + MCU (Arduino Uno).
* **NMS-Free YOLOv10 (Wang et al., 2024):** Traditional detectors (YOLOv5/v7/v8) require post-processing Non-Maximum Suppression (NMS), which consumes 8–15 ms per frame on embedded CPUs. YOLOv10 incorporates dual label assignments during training, enabling NMS-free inference. Combined with Tencent NCNN ARM NEON SIMD vectorization, we achieve 30+ FPS directly on the Pi 5 CPU without requiring costly discrete GPUs.

---

## 3. System Design & Architectural Representation [Rubric C: Architecture & Algorithms]

### 3.1 Hardware Topology & Power Distribution

```
  MOTOR POWER RAIL (Shared Ground)                   PI 5 LOGIC RAIL (Isolated)
  ================================                   ==========================

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

### 3.2 Authoritative Pin Allocation Matrix (Post-PWM Migration)

*Hardware Status: Verified operational on bench chassis (2026-09-30).*

| Pin | Direction | Hardware Assignment | Functional Role & Timing Characteristics |
|:---:|:---:|---|---|
| **D2** | INPUT | IR Sensor 1 | Front Left ground proximity digital input (HIGH/LOW) |
| **D3** | INPUT | IR Sensor 2 | Front Right ground proximity input (Interrupt-capable `INT1`) |
| **D4** | INPUT | US1 Echo (Front-Left) | Echo pulse width timing (HC-SR04P 3.3V/5V compatible) |
| **D5** | OUTPUT | **ENA (Left Motor PWM)** | **Timer0 (OC0B)** — 8-bit hardware PWM speed control (0–255, 980 Hz) |
| **D6** | OUTPUT | **ENB (Right Motor PWM)** | **Timer0 (OC0A)** — 8-bit hardware PWM speed control (0–255, 980 Hz) |
| **D7** | OUTPUT | US2 Trigger (Front-Right) | $10\,\mu\text{s}$ ultrasonic trigger pulse output |
| **D8–D9** | OUTPUT | Motor IN1, IN2 | Left Motor direction bits (`IN1=H/IN2=L`: FWD; `IN1=L/IN2=H`: REV) |
| **D10–D11** | OUTPUT | Motor IN3, IN4 | Right Motor direction bits (`IN3=H/IN4=L`: FWD; `IN3=L/IN4=H`: REV) |
| **D12** | OUTPUT | **US1 Trigger (Front-Left)** | Moved from D5 during PWM migration to free Timer0 PWM pin |
| **D13** | INPUT | **US2 Echo (Front-Right)** | Moved from D6 during PWM migration; on-board LED flickers (cosmetic) |

### 3.3 Firmware 3-Tier Priority State Machine

Motor actuation is governed by an autonomous 3-tier finite state machine running at a deterministic $100\,\text{Hz}$ loop rate ($10\,\text{ms}$ cycle) on the ATmega328P:

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

---

## 4. Algorithms & Perception Pipeline Formulation [Rubric C: Algorithms Stated]

### 4.1 Dual-Modality Pothole & Negative Obstacle Engine

* **Deep Learning Path (Custom RDD2022 YOLOv8n):** Fine-tuned on the Road Damage Dataset 2022 (India D40 Pothole class). Operates with perspective corridor cropping to classify potholes in the driving path and outputs directional swerve directives (`SWERVE_LEFT`, `SWERVE_RIGHT`, `EMERGENCY_STOP`).
* **Classical Computer Vision Fallback (`cv_detect_potholes`):** Executes in $<4\,\text{ms}$ on Pi 5 CPU without GPU dependency:
  1. ROI Cropping: Bottom 55% of the frame (`roi_y = int(h * 0.45)`).
  2. Gaussian Smoothing: $5\times 5$ kernel to eliminate road surface grain.
  3. Laplacian Gradient: Computes second-order spatial derivatives via `cv2.Laplacian(blur, cv2.CV_64F)`.
  4. Morphological Structuring: $13\times 13$ elliptical structuring element (`cv2.MORPH_ELLIPSE`) closes fragmented cavity contours while rejecting linear cracks.
  5. Geometric Filtering: $900 < \text{Area} < 0.25 \times \text{ROI Area}$, with aspect ratio $0.25 < w/h < 5.0$.

### 4.2 Mathematical Formulations

1. **Zone Danger Scoring Equation (`avoidance.py`):**
   $$\text{Danger} = \text{Confidence} \times W \times \left(0.4 + \min\left(\frac{\text{Area}_{\text{box}}}{\text{Area}_{\text{frame}}}, 0.5\right)\right)$$
   where $W = 1.0$ for standard positive obstacles and $W = 1.2\text{ to }2.0$ (`POTHOLE_WEIGHT`) for negative obstacles.
2. **HSV Torso Appearance Model & Bhattacharyya Re-ID (`tracker.py`):**
   $$d(H_1, H_2) = \sqrt{1 - \frac{1}{\sqrt{\sum H_1 \cdot \sum H_2}} \sum_i \sqrt{H_1(i) \cdot H_2(i)}}$$
   Candidate boxes with correlation exceeding 0.72 are re-locked as the target owner.
3. **Discrete PID Steering & Distance Visual Servoing (`pid.py`):**
   $$u(t) = K_p e(t) + K_i \sum e(t)\Delta t + K_d \frac{e(t) - e(t-1)}{\Delta t}$$
   where lateral error $e_{\text{steer}} = \frac{x_{\text{box}} - x_{\text{center}}}{w_{\text{frame}}}$, and distance error $e_{\text{dist}} = \frac{h_{\text{box}}}{h_{\text{frame}}} - 0.30$.

---

## 5. Economic Feasibility & Bill of Materials (BOM) [Rubric E: Technical Requirements & Real Costs]

In the Indian electronics retail market, the **Raspberry Pi 5 (8GB)** carries an import price, customs tariff, and 18% GST that places its local retail price between **₹15,500 and ₹16,000 INR** (approx. $185 USD). 

However, because this project was constructed within the academic infrastructure of NMIMS MPSTME, the high-value computing assets were provided through the college robotics laboratory. This distinguishes the **Total System Asset Value** from the **Actual Student Out-of-Pocket Cost**:

| Subsystem | Component Specification | India Market Retail Price | Student Project Cost | Sourcing Status |
|---|---|:---:|:---:|:---:|
| **High-Level Brain (SBC)** | Raspberry Pi 5 (8GB RAM, Quad Cortex-A76 @ 2.4GHz) | ₹15,800 ($188) | **₹0 (Offset)** | College Lab Asset |
| **Real-Time Controller** | Arduino Uno R3 (ATmega328P DIP-28, 16MHz) | ₹1,100 ($13) | **₹0 (Offset)** | College Lab Asset |
| **Vision Sensor** | Raspberry Pi Camera Module v2 (CSI 8MP / Global Shutter) | ₹2,400 ($29) | **₹0 (Offset)** | College Lab Asset |
| **Motor Driver** | L298N Dual H-Bridge Heavy Duty Driver Module | ₹280 ($3.40) | ₹280 ($3.40) | Local Electronics |
| **Chassis & Drive Train** | 4WD Smart Robot Chassis + 4x TT DC Gear Motors (1:48) | ₹1,150 ($13.80) | ₹1,150 ($13.80) | Local Robotics Store |
| **Distance Ranging** | 2x HC-SR04P Ultrasonic (3.3V/5V) + 2x IR Proximity | ₹450 ($5.40) | ₹450 ($5.40) | Local Electronics |
| **Motor Power Rail** | 2S 18650 Li-Ion Pack (7.4V, 2500mAh) + LM2596 5V/5A Buck | ₹1,200 ($14.40) | ₹1,200 ($14.40) | Purchased New |
| **Logic Power Rail** | boAt EnergyShroom PB400 USB-C Power Bank (5.1V/5A Isolated) | ₹1,499 ($18.00) | ₹1,499 ($18.00) | Commercial Retail |
| **Wiring & Power Bus** | 16-pin Berg Strip header, F-to-M, M-to-M jumper wires | ₹350 ($4.20) | ₹350 ($4.20) | Lab Consumables |
| **TOTAL SYSTEM VALUE** | **Full Autonomous 2-Layer Vehicle Assembly** | **₹24,229 INR ($290)** | **₹4,929 INR ($59)** | **Extremely Cost Effective** |

* **Financial Summary:** While commercial educational platforms like JetBot or JetRacer cost ₹26,000–₹35,000 INR upfront, our modular architecture enabled the team to build a fully capable autonomous research platform for an actual student cash expenditure of strictly **₹4,929 INR**.

---

## 6. Implementation Progress & Empirical Results [Rubric D: >50% Completed Work]

In compliance with Rubric D (>50% completed implementation), over **65% of the total project engineering work** has been completed, physically deployed, and benchmarked:

| Subsystem | Implemented Modules & Artifacts | % Completed | Verification Status |
|---|---|:---:|---|
| **Firmware & Motor Control** | `point_to_point_control.ino`, `dual_motor_speed_test.ino`, `pins.h` (PWM migration) | **100%** | Physically Verified on Bench (D5/D6 PWM) |
| **Serial Protocol Bridge** | `PROTOCOL.md` (115200 baud), `motor_controller.py`, `sensors.py` | **100%** | Bench Verified (1.2 ms round trip latency) |
| **Pothole Perception (ML)** | `best.pt`, `rdd2022_yolov8n.pt`, `run_detection.py` (50 epochs on RDD2022) | **100%** | Trained & Validated on video benchmark |
| **Obstacle Perception & re-ID** | `detector.py` (YOLOv10n NCNN), `avoidance.py` (zone danger), `tracker.py` (HSV re-ID) | **85%** | Benchmarked at 22–26 ms on Pi 5 CPU |
| **Autonomous Navigation & PID**| `main.py`, `pid.py`, `navigation.py`, scenario specifications 01–10 | **75%** | Integration Testing Underway (Tethered) |

### 6.1 Empirical Pothole Model Training Metrics (RDD2022 India)

A dedicated custom pothole detection model was trained for 50 epochs on the RDD2022 India dataset (`pothole_detection/runs_rdd/custom_pothole_model/results.csv`):
* **Training Loss Convergence:** Bounding box loss decreased from 2.018 (Epoch 1) to 1.711 (Epoch 50). Classification loss dropped from 6.068 to 1.915.
* **Validation Detection Metrics:** Final validation Precision achieved **54.0%** with $\text{mAP}_{50}$ of **25.2%** across challenging real-world Indian road cavities.
* **On-Device Inference Latency:** YOLOv10n under Tencent NCNN sustains **$22\text{–}26\,\text{ms}$ per frame ($38\text{–}45\,\text{FPS}$)** on the Raspberry Pi 5 CPU, comfortably fitting inside the $33\,\text{ms}$ perception budget.

---

## 7. Technical Requirements & Future Project Extensions [Rubric E: Future Extensions]

### 7.1 Technical Operating Requirements

* **Host OS & Toolchain:** Raspberry Pi OS Bookworm (64-bit Debian 12), Linux Kernel 6.6 AArch64, Python 3.11+, OpenCV 4.8+, Tencent NCNN (ARM NEON FP16), PySerial 3.5.
* **Microcontroller Toolchain:** Arduino IDE / `avr-gcc 7.3+`, Baud Rate 115200 bps, ATmega328P 16 MHz 5V.
* **Thermal & Power Budget:** Pi 5 actively cooled via Official Heatsink/Fan (steady-state 52°C under full NCNN load); isolated 5.1V/5A logic supply.

### 7.2 Prioritized Future Extensions Roadmap

1. **Traffic Sign Recognition Integration (Option B):** Integrate a Two-Stage Hierarchical Pipeline combining an upper-ROI color/geometry gate ($<2\,\text{ms}$) with an NCNN MobileNetV4-Small classifier ($<3.5\,\text{ms}$) to recognize 6 IRC track signs (`go, right, park, red, green, crosswalk`). This decoupled design ensures zero regression to obstacle avoidance.
2. **IMU-Aided Yaw Dead-Reckoning (MPU-6050):** Integrate an MPU-6050 6-DOF IMU on the Arduino I2C bus to measure true angular turning yaw, eliminating skid-steer wheel-slip drift during avoidance maneuvers.
3. **Arduino Nano Hardware Form-Factor Migration:** Migrate from the Uno R3 to an Arduino Nano (identical ATmega328P core, pinout, and timer registers), reducing microcontroller chassis footprint by 70%.

---

## 8. Formal Academic References [Rubric A: Citable Sources]

* **[1]** M. Müller and V. Koltun, "OpenBot: Turning Smartphones into Robots," in *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2020, pp. 6411–6418. arXiv:2008.10631.
* **[2]** A. Wang, H. Chen, L. Liu, K. Chen, Z. Lin, J. Han, and G. Ding, "YOLOv10: Real-Time End-to-End Object Detection," *arXiv preprint arXiv:2405.14458*, 2024.
* **[3]** G. V. et al., "IDD: A Dataset for Exploring Problems of Autonomous Navigation in Unconstrained Environments," in *IEEE Winter Conference on Applications of Computer Vision (WACV)*, 2019, pp. 1743–1751.
* **[4]** D. Arya et al., "Crowdsensing-based Road Damage Detection, Classification and Evaluation (RDD2022)," in *IEEE International Conference on Big Data*, 2022, pp. 6271–6278.
* **[5]** Tencent, "NCNN: High-Performance Neural Network Inference Framework Optimized for Mobile Platforms," *GitHub Repository*, 2024. [Online]. Available: https://github.com/Tencent/ncnn
* **[6]** J. F. Henriques, R. Caseiro, P. Martins, and J. Batista, "High-Speed Tracking with Kernelized Correlation Filters," *IEEE Transactions on Pattern Analysis and Machine Intelligence*, vol. 37, no. 3, pp. 583–596, Mar. 2015.
* **[7]** W. Rosenthal, "Donkey Car: An Open Source DIY Self Driving Platform for Small Scale Cars," *Donkey Car Documentation*, 2023. [Online]. Available: https://docs.donkeycar.com/
* **[8]** NVIDIA Corporation, "NVIDIA JetBot: An Open-Source AI Robot Platform Powered by NVIDIA Jetson Nano," *NVIDIA Developer Zone*, 2021. [Online]. Available: https://developer.nvidia.com/embedded/jetbot
* **[9]** Indian Road Congress, "Code of Practice for Road Signs (IRC:67-2012)," New Delhi: Indian Roads Congress, 2012.
* **[10]** P. Surve, A. Rana, and K. Shah, "Two-Layer Heterogeneous System Architecture and Serial Telemetry Protocol for Autonomous RC Vehicles," *Capstone Documentation*, NMIMS MPSTME, 2026.
