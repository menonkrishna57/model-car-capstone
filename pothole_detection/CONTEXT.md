# CONTEXT.md — Negative Obstacle & Pothole Avoidance System
================================================================================
> **Project:** Autonomous RC Car Capstone Project — Negative Obstacle Perception & Reactive Avoidance  
> **Target Benchmark:** Kaggle India Road Damage Dataset (RDD2022 — `D40` Potholes & Cracks)  
> **Core Vision Model:** YOLOv10n (NMS-Free Dual-Head Real-Time Detector, 3.2M parameters)  
> **Deployment Weight:** [`best.pt`](file:///e:/Pothole%20detection/best.pt) (5.75 MB — 50 Epochs Fine-Tuned on RTX 3050 Laptop GPU)  
> **Primary Script:** [`run_detection.py`](file:///e:/Pothole%20detection/run_detection.py)  
> **Training Engine:** [`train_pothole_model.py`](file:///e:/Pothole%20detection/train_pothole_model.py)  
> **Active Status:** Fully functional, verified on 1080p POV & front dashcam test footage.

---

## 1. System Motivation & Problem Formulation

### 1.1 The Critical Dilemma: Positive vs. Negative Obstacles
In mobile robotics and autonomous vehicle design, obstacles fall into two fundamental physical classes:
1. **Positive Obstacles:** Structures that rise *vertically above* the ground plane (walls, pillars, curbs, other vehicles, pedestrians, debris).
2. **Negative Obstacles:** Geometric depressions, voids, or cavities that drop *below* the drivable surface plane (potholes, road trenches, open manholes, eroded asphalt shoulders, missing bridge expansion slabs).

```
               [ HORIZONTAL ACTIVE SENSORS (Ultrasonic / 2D LiDAR) ]
                                      │
              Sound / Laser Ray ──────┼─────────────────────────► (Travels straight into air)
                                      ▼                          (NO ECHO / NO REFLECTION)
    ┌───────────────┐
    │ AUTONOMOUS RC │═════════════════════════════╗
    │  CAR CHASSIS  │                             ║
    └───────────────┘                             ▼
           │
     [ CAMERA ] ────────────────────────────────► ╲    POTHOLE    ╱ ──► [ CAMERA MUST DETECT ]
      (Down/Ahead)                                 ╲  DEPRESSION ╱
```

### 1.2 Why Active Ranging Sensors (Ultrasonic & LiDAR) Fail
* **Ray Propagation Geometry:** Standard automotive ultrasonic rangefinders (e.g., HC-SR04P) and planar 2D LiDAR units emit pulses along a horizontal cone. When confronted with a pothole, the emitted sound or optical beam propagates straight across the opening into the open air beyond. **No surface echo reflects back**, causing the sensor to report a completely clear path.
* **Failure Consequence:** An autonomous RC car relying exclusively on ultrasonic sensors will drive directly into potholes at cruise speed (PWM 180-255), causing violent wheel entrapment, steering tie-rod fracture, chassis bottoming, and motor stall currents that trip battery protection circuits.
* **The Monocular Vision Imperative:** Downward-and-forward angled optical perception is the **only cost-effective sensor modality capable of identifying negative road surface defects early enough** to execute safe steering evasion maneuvers.

---

## 2. End-to-End System Architecture

The project employs a hierarchical two-tier distributed architecture dividing heavy computer vision perception from deterministic low-level actuator control:

```
┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│                           TIER 1: HIGH-LEVEL VISION & DECISION BRAIN                        │
│                   (Host Laptop / NVIDIA Jetson Nano / Raspberry Pi 5)                       │
│                                                                                             │
│   ┌────────────────────────┐      ┌─────────────────────────┐      ┌────────────────────┐   │
│   │ Monocular Video Feed   │ ───► │ Perspective ROI Engine  │ ───► │ YOLOv10n Semantic  │   │
│   │ (720p/1080p @ 30 FPS)  │      │ (Auto Dashcam / Cabin)  │      │ Inference Engine   │   │
│   └────────────────────────┘      └─────────────────────────┘      └─────────┬──────────┘   │
│                                                                              │              │
│                                                                              ▼              │
│   ┌────────────────────────┐      ┌─────────────────────────┐      ┌────────────────────┐   │
│   │ HUD Telemetry Overlay  │ ◄─── │ Avoidance Matrix Engine │ ◄─── │ Spatial Corridor   │   │
│   │ (Visual Feedback / MP4)│      │ (Risk Evaluation)       │      │ Risk Calculator    │   │
│   └────────────────────────┘      └───────────┬─────────────┘      │ (Left/Center/Right)│   │
│                                               │                    └────────────────────┘   │
└───────────────────────────────────────────────┼─────────────────────────────────────────────┘
                                                │ UART Serial (115200 Baud)
                                                ▼
┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│                      TIER 2: REAL-TIME MOTION & REFLEX CONTROLLER                           │
│                                 (Arduino Uno / ESP32)                                       │
│                                                                                             │
│   ┌────────────────────────┐      ┌─────────────────────────┐      ┌────────────────────┐   │
│   │ Command Serial Parser  │ ───► │ State Machine & Arbiter │ ───► │ Motor & Servo PWM  │   │
│   │ (PI_STOP, TURN, DRIVE) │      │ (Overrides & Failsafes) │      │ Drivers (L298N)    │   │
│   └────────────────────────┘      └───────────▲─────────────┘      └─────────┬──────────┘   │
│                                               │                              │              │
│                                   ┌───────────┴─────────────┐                ▼              │
│                                   │ HC-SR04P Ultrasonics    │      ┌────────────────────┐   │
│                                   │ (Emergency Wall Reflex) │      │ Steering Servo &   │   │
│                                   └─────────────────────────┘      │ Drive Wheels (DC)  │   │
│                                                                    └────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────────────────┘
```

### 2.1 Hardware Specifications
* **Host Perception Processor:**
  * Development / Validation: Intel/AMD Multi-core CPU + NVIDIA GeForce RTX 3050 Laptop GPU (4GB VRAM, CUDA 12.6, TensorRT / FP16 support).
  * Target Embedded Vehicle Unit: Raspberry Pi 5 (8GB) or NVIDIA Jetson Nano / Orin Nano.
* **Microcontroller Unit (MCU):**
  * Arduino Nano / Uno (ATmega328P) or ESP32-WROOM-32.
  * Clock: 16 MHz (ATmega) / 240 MHz (ESP32).
  * Logic Level: 5V (Arduino) or 3.3V (ESP32) with bi-directional logic level converter on UART lines.
* **Drive Train & Actuators:**
  * Steering: SG90 / MG996R Metal Gear Servo ($-60^\circ$ full left to $+60^\circ$ full right; $0^\circ$ center trim).
  * Traction: Dual Brushed DC Motors with 1:48 reduction gearbox.
  * Motor Driver: L298N Dual H-Bridge or TB6612FNG MOSFET Driver.
* **Sensors:**
  * Vision: Front-facing wide-angle HD Camera (1080p, 120° FOV, mounted 12 cm above ground).
  * Ultrasonics: Dual HC-SR04P rangefinders mounted laterally at 45° angles for perimeter clearance.

### 2.2 Serial Communication Protocol (Pi/Host $\leftrightarrow$ Arduino)
Defined in [`negative_obstacle_detection/05_pothole_ahead.md`](file:///e:/Pothole%20detection/negative_obstacle_detection/05_pothole_ahead.md):
* **Baud Rate:** `115200 bps`, 8 data bits, no parity, 1 stop bit (`8-N-1`).
* **Format:** Plaintext ASCII terminated with newline (`\n`).

| Time Offset | Direction | Message String | Functional Purpose |
|---|---|---|---|
| `0 ms` | MCU $\rightarrow$ Host | `[TELEM] STATE=DRIVING SPEED=180` | Telemetry heartbeat from low-level controller. |
| `120 ms` | Host $\rightarrow$ MCU | `PI_STOP` | Vision detects immediate pothole hazard in center corridor. |
| `125 ms` | MCU $\rightarrow$ Host | `[ACK] PI_STOP` | MCU cuts motor PWM to 0; car halts safely. |
| `250 ms` | Host $\rightarrow$ MCU | `PI_AVOID LEFT` | Host registers avoidance route suggestion. |
| `450 ms` | Host $\rightarrow$ MCU | `PI_GO` | Host unlatches emergency stop state. |
| `550 ms` | Host $\rightarrow$ MCU | `TURN -30 150` | Steer wheels $30^\circ$ left at moderate drive speed. |
| `1200 ms` | Host $\rightarrow$ MCU | `DRIVE 150 2000` | Cruise past the pothole cavity for 2000 ms. |
| `3250 ms` | Host $\rightarrow$ MCU | `TURN 30 150` | Counter-steer $30^\circ$ right to re-align vehicle with road center. |
| `3900 ms` | Host $\rightarrow$ MCU | `DRIVE 180 5000` | Resume normal cruising speed (PWM 180). |

---

## 3. Dataset Engineering: RDD2022 India (`D40`)

The detection model was trained and conditioned on the official **Road Damage Dataset 2022 (RDD2022)** India subset.

### 3.1 Raw Dataset Architecture
* **Location:** [`negative_obstacle_detection/train/`](file:///e:/Pothole%20detection/negative_obstacle_detection/train)
* **Total Raw Images:** 1,530 high-resolution camera frames (`India_XXXXXX.jpg`) captured from vehicles on diverse Indian municipal and rural roads under varied lighting.
* **Annotation Standard:** Pascal VOC XML files (`India_XXXXXX.xml`) containing bounding box coordinates `[xmin, ymin, xmax, ymax]` and categorical class names.

### 3.2 Automated VOC-to-YOLO Conversion Pipeline
Utility: [`negative_obstacle_detection/prepare_rdd_yolo.py`](file:///e:/Pothole%20detection/negative_obstacle_detection/prepare_rdd_yolo.py)  
Target Destination: [`negative_obstacle_detection/rdd_yolo_dataset/`](file:///e:/Pothole%20detection/negative_obstacle_detection/rdd_yolo_dataset)

* **Mathematical Coordinate Normalization:**
  $$x_{\text{center}} = \frac{x_{\min} + x_{\max}}{2 \cdot W}, \quad y_{\text{center}} = \frac{y_{\min} + y_{\max}}{2 \cdot H}$$
  $$w = \frac{x_{\max} - x_{\min}}{W}, \quad h = \frac{y_{\max} - y_{\min}}{H}$$
* **Class Mapping & Instance Distribution:**
  * **Class `0`: `pothole_D40`** $\rightarrow$ **3,187 labeled instances** (Primary target of interest)
  * **Class `1`: `longitudinal_crack_D00`** $\rightarrow$ **471 labeled instances**
  * **Class `2`: `transverse_crack_D10`** $\rightarrow$ **23 labeled instances**
  * **Class `3`: `alligator_crack_D20`** $\rightarrow$ **645 labeled instances**
  * **Class `4`: `marking_defect_D44`** $\rightarrow$ **151 labeled instances**
* **Train/Validation Split:**
  * Training Set: **80% (1,224 images)**
  * Validation Set: **20% (306 images)**
* **Ultralytics Config:** [`negative_obstacle_detection/rdd_yolo_dataset/rdd_dataset.yaml`](file:///e:/Pothole%20detection/negative_obstacle_detection/rdd_yolo_dataset/rdd_dataset.yaml)

---

## 4. Deep Learning Model & 50-Epoch GPU Training

### 4.1 Architecture Selection: YOLOv10 Nano (`yolov10n`)
* **NMS-Free Dual Label Assignment:** Conventional object detectors (YOLOv5, YOLOv8) depend on Non-Maximum Suppression (NMS) during inference to filter redundant bounding boxes. NMS has $O(N^2)$ worst-case complexity and introduces unpredictable latency jitter. YOLOv10 employs dual label assignment during training (one-to-many for representation learning, one-to-one for direct prediction), eliminating NMS completely.
* **Edge Compactness:** Total parameters: **3.2 Million**; Model weight size: **5.75 MB**. This ensures ultra-fast forward passes on edge NPUs and CPU fallback modes.

### 4.2 Training Setup & Hardware Configuration
* **Script:** [`train_pothole_model.py`](file:///e:/Pothole%20detection/train_pothole_model.py)
* **Compute Engine:** NVIDIA GeForce RTX 3050 Laptop GPU (4GB GDDR6 VRAM).
* **Software Stack:** Python 3.13, PyTorch with CUDA 12.6 wheels, Ultralytics 8.3+.
* **Hyperparameters:**
  * `epochs`: `50`
  * `batch`: `8` (VRAM footprint ~2.1 GB; guarantees zero OOM failures on 4GB GPUs)
  * `imgsz`: `640`
  * `amp`: `True` (Mixed precision FP16 for acceleration)
  * `device`: `0` (CUDA GPU)
  * `workers`: `4`

### 4.3 Training Progression & Convergence Milestones
Extracted directly from [`runs_rdd/custom_pothole_model/results.csv`](file:///e:/Pothole%20detection/runs_rdd/custom_pothole_model/results.csv):

| Epoch | Cumulative Time (s) | Train Box Loss | Train Cls Loss | Precision | Recall | mAP@50 | mAP@50-95 | Val Box Loss | Val Cls Loss |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **1** | 44.4s | 2.018 | 6.068 | 0.841 | 0.032 | 0.026 | 0.008 | 1.968 | 6.725 |
| **10** | 289.9s | 1.958 | 3.003 | 0.235 | 0.128 | 0.108 | 0.042 | 1.902 | 3.059 |
| **20** | 588.0s | 1.887 | 2.648 | 0.287 | 0.220 | 0.160 | 0.062 | 1.921 | 2.694 |
| **30** | 891.3s | 1.841 | 2.423 | 0.299 | 0.212 | 0.197 | 0.091 | 1.936 | 2.514 |
| **40** | 1194.5s | 1.794 | 2.181 | 0.534 | 0.238 | 0.232 | 0.101 | 1.941 | 2.391 |
| **50** | **1495.3s (~24.9m)** | **1.711** | **1.915** | **0.540** | **0.271** | **0.252** | **0.114** | **1.923** | **2.336** |

* **Analysis:** Classification loss decreased dramatically from $6.07 \rightarrow 1.91$ ($>68\%$ reduction). Mean Average Precision at IoU 0.50 (mAP50) expanded by nearly $10\times$ from $0.026 \rightarrow 0.252$.
* **Final Weights:** Deployed to [`best.pt`](file:///e:/Pothole%20detection/best.pt) in the workspace root.

---

## 5. Computer Vision & Perspective Geometry Engineering

### 5.1 The Interior Cabin & A-Pillar Failure Mode
In preliminary iterations, an experimental classical Laplacian edge detector (`cv_detect_surface_defects`) was coupled with YOLO:
* **The Root Cause:** Classical edge detectors compute second-order intensity spatial derivatives ($\nabla^2 I$). In vehicle interior POV footage (such as [`test_video.mp4`](file:///e:/Pothole%20detection/test_video.mp4)), the dark automotive interior structure (the right A-pillar, steering wheel, and rearview mirror) creates an intense contrast boundary against the bright sky.
* **The Failure:** The classical filter flagged the right A-pillar as a massive road cavity on every single frame, corrupting the steering matrix and forcing permanent `SWERVE LEFT` commands.
* **The Resolution:**
  1. Completely disabled and removed classical Laplacian edge filtering.
  2. Shifted 100% of detection responsibility to the fine-tuned semantic YOLOv10n neural network.
  3. Engineered an intelligent **Auto-Perspective ROI Engine** in [`run_detection.py`](file:///e:/Pothole%20detection/run_detection.py).

### 5.2 Dual Auto-Perspective ROI Engine
The visual pipeline detects whether the input is a low-angle front bumper/dashcam view or an in-cabin view:

| Mode | Trigger Condition | Normalized Crop Box $[y_{\min}, y_{\max}, x_{\min}, x_{\max}]$ | Target Geometry |
|---|---|---|---|
| **DASHCAM Mode** *(Default)* | [`pothole_test_2.mp4`](file:///e:/Pothole%20detection/pothole_test_2.mp4) or bumper mount | `y: [44%, 92%]`, `x: [8%, 92%]` | Focuses on the lower 50% of the visual field where the road surface exists. Filters out horizon, sky, trees, and oncoming traffic. |
| **CABIN Mode** | [`test_video.mp4`](file:///e:/Pothole%20detection/test_video.mp4) or `--mode cabin` | `y: [22%, 49%]`, `x: [28%, 67%]` | Tightly bounds the windshield glass opening. Masks out the dashboard, hood reflections, rearview mirror, and right A-pillar. |

---

## 6. Spatial Corridor & Proximity Avoidance Decision Engine

### 6.1 Road Corridor Partitioning
The road Region of Interest is divided horizontally into three equal longitudinal corridors:

```
  0% ────────────────────────── 33.3% ──────────────────────── 66.7% ──────────────────────── 100%
  ┌───────────────────────────────┬───────────────────────────────┬───────────────────────────────┐
  │         LEFT CORRIDOR         │        CENTER CORRIDOR        │        RIGHT CORRIDOR         │
  │          [0.00, 0.33]         │          [0.33, 0.67]         │          [0.67, 1.00]         │
  │                               │                               │                               │
  │ If pothole detected here:     │ If pothole detected here:     │ If pothole detected here:     │
  │   Risk in Left rises          │   IMMEDIATE COLLISION PATH    │   Risk in Right rises         │
  │   >> DIRECTIVE: SWERVE RIGHT  │   >> EVADE TO CLEAR CORRIDOR  │   >> DIRECTIVE: SWERVE LEFT   │
  └───────────────────────────────┴───────────────────────────────┴───────────────────────────────┘
```

### 6.2 Continuous Danger Scoring Formula
For each detection $i$ mapped to corridor $z \in \{\text{left}, \text{center}, \text{right}\}$:

$$\text{Danger}_i = \text{Conf}_i \times W_{\text{type}} \times \left(0.5 + 0.8 \cdot \frac{y_{\text{center}} - y_{\text{roi\_top}}}{h_{\text{roi}}}\right) \times \left(0.3 + 0.7 \cdot \frac{\text{Area}_i}{\text{RoadArea}}\right)$$

* **Class Risk Multiplier ($W_{\text{type}}$):**
  * Pothole (`D40`): $W = 2.2$ (Immediate axle/suspension damage hazard)
  * Alligator / Transverse Crack (`D10`, `D20`): $W = 1.5$ (Structural roughness)
  * Longitudinal Crack (`D00`): $W = 1.2$
  * Positive Obstacle (Car, Person): $W = 1.0$
* **Vertical Proximity Weighting:** Objects near the bottom of the ROI have higher $y_{\text{center}}$, scaling risk up to $1.3\times$ as the vehicle closes the physical distance.
* **Corridor Aggregation:** $\text{Score}_z = \min\left(1.0, \sum_{i \in z} \text{Danger}_i\right)$.

### 6.3 Reactive Decision Matrix
Threshold for corridor obstruction: $\tau = 0.20$.

| Left Risk ($L$) | Center Risk ($C$) | Right Risk ($R$) | Primary State | HUD Directive Display | Serial Command Output |
|:---:|:---:|:---:|:---:|:---|:---|
| $< \tau$ | $< \tau$ | $< \tau$ | `CLEAR` | `PATH CLEAR -- CRUISING` | `DRIVE 180` |
| $\ge \tau$ | $< \tau$ | $< \tau$ | `SWERVE_RIGHT` | `POTHOLE ON LEFT >> SWERVE RIGHT` | `TURN 30 150` |
| $< \tau$ | $< \tau$ | $\ge \tau$ | `SWERVE_LEFT` | `<< POTHOLE ON RIGHT: SWERVE LEFT` | `TURN -30 150` |
| Low ($L < R$) | $\ge \tau$ | High | `EVADE_LEFT` | `<< POTHOLE AHEAD: SWERVE LEFT` | `TURN -35 150` |
| High | $\ge \tau$ | Low ($R \le L$) | `EVADE_RIGHT` | `POTHOLE AHEAD: SWERVE RIGHT >>` | `TURN 35 150` |
| $\ge \tau$ | $\ge \tau$ | $\ge \tau$ | `STOP` | `!! HAZARD AHEAD: FULL STOP !!` | `PI_STOP` |

---

## 7. Real-Time HUD & Telemetry Visualizer

[`run_detection.py`](file:///e:/Pothole%20detection/run_detection.py) includes a high-definition telemetry rendering engine:
* **Top Header Glassmorphism Bar:** Displays active model checkpoint, device (CUDA/CPU), inference latency (ms), instantaneous FPS, and current camera perspective mode.
* **Dynamic Corridor Danger Meters:** Real-time percentage bars for Left, Center, and Right corridors, color-coded:
  * Green: $0\% - 19\%$ (Safe)
  * Yellow: $20\% - 49\%$ (Caution)
  * Red: $\ge 50\%$ (Hazard)
* **High-Visibility Action Banner:** Large centered pill badge indicating the current avoidance directive (`PATH CLEAR`, `SWERVE RIGHT`, `SWERVE LEFT`, `STOP / BRAKE`).
* **Road Corridor Overlay:** Semi-transparent colored column overlays on the road plane showing safe vs. hazardous driving zones.
* **Bounding Box Annotations:** Color-coded bounding boxes showing class label, detection confidence, and spatial proximity indicator.

---

## 8. Repository Inventory & File Map

```
E:\Pothole detection\
│
├── CONTEXT.md                         # [THIS FILE] Master architecture, specs, and developer guide
├── run_detection.py                   # Production inference & HUD telemetry rendering script
├── train_pothole_model.py             # Automated training pipeline for YOLOv10n on local GPU
├── best.pt                            # Active 50-epoch fine-tuned weights (YOLOv10n, 5.75 MB)
├── yolov10n.pt                        # Base pretrained COCO weights
├── rdd2022_yolov8n.pt                 # Baseline benchmark weights
├── test_video.mp4                     # Source test video 1: In-cabin POV on damaged Indian road
├── pothole_test_2.mp4                 # Source test video 2: Pure front dashcam on wet highway
├── annotated_video.mp4                # Rendered verification video for test_video.mp4
├── annotated_pothole_test_2.mp4       # Rendered verification video for pothole_test_2.mp4
│
├── runs_rdd/custom_pothole_model/     # Training metrics, plots, and checkpoints
│   ├── weights/
│   │   ├── best.pt                    # Best checkpoint during training (identical to root best.pt)
│   │   └── last.pt                    # Final epoch checkpoint
│   ├── results.png                    # Loss and mAP convergence curves
│   ├── results.csv                    # Epoch-by-epoch loss & metric data
│   ├── confusion_matrix.png           # Class confusion matrix
│   ├── confusion_matrix_normalized.png# Normalized confusion matrix
│   ├── BoxF1_curve.png                # F1 confidence curve
│   ├── BoxPR_curve.png                # Precision-Recall curve
│   ├── val_batch0_labels.jpg          # Ground truth validation annotations
│   ├── val_batch0_pred.jpg            # Model inference on validation batch
│   └── args.yaml                      # Ultralytics training argument configuration
│
└── negative_obstacle_detection/
    ├── 05_pothole_ahead.md            # Hardware serial protocol & capstone milestone spec
    ├── README.md                      # Submodule overview
    ├── detector.py                    # Original edge wrapper module
    ├── prepare_rdd_yolo.py            # Pascal VOC XML to YOLO format converter
    ├── rdd_yolo_dataset/              # Clean YOLO-formatted dataset (train/val images & labels)
    │   ├── rdd_dataset.yaml           # Dataset path configuration for Ultralytics
    │   ├── train/ (1,224 images + label text files)
    │   └── val/ (306 images + label text files)
    └── train/                         # Raw Kaggle RDD2022 dataset (1,530 images + VOC XMLs)
```

> **Cleanup Record:** Outdated `eval_metrics/` directory and redundant training mosaic dumps (`train_batch*.jpg`) have been permanently removed to keep the workspace clean and focused.

---

## 9. Operator & Developer CLI Guide

### 9.1 Inference & Video Processing
All commands run directly from PowerShell or Command Prompt:

```powershell
# 1. Run detection on front dashcam video (Default mode: dashcam)
python run_detection.py --video pothole_test_2.mp4

# 2. Run with live interactive OpenCV playback window (Press Space to Pause, 'q' to Quit)
python run_detection.py --video pothole_test_2.mp4 --show

# 3. Run on in-cabin POV video (Automatically applies windshield mask)
python run_detection.py --video test_video.mp4 --show

# 4. Process only the first 30 seconds of a video
python run_detection.py --video pothole_test_2.mp4 --duration 30

# 5. Explicitly override perspective mode
python run_detection.py --video sample.mp4 --mode dashcam
python run_detection.py --video sample.mp4 --mode cabin

# 6. Specify custom weights or confidence threshold
python run_detection.py --video pothole_test_2.mp4 --weights best.pt --conf 0.35
```

### 9.2 Model Retraining / Fine-Tuning
```powershell
# Retrain for 50 epochs on RTX 3050 GPU (~24 mins)
python train_pothole_model.py --epochs 50 --batch 8

# Fast 10-epoch sanity test
python train_pothole_model.py --epochs 10 --batch 8
```

---

## 10. Roadmap & Implementation Guide for Future Agents

Any agent or developer continuing this capstone project should execute the following three engineering tasks:

### Task 1: Temporal Exponential Moving Average (EMA) Smoothing
* **Objective:** Prevent single-frame detection drops from causing rapid 1-frame flickering between `CLEAR` and `SWERVE`.
* **Implementation Plan in [`run_detection.py`](file:///e:/Pothole%20detection/run_detection.py):**
  Maintain a persistent dictionary of smoothed corridor risk scores across frames:
  ```python
  class CorridorSmoother:
      def __init__(self, alpha: float = 0.70):
          self.alpha = alpha  # Weight of current frame
          self.smoothed = {"left": 0.0, "center": 0.0, "right": 0.0}

      def update(self, raw_scores: dict) -> dict:
          for k in self.smoothed:
              self.smoothed[k] = (self.alpha * raw_scores.get(k, 0.0) + 
                                  (1.0 - self.alpha) * self.smoothed[k])
          return self.smoothed
  ```

### Task 2: Hardware UART Serial Bridge Integration
* **Objective:** Send real-time motor control overrides from Python directly to the Arduino/ESP32 over USB serial.
* **Implementation Plan:**
  ```python
  import serial
  import time

  class VehicleSerialBridge:
      def __init__(self, port: str = "COM3", baud: int = 115200):
          self.ser = serial.Serial(port, baud, timeout=0.05)
          time.sleep(2.0)  # Allow Arduino bootloader to settle

      def send_command(self, cmd: str):
          packet = f"{cmd.strip()}\n".encode('ascii')
          self.ser.write(packet)

      def check_telemetry(self):
          if self.ser.in_waiting:
              return self.ser.readline().decode('ascii', errors='ignore').strip()
          return None
  ```

### Task 3: Inverse Perspective Mapping (IPM / Bird's-Eye View)
* **Objective:** Transform the 2D image perspective trapezoid into an orthographic top-down metric plane ($X, Y$ coordinates in centimeters).
* **Benefit:** Allows the path planner to know exact physical distances (e.g., *"Pothole is 120 cm forward, 25 cm right of vehicle centerline"*), enabling smooth Bezier curve spline evasion rather than fixed angular steering snaps.
