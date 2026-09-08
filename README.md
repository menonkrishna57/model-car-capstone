# Autonomous RC Car — Capstone Project (MPSTME / NMIMS)

Two-layer autonomous 4WD RC vehicle powered by an **Arduino Uno** for real-time motor control and a **Raspberry Pi 5** for edge AI perception and navigation.

---

## 1. System Architecture

| Layer | Hardware | Role | Owner |
|---|---|---|---|
| **Real-Time Control** | Arduino Uno (ATmega328P) | Deterministic PWM via L298N (skid-steer 4WD), turn-then-drive state machine, IMU heading hooks, serial telemetry | `firmware-agent` |
| **Edge Perception** | Raspberry Pi 5 (8GB) | YOLOv10n inference via NCNN, CSI global-shutter camera, ICM20948 IMU (I2C), 3× HC-SR04P ultrasonic array | `perception-agent` |
| **Integration** | Pi ↔ Uno Bridge | Serial command/telemetry handshake, point-to-point dead reckoning navigation | `integration-agent` |

---

## 2. Repository Layout

```
.
├── arduino/                       # Uno firmware (future Nano drop-in) — owned by firmware-agent
│   ├── motor_test/                # Motor forward/reverse bench sketch
│   ├── point_to_point_control/    # Turn-then-drive state machine sketch
│   ├── lib/                       # Centralized pin maps & protocol headers
│   └── PROTOCOL.md                # Serial telemetry / command protocol (source of truth)
│
├── car/                           # Pi 5 runtime — owned by perception-agent & integration-agent
│   ├── main.py                    # Entry point: camera → detect → decide → drive
│   ├── perception/                # YOLOv10n NCNN detector, ICM20948 IMU, 3x ultrasonic array
│   ├── control/                   # Pi-side serial handshake & point-to-point dead reckoning
│   └── requirements.txt           # Runtime-only dependencies (picamera2, pyserial, ncnn, etc.)
│
├── benchmarking/                  # Hardware benchmarking on real Pi 5 — owned by bench-agent
│   ├── benchmark.py               # Pure inference & multi-core benchmark runner
│   ├── requirements.txt           # Benchmark environment dependencies
│   ├── models/                    # Exported weights: PyTorch, NCNN FP16, OpenVINO FP16
│   ├── test_videos/               # Dashcam evaluation videos
│   ├── results/                   # Dated on-device benchmark run outputs
│   └── legacy/                    # Archived Oracle Cloud A1 baseline benchmark
│
├── infra/                         # Pi headless bring-up & networking — owned by network-agent
│   ├── pi-image/                  # Cloud-init network-config & user-data (one-shot flash)
│   └── SETUP.md                   # SSH, Wi-Fi bring-up, and raspi-config interface guide
│
├── tools/                         # Project tools — owned by docs-tracker-agent
│   └── component_tracker/         # Lab borrowing rotation tracker (15/60-day policy)
│
├── docs/                          # Project documentation — owned by docs-tracker-agent
│   ├── architecture.md            # Two-layer design rationale & power rail isolation
│   └── decisions/                 # Architectural Decision Records (ADRs)
│
├── AGENTS.md                      # Multi-agent role boundaries and constraints
└── README.md
```

---

## 3. Quick Start

### A. Benchmarking (Pi 5 Hardware)
```bash
cd benchmarking
pip install -r requirements.txt
python benchmark.py --video test_videos/dashcam_test.mp4 --imgsz 320
```

### B. Car Runtime (Raspberry Pi 5)
```bash
cd car
pip install -r requirements.txt
python main.py
```

### C. Arduino Control Firmware
1. Open `arduino/point_to_point_control/point_to_point_control.ino` in Arduino IDE.
2. Select **Arduino Uno** board and correct COM/TTY port.
3. Upload and open Serial Monitor at **9600 baud**.

---

## 4. Team

- **Krishna**: Hardware + Integration + C++ control loop (Lead, primary agent user)
- **Aditya**: Hardware / Chassis
- **Parth**: Software
