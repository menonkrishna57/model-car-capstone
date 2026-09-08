# Autonomous RC Car — Capstone Project

MPSTME / NMIMS autonomous RC car with two-layer architecture:

| Layer | Hardware | Role |
|-------|----------|------|
| **Control** | Arduino Nano (ATmega328P) | Real-time motor PWM via L298N, skid-steer, IMU heading hooks, serial telemetry |
| **Perception** | Raspberry Pi 5 | YOLOv10n (NCNN), CSI global-shutter camera, ICM20948 IMU, 2× HC-SR04P ultrasonic |

## Repo Layout

```
firmware/arduino/           ← Nano sketches (motor test, future control state machines)
perception/                 ← Pi-side Python: camera, YOLO/NCNN, IMU, ultrasonic
perception/serial_bridge/   ← Pi ↔ Nano serial handshake
sim/                        ← Oracle Cloud benchmarking scripts, models, test videos
sim/models/                 ← Exported model weights (NCNN, OpenVINO, PyTorch)
sim/test_videos/            ← Dashcam test footage for benchmarking
sim/legacy/                 ← Older Oracle Ampere A1 benchmark (baseline)
tools/component_tracker/    ← Lab equipment tracking (Excel + scripts)
docs/                       ← Architecture notes, wiring diagrams, ADRs
```

## Quick Start

### Benchmarking (on Pi 5 or Oracle Cloud A1 proxy)

```bash
cd sim
pip install -r requirements.txt
python benchmark.py --video test_videos/dashcam_test.mp4
```

### Perception (on Pi 5 — requires camera connector)

```bash
cd perception
pip install -r requirements.txt
# Camera connector not yet available — skeleton code only
```

### Arduino Motor Test

1. Open `firmware/arduino/motor_test/motor_test.ino` in the Arduino IDE
2. Select **Arduino Nano** as target board
3. Upload and open Serial Monitor at 9600 baud

## Team

- **Krishna** — hardware, integration, C++ control loop (lead)
- **Aditya** — YOLO pipeline / fine-tuning
- **Parth** — ultrasonic logic + assembly
