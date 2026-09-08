# Architecture Overview — Autonomous RC Car

**Owned by**: `docs-tracker-agent`

---

## 1. Two-Layer Architecture Rationale

The vehicle splits autonomy into two distinct processing layers:

```
┌────────────────────────────────────────────────────────┐
│                   High-Level Layer                     │
│                   Raspberry Pi 5                       │
│  - Perception (YOLOv10n via NCNN on ARM Cortex-A76)    │
│  - CSI Global Shutter Camera                           │
│  - ICM20948 IMU (I2C) & 3× HC-SR04P Ultrasonics (GPIO) │
│  - Navigation State Machine & Decision Logic           │
└──────────────────────────┬─────────────────────────────┘
                           │ Serial Link (9600 baud)
                           │ (USB-Serial or GPIO UART)
┌──────────────────────────┴─────────────────────────────┐
│                   Real-Time Control Layer              │
│                   Arduino Uno (ATmega328P)             │
│  - Deterministic PWM Motor Driver (L298N H-Bridge)     │
│  - Skid-Steer 4WD Pairing (Left Pair / Right Pair)     │
│  - Millisecond-accurate turn-then-drive execution      │
│  - Serial Command Parser & Telemetry Feedback          │
└────────────────────────────────────────────────────────┘
```

### Why Not Pi 5 Alone?
1. **PWM Jitter**: Linux is not a hard real-time OS. Under heavy CPU load (e.g. YOLOv10n 4-core inference), software PWM or user-space GPIO timing jitters significantly, causing uneven motor pulses and jerky trajectory errors.
2. **Resource Contention**: The ATmega328P guarantees microsecond-level determinism for motor pulse-width and direction transitions without competing with heavy machine learning inference threads.
3. **Future Nano Migration**: Arduino Uno is currently used; a drop-in migration to Arduino Nano (identical ATmega328P microcontroller) is planned to reduce chassis footprint without requiring firmware logic changes.

---

## 2. Power Architecture

The car uses **strictly separated power rails** to protect sensitive logic from motor inductive voltage spikes (back-EMF):

1. **Motor Rail & Arduino**:
   - 2S 18650 Li-ion battery pack (7.4V nominal, up to 8.4V fully charged).
   - High-efficiency 5V / 5A buck converter stepping down 2S battery to 5V.
   - Powers the L298N motor driver `+12V` / motor power terminal and Arduino `VIN` / `5V`.
2. **Raspberry Pi 5 Rail**:
   - boAt EnergyShroom PB400 power bank (dedicated 5V / 3A+ delivery over USB-C).
   - Completely isolated ground loop from motor noise. Common ground tied only at the serial / logic interface.

---

## 3. Communication & Control Contract

- Protocol source of truth: [`arduino/PROTOCOL.md`](../arduino/PROTOCOL.md)
- Baud rate: `9600` baud, newline-terminated ASCII packets.
- Telemetry prefix: `[TELEM]`
- Acknowledge prefix: `[ACK]`
