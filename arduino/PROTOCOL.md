# Serial Protocol Specification (Source of Truth)

**Layer Contract**: Raspberry Pi 5 ↔ Arduino Uno (ATmega328P)  
**Baud Rate**: 9600 baud, 8-N-1  
**Line Ending**: `\n` (newline, ASCII `0x0A`)  
**Transport**: Swappable — USB-Serial (`/dev/ttyUSB0` or `/dev/ttyACM0`) or GPIO-UART (`/dev/ttyAMA0`).

---

## 1. Command Priority Model

Three tiers, highest priority wins:

| Tier | Authority | Commands | Override behavior |
|------|-----------|----------|-------------------|
| **1 — Pi Camera Latch** | Raspberry Pi 5 (camera + YOLO) | `PI_STOP`, `PI_GO`, `PI_AVOID` | Absolute. Freezes all motor output. Only `PI_GO` releases. |
| **2 — Local Ultrasonic** | Arduino Uno (2× HC-SR04P) | *(automatic — no command)* | Triggers autonomous avoidance. Preempted by Tier 1. |
| **3 — Normal Driving** | Pi 5 or Serial Monitor | `DRIVE`, `REV`, `TURN`, `STOP`, `PING` | Standard timed actions. Preempted by Tier 1 and 2. |

---

## 2. Commands (Pi 5 → Arduino Uno)

Commands are sent as ASCII strings terminated with `\n`.

### Tier 1 — Pi Camera Latch

| Command | Format | Example | Description |
|---|---|---|---|
| **PI_STOP** | `PI_STOP` | `PI_STOP` | Latched emergency stop from camera algorithm (obstacle / negative obstacle). Freezes all motors. Only `PI_GO` releases. Overrides everything, including mid-avoidance. |
| **PI_GO** | `PI_GO` | `PI_GO` | Release Pi latch. Returns to `STATE_IDLE`. Ignored if not in `PI_STOPPED` state. |
| **PI_AVOID** | `PI_AVOID <LEFT\|RIGHT>` | `PI_AVOID LEFT` | Directional hint for next avoidance maneuver. Stored and consumed once. If set when ultrasonic avoidance triggers, car turns this direction instead of deciding locally. |

### Tier 3 — Normal Driving

| Command | Format | Example | Description |
|---|---|---|---|
| **DRIVE** | `DRIVE <speed> <duration_ms>` | `DRIVE 180 1500` | Drive forward at PWM `<speed>` (0–255) for `<duration_ms>` ms |
| **REVERSE** | `REV <speed> <duration_ms>` | `REV 180 1000` | Drive reverse at PWM `<speed>` (0–255) for `<duration_ms>` ms |
| **TURN** | `TURN <angle_deg> <speed>` | `TURN 45 150` | Skid-steer turn by `<angle_deg>` (-180 to +180; positive = right, negative = left) |
| **STOP** | `STOP` | `STOP` | Immediate non-latched stop (goes to `IDLE`; does NOT lock out further commands) |
| **PING** | `PING` | `PING` | Liveness check |

> **Note**: All Tier 3 commands are blocked while `PI_STOPPED` is active. Arduino responds with `[ACK] BLOCKED (PI_STOPPED)`.

---

## 3. Telemetry (Arduino Uno → Pi 5)

Arduino broadcasts or responds with single-line telemetry events prefixed by `[TELEM]` or `[ACK]`.

### Acknowledgments

| Message | Example | Description |
|---|---|---|
| **ACK** | `[ACK] DRIVE 180 1500` | Command accepted and executed/queued |
| **PONG** | `[ACK] PONG` | Heartbeat response to `PING` |
| **BLOCKED** | `[ACK] BLOCKED (PI_STOPPED)` | Command rejected — Pi latch is active |

### State & Lifecycle

| Message | Example | Description |
|---|---|---|
| **READY** | `[TELEM] READY Uno-ATmega328P` | Broadcast once bootloader finishes in `setup()` |
| **STATE** | `[TELEM] STATE=DRIVING SPEED=180` | Current state machine status |
| **DONE** | `[TELEM] DONE DRIVE` | Timed drive/turn/rev action finished |

### Ultrasonic & Avoidance

| Message | Example | Description |
|---|---|---|
| **US** | `[TELEM] US LEFT=15.3 RIGHT=42.7` | Periodic ultrasonic distance readings (every ~100ms) |
| **AVOID** | `[TELEM] AVOID RIGHT` | Avoidance maneuver started, turning in stated direction |
| **DONE AVOID** | `[TELEM] DONE AVOID` | Avoidance maneuver complete, resuming forward |
| **PI_STOPPED** | `[TELEM] STATE=PI_STOPPED` | Acknowledges Pi latch is active |

---

## 4. State Machine

```
States:
  IDLE         — Motors off, awaiting commands
  DRIVING      — Forward at PWM speed for timed duration
  REVERSING    — Reverse at PWM speed for timed duration
  TURNING      — Skid-steer turn for timed duration
  AVOIDING     — Autonomous ultrasonic avoidance (reverse → turn → resume)
  PI_STOPPED   — Latched stop from Pi camera (frozen until PI_GO)

Transitions:
  IDLE ──[DRIVE/REV/TURN]──► DRIVING/REVERSING/TURNING
  DRIVING/REVERSING/TURNING ──[timer expires]──► IDLE
  DRIVING/REVERSING/TURNING ──[ultrasonic < 20cm]──► AVOIDING
  AVOIDING ──[maneuver complete]──► DRIVING (resume)
  ANY STATE ──[PI_STOP]──► PI_STOPPED
  PI_STOPPED ──[PI_GO]──► IDLE
  ANY STATE ──[STOP]──► IDLE
```

---

## 5. Conventions
- Both left motors and both right motors are paired channel-wise on a single L298N driver board (skid-steer mode).
- Pin mappings are centralized in `arduino/lib/pins.h`.
- Ultrasonic and avoidance timing constants are in `arduino/lib/protocol.h`.
- Any modification to this protocol must originate from `firmware-agent` and be mirrored in `car/control/motor_controller.py`.
