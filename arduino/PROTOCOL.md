# Serial Protocol Specification (Source of Truth)

**Layer Contract**: Raspberry Pi 5 ↔ Arduino Uno (ATmega328P)  
**Baud Rate**: 9600 baud, 8-N-1  
**Line Ending**: `\n` (newline, ASCII `0x0A`)  
**Transport**: Swappable — USB-Serial (`/dev/ttyUSB0` or `/dev/ttyACM0`) or GPIO-UART (`/dev/ttyAMA0`).

---

## 1. Commands (Pi 5 → Arduino Uno)

Commands are sent as ASCII strings terminated with `\n`.

| Command | Format | Example | Description |
|---|---|---|---|
| **DRIVE** | `DRIVE <speed> <duration_ms>` | `DRIVE 180 1500` | Drive forward at PWM `<speed>` (0–255) for `<duration_ms>` ms |
| **REVERSE** | `REV <speed> <duration_ms>` | `REV 180 1000` | Drive reverse at PWM `<speed>` (0–255) for `<duration_ms>` ms |
| **TURN** | `TURN <angle_deg> <speed>` | `TURN 45 150` | Skid-steer turn by `<angle_deg>` (-180 to +180; positive = right, negative = left) |
| **STOP** | `STOP` | `STOP` | Immediate emergency / normal stop (PWM = 0) |
| **PING** | `PING` | `PING` | Liveness check |

---

## 2. Telemetry (Arduino Uno → Pi 5)

Arduino broadcasts or responds with single-line telemetry events prefixed by `[TELEM]` or `[ACK]`.

| Message | Example | Description |
|---|---|---|
| **ACK** | `[ACK] DRIVE 180 1500` | Command accepted and executed/queued |
| **STATUS** | `[TELEM] STATE=DRIVING SPEED=180` | Current state machine status |
| **DONE** | `[TELEM] DONE DRIVE` | Timed drive/turn action finished |
| **PONG** | `[ACK] PONG` | Heartbeat response to `PING` |
| **READY** | `[TELEM] READY Uno-ATmega328P` | Broadcast once bootloader finishes in `setup()` |

---

## 3. Conventions
- Both left motors and both right motors are paired channel-wise on a single L298N driver board (skid-steer mode).
- Pin mappings are centralized in `arduino/lib/pins.h`.
- Any modification to this protocol must originate from `firmware-agent` and be mirrored in `car/control/motor_controller.py`.
