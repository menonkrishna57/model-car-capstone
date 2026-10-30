# Serial Protocol Specification (Source of Truth)

**Layer Contract**: Raspberry Pi 5 ↔ Arduino Uno (ATmega328P)  
**Baud Rate**: 115200 baud, 8-N-1  
**Line Ending**: `\n` (newline, ASCII `0x0A`)  
**Transport**: Swappable — USB-Serial (`/dev/ttyUSB0` or `/dev/ttyACM0`) or GPIO-UART (`/dev/ttyAMA0`).

---

## 1. Command Priority Model

Multi-tier priority stack, highest priority wins:

| Tier | Authority | Commands / Triggers | Override behavior |
|------|-----------|---------------------|-------------------|
| **1 — Pi Camera Latch** | Raspberry Pi 5 (camera + YOLO) | `PI_STOP`, `PI_GO`, `PI_AVOID` | Absolute. Freezes all motor output. Only `PI_GO` releases. |
| **1.5 — Cliff Detection Reflex** | Arduino Uno (angled 2× HC-SR04P) | *(automatic — drop-off detected)* | Immediate reverse burst, then latches (`CLIFF_STOPPED`). Only `PI_GO` releases. Preempted only by Tier 1. |
| **2 — Local Ultrasonic Avoidance** | Arduino Uno (2× HC-SR04P) | *(automatic — obstacle < threshold)* | Autonomous 3-phase avoidance (reverse → turn → resume). Preempted by Tier 1 & 1.5. |
| **3 — Normal Driving** | Pi 5 or Serial Monitor | `DRIVE`, `REV`, `TURN`, `STOP`, `PING`, `STATUS` | Standard timed actions. Preempted by Tier 1, 1.5, and 2. |
| **— — Meta / Runtime Tuning** | Pi 5 or Serial Monitor | `TUNE GET`, `TUNE SET`, `TUNE RECAL` | Runtime parameter adjustments without reflashing. |

---

## 2. Commands (Pi 5 → Arduino Uno)

Commands are sent as ASCII strings terminated with `\n`.

### Tier 1 — Pi Camera Latch

| Command | Format | Example | Description |
|---|---|---|---|
| **PI_STOP** | `PI_STOP` | `PI_STOP` | Latched emergency stop from camera algorithm. Freezes all motors. Only `PI_GO` releases. Overrides everything, including mid-avoidance and mid-cliff reverse. |
| **PI_GO** | `PI_GO` | `PI_GO` | Release Pi or Cliff latch. Returns to `STATE_IDLE`. Ignored if not in `PI_STOPPED` or `CLIFF_STOPPED`. |
| **PI_AVOID** | `PI_AVOID <LEFT\|RIGHT>` | `PI_AVOID LEFT` | Directional hint for next ultrasonic avoidance maneuver. Stored and consumed once. If set when ultrasonic avoidance triggers, car turns this direction instead of deciding locally. |

### Tier 3 — Normal Driving

| Command | Format | Example | Description |
|---|---|---|---|
| **DRIVE** | `DRIVE <speed> <duration_ms>` | `DRIVE 180 1500` | Drive forward at PWM `<speed>` (180–255, 0 = stop) for `<duration_ms>` ms. Active speeds below 180 are clamped to 180 floor. |
| **REVERSE** | `REV <speed> <duration_ms>` | `REV 180 1000` | Drive reverse at PWM `<speed>` (180–255, 0 = stop) for `<duration_ms>` ms. Active speeds below 180 are clamped to 180 floor. |
| **TURN** | `TURN <angle_deg> <speed>` | `TURN 45 200` | Skid-steer turn by `<angle_deg>` (-180 to +180; positive = right, negative = left) at PWM `<speed>` (200–255; clamped to 200 floor, 0 = stop). All turn commands enforce a minimum motor PWM floor of 200 to overcome vehicle weight and friction during turns. |
| **STOP** | `STOP` | `STOP` | Immediate non-latched stop (goes to `IDLE`; does NOT lock out further commands). Blocked if in `PI_STOPPED` or `CLIFF_STOPPED`. |
| **PING** | `PING` | `PING` | Liveness check (responds with `[ACK] PONG`). |
| **STATUS** | `STATUS` | `STATUS` | Queries current state and ultrasonic readings. |

> **Note**: All Tier 3 commands are blocked while `PI_STOPPED` or `CLIFF_STOPPED` is active. Arduino responds with `[ACK] BLOCKED (PI_STOPPED)` or `[ACK] BLOCKED (CLIFF_STOPPED)`. The minimum driving speed floor is 180 PWM for straight movement commands (DRIVE, REV) to overcome TT motor static friction under vehicle payload; all turn commands (TURN) enforce a minimum motor PWM floor of 200 to overcome vehicle weight and tire friction during skid-steer turns; speed 0 is reserved for stop.

### Meta — Runtime Parameter Tuning (`TUNE`)

Enables on-the-fly field calibration without reflashing the Arduino Uno. Values can be saved to internal EEPROM to persist across power cycles.

| Command | Format | Example | Description |
|---|---|---|---|
| **TUNE GET** | `TUNE GET` | `TUNE GET` | Dumps all active tunable parameters across several `[TELEM] TUNE ...` lines. |
| **TUNE SET** | `TUNE SET <PARAM> <VALUE>` | `TUNE SET US_THRESH 18.0` | Updates a runtime parameter value. Responds with `[ACK] TUNE SET <PARAM>=<VALUE>`. |
| **TUNE RECAL** | `TUNE RECAL` | `TUNE RECAL` | Triggers a fresh ground baseline calibration cycle for angled ultrasonic sensors. Must be in `IDLE` or `CLIFF_STOPPED` state. |
| **TUNE SAVE** | `TUNE SAVE` | `TUNE SAVE` | Commits current parameters to ATmega328P non-volatile EEPROM. Responds with `[ACK] TUNE SAVE SUCCESS`. |
| **TUNE LOAD** | `TUNE LOAD` | `TUNE LOAD` | Reloads stored parameters from EEPROM. Responds with `[ACK] TUNE LOAD SUCCESS`. |
| **TUNE RESET** | `TUNE RESET` | `TUNE RESET` | Resets parameters to compile-time factory defaults. Responds with `[ACK] TUNE RESET_TO_DEFAULTS`. |

#### Tunable Parameter Names (`<PARAM>`):

| Name | Type | Default | Description |
|---|---|---|---|
| `US_THRESH` | `float` | 20.0 (auto-adjusted) | Obstacle trigger distance threshold (cm) |
| `US_POLL_MS` | `unsigned long` | 100 | Ultrasonic sensor poll interval (ms) |
| `CLIFF_MARGIN` | `float` | 15.0 | Cliff detection margin above ground baseline (cm) |
| `CLIFF_BASE_L` | `float` | Auto-calibrated | Left sensor ground baseline distance (cm) |
| `CLIFF_BASE_R` | `float` | Auto-calibrated | Right sensor ground baseline distance (cm) |
| `CLIFF_REV_MS` | `unsigned long` | 400 | Cliff reverse burst duration before latching (ms) |
| `CLIFF_REV_SPD` | `int` | 180 | Cliff reverse burst PWM speed (0–255, minimum operational floor 180; header default 185) |
| `CLIFF_CONFIRM` | `int` | 2 | Consecutive cliff readings needed to confirm trigger |
| `AVOID_SPD` | `int` | 180 | Avoidance PWM speed (0–255, minimum operational floor 180; header default 190) |
| `AVOID_REV_MS` | `unsigned long` | 400 | Phase 1: Avoidance reverse burst duration (ms) |
| `AVOID_TURN_MS` | `unsigned long` | 500 | Phase 2: Avoidance skid-turn duration (ms) |
| `AVOID_FWD_MS` | `unsigned long` | 2000 | Phase 3: Avoidance resume forward duration (ms) |
| `AVOID_COOL_MS`| `unsigned long` | 500 | Cooldown suppression window after avoidance (ms) |
| `WDG_MS` | `unsigned long` | 5000 | Serial watchdog timeout for Pi silence (ms, 0=disabled) |
| `TURN_CAL` | `int` | 10 | Skid-turn ms duration per degree |
| `MOTOR_TRIM` | `int` | 0 | Chassis straight-line trim (-50 to +50; >0 turns right faster, <0 turns left faster) |
| `TELEM_STREAM` | `int` | 1 | Periodic US telemetry broadcast (1 = streaming, 0 = silent/quiet mode) |
| `CLIFF_ENABLE` | `int` | 1 | Cliff detection reflex toggle (1 = enabled for table/edges, 0 = disabled for floor driving) |

---

## 3. Telemetry (Arduino Uno → Pi 5)

Arduino broadcasts or responds with single-line telemetry events prefixed by `[TELEM]` or `[ACK]`.

### Acknowledgments

| Message | Example | Description |
|---|---|---|
| **ACK** | `[ACK] DRIVE 180 1500` | Command accepted and executed/queued |
| **PONG** | `[ACK] PONG` | Heartbeat response to `PING` |
| **BLOCKED** | `[ACK] BLOCKED (PI_STOPPED)` | Command rejected — Pi latch is active |
| **BLOCKED** | `[ACK] BLOCKED (CLIFF_STOPPED)` | Command rejected — Cliff latch is active |
| **TUNE SET** | `[ACK] TUNE SET US_THRESH=18.0` | Parameter successfully updated |
| **ERROR** | `[ACK] ERROR (BAD_DRIVE_ARGS)` | Command arguments invalid or malformed |

### State & Lifecycle

| Message | Example | Description |
|---|---|---|
| **READY** | `[TELEM] READY Uno-ATmega328P v2.0` | Broadcast once bootloader finishes in `setup()` |
| **CAL_CLIFF** | `[TELEM] CAL_CLIFF BASE_L=12.4 BASE_R=12.1 OBS_THRESH=7.3` | Ground calibration result at boot or after `TUNE RECAL` |
| **STATE** | `[TELEM] STATE=DRIVING SPEED=180` | Current state machine status |
| **DONE** | `[TELEM] DONE DRIVE` | Timed drive/turn/rev action finished |
| **HEARTBEAT** | `[TELEM] HEARTBEAT_TIMEOUT` | Pi watchdog triggered due to silence; car safely stopped |

### Ultrasonic, Avoidance & Cliff Detection

| Message | Example | Description |
|---|---|---|
| **US** | `[TELEM] US LEFT=12.3 RIGHT=12.5` | Periodic ultrasonic distance readings (every ~100ms) |
| **CLIFF** | `[TELEM] CLIFF LEFT=999.0 RIGHT=12.1 BASE_L=12.4 BASE_R=12.1` | Cliff edge detected; triggering reverse burst |
| **AVOID** | `[TELEM] AVOID RIGHT` | Avoidance maneuver started, turning in stated direction |
| **DONE AVOID** | `[TELEM] DONE AVOID` | Avoidance maneuver complete, resuming forward |
| **CLIFF_STOPPED**| `[TELEM] STATE=CLIFF_STOPPED` | Latched stop after cliff reverse burst; waiting for `PI_GO` |
| **PI_STOPPED** | `[TELEM] STATE=PI_STOPPED` | Latched stop from Pi camera; waiting for `PI_GO` |

---

## 4. State Machine

```
States:
  IDLE              — Motors off, awaiting commands
  DRIVING           — Forward at PWM speed for timed duration
  REVERSING         — Reverse at PWM speed for timed duration
  TURNING           — Skid-steer turn for timed duration
  AVOIDING          — Autonomous ultrasonic avoidance (reverse → turn → resume)
  CLIFF_REVERSING   — Backing away from detected cliff edge
  CLIFF_STOPPED     — Latched stop after cliff reverse (frozen until PI_GO)
  PI_STOPPED        — Latched stop from Pi camera (frozen until PI_GO)

Transitions:
  IDLE ──[DRIVE/REV/TURN]──► DRIVING/REVERSING/TURNING
  DRIVING/REVERSING/TURNING ──[timer expires]──► IDLE
  DRIVING/TURNING/AVOID-PHASE3 ──[cliff confirmed]──► CLIFF_REVERSING
  CLIFF_REVERSING ──[CLIFF_REV_MS expires]──► CLIFF_STOPPED
  CLIFF_STOPPED ──[PI_GO]──► IDLE
  DRIVING/TURNING ──[ultrasonic < US_THRESH]──► AVOIDING
  AVOIDING ──[maneuver complete]──► IDLE
  ANY STATE ──[PI_STOP]──► PI_STOPPED
  PI_STOPPED ──[PI_GO]──► IDLE
  ANY NON-LATCHED STATE ──[STOP]──► IDLE
```

---

## 5. Conventions
- Both left motors and both right motors are paired channel-wise on a single L298N driver board (skid-steer mode).
- **Directional Pin Polarity**:
  - Forward: Right motor Channel A (`IN1=LOW, IN2=HIGH`), Left motor Channel B (`IN3=LOW, IN4=HIGH`).
  - Reverse: Right motor Channel A (`IN1=HIGH, IN2=LOW`), Left motor Channel B (`IN3=HIGH, IN4=LOW`).
- **Startup State**: Motors initialize disarmed and stopped (PWM 0, all IN pins LOW) upon MCU boot/reset; no autonomous movement occurs until explicit user command is received.
- **Minimum Speed Floor**: All active driving PWM speeds are clamped to $\ge 180$ PWM for straight drive (DRIVE, REV), while all skid-steer turn commands (TURN) enforce a minimum motor PWM floor of $\ge 200$ to overcome vehicle weight and friction during turns (speed 0 is reserved for stop).
- Pin mappings are centralized in `arduino/lib/pins.h`.
- Ultrasonic, cliff detection, and avoidance timing constants are in `arduino/lib/protocol.h`.
- Any modification to this protocol must originate from `firmware-agent` and be mirrored in `car/control/motor_controller.py`.
