# AGENTS.md — Autonomous RC Car Capstone

This file configures the Antigravity coding agent (and its subagents) for the
MPSTME/NMIMS autonomous RC car capstone project. It gives the top-level agent
enough context to work independently and defines a set of subagents that own
distinct parts of the stack. Route work to the narrowest subagent that fits;
use the top-level agent only for orchestration, cross-cutting changes, and
final integration.

## 1. Project summary

Two-layer autonomous RC car:
- **Arduino Nano** — real-time motor control (PWM via L298N, turn-then-drive
  state machine, IMU heading hooks, serial telemetry).
- **Raspberry Pi 5** — perception (YOLOv10n via NCNN, CSI global-shutter
  camera, ICM20948 IMU over I2C, 3x HC-SR04P ultrasonic array) and high-level
  navigation, talking to the Nano over serial (USB or GPIO UART — undecided).

Team: Krishna (hardware + integration + C++ control loop, lead), Aditya
(YOLO pipeline/fine-tuning), Parth (ultrasonic logic + assembly). Krishna is
the primary user of this agent and has the strongest hardware/software
background on the team.

Chassis: 2-layer acrylic 4WD, TT motors, single L298N in skid-steer
(both-left / both-right channel pairing). Power: 2S 18650 + 5V/5A buck
converter for motor/Arduino rail; boAt EnergyShroom PB400 dedicated to the
Pi 5. No wheel encoders — navigation fallback is IMU-aided dead reckoning
(gyro heading + PWM-calibrated distance estimate).

## 2. Hard constraints (apply to every subagent)

- **No destructive changes to shared hardware.** Components are borrowed
  from a college lab under a 15/60-day ID rotation policy shared across the
  team. Never suggest cutting cables, desoldering, or any other
  irreversible modification. Flag anything destructive instead of doing it.
- **Two-layer architecture is not negotiable.** Do not propose collapsing
  control into the Pi alone — GPIO PWM jitter under YOLO load and resource
  contention are the reason the Nano owns real-time control.
- **Uno→Nano is a drop-in swap** (same ATmega328P) — don't reintroduce
  Uno-specific assumptions (e.g. shield stacking) in new code or docs.
- Once a solution path is confirmed unavailable (currently: micro-HDMI —
  none on hand, hard to source), do not re-suggest it. Treat it as closed
  unless the user reopens it.
- Prefer granular, step-by-step output with exact pin names / wire-by-wire
  instructions when producing hardware guidance — match Krishna's stated
  working style.
- Benchmarking claims about the Pi 5 must be framed honestly: Oracle Cloud
  Ampere A1 results are an **optimistic upper bound** (same ARM lineage,
  not the same SoC), paired with published Pi 5 literature numbers
  (~125–145 ms/frame for a YOLO-nano model via NCNN) as the realistic range.

## 3. Repo layout (target — create if missing)

```
/firmware/arduino/          # Nano sketches (point_to_point_control.ino, bench tests)
/perception/                # Pi-side Python: camera, YOLO/NCNN, IMU, ultrasonic
/perception/serial_bridge/  # Pi<->Nano serial handshake code
/sim/                       # Oracle Cloud benchmark notebooks, yolo-env setup scripts
/tools/component_tracker/   # Excel workbook + any scripts that regenerate it
/docs/                      # Architecture notes, wiring diagrams, ADRs
AGENTS.md
README.md
```

## 4. Global conventions

- **Arduino/C++**: state machines over deep call chains; every motor/IMU
  action gets a serial telemetry line for debugging (9600 baud). Keep pin
  mappings as named constants at the top of the file, matching whatever the
  current wiring doc says (see `/docs`).
- **Python (Pi + sim)**: use the `yolo-env` venv conventions (ultralytics,
  onnxruntime). Keep inference code swappable between the Oracle Cloud proxy
  and the real Pi 5 — no hardcoded ARM/Pi-only assumptions in the model code
  itself; isolate GPIO/CSI/I2C calls behind a thin hardware interface so the
  same perception code can run on the cloud proxy for benchmarking.
- **Commits/PRs**: one subagent's work per commit where possible, so
  integration diffs stay reviewable by teammates who aren't Krishna.
- Never fabricate benchmark numbers, sensor readings, or test results —
  mark anything not yet run on real hardware as "unverified / simulated."

## 5. Subagents

### 5.1 `firmware-agent` — Arduino / real-time control
**Owns:** `/firmware/arduino/**`
**Responsibilities:**
- Motor control state machines (turn-then-drive, skid-steer mixing).
- IMU heading integration hooks (consumes heading from Pi or onboard calc).
- Serial telemetry protocol definition and implementation.
- Bench test sketches (single motor → full 4WD scale-up).
**Constraints:** Target ATmega328P (Nano). No shield-stacking assumptions.
Keep serial protocol documented in `/docs/serial_protocol.md` and update it
whenever the message format changes — the perception-agent depends on it.
**Does not:** touch Python/perception code, decide the Wi-Fi/networking
approach, or modify the component tracker.

### 5.2 `perception-agent` — Pi 5 / camera / YOLO / sensors
**Owns:** `/perception/**` (excluding `serial_bridge/`, see 5.3)
**Responsibilities:**
- Camera capture pipeline (CSI, global shutter).
- YOLOv10n via NCNN inference: loading, pre/post-processing, lane
  detection, obstacle avoidance, traffic sign recognition.
- ICM20948 IMU reads over I2C; HC-SR04P ultrasonic array reads/filtering.
- IMU-aided dead-reckoning fallback logic (no encoders available).
**Constraints:** Must run against the confirmed interface set (Camera/CSI,
I2C, Serial Port hardware with login-shell-over-serial disabled). Do not
assume the Pi has network access when writing core perception logic — that
is a separate concern owned by `network-agent`.
**Does not:** own the Nano-facing serial framing (defers to the protocol
doc from `firmware-agent`), touch cloud benchmarking scripts.

### 5.3 `integration-agent` — Pi↔Arduino bridge & full-stack wiring
**Owns:** `/perception/serial_bridge/**`
**Responsibilities:**
- Implements the Pi-side half of the serial handshake against the protocol
  documented by `firmware-agent`.
- Resolves and implements the USB-serial vs GPIO-UART decision once made;
  until decided, keep the transport behind an interface so either works.
- End-to-end wiring of perception outputs → control commands.
- Owns the eventual full 4WD integration test plan.
**Constraints:** Must not modify `firmware-agent`'s sketch files or
`perception-agent`'s inference code directly — request changes from those
subagents via the protocol/interface doc instead of reaching across
boundaries.

### 5.4 `network-agent` — Pi headless setup & connectivity
**Owns:** `/docs/networking/**`, any `network-config`/cloud-init assets
**Responsibilities:**
- Headless Wi-Fi bring-up for the Pi 5 (currently targeting hotspot SSID
  "Krogs57's Network").
- cloud-init `network-config` authoring — remember this is **one-shot**:
  once the OS has booted with a given config, edits require a full reflash,
  not a live edit. Always produce a fresh image-writing plan rather than
  "just edit the file on the running system."
- SSH/Raspberry Pi Connect setup once network access is confirmed.
- Distinct-hostname assignment per team member to avoid SSH collisions on
  shared lab Wi-Fi; Asia/Kolkata timezone.
**Constraints:** No micro-HDMI is available and is currently out of scope
to source — do not propose display-cable-based debugging. A Caps-Lock LED
test has already confirmed the OS itself boots fine, so treat this purely
as a Wi-Fi/connectivity problem, not a boot problem. Required interfaces to
(re)enable on every reflash: Camera, I2C, Serial Port hardware (with login
shell over serial **disabled**, to keep UART free for the Arduino link).
**Escalates to user:** any step that requires physical access to the Pi
(swapping storage, plugging in a cable) rather than something scriptable.

### 5.5 `sim-bench-agent` — Oracle Cloud benchmarking proxy
**Owns:** `/sim/**`
**Responsibilities:**
- Maintains the `raspberrypi5-simulation` Oracle Cloud A1 instance setup
  (Ubuntu 24.04 aarch64, `yolo-env` venv: ultralytics, onnxruntime,
  JupyterLab).
- Writes/updates the YOLOv10n benchmark notebook(s).
- Reports results paired with published Pi 5 NCNN benchmarks, always
  labeled as an optimistic upper bound, never presented as equivalent to
  real Pi 5 performance.
**Constraints:** Remember the instance has an **ephemeral IP** that can
change on restart — any setup script or doc should not hardcode the IP.
"Stop" preserves the boot volume; "Terminate" is destructive and
irreversible — never invoke/recommend Terminate without explicit
confirmation from the user.

### 5.6 `docs-tracker-agent` — Component tracking & documentation
**Owns:** `/tools/component_tracker/**`, `/docs/*.md` (non-networking)
**Responsibilities:**
- Maintains the four-sheet Excel component tracker (paste-and-parse entry,
  ID rotation log, rotation summary dashboard, policy reference).
- Keeps the 15/60-day ID rotation auto-logic correct: flag caps hit, and
  suggest the next available team member by cross-referencing full
  rotation history.
- Writes/updates architecture notes, wiring diagrams, ADRs, and the serial
  protocol doc (in coordination with `firmware-agent`).
**Constraints:** This is a shared-equipment logistics tool, not a hardware
control path — never let it drive actual pin/wiring decisions, only record
them.

## 6. Delegation rules for the top-level agent

1. If a task is entirely within one subagent's `Owns:` path, delegate to it
   directly and let it report back a short summary.
2. If a task spans subagents (e.g. "get the car driving end to end"), the
   top-level agent breaks it into per-subagent subtasks, sequences them,
   and calls `integration-agent` last to wire the pieces together.
3. If a task changes the serial protocol, the top-level agent must route
   the change through `firmware-agent` first, then notify
   `perception-agent`/`integration-agent` of the updated contract.
4. Any task involving Terminating cloud resources, reflashing storage, or
   physically modifying borrowed lab hardware requires explicit user
   confirmation before a subagent proceeds.

## 7. Current priority (as of last sync)

**Blocker:** Pi 5 headless Wi-Fi connectivity (`network-agent`). This gates
`perception-agent`, `integration-agent`, and any real (non-simulated)
benchmarking. Everything else can proceed in parallel using the Oracle
Cloud proxy (`sim-bench-agent`) and bench-top Arduino testing
(`firmware-agent`).

**Next up once unblocked:** run the YOLOv10n benchmark notebook on real Pi
5 hardware, implement the Pi↔Arduino serial handshake, and begin scaling
the single-motor bench test to full 4WD.
