# AGENTS.md — Autonomous RC Car Capstone

Configuration for the Antigravity coding agent and its subagents on the
MPSTME/NMIMS autonomous RC car capstone. Route work to the narrowest
subagent whose `Owns:` path covers it. The top-level agent is only for
orchestration, cross-cutting changes, and final integration.

## 1. Project summary

Two-layer autonomous RC car:
- **Arduino Uno** — real-time motor control (PWM via L298N, turn-then-drive
  state machine, IMU heading hooks, serial telemetry). *(Future scope: swap
  to a Nano for a smaller footprint — same ATmega328P, drop-in, no code
  changes expected.)*
- **Raspberry Pi 5** — perception (YOLOv10n via NCNN, CSI global-shutter
  camera, ICM20948 IMU over I2C, 2x HC-SR04P ultrasonic sensors, both front-facing) and
  high-level navigation, talking to the Uno over serial (USB or GPIO
  UART — undecided; keep transport swappable until settled).

Team: Krishna (hardware + integration + C++ control loop, lead, primary
agent user), Aditya (hardware/chassis), Parth (software).

Chassis: 2-layer acrylic 4WD, TT motors, single L298N in skid-steer
(both-left / both-right channel pairing). Power: 2S 18650 + 5V/5A buck
converter for motor/Arduino rail; boAt EnergyShroom PB400 dedicated to the
Pi 5. No wheel encoders — navigation fallback is IMU-aided dead reckoning
(gyro heading + PWM-calibrated distance estimate).

## 2. Hard constraints (apply to every subagent)

- **No destructive changes to shared hardware.** Components are borrowed
  from a college lab under a 15/60-day ID rotation policy shared across the
  team. Never cut, desolder, or otherwise irreversibly modify borrowed
  gear — flag it instead.
- **Two-layer architecture is not negotiable.** Never propose collapsing
  control into the Pi alone; GPIO PWM jitter under YOLO load and resource
  contention are exactly why the Uno owns real-time control.
- **Currently on Arduino Uno.** A future Nano migration is in scope (same
  ATmega328P, drop-in, zero code changes expected) but not yet done — don't
  write Uno-only assumptions (e.g. shield stacking) that would block it, and
  don't refer to the board as a Nano until the swap actually happens.
- **micro-HDMI is a closed path** (none on hand, hard to source) — do not
  re-suggest display-cable-based debugging.
- Prefer granular, step-by-step output with exact pin names and
  wire-by-wire breakdowns for hardware guidance.
- Benchmark against real Pi 5 hardware — no cloud/simulated proxy is in
  use for this project right now.
- Never fabricate benchmark numbers, sensor readings, or test results —
  mark anything not yet run on real hardware as "unverified / simulated."

## 3. Repo layout (authoritative)

```
.
├── arduino/                       # Uno firmware (Nano migration = future scope) — owned by firmware-agent
│   ├── motor_test/
│   │   └── motor_test.ino         # Motor forward/reverse bench sketch
│   ├── point_to_point_control/
│   │   └── point_to_point_control.ino
│   ├── lib/                       # Shared Arduino headers (pin maps, protocol constants)
│   └── PROTOCOL.md                # Serial telemetry / command format (source of truth)
│
├── car/                           # Pi 5 runtime — split across perception & integration
│   ├── main.py                    # Entry point: orchestrates STOP > AVOID > FOLLOW priority stack
│   ├── perception/                # owned by perception-agent
│   │   ├── detector.py            # NCNN detection wrapper + pothole classical CV
│   │   ├── avoidance.py           # Zone-scoring decision engine
│   │   ├── tracker.py             # KCF person tracker for follow-me
│   │   ├── sensors.py             # Parses Arduino telemetry for ultrasonic/IMU data
│   │   └── config.py              # Resolution, thresholds, serial port/baud
│   ├── control/                   # owned by integration-agent
│   │   ├── motor_controller.py    # Pi-side serial handshake (implements arduino/PROTOCOL.md)
│   │   ├── pid.py                 # PID steering controller
│   │   └── navigation.py          # IMU-aided dead-reckoning point-to-point logic
│   └── requirements.txt           # Runtime-only deps (picamera2, pyserial, ncnn, etc.)
│
├── benchmarking/                  # owned by bench-agent
│   ├── benchmark.py                # Latest benchmark script (runs directly on Pi 5 hardware)
│   ├── requirements.txt
│   ├── test_videos/
│   │   ├── dashcam_test.mp4
│   │   └── testvideovid1.mp4
│   ├── models/                     # Model weights & exports
│   │   ├── yolov10n.pt
│   │   ├── yolov10n_ncnn_model/
│   │   └── yolov10n_openvino_model/
│   ├── results/                    # Dated on-device benchmark run outputs
│   └── legacy/                     # Old Oracle Cloud proxy benchmark — inactive, kept for reference only
│       ├── benchmark.py
│       ├── NCNN_Model/
│       └── OpenVINO/
│
├── infra/                          # owned by network-agent
│   ├── pi-image/
│   │   ├── network-config          # cloud-init Wi-Fi config (one-shot — reflash, don't hot-edit)
│   │   └── user-data
│   └── SETUP.md                    # Headless bring-up, SSH/Pi Connect, interface enablement steps
│
├── tools/
│   └── component_tracker/          # owned by docs-tracker-agent
│       └── lab_component_tracker.xlsx
│
├── docs/                            # owned by docs-tracker-agent
│   ├── architecture.md              # Two-layer design rationale, wiring diagrams
│   └── decisions/                   # Short ADRs (e.g. USB-serial vs GPIO-UART)
│
├── .venv/
├── AGENTS.md
└── README.md                        # Project overview, current status, quickstart
```

## 4. Global conventions

- **Arduino/C++**: state machines over deep call chains; every motor/IMU
  action gets a serial telemetry line (9600 baud). Pin mappings are named
  constants, defined once in `arduino/lib/` and never duplicated inline.
- **Python (`car/`, `benchmarking/`)**: hardware access (GPIO/CSI/I2C/
  serial) stays behind a thin interface so `car/perception/detector.py`'s
  core logic can also run unmodified inside `benchmarking/` for offline
  testing. `car/requirements.txt` and `benchmarking/requirements.txt` stay
  independent — runtime-only deps never leak into the benchmarking env.
- **Serial protocol**: `arduino/PROTOCOL.md` is the single source of truth
  for message format and baud rate. Any change to it is a `firmware-agent`
  change, propagated to `car/control/motor_controller.py` by
  `integration-agent`.
- **Commits/PRs**: one subagent's work per commit where possible, so
  integration diffs stay reviewable by teammates who aren't Krishna.

## 5. Subagents

### 5.1 `firmware-agent` — Arduino / real-time control
**Owns:** `arduino/**`
**Responsibilities:** motor control state machines, 2x HC-SR04P ultrasonic polling (front-left, front-right), local ultrasonic avoidance FSM (Tier 2 priority), serial telemetry (115200 baud), bench sketches; keeps `arduino/PROTOCOL.md` current.
**Constraints:** ATmega328P (Uno, currently) target, no shield-stacking
assumptions. Nano migration is future scope — flag any change that would
make that swap harder, but don't do the migration itself unprompted.
**Does not:** touch `car/**` or `benchmarking/**`.

### 5.2 `perception-agent` — detection & sensing
**Owns:** `car/perception/**`
**Responsibilities:** NCNN detection wrapper loading exports from
`benchmarking/models/`, pothole classical CV detection, `avoidance.py` zone-scoring engine, `tracker.py` person tracker; runtime thresholds/config.
**Constraints:** must run against the confirmed interface set (Camera/CSI,
I2C, Serial Port with login-shell-over-serial disabled); no Pi-only
assumptions that would break running the same code path under
`benchmarking/`.
**Does not:** own `car/main.py`'s orchestration loop or anything in
`car/control/`. Ultrasonic hardware reads have moved to Arduino.

### 5.3 `integration-agent` — runtime wiring & Pi↔Arduino bridge
**Owns:** `car/main.py`, `car/control/**`, `car/requirements.txt`
**Responsibilities:** `motor_controller.py` implements the Pi-side half of
the handshake defined in `arduino/PROTOCOL.md`; `navigation.py` implements
IMU-aided dead-reckoning point-to-point driving; `main.py` wires
camera → `perception/` → decision → `control/` → Arduino end to end. Owns
the integration test plan (bench → tethered → free-run).
**Constraints:** requests changes to `arduino/**` or `car/perception/**`
from the owning subagent rather than editing across the boundary.

### 5.4 `bench-agent` — benchmarking
**Owns:** `benchmarking/**`
**Responsibilities:** maintains `benchmark.py`, `models/`, `test_videos/`,
and dated `results/`; runs and reports YOLOv10n/NCNN inference performance
directly on the real Pi 5. `legacy/` (old Oracle Cloud proxy benchmark) is
inactive and kept for reference only — never the source of truth, and no
new work goes into it.
**Constraints:** no cloud proxy is in use for this project — don't stand
one up or route benchmarking through it unless the user reopens that
option.
**Does not:** touch `car/**` — exported artifacts flow one-way from
`benchmarking/models/` into what `perception-agent` loads at runtime.

### 5.5 `network-agent` — Pi headless setup & connectivity
**Owns:** `infra/**`
**Responsibilities:** headless Wi-Fi bring-up; `network-config`/
`user-data` cloud-init authoring (this is **one-shot** — a full reflash is
required after any change, never a live edit of the running system); SSH /
Pi Connect setup; distinct per-teammate hostnames to avoid SSH collisions
on shared lab Wi-Fi; documents required interface enablement (Camera, I2C,
Serial Port with login-shell-over-serial disabled) in `infra/SETUP.md`.
**Constraints:** no micro-HDMI available — never propose display-cable
debugging.
**Escalates to user:** anything requiring physical access to the Pi
(storage swap, cable plug-in).

### 5.6 `docs-tracker-agent` — documentation & component tracking
**Owns:** `README.md`, `docs/**`, `tools/component_tracker/**`
**Responsibilities:** keeps `README.md` current (architecture summary,
repo map, status/blockers); maintains `lab_component_tracker.xlsx`
(15/60-day ID rotation across Krishna/Aditya/Parth — flags caps hit,
suggests next holder); writes architecture notes and short ADRs in
`docs/decisions/` (e.g. USB-serial vs GPIO-UART once decided).
**Constraints:** records decisions made by other subagents — never makes
pin/wiring/code decisions itself.

## 6. Delegation rules for the top-level agent

1. Task fits entirely inside one subagent's `Owns:` path → delegate
   directly, get a short summary back.
2. Task spans subagents (e.g. "get the car driving end to end") → break
   into per-subagent subtasks, sequence them, call `integration-agent`
   last to wire pieces together.
3. Task changes the serial protocol → route through `firmware-agent`
   first (updates `arduino/PROTOCOL.md`), then notify `perception-agent`/
   `integration-agent` of the new contract.
4. Reflashing storage or physically modifying borrowed lab hardware →
   requires explicit user confirmation before any subagent proceeds.

## 7. Current priority (as of last sync)

**Blocker:** Pi 5 headless Wi-Fi connectivity (`network-agent`, `infra/`).
This gates `perception-agent` and `integration-agent`. Benchmarking
(`bench-agent`) and bench-top Arduino testing (`firmware-agent`) can
proceed in parallel directly on the real Pi 5 / Uno hardware, no
simulation needed.

**Next up once unblocked:** run `benchmarking/benchmark.py` on real Pi 5
hardware, implement `car/control/motor_controller.py` against
`arduino/PROTOCOL.md`, and scale `arduino/motor_test/motor_test.ino`
toward `point_to_point_control.ino`, then wire it all through
`car/main.py`.