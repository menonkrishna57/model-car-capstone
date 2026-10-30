# Traffic Sign

This folder contains an isolated copy of the car runtime with automatic four-sign decisions and dashboard integration. Launch this version to use the new feature; the repository's original `car/` runtime is unchanged by this folder.

**No trained traffic-sign weights or training images are included.** Real sign recognition requires a custom four-class checkpoint or NCNN export. The tests use simulated predictions and hardware. They do not demonstrate recognition accuracy, Raspberry Pi FPS, or physical turn accuracy.

## Files

- `run.py`: entry point for this runtime.
- `car/main.py`: driving decisions, sign arbitration, camera capture and dashboard.
- `car/perception/traffic_signs.py`: optional rate-limited detector and camera geometry.
- `car/control/traffic_sign_policy.py`: confirmation, STOP dwell and No Entry state.
- `car/control/motor_controller.py`: existing Uno protocol with sonar receipt timestamps.
- `tests/`: sign tests and existing driving regression tests.
- `GUIDE.md`: dataset, training, calibration and operating details.
- `dataset.example.yaml`: four-class YOLO dataset configuration template.

## Setup and run

Use a Python environment with the repository's Pi runtime dependencies. On Raspberry Pi OS, follow the repository's camera/interface setup and ensure Picamera2 is accessible from your Python environment.

From the **repository root**:

```bash
python -m pip install -r "Traffic Sign/car/requirements.txt"
python "Traffic Sign/run.py" --help
python "Traffic Sign/run.py" --mode lane --bench-safe --traffic-sign-model /absolute/path/to/trained_sign_ncnn_model
```

The sign model must have the classes `stop`, `no_entry`, `turn_left`, and `turn_right`. Stock COCO obstacle weights do not supply this four-class model. Omitting `--traffic-sign-model` disables sign handling. `--bench-safe` is for stationary inspection; movement is initially disabled. Enable movement only after checking detection, actual camera orientation, motor direction and calibration on the physical track.

The copied runtime resolves general obstacle weights from `../benchmarking/models/` and pothole weights from `../pothole_detection/`, relative to this folder. Keep `Traffic Sign` inside this repository. It does not download or duplicate those assets.

## Automatic decisions

| Sign | Behavior |
|---|---|
| STOP | Confirm across fresh observations, command STOP, hold three seconds, then allow normal safety checks to govern resuming. |
| No Entry | Stop until explicit sign-policy reset. |
| Compulsory Left / Right | Confirm and approach; automatically turn when apparent size and camera bearing meet the approach gate. No junction button is required. |

The dashboard shows the sign, bearing, pending instruction, action and faults; the stream overlays sign boxes. Existing obstacle decisions remain in control when no sign maneuver is active. Obstacles, unavailable sensing or disabled motors inhibit sign turns.

Lane mode uses bounded turn pulses, observes lane departure and then requires centered lane reacquisition. Timeout or interruption holds a fault until reset. Autonomous mode uses timed turning and cannot verify physical yaw. The camera's configured HFOV is 53.5 degrees; apparent-size gating is a track heuristic, not metric ranging or a general junction detector.

The documented chassis pivot default is PWM 210; `--traffic-turn-pwm` and `--traffic-turn-ms-per-degree` allow calibration. Timing must match the Uno's actual `TURN_CAL`. See `GUIDE.md` before running on the car.

## Tests

Run from this folder so imports exercise this copy of the runtime:

```bash
cd "Traffic Sign"
python -m pip install pytest
python -m pytest -q tests
```

For model training, collect and label examples of the actual Indian-standard prints. Keep left/right arrow labels correct when augmenting images. Copy and edit `dataset.example.yaml`; it is only a template, not a supplied dataset.
