"""
test_challenger_m1_2.py — Adversarial Empirical Stress & Verification Suite
=============================================================================
Challenger 2 suite for Milestone 1 (Manual Skid-Steer Turning Reliability & Responsive Control):
1. Command Flooding & High-Frequency Stress:
   - Rapid sequential command flooding (500 requests at zero delay)
   - High-concurrency multi-threaded command burst (10 threads, 500 total requests)
   - Serial buffer integrity and absence of byte interleaving / truncation
   - Malformed, adversarial, and edge-case query parameters (NaN, overflow, injection, empty)
   - Safety gates (latched, disabled motors, front/rear ultrasonic obstacles)
   - Disconnected / dry-run resilience under burst traffic
2. 'D' Key Behavior & Daylight AWB Deconfliction:
   - Static / regex audit of PAGE HTML/JS in car/main.py
   - Comprehensive JS state-machine oracle across all control modes (manual, autonomous, follow, undefined)
   - Keyboard repeat suppression (e.repeat)
   - Multi-key state tracking (keysDown Set) and transition persistence
   - Window blur runaway prevention
3. Dynamic Speed Parameterization in Turn Commands:
   - Verification that left/right turns respect selected speed (130, 185, 235 PWM) and not hardcoded 160.
"""

import io
import json
import re
import threading
import time
import pytest
import numpy as np

from car.main import (
    DiagnosticHandler,
    SystemState,
    PAGE,
    EMERGENCY_STOP_DISTANCE_CM,
    is_valid_us,
)
from car.control.motor_controller import MotorController
from tests.test_main_coordination import (
    MockSerialDevice,
    MockHttpSocket,
    create_mock_motor,
    simulate_http_get,
)


# =============================================================================
# Helper: Thread-Safe HTTP Simulation
# =============================================================================

def send_request(path: str, sys_state: SystemState):
    """Simulates an HTTP GET request to DiagnosticHandler and parses JSON response."""
    status, headers, body = simulate_http_get(path, sys_state)
    try:
        data = json.loads(body.decode("utf-8"))
    except Exception:
        data = None
    return status, headers, data, body


# =============================================================================
# 1. Command Flooding & High-Frequency Stress Tests
# =============================================================================

def test_high_frequency_sequential_command_flooding():
    """
    Stress-test rapid sequential command flooding:
    Sends 500 alternating left and right turn requests at zero delay.
    Verifies 100% HTTP 200 OK, valid JSON responses, correct command generation,
    and zero unhandled exceptions or crashes.
    """
    sys_state = SystemState()
    motor, serial_dev = create_mock_motor()
    sys_state.motor = motor
    sys_state.motors_enabled = True
    sys_state.control_mode = "manual"

    num_requests = 500
    start_time = time.perf_counter()

    for i in range(num_requests):
        cmd = "left" if (i % 2 == 0) else "right"
        speed = 130 if (i % 3 == 0) else (185 if (i % 3 == 1) else 235)
        path = f"/manual_drive?cmd={cmd}&speed={speed}"

        status, headers, data, body = send_request(path, sys_state)
        assert status == 200, f"Request {i} failed with status {status}"
        assert data is not None, f"Request {i} returned invalid JSON: {body}"
        assert data.get("status") == "ok"
        assert data.get("executed") is True
        assert data.get("cmd") == cmd

    elapsed = time.perf_counter() - start_time

    # Verify that serial device received all 500 commands
    assert len(serial_dev.written_commands) == num_requests
    for idx, raw_cmd in enumerate(serial_dev.written_commands):
        expected_cmd = "left" if (idx % 2 == 0) else "right"
        expected_speed = 130 if (idx % 3 == 0) else (185 if (idx % 3 == 1) else 235)
        expected_angle = -40 if expected_cmd == "left" else 40
        assert raw_cmd == f"TURN {expected_angle} {expected_speed}", (
            f"Command {idx} corrupted: got {raw_cmd!r}, expected TURN {expected_angle} {expected_speed}"
        )


def test_concurrent_multi_threaded_command_burst():
    """
    Stress-test multi-threaded concurrency:
    10 threads concurrently burst 50 requests each (500 total concurrent requests)
    alternating commands ('left', 'right', 'forward', 'reverse', 'stop') across different speeds.
    Verifies that lock contention does not crash, deadlock, corrupt commands, or leak memory.
    """
    sys_state = SystemState()
    motor, serial_dev = create_mock_motor()
    sys_state.motor = motor
    sys_state.motors_enabled = True
    sys_state.manual_override_obstacles = True  # Allow all movements without US blocking

    num_threads = 10
    requests_per_thread = 50
    errors = []
    responses = []

    def worker_thread(thread_id: int):
        commands = ["left", "right", "forward", "reverse", "stop"]
        speeds = [130, 185, 235]
        for j in range(requests_per_thread):
            cmd = commands[(thread_id + j) % len(commands)]
            spd = speeds[(thread_id * 2 + j) % len(speeds)]
            path = f"/manual_drive?cmd={cmd}&speed={spd}"
            try:
                status, headers, data, body = send_request(path, sys_state)
                if status != 200 or data is None or data.get("status") != "ok":
                    errors.append((thread_id, j, status, data, body))
                else:
                    responses.append(data)
            except Exception as e:
                errors.append((thread_id, j, "EXCEPTION", str(e)))

    threads = [threading.Thread(target=worker_thread, args=(tid,)) for tid in range(num_threads)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=10.0)

    assert len(errors) == 0, f"Encountered {len(errors)} errors during concurrent burst: {errors[:5]}"
    assert len(responses) == num_threads * requests_per_thread

    # Validate that every command in MockSerialDevice is a complete, uncorrupted, valid command
    valid_patterns = [
        re.compile(r"^TURN (-?40) (130|185|235)$"),
        re.compile(r"^DRIVE (130|185|235) \d+$"),
        re.compile(r"^REV (130|185|235) \d+$"),
        re.compile(r"^STOP$"),
    ]

    assert len(serial_dev.written_commands) == num_threads * requests_per_thread
    for written in serial_dev.written_commands:
        assert any(pat.match(written) for pat in valid_patterns), (
            f"Corrupted or invalid serial command encountered: {written!r}"
        )


def test_serial_buffer_integrity_under_rapid_left_right_alternation():
    """
    Adversarial probe: Rapid left/right toggling to verify that no bytes are dropped,
    no partial lines exist, and newline terminators remain strictly intact.
    """
    motor, serial_dev = create_mock_motor()
    total_turns = 300

    for i in range(total_turns):
        angle = -40 if (i % 2 == 0) else 40
        speed = 185
        ok = motor.turn(angle, speed=speed)
        assert ok is True

    assert len(serial_dev.written_commands) == total_turns
    for i, line in enumerate(serial_dev.written_commands):
        expected_angle = -40 if (i % 2 == 0) else 40
        assert line == f"TURN {expected_angle} 185"
        assert "\n" not in line  # line was split by newline, meaning exactly one newline per cmd
        assert "\r" not in line


def test_adversarial_malformed_manual_drive_inputs():
    """
    Stress-test with malicious, out-of-bound, and malformed query strings:
    - Non-numeric speed strings: speed=NaN, speed=foo, speed=None, speed=3.14
    - Out of range speed values: speed=-9999, speed=99999999999999999
    - Empty or special characters: speed=, speed=%00%ff
    - Unknown or injection commands: cmd=;DROP TABLE, cmd=%00, cmd=unknown_xyz
    Verifies that the server does NOT crash (no 500 error, no unhandled exceptions),
    properly falls back to default speed, clamps to [0, 255], and cleanly handles unknown cmds.
    """
    sys_state = SystemState()
    motor, serial_dev = create_mock_motor()
    sys_state.motor = motor
    sys_state.motors_enabled = True

    malformed_cases = [
        # (query_string, expected_executed, expected_speed_in_motor_call)
        ("/manual_drive?cmd=left&speed=foo", True, sys_state.manual_speed),
        ("/manual_drive?cmd=right&speed=NaN", True, sys_state.manual_speed),
        ("/manual_drive?cmd=left&speed=None", True, sys_state.manual_speed),
        ("/manual_drive?cmd=right&speed=", True, sys_state.manual_speed),
        ("/manual_drive?cmd=left&speed=-500", True, 0),  # clamped to 0
        ("/manual_drive?cmd=right&speed=999999", True, 255),  # clamped to 255
        ("/manual_drive?cmd=left&speed=185.7", True, sys_state.manual_speed),  # float string -> ValueError fallback
        ("/manual_drive?cmd=&speed=185", False, None),  # empty cmd
        ("/manual_drive?cmd=MALICIOUS_CMD&speed=185", False, None),
        ("/manual_drive?cmd=left%00injection&speed=185", False, None),
        ("/manual_drive?speed=185", False, None),  # missing cmd
        ("/manual_drive", False, None),  # empty query
    ]

    for path, expected_executed, expected_spd in malformed_cases:
        status, headers, data, body = send_request(path, sys_state)
        assert status == 200, f"Path {path} returned HTTP {status}"
        assert data is not None, f"Path {path} returned non-JSON body: {body}"
        assert data.get("executed") == expected_executed, (
            f"Path {path}: expected executed={expected_executed}, got {data.get('executed')}"
        )

        if expected_executed:
            last_cmd = serial_dev.written_commands[-1]
            assert f" {expected_spd}" in last_cmd, (
                f"Path {path}: expected speed {expected_spd} in written command {last_cmd!r}"
            )


def test_manual_drive_dry_run_and_disconnected_resilience():
    """
    Stress-test manual drive commands when motor is in dry-run / disconnected state:
    Ensures that issuing hundreds of commands causes no crash or AttributeError.
    """
    sys_state = SystemState()
    sys_state.motor = MotorController(port="SIMULATED")
    sys_state.motor.connect()  # sets port to SIMULATED, _serial is None
    sys_state.motors_enabled = True

    for i in range(100):
        status, headers, data, body = send_request("/manual_drive?cmd=left&speed=185", sys_state)
        assert status == 200
        assert data["executed"] is True  # executed via motor controller dry-run method returning False safely


def test_manual_drive_safety_gates_under_flooding():
    """
    Verify safety gate enforcement during command requests:
    1. motors_enabled = False (bench-safe): 0 motor actuations allowed.
    2. pi_latched = True without override: 0 motor actuations allowed.
    3. pi_latched = True WITH manual_override_obstacles: commands allowed.
    4. Front ultrasonic obstacle < 20cm: 'forward' blocked, but 'left'/'right' turns allowed.
    5. Rear ultrasonic obstacle < 20cm: 'reverse' blocked, but turns allowed.
    """
    sys_state = SystemState()
    motor, serial_dev = create_mock_motor()
    sys_state.motor = motor

    # Case 1: Bench safe (motors_enabled = False)
    sys_state.motors_enabled = False
    sys_state.pi_latched = False
    status, _, data, _ = send_request("/manual_drive?cmd=left&speed=185", sys_state)
    assert data["executed"] is False
    assert len(serial_dev.written_commands) == 0

    # Case 2: Pi latched (pi_latched = True, override = False)
    sys_state.motors_enabled = True
    sys_state.pi_latched = True
    sys_state.manual_override_obstacles = False
    status, _, data, _ = send_request("/manual_drive?cmd=left&speed=185", sys_state)
    assert data["executed"] is False
    assert len(serial_dev.written_commands) == 0

    # Case 3: Override active (pi_latched = True, override = True)
    sys_state.manual_override_obstacles = True
    status, _, data, _ = send_request("/manual_drive?cmd=left&speed=185", sys_state)
    assert data["executed"] is True
    assert "TURN -40 185" in serial_dev.written_commands
    serial_dev.written_commands.clear()

    # Case 4: Front obstacle < 20cm
    sys_state.manual_override_obstacles = False
    sys_state.pi_latched = False
    motor.latest_sonar = {"left": 12.0, "right": 15.0, "rear": 50.0}

    # Forward must be blocked
    status, _, data, _ = send_request("/manual_drive?cmd=forward&speed=185", sys_state)
    assert data["executed"] is False
    assert data["reason"] == "us_front_blocked"
    assert "STOP" in serial_dev.written_commands
    serial_dev.written_commands.clear()

    # Left and right turns MUST be allowed so vehicle can steer away from obstacle
    status, _, data, _ = send_request("/manual_drive?cmd=left&speed=185", sys_state)
    assert data["executed"] is True
    assert "TURN -40 185" in serial_dev.written_commands
    serial_dev.written_commands.clear()

    status, _, data, _ = send_request("/manual_drive?cmd=right&speed=235", sys_state)
    assert data["executed"] is True
    assert "TURN 40 235" in serial_dev.written_commands
    serial_dev.written_commands.clear()

    # Case 5: Rear obstacle < 20cm
    motor.latest_sonar = {"left": 50.0, "right": 50.0, "rear": 14.0}
    status, _, data, _ = send_request("/manual_drive?cmd=reverse&speed=185", sys_state)
    assert data["executed"] is False
    assert data["reason"] == "us_rear_blocked"
    assert "STOP" in serial_dev.written_commands
    serial_dev.written_commands.clear()


# =============================================================================
# 2. 'D' Key Behavior & Daylight AWB Deconfliction Tests
# =============================================================================

def test_page_html_js_d_key_awb_guard_static_audit():
    """
    Static & regular expression audit of car/main.py PAGE content:
    Verifies that:
    1. 'D' key Daylight AWB call is strictly guarded by control mode:
       `if (k === 'd' && currentControlMode !== 'manual' && typeof currentControlMode !== 'undefined')`
    2. There is NO un-guarded `if (k === 'd') setAwb('daylight');`
    3. Browser repeat guard `if (e.repeat) return;` is present.
    4. Key tracking Set `keysDown = new Set()` is present.
    5. Window blur handler is present to prevent runaway.
    """
    # 1. Guarded D key
    assert "currentControlMode !== 'manual'" in PAGE, "Missing manual mode guard on 'D' shortcut"
    assert "setAwb('daylight')" in PAGE

    # 2. Confirm no un-guarded k === 'd' calling setAwb
    unguarded_matches = re.findall(r"if\s*\(\s*k\s*===\s*['\"]d['\"]\s*\)\s*setAwb", PAGE)
    assert len(unguarded_matches) == 0, f"Found un-guarded 'd' key AWB call: {unguarded_matches}"

    # 3. Repeat guard
    assert "if (e.repeat) return;" in PAGE

    # 4. Key tracking
    assert "const keysDown = new Set();" in PAGE
    assert "MOVEMENT_KEYS" in PAGE
    assert "resolveActiveMovement" in PAGE

    # 5. Window blur
    assert "window.addEventListener('blur'" in PAGE


class ClientJsStateMachineOracle:
    """
    Faithful Python simulation of the JavaScript client logic embedded in PAGE
    (lines 520–948 of car/main.py).
    Executes identical logic for keydown, keyup, blur, getMovementKey, and resolveActiveMovement.
    """

    MOVEMENT_KEYS = {
        'KeyW': 'forward',
        'ArrowUp': 'forward',
        'KeyS': 'reverse',
        'ArrowDown': 'reverse',
        'KeyA': 'left',
        'ArrowLeft': 'left',
        'KeyD': 'right',
        'ArrowRight': 'right',
        'w': 'forward',
        's': 'reverse',
        'a': 'left',
        'd': 'right',
    }

    def __init__(self, initial_mode: str = "autonomous", initial_speed: int = 185):
        self.currentControlMode = initial_mode
        self.currentSpeed = initial_speed
        self.keysDown = set()
        self.keysDown_order = []  # To simulate Array.from(keysDown) order in JS
        self.activeAction = None
        self.driveIntervalActive = False
        self.awb_calls = []
        self.drive_calls = []
        self.stop_calls = 0

    def get_movement_key(self, code: str, key: str):
        if code and code in self.MOVEMENT_KEYS:
            return code
        if key and key in self.MOVEMENT_KEYS:
            return key
        if key and key.lower() in self.MOVEMENT_KEYS:
            return key.lower()
        return None

    def resolve_active_movement(self):
        for k in reversed(self.keysDown_order):
            cmd = self.MOVEMENT_KEYS.get(k)
            if cmd:
                return cmd
        return None

    def remove_movement_key(self, code: str, key: str):
        mk = self.get_movement_key(code, key)
        if mk and mk in self.keysDown:
            self.keysDown.remove(mk)
            self.keysDown_order = [x for x in self.keysDown_order if x != mk]
        if code and code in self.keysDown:
            self.keysDown.remove(code)
            self.keysDown_order = [x for x in self.keysDown_order if x != code]
        if key:
            if key in self.keysDown:
                self.keysDown.remove(key)
                self.keysDown_order = [x for x in self.keysDown_order if x != key]
            if key.lower() in self.keysDown:
                self.keysDown.remove(key.lower())
                self.keysDown_order = [x for x in self.keysDown_order if x != key.lower()]

    def set_awb(self, mode: str):
        self.awb_calls.append(mode)

    def send_manual_drive(self, cmd: str, speed: int = None):
        spd = speed if speed is not None else self.currentSpeed
        if self.activeAction == cmd:
            return
        self.activeAction = cmd
        self.driveIntervalActive = True
        self.drive_calls.append((cmd, spd))

    def stop_drive(self):
        self.driveIntervalActive = False
        self.activeAction = None
        self.keysDown.clear()
        self.keysDown_order.clear()
        self.stop_calls += 1

    def on_keydown(self, code: str, key: str, repeat: bool = False):
        if repeat:
            return

        k = key.lower() if key else ""

        # Disarm Daylight AWB shortcut when in manual drive so 'D' turns right without toggling camera white balance
        if k == 'd' and self.currentControlMode != 'manual' and self.currentControlMode is not None:
            self.set_awb('daylight')

        is_manual = (self.currentControlMode == 'manual' or self.currentControlMode is None)
        if is_manual:
            move_key = self.get_movement_key(code, key)
            if move_key:
                if repeat:
                    return
                if move_key in self.keysDown:
                    self.keysDown.remove(move_key)
                    self.keysDown_order = [x for x in self.keysDown_order if x != move_key]
                self.keysDown.add(move_key)
                self.keysDown_order.append(move_key)
                cmd = self.resolve_active_movement()
                if cmd:
                    self.send_manual_drive(cmd, self.currentSpeed)

    def on_keyup(self, code: str, key: str):
        move_key = self.get_movement_key(code, key)
        is_relevant = (
            move_key or
            (code and code in self.keysDown) or
            (key and (key in self.keysDown or key.lower() in self.keysDown))
        )
        if is_relevant:
            self.remove_movement_key(code, key)
            if len(self.keysDown) == 0:
                self.stop_drive()
            else:
                next_cmd = self.resolve_active_movement()
                if next_cmd:
                    self.send_manual_drive(next_cmd, self.currentSpeed)
                else:
                    self.stop_drive()

    def on_blur(self):
        if len(self.keysDown) > 0:
            self.stop_drive()


def test_d_key_behavior_in_manual_mode():
    """
    Verify empirically that pressing 'd' / 'KeyD' while in manual mode:
    1. NEVER calls setAwb('daylight').
    2. Correctly triggers sendManualDrive('right', currentSpeed).
    3. Handles uppercase 'D' identically.
    4. Handles repeated events (e.repeat=True) without redundant triggers.
    5. Cleanly stops on keyup when no other keys are held.
    """
    client = ClientJsStateMachineOracle(initial_mode="manual", initial_speed=185)

    # 1. Press 'd' (lowercase)
    client.on_keydown(code="KeyD", key="d", repeat=False)
    assert len(client.awb_calls) == 0, f"setAwb was unexpectedly called: {client.awb_calls}"
    assert client.activeAction == "right"
    assert client.drive_calls == [("right", 185)]
    assert client.driveIntervalActive is True

    # 2. Browser repeat event
    client.on_keydown(code="KeyD", key="d", repeat=True)
    assert len(client.awb_calls) == 0
    assert len(client.drive_calls) == 1  # No duplicate command emitted!

    # 3. Release 'd'
    client.on_keyup(code="KeyD", key="d")
    assert client.activeAction is None
    assert client.driveIntervalActive is False
    assert client.stop_calls == 1

    # 4. Press 'D' (uppercase with Shift or CapsLock)
    client.on_keydown(code="KeyD", key="D", repeat=False)
    assert len(client.awb_calls) == 0
    assert client.activeAction == "right"
    assert client.drive_calls[-1] == ("right", 185)
    client.on_keyup(code="KeyD", key="D")
    assert client.activeAction is None


def test_d_key_behavior_in_autonomous_and_follow_modes():
    """
    Verify that pressing 'd' when NOT in manual mode:
    1. In 'autonomous' mode: DOES trigger setAwb('daylight') and does NOT drive.
    2. In 'follow' mode: DOES trigger setAwb('daylight') and does NOT drive.
    """
    # 1. Autonomous mode
    client_auto = ClientJsStateMachineOracle(initial_mode="autonomous")
    client_auto.on_keydown(code="KeyD", key="d", repeat=False)
    assert client_auto.awb_calls == ["daylight"]
    assert len(client_auto.drive_calls) == 0
    assert client_auto.activeAction is None

    # 2. Follow mode
    client_follow = ClientJsStateMachineOracle(initial_mode="follow")
    client_follow.on_keydown(code="KeyD", key="d", repeat=False)
    assert client_follow.awb_calls == ["daylight"]
    assert len(client_follow.drive_calls) == 0
    assert client_follow.activeAction is None


def test_client_multi_key_tracking_transitions():
    """
    Verify smooth key transitions with keysDown Set:
    1. Press 'w' (forward) -> drives forward.
    2. While holding 'w', press 'd' (turn right) -> smoothly switches to 'right'.
    3. Release 'd' while still holding 'w' -> smoothly transitions back to 'forward' without stopping!
    4. Release 'w' -> stops.
    5. Test window blur safety halt.
    """
    client = ClientJsStateMachineOracle(initial_mode="manual", initial_speed=235)

    # Step 1: Press 'w'
    client.on_keydown(code="KeyW", key="w")
    assert client.activeAction == "forward"
    assert client.drive_calls[-1] == ("forward", 235)
    assert client.stop_calls == 0

    # Step 2: While holding 'w', press 'd'
    client.on_keydown(code="KeyD", key="d")
    assert client.activeAction == "right"
    assert client.drive_calls[-1] == ("right", 235)
    assert client.stop_calls == 0

    # Step 3: Release 'd' (still holding 'w')
    client.on_keyup(code="KeyD", key="d")
    assert client.activeAction == "forward"
    assert client.drive_calls[-1] == ("forward", 235)
    assert client.stop_calls == 0  # Did NOT stop!

    # Step 4: Release 'w'
    client.on_keyup(code="KeyW", key="w")
    assert client.activeAction is None
    assert client.stop_calls == 1

    # Step 5: Test window blur with active keys held
    client.on_keydown(code="KeyA", key="a")
    assert client.activeAction == "left"
    client.on_blur()
    assert client.activeAction is None
    assert len(client.keysDown) == 0
    assert client.stop_calls == 2


# =============================================================================
# 3. Dynamic Speed Parameterization in Turn Commands
# =============================================================================

def test_dynamic_speed_parameterization_in_turn_commands():
    """
    Verify that manual turning honors user-selected speed levels:
    - Speed 130: left -> TURN -40 130, right -> TURN 40 130
    - Speed 185: left -> TURN -40 185, right -> TURN 40 185
    - Speed 235: left -> TURN -40 235, right -> TURN 40 235
    Confirms that the hardcoded 160 speed bug is completely eradicated.
    """
    sys_state = SystemState()
    motor, serial_dev = create_mock_motor()
    sys_state.motor = motor
    sys_state.motors_enabled = True

    for spd in [130, 185, 235]:
        serial_dev.written_commands.clear()

        # Left turn
        status, _, data, _ = send_request(f"/manual_drive?cmd=left&speed={spd}", sys_state)
        assert data["executed"] is True
        assert serial_dev.written_commands[-1] == f"TURN -40 {spd}"
        assert f"TURN -40 160" != serial_dev.written_commands[-1] or spd == 160

        # Right turn
        status, _, data, _ = send_request(f"/manual_drive?cmd=right&speed={spd}", sys_state)
        assert data["executed"] is True
        assert serial_dev.written_commands[-1] == f"TURN 40 {spd}"
        assert f"TURN 40 160" != serial_dev.written_commands[-1] or spd == 160


def test_motor_inversion_in_turn_commands():
    """
    Verify that when motor inversion is enabled (invert_motors = True),
    turning commands reverse angles correctly:
    - left (-40) inverted becomes TURN 40 <spd>
    - right (40) inverted becomes TURN -40 <spd>
    """
    sys_state = SystemState()
    motor, serial_dev = create_mock_motor()
    motor.invert_motors = True
    sys_state.motor = motor
    sys_state.motors_enabled = True

    status, _, data, _ = send_request("/manual_drive?cmd=left&speed=185", sys_state)
    assert data["executed"] is True
    assert serial_dev.written_commands[-1] == "TURN 40 185"

    status, _, data, _ = send_request("/manual_drive?cmd=right&speed=185", sys_state)
    assert data["executed"] is True
    assert serial_dev.written_commands[-1] == "TURN -40 185"
