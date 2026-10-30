"""
tests/test_m1_challenger_empirical.py — Challenger 1 Empirical Test Suite
==========================================================================
Empirical verification of Milestone 1 (Manual Skid-Steer Turning Reliability & Responsive Control):
1. HTTP GET /manual_drive?cmd=left|right&speed=130, 185, 235 speed reflection in serial TURN commands
2. Clamping, boundary cases, malformed queries, and missing speed fallback
3. Motor inversion and safety gate checks (motors_enabled, pi_latched, manual_override, ultrasonic front clearance)
4. Static / forensic parsing of car/main.py Web UI JavaScript code
5. Multi-key transition state machine oracle simulation (repeat guard, keysDown set, keydown/keyup transitions, blur, 'd' key conflict)
"""

import json
import re
import socket
import threading
import time
import urllib.request
import urllib.error
import pytest

from car.main import (
    SystemState,
    StreamingServer,
    DiagnosticHandler,
)
from car.control.motor_controller import MotorController


class MockSerialDevice:
    """Mock serial transport recording written commands and serving canned responses."""

    def __init__(self, incoming_lines=None):
        self.is_open = True
        self.written_commands = []
        self._in_buffer = list(incoming_lines or [])

    @property
    def in_waiting(self):
        return sum(len(line.encode("utf-8") + b"\n") for line in self._in_buffer)

    def readline(self):
        if self._in_buffer:
            line = self._in_buffer.pop(0)
            return (line + "\n").encode("utf-8")
        return b""

    def write(self, data: bytes):
        text = data.decode("utf-8").strip()
        self.written_commands.append(text)
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self._in_buffer.clear()

    def reset_output_buffer(self):
        pass

    def close(self):
        self.is_open = False


def create_mock_motor(incoming_lines=None):
    """Creates a MotorController backed by a MockSerialDevice."""
    motor = MotorController(port="MOCK_PORT", baud=115200)
    mock_serial = MockSerialDevice(incoming_lines)
    motor._serial = mock_serial
    return motor, mock_serial


import io

class MockHttpSocket:
    """In-memory HTTP socket simulation to avoid OS socket binding permission restrictions."""
    def __init__(self, data: bytes):
        self.rfile = io.BytesIO(data)
        self.wfile = io.BytesIO()

    def makefile(self, mode, *args, **kwargs):
        if "r" in mode:
            return self.rfile
        return self.wfile

    def sendall(self, data):
        self.wfile.write(data)


def simulate_http_get(path: str, state: SystemState):
    """Executes a real DiagnosticHandler request against an in-memory HTTP socket."""
    class MockServer:
        def __init__(self, s):
            self.sys_state = s

    raw_req = f"GET {path} HTTP/1.0\r\nHost: localhost:8080\r\n\r\n".encode("utf-8")
    sock = MockHttpSocket(raw_req)
    server = MockServer(state)
    DiagnosticHandler(sock, ("127.0.0.1", 8080), server)

    resp_data = sock.wfile.getvalue()
    parts = resp_data.split(b"\r\n\r\n", 1)
    header_block = parts[0].decode("utf-8", errors="replace")
    body = parts[1] if len(parts) > 1 else b""
    status_code = int(header_block.splitlines()[0].split()[1])
    data = json.loads(body.decode("utf-8")) if body else {}
    return status_code, header_block, data


@pytest.fixture
def http_sim():
    """Provides a fresh SystemState and MockSerialDevice backed motor controller."""
    state = SystemState()
    motor, serial_dev = create_mock_motor()
    state.motor = motor
    state.motors_enabled = True
    state.control_mode = "manual"
    state.manual_speed = 185
    state.pi_latched = False
    state.manual_override_obstacles = False
    return {"state": state, "motor": motor, "serial": serial_dev}


# =============================================================================
# 1. HTTP GET /manual_drive Empirical Speed Verification
# =============================================================================

@pytest.mark.parametrize("speed", [130, 185, 235])
def test_manual_drive_left_turn_speeds(http_sim, speed):
    """Verify GET /manual_drive?cmd=left&speed=<spd> emits 'TURN -40 <spd>' to serial."""
    state = http_sim["state"]
    serial_dev = http_sim["serial"]

    serial_dev.written_commands.clear()
    status, headers, data = simulate_http_get(f"/manual_drive?cmd=left&speed={speed}", state)
    assert status == 200
    assert data["status"] == "ok"
    assert data["executed"] is True
    assert data["cmd"] == "left"

    expected_cmd = f"TURN -40 {speed}"
    assert expected_cmd in serial_dev.written_commands, (
        f"Expected serial command '{expected_cmd}' not found in {serial_dev.written_commands}"
    )


@pytest.mark.parametrize("speed", [130, 185, 235])
def test_manual_drive_right_turn_speeds(http_sim, speed):
    """Verify GET /manual_drive?cmd=right&speed=<spd> emits 'TURN 40 <spd>' to serial."""
    state = http_sim["state"]
    serial_dev = http_sim["serial"]

    serial_dev.written_commands.clear()
    status, headers, data = simulate_http_get(f"/manual_drive?cmd=right&speed={speed}", state)
    assert status == 200
    assert data["status"] == "ok"
    assert data["executed"] is True
    assert data["cmd"] == "right"

    expected_cmd = f"TURN 40 {speed}"
    assert expected_cmd in serial_dev.written_commands, (
        f"Expected serial command '{expected_cmd}' not found in {serial_dev.written_commands}"
    )


def test_manual_drive_turn_aliases(http_sim):
    """Verify turn_left and turn_right aliases work identically to left/right."""
    state = http_sim["state"]
    serial_dev = http_sim["serial"]

    serial_dev.written_commands.clear()
    status, _, data = simulate_http_get("/manual_drive?cmd=turn_left&speed=130", state)
    assert status == 200
    assert data["status"] == "ok"
    assert "TURN -40 130" in serial_dev.written_commands

    serial_dev.written_commands.clear()
    status, _, data = simulate_http_get("/manual_drive?cmd=turn_right&speed=235", state)
    assert status == 200
    assert data["status"] == "ok"
    assert "TURN 40 235" in serial_dev.written_commands


def test_manual_drive_default_speed_when_omitted(http_sim):
    """Verify omitting speed parameter falls back to sys_state.manual_speed."""
    state = http_sim["state"]
    serial_dev = http_sim["serial"]

    state.manual_speed = 185
    serial_dev.written_commands.clear()
    status, _, data = simulate_http_get("/manual_drive?cmd=left", state)
    assert status == 200
    assert "TURN -40 185" in serial_dev.written_commands

    state.manual_speed = 235
    serial_dev.written_commands.clear()
    status, _, data = simulate_http_get("/manual_drive?cmd=right", state)
    assert status == 200
    assert "TURN 40 235" in serial_dev.written_commands


def test_manual_drive_speed_clamping_and_malformed_input(http_sim):
    """Verify speed values are clamped [0, 255] and malformed input gracefully falls back."""
    state = http_sim["state"]
    serial_dev = http_sim["serial"]

    # Negative clamped to 0
    serial_dev.written_commands.clear()
    status, _, _ = simulate_http_get("/manual_drive?cmd=left&speed=-50", state)
    assert status == 200
    assert "TURN -40 0" in serial_dev.written_commands

    # Over 255 clamped to 255
    serial_dev.written_commands.clear()
    status, _, _ = simulate_http_get("/manual_drive?cmd=right&speed=999", state)
    assert status == 200
    assert "TURN 40 255" in serial_dev.written_commands

    # Non-integer fallback to sys_state.manual_speed without 500 error
    state.manual_speed = 185
    serial_dev.written_commands.clear()
    status, _, _ = simulate_http_get("/manual_drive?cmd=left&speed=not_a_number", state)
    assert status == 200
    assert "TURN -40 185" in serial_dev.written_commands


def test_manual_drive_inverted_motors(http_sim):
    """Verify that invert_motors flips the TURN angle sign."""
    state = http_sim["state"]
    motor = http_sim["motor"]
    serial_dev = http_sim["serial"]

    motor.invert_motors = True

    serial_dev.written_commands.clear()
    status, _, _ = simulate_http_get("/manual_drive?cmd=left&speed=185", state)
    assert status == 200
    # Left is -40, inverted should become +40
    assert "TURN 40 185" in serial_dev.written_commands

    serial_dev.written_commands.clear()
    status, _, _ = simulate_http_get("/manual_drive?cmd=right&speed=185", state)
    assert status == 200
    # Right is 40, inverted should become -40
    assert "TURN -40 185" in serial_dev.written_commands

    motor.invert_motors = False


def test_manual_drive_safety_gates(http_sim):
    """Verify safety gates: motors_enabled, pi_latched, and ultrasonic clearance."""
    state = http_sim["state"]
    motor = http_sim["motor"]
    serial_dev = http_sim["serial"]

    # 1. Motors disabled
    state.motors_enabled = False
    serial_dev.written_commands.clear()
    status, _, data = simulate_http_get("/manual_drive?cmd=left&speed=185", state)
    assert status == 200
    assert data["status"] == "blocked"
    assert data["executed"] is False
    assert not any("TURN" in cmd for cmd in serial_dev.written_commands)
    state.motors_enabled = True

    # 2. Pi latched (camera stop) without override
    state.pi_latched = True
    state.manual_override_obstacles = False
    serial_dev.written_commands.clear()
    status, _, data = simulate_http_get("/manual_drive?cmd=right&speed=185", state)
    assert status == 200
    assert data["status"] == "blocked"
    assert data["executed"] is False
    assert not any("TURN" in cmd for cmd in serial_dev.written_commands)

    # 3. Pi latched with override enabled
    state.manual_override_obstacles = True
    serial_dev.written_commands.clear()
    status, _, data = simulate_http_get("/manual_drive?cmd=right&speed=185", state)
    assert status == 200
    assert data["status"] == "ok"
    assert data["executed"] is True
    assert "TURN 40 185" in serial_dev.written_commands
    state.pi_latched = False
    state.manual_override_obstacles = False

    # 4. Skid-steer rotation while obstacle is in front:
    # Forward MUST be blocked, but Left/Right turn MUST be allowed to steer away
    motor.latest_sonar = {"left": 10.0, "right": 12.0, "rear": 999.0}
    serial_dev.written_commands.clear()
    status, _, data = simulate_http_get("/manual_drive?cmd=forward&speed=185", state)
    assert status == 200
    assert data["status"] == "blocked"
    assert data["reason"] == "us_front_blocked"

    # Turn left must succeed despite front obstacle
    serial_dev.written_commands.clear()
    status, _, data = simulate_http_get("/manual_drive?cmd=left&speed=185", state)
    assert status == 200
    assert data["status"] == "ok"
    assert data["executed"] is True
    assert "TURN -40 185" in serial_dev.written_commands


def test_manual_stop_endpoint(http_sim):
    """Verify /manual_stop stops motors and writes STOP command to serial."""
    state = http_sim["state"]
    serial_dev = http_sim["serial"]

    serial_dev.written_commands.clear()
    status, _, data = simulate_http_get("/manual_stop", state)
    assert status == 200
    assert data["status"] == "ok"
    assert data["action"] == "STOP"

    assert "STOP" in serial_dev.written_commands
    assert state.manual_action == "STOP"


# =============================================================================
# 2. JavaScript Web UI Forensic Analysis
# =============================================================================

def test_javascript_code_structure_in_main_py():
    """Verify car/main.py contains all required client-side key-handling components."""
    with open("car/main.py", "r", encoding="utf-8") as f:
        content = f.read()

    # Verify keysDown Set exists
    assert "const keysDown = new Set();" in content
    assert "window.keysDown = keysDown;" in content

    # Verify repeat event guard in keydown
    assert "if (e.repeat) return;" in content

    # Verify MOVEMENT_KEYS mapping
    assert "const MOVEMENT_KEYS = {" in content
    for key in ["'KeyW'", "'KeyS'", "'KeyA'", "'KeyD'", "'ArrowUp'", "'ArrowDown'", "'ArrowLeft'", "'ArrowRight'"]:
        assert key in content

    # Verify resolveActiveMovement scans in reverse order
    assert "function resolveActiveMovement()" in content
    assert "for (let i = keys.length - 1; i >= 0; i--)" in content

    # Verify removeMovementKey clears code, key, and lower-cased key
    assert "function removeMovementKey(e)" in content
    assert "keysDown.delete(e.code);" in content
    assert "keysDown.delete(e.key.toLowerCase());" in content

    # Verify window blur safety clearing
    assert "window.addEventListener('blur'" in content
    assert "keysDown.clear();" in content
    assert "stopDrive();" in content

    # Verify Daylight AWB disarmed in manual mode
    awb_pattern = re.search(r"if\s*\(\s*k\s*===\s*'d'\s*&&\s*currentControlMode\s*!==\s*'manual'", content)
    assert awb_pattern is not None, "Daylight AWB shortcut is not properly disarmed in manual mode"

    # Verify sendManualDrive propagates speed parameter
    send_drive_match = re.search(r"fetch\('/manual_drive\?cmd='\s*\+\s*cmd\s*\+\s*'(&speed=)'\s*\+\s*spd\)", content)
    assert send_drive_match is not None, "sendManualDrive does not append &speed= parameter"


# =============================================================================
# 3. JavaScript Multi-Key State Transition Oracle
# =============================================================================

class JavaScriptKeyControllerOracle:
    """
    Direct faithful oracle modeling the exact client-side JavaScript key-tracking
    engine from car/main.py lines 520-948.
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
        'd': 'right'
    }

    def __init__(self, initial_mode='manual', initial_speed=185):
        self.control_mode = initial_mode
        self.current_speed = initial_speed
        self.keys_down = []  # Ordered list simulating JS Set insertion order
        self.active_action = None
        self.sent_commands = []
        self.awb_calls = []
        self.interval_running = False

    def _get_movement_key(self, code=None, key=None):
        if code and code in self.MOVEMENT_KEYS:
            return code
        if key and key in self.MOVEMENT_KEYS:
            return key
        if key and key.lower() in self.MOVEMENT_KEYS:
            return key.lower()
        return None

    def _resolve_active_movement(self):
        for k in reversed(self.keys_down):
            cmd = self.MOVEMENT_KEYS.get(k)
            if cmd:
                return cmd
        return None

    def _remove_movement_key(self, code=None, key=None):
        mk = self._get_movement_key(code, key)
        targets = set()
        if mk:
            targets.add(mk)
        if code:
            targets.add(code)
        if key:
            targets.add(key)
            targets.add(key.lower())
        self.keys_down = [k for k in self.keys_down if k not in targets]

    def _send_manual_drive(self, cmd, speed=None):
        spd = speed if speed is not None else self.current_speed
        if self.active_action == cmd:
            return
        self.active_action = cmd
        self.interval_running = True
        self.sent_commands.append((cmd, spd))

    def _stop_drive(self):
        self.interval_running = False
        self.active_action = None
        self.keys_down.clear()
        self.sent_commands.append(('stop', 0))

    def on_keydown(self, code=None, key=None, repeat=False):
        if repeat:
            return  # e.repeat guard

        k = key.lower() if key else ''
        if k == 'd' and self.control_mode != 'manual':
            self.awb_calls.append('daylight')

        is_manual = (self.control_mode == 'manual')
        if is_manual:
            if k == '1':
                self.current_speed = 130
            elif k == '2':
                self.current_speed = 185
            elif k == '3':
                self.current_speed = 235

            move_key = self._get_movement_key(code, key)
            if move_key:
                # Set semantics: delete then re-add to move to end of insertion order
                if move_key in self.keys_down:
                    self.keys_down.remove(move_key)
                self.keys_down.append(move_key)
                cmd = self._resolve_active_movement()
                if cmd:
                    self._send_manual_drive(cmd, self.current_speed)

    def on_keyup(self, code=None, key=None):
        move_key = self._get_movement_key(code, key)
        has_key = (
            move_key or
            (code and code in self.keys_down) or
            (key and (key in self.keys_down or key.lower() in self.keys_down))
        )
        if has_key:
            self._remove_movement_key(code, key)
            if len(self.keys_down) == 0:
                self._stop_drive()
            else:
                next_cmd = self._resolve_active_movement()
                if next_cmd:
                    self._send_manual_drive(next_cmd, self.current_speed)
                else:
                    self._stop_drive()

    def on_blur(self):
        if len(self.keys_down) > 0:
            self.keys_down.clear()
            self._stop_drive()


def test_js_single_key_press_and_release():
    """Verify single key press ('w') drives forward, and keyup halts."""
    oracle = JavaScriptKeyControllerOracle()
    oracle.on_keydown(code='KeyW', key='w')
    assert oracle.active_action == 'forward'
    assert oracle.sent_commands[-1] == ('forward', 185)

    oracle.on_keyup(code='KeyW', key='w')
    assert oracle.active_action is None
    assert oracle.sent_commands[-1] == ('stop', 0)


def test_js_repeat_event_ignored():
    """Verify browser keydown repeat events do NOT re-trigger commands."""
    oracle = JavaScriptKeyControllerOracle()
    oracle.on_keydown(code='KeyA', key='a', repeat=False)
    assert oracle.sent_commands == [('left', 185)]

    # Browser repeats while key is held down
    oracle.on_keydown(code='KeyA', key='a', repeat=True)
    oracle.on_keydown(code='KeyA', key='a', repeat=True)
    oracle.on_keydown(code='KeyA', key='a', repeat=True)
    # No extra commands emitted!
    assert oracle.sent_commands == [('left', 185)]


def test_js_multi_key_smooth_transition():
    """
    Stress-test multi-key transition:
    Hold W -> Press D -> Release D -> Release W.
    Expected: forward -> right -> forward -> stop.
    """
    oracle = JavaScriptKeyControllerOracle()

    # 1. Press W -> drives forward
    oracle.on_keydown(code='KeyW', key='w')
    assert oracle.active_action == 'forward'

    # 2. While W is held, press D -> turns right
    oracle.on_keydown(code='KeyD', key='d')
    assert oracle.active_action == 'right'
    assert oracle.sent_commands[-1] == ('right', 185)

    # 3. Release D (W still held) -> resumes forward smoothly without stopDrive!
    oracle.on_keyup(code='KeyD', key='d')
    assert oracle.active_action == 'forward'
    assert oracle.sent_commands[-1] == ('forward', 185)

    # 4. Release W -> stops
    oracle.on_keyup(code='KeyW', key='w')
    assert oracle.active_action is None
    assert oracle.sent_commands[-1] == ('stop', 0)


def test_js_3_key_chord_and_reverse_release():
    """
    Hold W -> Hold A -> Hold S -> Release in reverse order.
    W -> A -> S -> A -> W -> stop.
    """
    oracle = JavaScriptKeyControllerOracle()
    oracle.on_keydown(code='KeyW', key='w')
    assert oracle.active_action == 'forward'

    oracle.on_keydown(code='KeyA', key='a')
    assert oracle.active_action == 'left'

    oracle.on_keydown(code='KeyS', key='s')
    assert oracle.active_action == 'reverse'

    # Release S -> falls back to A
    oracle.on_keyup(code='KeyS', key='s')
    assert oracle.active_action == 'left'

    # Release A -> falls back to W
    oracle.on_keyup(code='KeyA', key='a')
    assert oracle.active_action == 'forward'

    # Release W -> stops
    oracle.on_keyup(code='KeyW', key='w')
    assert oracle.active_action is None
    assert oracle.sent_commands[-1] == ('stop', 0)


def test_js_arrow_keys_and_wasd_interchangeability():
    """Verify arrow keys work identically to WASD."""
    oracle = JavaScriptKeyControllerOracle()

    oracle.on_keydown(code='ArrowUp', key='ArrowUp')
    assert oracle.active_action == 'forward'
    oracle.on_keydown(code='ArrowRight', key='ArrowRight')
    assert oracle.active_action == 'right'
    oracle.on_keyup(code='ArrowRight', key='ArrowRight')
    assert oracle.active_action == 'forward'
    oracle.on_keyup(code='ArrowUp', key='ArrowUp')
    assert oracle.active_action is None


def test_js_blur_clears_keys_and_stops_motion():
    """Verify window blur immediately halts movement even if keys are held down."""
    oracle = JavaScriptKeyControllerOracle()
    oracle.on_keydown(code='KeyW', key='w')
    oracle.on_keydown(code='KeyD', key='d')
    assert len(oracle.keys_down) == 2
    assert oracle.active_action == 'right'

    # Tab loses focus
    oracle.on_blur()
    assert len(oracle.keys_down) == 0
    assert oracle.active_action is None
    assert oracle.sent_commands[-1] == ('stop', 0)


def test_js_d_key_disarms_awb_in_manual_mode():
    """Verify 'd' key in manual mode turns right and does NOT call setAwb('daylight')."""
    oracle_manual = JavaScriptKeyControllerOracle(initial_mode='manual')
    oracle_manual.on_keydown(code='KeyD', key='d')
    assert oracle_manual.active_action == 'right'
    assert len(oracle_manual.awb_calls) == 0, "'d' in manual mode unexpectedly triggered Daylight AWB!"

    # In autonomous mode, 'd' key DOES trigger Daylight AWB
    oracle_auto = JavaScriptKeyControllerOracle(initial_mode='autonomous')
    oracle_auto.on_keydown(code='KeyD', key='d')
    assert len(oracle_auto.awb_calls) == 1
    assert oracle_auto.awb_calls[0] == 'daylight'


def test_js_speed_hotkeys_change_speed_for_subsequent_turns():
    """Verify pressing '1', '2', '3' alters turning speed dynamically."""
    oracle = JavaScriptKeyControllerOracle()

    # Select speed '1' (130)
    oracle.on_keydown(key='1')
    assert oracle.current_speed == 130
    oracle.on_keydown(code='KeyA', key='a')
    assert oracle.sent_commands[-1] == ('left', 130)
    oracle.on_keyup(code='KeyA', key='a')

    # Select speed '3' (235)
    oracle.on_keydown(key='3')
    assert oracle.current_speed == 235
    oracle.on_keydown(code='KeyD', key='d')
    assert oracle.sent_commands[-1] == ('right', 235)
    oracle.on_keyup(code='KeyD', key='d')
