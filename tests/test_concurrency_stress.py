"""
test_concurrency_stress.py — Stress & Concurrency Verification for car/main.py
=============================================================================
Milestone 4: Empirical Challenger 2 Verification Suite.

Tests:
1. High-concurrency race condition testing across manual drive commands and toggle_swap_rb.
2. Rapid key events, repeat event filtering, and simultaneous key tracking state machine.
3. Server resilience under malformed queries and high-frequency command bursts.
4. Emergency stop race conditions under active concurrent manual drive load.
5. In-process thread safety and state invariant verification.
"""

import io
import json
import random
import threading
import time
import pytest
import numpy as np

from car.main import (
    SystemState,
    DiagnosticHandler,
    CommandThrottler,
    CRUISE_SPEED,
)
from car.control.motor_controller import MotorController
from tests.test_main_coordination import MockSerialDevice, create_mock_motor


# =============================================================================
# In-Memory HTTP Socket Mock for Concurrency (Bypasses WinError 10013)
# =============================================================================

class ConcurrentMockSocket:
    """Thread-isolated mock socket simulating a client connection."""
    def __init__(self, request_bytes: bytes):
        self.rfile = io.BytesIO(request_bytes)
        self.wfile = io.BytesIO()

    def makefile(self, mode, *args, **kwargs):
        if 'r' in mode:
            return self.rfile
        return self.wfile

    def sendall(self, data):
        self.wfile.write(data)


class MockStreamingServer:
    """Mock container for StreamingServer sharing sys_state across threads."""
    def __init__(self, sys_state: SystemState):
        self.sys_state = sys_state


def dispatch_request(path: str, server: MockStreamingServer):
    """Dispatches a single HTTP request through DiagnosticHandler."""
    raw_req = f"GET {path} HTTP/1.0\r\nHost: localhost:8080\r\n\r\n".encode("utf-8")
    sock = ConcurrentMockSocket(raw_req)
    try:
        DiagnosticHandler(sock, ("127.0.0.1", 8080), server)
    except Exception as e:
        return 500, {}, str(e)

    resp_data = sock.wfile.getvalue()
    if not resp_data:
        return 500, {}, "empty response"

    parts = resp_data.split(b"\r\n\r\n", 1)
    header_block = parts[0].decode("utf-8", errors="replace")
    body = parts[1] if len(parts) > 1 else b""
    try:
        status_code = int(header_block.splitlines()[0].split()[1])
    except (IndexError, ValueError):
        status_code = 500

    parsed_json = {}
    if body:
        try:
            parsed_json = json.loads(body.decode("utf-8"))
        except Exception:
            parsed_json = {"raw": body.decode("utf-8", errors="replace")}

    return status_code, parsed_json, header_block


# =============================================================================
# 1. High-Concurrency Stress: Manual Drive & toggle_swap_rb
# =============================================================================

def test_concurrent_manual_drive_and_toggle_swap_rb():
    """
    Stress-test concurrent HTTP requests across manual drive commands and toggle_swap_rb.
    Fires 600 concurrent requests across 30 client threads:
    - 15 threads spamming /manual_drive with various commands and speeds
    - 10 threads spamming /toggle_swap_rb and /set_swap_rb
    - 5 threads spamming /telemetry
    """
    sys_state = SystemState()
    motor, serial_dev = create_mock_motor()
    sys_state.motor = motor
    sys_state.motors_enabled = True
    sys_state.control_mode = "manual"
    server = MockStreamingServer(sys_state)

    results = []
    errors = []
    thread_count = 30
    reqs_per_thread = 20

    def drive_worker(worker_id):
        cmds = ["forward", "reverse", "left", "right", "stop"]
        speeds = [130, 185, 235]
        for i in range(reqs_per_thread):
            cmd = random.choice(cmds)
            spd = random.choice(speeds)
            url = f"/manual_drive?cmd={cmd}&speed={spd}"
            code, data, _ = dispatch_request(url, server)
            if code != 200:
                errors.append(f"DriveWorker {worker_id} req {i} returned {code}: {data}")
            results.append((code, data))

    def swap_worker(worker_id):
        for i in range(reqs_per_thread):
            if i % 2 == 0:
                url = "/toggle_swap_rb"
            else:
                val = "true" if (i % 4 == 1) else "false"
                url = f"/set_swap_rb?enabled={val}"
            code, data, _ = dispatch_request(url, server)
            if code != 200:
                errors.append(f"SwapWorker {worker_id} req {i} returned {code}: {data}")
            results.append((code, data))

    def telemetry_worker(worker_id):
        for i in range(reqs_per_thread):
            url = "/telemetry"
            code, data, _ = dispatch_request(url, server)
            if code != 200:
                errors.append(f"TelemetryWorker {worker_id} req {i} returned {code}: {data}")
            results.append((code, data))

    threads = []
    for t_id in range(15):
        threads.append(threading.Thread(target=drive_worker, args=(t_id,)))
    for t_id in range(10):
        threads.append(threading.Thread(target=swap_worker, args=(t_id,)))
    for t_id in range(5):
        threads.append(threading.Thread(target=telemetry_worker, args=(t_id,)))

    t0 = time.monotonic()
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=10.0)
    elapsed = time.monotonic() - t0

    assert len(errors) == 0, f"Encountered {len(errors)} thread errors:\n" + "\n".join(errors[:5])
    assert len(results) == thread_count * reqs_per_thread, f"Expected {thread_count * reqs_per_thread} results, got {len(results)}"

    # Verify all responses succeeded with 200 OK
    status_codes = [r[0] for r in results]
    assert all(code == 200 for code in status_codes), f"Non-200 status codes found: {set(status_codes)}"

    # Verify state invariants
    with sys_state.lock:
        assert isinstance(sys_state.swap_rb, bool), f"swap_rb corrupted: {sys_state.swap_rb}"
        assert sys_state.control_mode == "manual"
        assert isinstance(sys_state.manual_action, str)

    # Verify motor commands were actually emitted to mock serial transport
    assert len(serial_dev.written_commands) > 0, "No motor commands were recorded!"
    for cmd in serial_dev.written_commands:
        assert any(cmd.startswith(prefix) for prefix in ("DRIVE", "TURN", "REV", "STOP", "PI_")), f"Corrupted serial command: {cmd}"


# =============================================================================
# 2. Client Key State Machine Simulation & Invariants
# =============================================================================

class ClientKeyTracker:
    """
    Direct Python implementation of the browser client key-state tracking
    defined in car/main.py lines 1218–1327.
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

    def __init__(self):
        self.keys_down = []  # preserves insertion order, mirroring Set.prototype[Symbol.iterator]
        self.active_action = None
        self.commands_sent = []

    def get_movement_key(self, code, key):
        if code and code in self.MOVEMENT_KEYS:
            return code
        if key and key in self.MOVEMENT_KEYS:
            return key
        if key and key.lower() in self.MOVEMENT_KEYS:
            return key.lower()
        return None

    def resolve_active_movement(self):
        for k in reversed(self.keys_down):
            cmd = self.MOVEMENT_KEYS.get(k)
            if cmd:
                return cmd
        return None

    def keydown(self, code, key, repeat=False):
        if repeat:
            return  # Repeat guard
        mk = self.get_movement_key(code, key)
        if mk:
            if mk in self.keys_down:
                self.keys_down.remove(mk)
            self.keys_down.append(mk)
            cmd = self.resolve_active_movement()
            if cmd:
                self.active_action = cmd
                self.commands_sent.append(cmd)

    def keyup(self, code, key):
        mk = self.get_movement_key(code, key)
        if mk and mk in self.keys_down:
            self.keys_down.remove(mk)
        if code and code in self.keys_down:
            self.keys_down.remove(code)
        if key and key in self.keys_down:
            self.keys_down.remove(key)
        if key and key.lower() in self.keys_down:
            self.keys_down.remove(key.lower())

        if len(self.keys_down) == 0:
            self.active_action = None
            self.commands_sent.append("stop")
        else:
            next_cmd = self.resolve_active_movement()
            if next_cmd:
                self.active_action = next_cmd
                self.commands_sent.append(next_cmd)
            else:
                self.active_action = None
                self.commands_sent.append("stop")


def test_client_key_tracking_simultaneous_and_transitions():
    """
    Verify client key tracking state machine:
    - Pressing 'W' then 'A' transitions to 'left' while maintaining 'W'
    - Releasing 'A' smoothly resumes 'forward' without stopping
    - Holding multiple keys never results in premature stop
    - Releasing final key triggers 'stop'
    """
    tracker = ClientKeyTracker()

    # 1. Press W (forward)
    tracker.keydown(code="KeyW", key="w")
    assert tracker.active_action == "forward"
    assert tracker.keys_down == ["KeyW"]

    # 2. Press A while holding W (turn left)
    tracker.keydown(code="KeyA", key="a")
    assert tracker.active_action == "left"
    assert tracker.keys_down == ["KeyW", "KeyA"]

    # 3. Release A (should immediately resume forward!)
    tracker.keyup(code="KeyA", key="a")
    assert tracker.active_action == "forward"
    assert tracker.keys_down == ["KeyW"]
    assert tracker.commands_sent[-1] == "forward"

    # 4. Press D while holding W (turn right)
    tracker.keydown(code="KeyD", key="d")
    assert tracker.active_action == "right"
    assert tracker.keys_down == ["KeyW", "KeyD"]

    # 5. Release W while holding D (should remain turning right!)
    tracker.keyup(code="KeyW", key="w")
    assert tracker.active_action == "right"
    assert tracker.keys_down == ["KeyD"]
    assert tracker.commands_sent[-1] == "right"

    # 6. Release D (all keys released -> stop!)
    tracker.keyup(code="KeyD", key="d")
    assert tracker.active_action is None
    assert len(tracker.keys_down) == 0
    assert tracker.commands_sent[-1] == "stop"


def test_client_key_repeat_guard():
    """Verify that browser repeat events (30-50Hz) are strictly ignored and do not reset maneuvers."""
    tracker = ClientKeyTracker()

    # Initial press
    tracker.keydown(code="KeyA", key="a", repeat=False)
    assert len(tracker.commands_sent) == 1
    assert tracker.commands_sent[0] == "left"

    # 50 simulated repeat events (e.g. holding key down)
    for _ in range(50):
        tracker.keydown(code="KeyA", key="a", repeat=True)

    # Must NOT have sent additional commands on repeat
    assert len(tracker.commands_sent) == 1

    tracker.keyup(code="KeyA", key="a")
    assert tracker.commands_sent[-1] == "stop"
    assert len(tracker.commands_sent) == 2


def test_client_key_fuzz_random_interleaving():
    """
    Adversarial fuzz test: 1,000 randomized chaotic key combinations.
    Verifies that the key state machine never crashes, deadlocks, or desynchronizes.
    """
    tracker = ClientKeyTracker()
    test_keys = [
        ("KeyW", "w"),
        ("KeyA", "a"),
        ("KeyS", "s"),
        ("KeyD", "d"),
        ("ArrowUp", "ArrowUp"),
        ("ArrowDown", "ArrowDown"),
        ("ArrowLeft", "ArrowLeft"),
        ("ArrowRight", "ArrowRight"),
    ]

    random.seed(42)
    for step in range(1000):
        action = random.choice(["down", "repeat", "up"])
        code, key = random.choice(test_keys)
        if action == "down":
            tracker.keydown(code, key, repeat=False)
        elif action == "repeat":
            tracker.keydown(code, key, repeat=True)
        else:
            tracker.keyup(code, key)

        # Invariant checks
        if len(tracker.keys_down) == 0:
            assert tracker.active_action is None
        else:
            assert tracker.active_action in ("forward", "reverse", "left", "right")


# =============================================================================
# 3. Server-Side Rapid Burst Stress & Malformed Query Resilience
# =============================================================================

def test_server_rapid_burst_and_malformed_queries():
    """
    Verify server handles bursts of 100 requests in rapid succession with:
    - Normal commands
    - Unknown/malformed commands
    - Extreme/malformed speeds (negative, infinite, NaN, non-integer)
    - Non-ASCII unicode strings
    Ensures zero 500 Internal Server Errors and zero unhandled exceptions.
    """
    sys_state = SystemState()
    motor, serial_dev = create_mock_motor()
    sys_state.motor = motor
    sys_state.motors_enabled = True
    server = MockStreamingServer(sys_state)

    malformed_urls = [
        "/manual_drive?cmd=forward&speed=185",
        "/manual_drive?cmd=left&speed=999999",  # Extreme speed
        "/manual_drive?cmd=right&speed=-50",    # Negative speed
        "/manual_drive?cmd=reverse&speed=abc",  # Non-numeric speed
        "/manual_drive?cmd=drift&speed=185",    # Unknown command
        "/manual_drive?cmd=&speed=",            # Empty params
        "/manual_drive?cmd=qianjin&speed=185",  # Unknown command text
        "/manual_drive",                        # No params
        "/set_speed?speed=99999",               # Clamped max
        "/set_speed?speed=-100",                # Clamped min
        "/set_speed?speed=NaN",                 # Invalid speed
        "/set_swap_rb?enabled=xyz",             # Invalid boolean
        "/set_swap_rb?enable=1",                # Alternative param
        "/toggle_override",
        "/test_turn?angle=abc",                 # Invalid angle
        "/test_turn?angle=720",                 # Extreme angle
    ]

    for i in range(100):
        url = malformed_urls[i % len(malformed_urls)]
        code, data, _ = dispatch_request(url, server)
        assert code in (200, 400), f"Unexpected status code {code} for URL {url}"
        assert isinstance(data, dict), f"Expected JSON response, got: {data}"

    # Verify manual_speed remains within safe bounds [80, 255]
    with sys_state.lock:
        assert 80 <= sys_state.manual_speed <= 255


# =============================================================================
# 4. Emergency Stop Concurrency Race
# =============================================================================

def test_emergency_stop_concurrency_race():
    """
    Verify that an emergency stop triggered asynchronously while multiple threads
    are actively sending /manual_drive instantly stops motors and blocks subsequent drive commands.
    """
    sys_state = SystemState()
    motor, serial_dev = create_mock_motor()
    sys_state.motor = motor
    sys_state.motors_enabled = True
    sys_state.control_mode = "manual"
    server = MockStreamingServer(sys_state)

    stop_triggered = threading.Event()
    drive_results = []

    def spam_drive():
        while not stop_triggered.is_set():
            code, data, _ = dispatch_request("/manual_drive?cmd=forward&speed=185", server)
            drive_results.append(data)
            time.sleep(0.005)

    workers = [threading.Thread(target=spam_drive) for _ in range(5)]
    for w in workers:
        w.start()

    time.sleep(0.05)
    # Trigger emergency stop
    code, estop_data, _ = dispatch_request("/emergency_stop", server)
    assert code == 200
    assert estop_data["motors_enabled"] is False
    assert estop_data["pi_latched"] is True

    # Allow workers to send a few more requests under latched condition
    time.sleep(0.05)
    stop_triggered.set()
    for w in workers:
        w.join(timeout=2.0)

    # Verify that motors_enabled is False and pi_latched is True
    with sys_state.lock:
        assert sys_state.motors_enabled is False
        assert sys_state.pi_latched is True

    # Verify that any manual drive sent after e-stop was blocked
    code, post_estop, _ = dispatch_request("/manual_drive?cmd=forward&speed=185", server)
    assert post_estop["executed"] is False
    assert post_estop["status"] == "blocked"

    # Verify STOP and PI_STOP were sent to Arduino
    assert "STOP" in serial_dev.written_commands
    assert "PI_STOP" in serial_dev.written_commands

    # Now release latch
    code, release_data, _ = dispatch_request("/release_latch", server)
    assert code == 200
    assert release_data["pi_latched"] is False
    with sys_state.lock:
        assert sys_state.pi_latched is False
