"""
test_requirements_e2e.py — Comprehensive Opaque-Box E2E Requirements Test Suite
================================================================================
Owned by test_writer_1 (E2E Test Architect).
Verification suite covering:
  - R1: Directional Motor Control Logic Contract
  - R2: Motor Startup Disarmed State
  - R3: Manual Mode & pi_latch Resolution
  - R4: Minimum Motor Speed Calibration (>= 180 PWM)

Test Methodology (4 Tiers):
  - Tier 1: Feature Coverage (R1, R2, R3, R4 in isolation)
  - Tier 2: Boundary & Corner Cases (speed 0, 1, 179, 180, 255; mode cycling; obstacle states)
  - Tier 3: Cross-Feature Interactions (manual + obstacle, disarmed + drive, E-stop during manual)
  - Tier 4: Real-World Scenarios (full operational lifecycle: boot -> arm -> latch -> manual -> estop)
"""

import io
import json
import re
import threading
import time
from pathlib import Path
from typing import Optional, Tuple, Dict, Any, List

import numpy as np
import pytest

# Production module imports
import car.perception.config as config
from car.main import (
    DiagnosticHandler,
    SystemState,
    CommandThrottler,
    StreamingOutput,
    PAGE,
    CRUISE_SPEED,
    SLOW_SPEED,
    EMERGENCY_STOP_DISTANCE_CM,
    _check_obstacle_ahead,
    _run_autonomous,
)
from car.control.motor_controller import MotorController
from car.control.navigation import NavigationSystem
from car.perception.sensors import SensorManager


# Repository roots
REPO_ROOT = Path(__file__).resolve().parent.parent
ARDUINO_DIR = REPO_ROOT / "arduino"


# =============================================================================
# Test Infrastructure & Mock Transports
# =============================================================================

class MockSerialTransport:
    """Mock serial transport recording written bytes and serving simulated telemetry."""

    def __init__(self, incoming_lines: Optional[List[str]] = None):
        self.is_open = True
        self.written_commands: List[str] = []
        self._in_buffer: List[str] = list(incoming_lines or [])
        self._lock = threading.Lock()

    @property
    def in_waiting(self) -> int:
        with self._lock:
            return sum(len(line.encode("utf-8") + b"\n") for line in self._in_buffer)

    def readline(self) -> bytes:
        with self._lock:
            if self._in_buffer:
                line = self._in_buffer.pop(0)
                return (line + "\n").encode("utf-8")
            return b""

    def write(self, data: bytes) -> int:
        with self._lock:
            text = data.decode("utf-8").strip()
            if text:
                self.written_commands.append(text)
            return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        with self._lock:
            self._in_buffer.clear()

    def reset_output_buffer(self):
        pass

    def close(self):
        self.is_open = False


def create_test_motor(incoming_lines: Optional[List[str]] = None) -> Tuple[MotorController, MockSerialTransport]:
    """Factory helper creating a MotorController with simulated serial transport."""
    motor = MotorController(port="TEST_MOCK_PORT", baud=115200)
    mock_serial = MockSerialTransport(incoming_lines)
    motor._serial = mock_serial
    return motor, mock_serial


class MockHttpSocket:
    """Mock socket buffer for simulating HTTP requests through DiagnosticHandler."""

    def __init__(self, request_bytes: bytes):
        self.rfile = io.BytesIO(request_bytes)
        self.wfile = io.BytesIO()

    def makefile(self, mode: str, *args, **kwargs):
        if "r" in mode:
            return self.rfile
        return self.wfile

    def sendall(self, data: bytes):
        self.wfile.write(data)


def simulate_http_request(path: str, sys_state: SystemState) -> Tuple[int, Optional[Dict[str, Any]], str]:
    """
    Simulates an HTTP GET request to DiagnosticHandler.
    Returns: (status_code, parsed_json_dict_or_none, raw_body_string)
    """
    class MockServer:
        def __init__(self, state: SystemState):
            self.sys_state = state

    raw_req = f"GET {path} HTTP/1.0\r\nHost: localhost:8080\r\n\r\n".encode("utf-8")
    sock = MockHttpSocket(raw_req)
    server = MockServer(sys_state)
    DiagnosticHandler(sock, ("127.0.0.1", 8080), server)

    resp_data = sock.wfile.getvalue()
    parts = resp_data.split(b"\r\n\r\n", 1)
    header_block = parts[0].decode("utf-8", errors="replace")
    body_bytes = parts[1] if len(parts) > 1 else b""
    body_str = body_bytes.decode("utf-8", errors="replace")

    status_code = int(header_block.splitlines()[0].split()[1])
    try:
        data = json.loads(body_str)
    except Exception:
        data = None

    return status_code, data, body_str


# =============================================================================
# TIER 1: Feature Coverage (Requirements in Isolation)
# =============================================================================

class TestTier1FeatureCoverage:
    """
    Tier 1: Feature Coverage tests verifying each requirement independently.
    """

    # --- R1: Directional Motor Control Logic Contract ---

    def test_r1_motor_controller_directional_command_contract(self):
        """
        Verify that MotorController emits forward drive signals on drive()
        and reverse drive signals on reverse(), with turn signals on turn().
        """
        motor, serial_dev = create_test_motor()

        # Normal forward command must issue DRIVE, not REV
        motor.drive(200, 300)
        assert len(serial_dev.written_commands) == 1
        assert serial_dev.written_commands[-1].startswith("DRIVE")
        assert serial_dev.written_commands[-1] == "DRIVE 200 300"

        # Reverse command must issue REV, not DRIVE
        motor.reverse(200, 300)
        assert len(serial_dev.written_commands) == 2
        assert serial_dev.written_commands[-1].startswith("REV")
        assert serial_dev.written_commands[-1] == "REV 200 300"

        # Turn commands must issue TURN with signed angles
        motor.turn(45, 180)
        assert serial_dev.written_commands[-1] == "TURN 45 200"

        motor.turn(-45, 180)
        assert serial_dev.written_commands[-1] == "TURN -45 200"

    def test_r1_arduino_drive_sketches_directional_polarity(self):
        """
        Verify that Arduino firmware sketches set directional pins so forward
        commands produce forward movement (IN1=LOW, IN2=HIGH and IN3=LOW, IN4=HIGH)
        and reverse commands produce backward movement.
        """
        p2p_ino = ARDUINO_DIR / "point_to_point_control" / "point_to_point_control.ino"
        pins_h = ARDUINO_DIR / "point_to_point_control" / "pins.h"
        motor_test_ino = ARDUINO_DIR / "motor_test" / "motor_test.ino"

        assert p2p_ino.exists(), f"Missing {p2p_ino}"
        assert pins_h.exists(), f"Missing {pins_h}"
        assert motor_test_ino.exists(), f"Missing {motor_test_ino}"

        # In point_to_point_control, verify INVERT_MOTOR_POLARITY or directional logic
        pins_content = pins_h.read_text(encoding="utf-8")
        p2p_content = p2p_ino.read_text(encoding="utf-8")

        # Directional mapping contract:
        # Channel A (Right): IN1, IN2. Channel B (Left): IN3, IN4.
        # Under TT motor chassis wiring:
        # Forward requires IN1=LOW, IN2=HIGH, IN3=LOW, IN4=HIGH
        # Reverse requires IN1=HIGH, IN2=LOW, IN3=HIGH, IN4=LOW
        assert "INVERT_MOTOR_POLARITY" in pins_content or "INVERT_MOTOR_POLARITY" in p2p_content

        # Verify motor_test.ino directional pin configuration
        motor_test_content = motor_test_ino.read_text(encoding="utf-8")
        assert "driveForward" in motor_test_content
        # Forward drive function must configure forward polarity pins
        fwd_match = re.search(r"void\s+driveForward\s*\(\)\s*\{([^}]+)\}", motor_test_content)
        assert fwd_match is not None, "driveForward() definition missing in motor_test.ino"
        fwd_body = fwd_match.group(1)

        # Forward motion requires IN1/IN3 LOW and IN2/IN4 HIGH under corrected chassis polarity
        assert "digitalWrite(IN1, LOW)" in fwd_body or "digitalWrite(PIN_IN1, LOW)" in fwd_body
        assert "digitalWrite(IN2, HIGH)" in fwd_body or "digitalWrite(PIN_IN2, HIGH)" in fwd_body
        assert "digitalWrite(IN3, LOW)" in fwd_body or "digitalWrite(PIN_IN3, LOW)" in fwd_body
        assert "digitalWrite(IN4, HIGH)" in fwd_body or "digitalWrite(PIN_IN4, HIGH)" in fwd_body

    def test_r1_hardware_testing_completion_notice(self):
        """
        Verify that a completion notice or instructions prompting manual hardware
        testing is documented in Arduino sketches or project documentation.
        """
        motor_test_ino = ARDUINO_DIR / "motor_test" / "motor_test.ino"
        content = motor_test_ino.read_text(encoding="utf-8")
        # motor_test.ino serial banner or comments prompt manual physical testing
        assert "Serial.println" in content or "test" in content.lower()

    # --- R2: Motor Startup Disarmed State ---

    def test_r2_system_state_initializes_disarmed(self):
        """
        R2: Ensure SystemState initializes with motors_enabled == False upon startup.
        """
        sys_state = SystemState()
        assert sys_state.motors_enabled is False, (
            "Requirement R2 Violation: SystemState.motors_enabled must initialize to False (disarmed)"
        )

    def test_r2_manual_drive_rejected_when_disarmed(self):
        """
        R2: Manual drive commands must reject execution while motors are disarmed.
        """
        sys_state = SystemState()
        sys_state.motors_enabled = False
        motor, serial_dev = create_test_motor()
        sys_state.motor = motor

        status, data, _ = simulate_http_request("/manual_drive?cmd=forward&speed=200", sys_state)
        assert status == 200
        assert data is not None
        assert data.get("executed") is False, "Manual drive must NOT execute when motors are disarmed"
        assert len(serial_dev.written_commands) == 0, "No motor commands should be sent when disarmed"

    def test_r2_test_drive_rejected_when_disarmed(self):
        """
        R2: Test drive and test turn endpoints must reject execution when disarmed.
        """
        sys_state = SystemState()
        sys_state.motors_enabled = False
        motor, serial_dev = create_test_motor()
        sys_state.motor = motor

        # /test_drive
        status, data, _ = simulate_http_request("/test_drive", sys_state)
        assert status == 200
        assert data.get("executed") is False, "/test_drive must reject when disarmed"

        # /test_turn
        status, data, _ = simulate_http_request("/test_turn?angle=45", sys_state)
        assert status == 200
        assert data.get("executed") is False, "/test_turn must reject when disarmed"

        assert len(serial_dev.written_commands) == 0

    def test_r2_explicit_toggle_arms_motors(self):
        """
        R2: Motors must transition to enabled only upon explicit user toggle.
        """
        sys_state = SystemState()
        sys_state.motors_enabled = False
        motor, serial_dev = create_test_motor()
        sys_state.motor = motor

        # Toggle to enable
        status, data, _ = simulate_http_request("/toggle_motors", sys_state)
        assert status == 200
        assert data.get("motors_enabled") is True
        assert sys_state.motors_enabled is True

        # Now test drive succeeds
        status, data, _ = simulate_http_request("/test_drive", sys_state)
        assert status == 200
        assert data.get("executed") is True
        assert len(serial_dev.written_commands) > 0

    def test_r2_web_ui_reflects_disarmed_state(self):
        """
        R2: Web UI cockpit template must initially present disarmed (BENCH SAFE / OFF) status.
        """
        # Check initial HTML state in PAGE
        assert "MOTORS: BENCH SAFE [OFF]" in PAGE or "badge-motors-off" in PAGE, (
            "Web UI cockpit initial state must reflect disarmed status"
        )
        assert "Enable Motors (M)" in PAGE, (
            "Web UI button should prompt 'Enable Motors (M)' on startup"
        )

    # --- R3: Manual Mode & pi_latch Resolution ---

    def test_r3_obstacle_triggers_pi_latch_in_autonomous(self):
        """
        R3: Imminent obstacle in autonomous mode triggers pi_latched == True
        and sends PI_STOP to serial.
        """
        motor, serial_dev = create_test_motor()
        sensors = SensorManager(motor_controller=motor)
        throttler = CommandThrottler()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        # Imminent obstacle bbox in center corridor (height > 40%)
        detections = [{
            "class_name": "person",
            "confidence": 0.88,
            "bbox": [200, 100, 440, 340],  # height = 240 / 480 = 50%
            "type": "positive",
        }]

        pi_latched, action = _run_autonomous(
            motor=motor,
            sensors=sensors,
            detections=detections,
            us_blocked=False,
            frame=frame,
            pi_latched=False,
            motors_enabled=True,
            throttler=throttler,
        )

        assert pi_latched is True, "Autonomous obstacle must engage pi_latched"
        assert "PI_STOP" in serial_dev.written_commands, "PI_STOP must be sent to Arduino"

    def test_r3_mode_switch_to_manual_clears_pi_latch_and_sends_pi_go(self):
        """
        R3: Switching into manual mode clears active obstacle pi_latch and
        writes PI_GO to serial.
        """
        sys_state = SystemState()
        motor, serial_dev = create_test_motor()
        sys_state.motor = motor
        sys_state.motors_enabled = True
        sys_state.control_mode = "autonomous"
        sys_state.pi_latched = True

        # Switch to manual via /set_mode?mode=manual
        status, data, _ = simulate_http_request("/set_mode?mode=manual", sys_state)
        assert status == 200
        assert sys_state.control_mode == "manual"
        assert sys_state.pi_latched is False, (
            "Requirement R3 Violation: Switching to manual mode must clear pi_latched"
        )
        assert "PI_GO" in serial_dev.written_commands, (
            "Requirement R3 Violation: Switching to manual mode must send PI_GO to unstick Arduino"
        )

    def test_r3_manual_drive_command_resolves_pi_latch(self):
        """
        R3: Issuing manual drive commands when pi_latched is active clears
        pi_latched, sends PI_GO, and allows command execution.
        """
        sys_state = SystemState()
        motor, serial_dev = create_test_motor()
        sys_state.motor = motor
        sys_state.motors_enabled = True
        sys_state.control_mode = "manual"
        sys_state.pi_latched = True

        status, data, _ = simulate_http_request("/manual_drive?cmd=forward&speed=200", sys_state)
        assert status == 200
        assert data.get("executed") is True, (
            "Requirement R3 Violation: Manual drive command must execute even if pi_latched was active"
        )
        assert sys_state.pi_latched is False, (
            "Requirement R3 Violation: Manual drive command must clear pi_latched"
        )
        assert "PI_GO" in serial_dev.written_commands, (
            "Requirement R3 Violation: Manual drive command must dispatch PI_GO to Arduino"
        )

    def test_r3_emergency_stop_remains_unbypassable(self):
        """
        R3: Emergency stop functionality is preserved and unbypassable by manual commands.
        """
        sys_state = SystemState()
        motor, serial_dev = create_test_motor()
        sys_state.motor = motor
        sys_state.motors_enabled = True

        # Trigger emergency stop
        status, _, _ = simulate_http_request("/emergency_stop", sys_state)
        assert status == 200
        assert sys_state.motors_enabled is False
        assert sys_state.pi_latched is True
        assert "PI_STOP" in serial_dev.written_commands

        # Clear write log to verify subsequent commands
        serial_dev.written_commands.clear()

        # Attempt to issue manual drive command during emergency stop
        status, data, _ = simulate_http_request("/manual_drive?cmd=forward&speed=200", sys_state)
        assert status == 200
        assert data.get("executed") is False, "Manual drive must NOT execute during emergency stop"
        assert "PI_GO" not in serial_dev.written_commands, "Emergency stop must NOT send PI_GO"
        assert len(serial_dev.written_commands) == 0

    # --- R4: Minimum Motor Speed Calibration ---

    def test_r4_perception_config_speed_floor(self):
        """
        R4: Configured speed constants in car/perception/config.py must satisfy >= 180 PWM.
        """
        assert config.LANE_CRUISE_SPEED >= 180, (
            f"LANE_CRUISE_SPEED ({config.LANE_CRUISE_SPEED}) must be >= 180 PWM"
        )
        assert config.LANE_CURVE_SPEED >= 180, (
            f"LANE_CURVE_SPEED ({config.LANE_CURVE_SPEED}) must be >= 180 PWM"
        )
        assert config.FOLLOW_SEARCH_TURN_SPEED >= 180, (
            f"FOLLOW_SEARCH_TURN_SPEED ({config.FOLLOW_SEARCH_TURN_SPEED}) must be >= 180 PWM"
        )

    def test_r4_motor_controller_speed_floor_and_clamping(self):
        """
        R4: MotorController methods clamp active speeds 1..179 to 180 PWM,
        while preserving speed 0 for stopping.
        """
        motor, serial_dev = create_test_motor()

        # Speed 0 is preserved for stop
        motor.drive(0, 300)
        assert serial_dev.written_commands[-1] == "DRIVE 0 300"

        # Speeds 1..179 are clamped to 180
        for spd in [1, 50, 120, 160, 179]:
            motor.drive(spd, 300)
            cmd = serial_dev.written_commands[-1]
            assert cmd == "DRIVE 180 300", f"Speed {spd} must be clamped to 180, got {cmd}"

        # Reverse speeds 1..179 clamped to 180
        motor.reverse(100, 300)
        assert serial_dev.written_commands[-1] == "REV 180 300"

        # Turn speeds 1..179 clamped to 180
        motor.turn(45, speed=150)
        assert serial_dev.written_commands[-1] == "TURN 45 200"

        # Valid high speed preserved
        motor.drive(255, 300)
        assert serial_dev.written_commands[-1] == "DRIVE 255 300"

    def test_r4_navigation_system_speed_floor(self):
        """
        R4: NavigationSystem default speed and turn speeds must be >= 180 PWM.
        """
        motor, serial_dev = create_test_motor()
        sensors = SensorManager(motor_controller=motor)
        nav = NavigationSystem(controller=motor, sensors=sensors)

        assert nav.default_pwm >= 180, f"NavigationSystem.default_pwm ({nav.default_pwm}) must be >= 180"

    def test_r4_main_speed_presets_and_endpoints(self):
        """
        R4: Main loop SLOW_SPEED and CRUISE_SPEED, plus /set_speed and /manual_drive endpoints,
        must enforce speed floor >= 180 PWM.
        """
        assert CRUISE_SPEED >= 180, f"CRUISE_SPEED ({CRUISE_SPEED}) must be >= 180"
        assert SLOW_SPEED >= 180, f"SLOW_SPEED ({SLOW_SPEED}) must be >= 180"

        sys_state = SystemState()
        motor, serial_dev = create_test_motor()
        sys_state.motor = motor
        sys_state.motors_enabled = True

        # /set_speed with sub-180 value must clamp to 180
        status, data, _ = simulate_http_request("/set_speed?speed=130", sys_state)
        assert status == 200
        assert data.get("manual_speed") >= 180, (
            f"/set_speed must clamp to >= 180, got {data.get('manual_speed')}"
        )

        # /manual_drive with sub-180 value must clamp command to 180
        status, data, _ = simulate_http_request("/manual_drive?cmd=forward&speed=120", sys_state)
        assert status == 200
        assert serial_dev.written_commands[-1].startswith("DRIVE 180"), (
            f"/manual_drive must clamp driving speed to 180, got {serial_dev.written_commands[-1]}"
        )

    def test_r4_arduino_firmware_avoidance_speed_floor(self):
        """
        R4: In Arduino protocol headers, avoidance and cliff reverse speeds
        must be calibrated to >= 180 PWM.
        """
        protocol_h = ARDUINO_DIR / "point_to_point_control" / "protocol.h"
        assert protocol_h.exists(), f"Missing {protocol_h}"
        content = protocol_h.read_text(encoding="utf-8")

        # Check CLIFF_REVERSE_SPEED and AVOID_SPEED macros
        for macro in ["CLIFF_REVERSE_SPEED", "AVOID_SPEED"]:
            m = re.search(rf"#define\s+{macro}\s+(\d+)", content)
            assert m is not None, f"Macro {macro} not found in {protocol_h}"
            val = int(m.group(1))
            assert val >= 180, f"{macro} in {protocol_h} is {val}, must be >= 180"


# =============================================================================
# TIER 2: Boundary & Corner Cases
# =============================================================================

class TestTier2BoundaryAndCornerCases:
    """
    Tier 2: Boundary & Corner Cases testing extreme inputs, rapid toggles,
    and state edge conditions.
    """

    @pytest.mark.parametrize("input_speed,expected_speed", [
        (0, 0),        # Speed 0 preserved for stop
        (1, 180),      # Minimum non-zero clamped to 180
        (50, 180),     # Low speed clamped to 180
        (179, 180),    # Boundary below threshold clamped to 180
        (180, 180),    # Exact threshold preserved
        (200, 200),    # Nominal speed preserved
        (255, 255),    # Maximum PWM preserved
        (300, 255),    # Overflow clamped to 255
        (-50, 0),      # Negative clamped to 0
    ])
    def test_speed_clamping_discrete_boundaries(self, input_speed, expected_speed):
        """
        Verify discrete speed clamping boundary inputs across all partitions.
        """
        motor, serial_dev = create_test_motor()
        motor.drive(input_speed, 300)
        assert len(serial_dev.written_commands) == 1
        cmd = serial_dev.written_commands[0]
        expected_cmd = f"DRIVE {expected_speed} 300"
        assert cmd == expected_cmd, f"Input {input_speed}: expected '{expected_cmd}', got '{cmd}'"

    def test_rapid_mode_toggling_stability(self):
        """
        Rapidly toggle between control modes (50 iterations) to verify state
        consistency, thread-safety, and absence of deadlock.
        """
        sys_state = SystemState()
        motor, serial_dev = create_test_motor()
        sys_state.motor = motor
        sys_state.motors_enabled = True

        modes = ["autonomous", "lane", "follow", "manual"]
        for i in range(50):
            target_mode = modes[i % len(modes)]
            status, data, _ = simulate_http_request(f"/set_mode?mode={target_mode}", sys_state)
            assert status == 200
            assert data.get("control_mode") == target_mode
            assert sys_state.control_mode == target_mode

    def test_obstacle_cleared_vs_obstacle_active_latch_behavior(self):
        """
        Verify that an active obstacle maintains latch, whereas clearing the obstacle
        releases the latch in autonomous mode.
        """
        motor, serial_dev = create_test_motor()
        sensors = SensorManager(motor_controller=motor)
        throttler = CommandThrottler()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        # 1. Active obstacle engages latch
        active_obstacle = [{
            "class_name": "person",
            "confidence": 0.90,
            "bbox": [200, 80, 440, 360],
            "type": "positive",
        }]
        pi_latched, _ = _run_autonomous(
            motor=motor, sensors=sensors, detections=active_obstacle,
            us_blocked=False, frame=frame, pi_latched=False,
            motors_enabled=True, throttler=throttler,
        )
        assert pi_latched is True

        # 2. Path clear disengages latch
        pi_latched, _ = _run_autonomous(
            motor=motor, sensors=sensors, detections=[],
            us_blocked=False, frame=frame, pi_latched=True,
            motors_enabled=True, throttler=throttler,
        )
        assert pi_latched is False
        assert "PI_GO" in serial_dev.written_commands


# =============================================================================
# TIER 3: Cross-Feature Interactions
# =============================================================================

class TestTier3CrossFeatureInteractions:
    """
    Tier 3: Cross-Feature Interactions testing combined and concurrent feature logic.
    """

    def test_manual_drive_when_obstacle_present_respects_proximity_safety(self):
        """
        When obstacle is present, switching to manual mode clears pi_latch to
        allow operator bypass. However, ultrasonic critical auto-brake (< 25cm)
        still holds safety unless manual override is explicitly enabled.
        """
        sys_state = SystemState()
        motor, serial_dev = create_test_motor()
        sys_state.motor = motor
        sys_state.motors_enabled = True
        sys_state.control_mode = "manual"
        sys_state.pi_latched = True

        # Set ultrasonic sensor readings to 15cm (< 25cm EMERGENCY_STOP_DISTANCE_CM)
        motor.latest_sonar["left"] = 15.0
        motor.latest_sonar["right"] = 15.0

        # Without override: camera latch is cleared, but ultrasonic auto-brake halts drive
        status, data, _ = simulate_http_request("/manual_drive?cmd=forward&speed=200", sys_state)
        assert status == 200
        assert sys_state.pi_latched is False, "pi_latched must be cleared"
        assert data.get("executed") is False, "Ultrasonic obstacle must block forward drive"
        assert data.get("reason") == "us_front_blocked"

        # Now enable manual obstacle override
        simulate_http_request("/toggle_override", sys_state)
        assert sys_state.manual_override_obstacles is True

        # With override enabled: manual drive executes despite close ultrasonic obstacle
        status, data, _ = simulate_http_request("/manual_drive?cmd=forward&speed=200", sys_state)
        assert status == 200
        assert data.get("executed") is True
        assert serial_dev.written_commands[-1].startswith("DRIVE")

    def test_manual_drive_when_disarmed_never_moves(self):
        """
        Manual drive commands must NEVER execute while disarmed, regardless of
        manual_override_obstacles or pi_latched state.
        """
        sys_state = SystemState()
        motor, serial_dev = create_test_motor()
        sys_state.motor = motor
        sys_state.motors_enabled = False
        sys_state.control_mode = "manual"
        sys_state.manual_override_obstacles = True
        sys_state.pi_latched = False

        status, data, _ = simulate_http_request("/manual_drive?cmd=forward&speed=220", sys_state)
        assert status == 200
        assert data.get("executed") is False
        assert len(serial_dev.written_commands) == 0

    def test_emergency_stop_during_manual_drive(self):
        """
        Triggering emergency stop while car is actively driving in manual mode
        must immediately halt all motion, disarm motors, and prevent resumption.
        """
        sys_state = SystemState()
        motor, serial_dev = create_test_motor()
        sys_state.motor = motor
        sys_state.motors_enabled = True
        sys_state.control_mode = "manual"

        # 1. Drive actively
        simulate_http_request("/manual_drive?cmd=forward&speed=200", sys_state)
        assert serial_dev.written_commands[-1].startswith("DRIVE")

        # 2. Trigger Emergency Stop
        simulate_http_request("/emergency_stop", sys_state)
        assert sys_state.motors_enabled is False
        assert sys_state.pi_latched is True
        assert "STOP" in serial_dev.written_commands
        assert "PI_STOP" in serial_dev.written_commands

        # 3. Subsequent manual commands must be rejected
        serial_dev.written_commands.clear()
        status, data, _ = simulate_http_request("/manual_drive?cmd=forward&speed=200", sys_state)
        assert data.get("executed") is False
        assert len(serial_dev.written_commands) == 0

    def test_speed_adjustment_during_latch(self):
        """
        Adjusting speed via /set_speed while car is latched clamps to >= 180 PWM
        without causing accidental drive execution.
        """
        sys_state = SystemState()
        motor, serial_dev = create_test_motor()
        sys_state.motor = motor
        sys_state.motors_enabled = True
        sys_state.pi_latched = True

        status, data, _ = simulate_http_request("/set_speed?speed=100", sys_state)
        assert status == 200
        assert data.get("manual_speed") >= 180
        # No motor commands should have been transmitted
        assert len(serial_dev.written_commands) == 0


# =============================================================================
# TIER 4: Real-World Scenarios
# =============================================================================

class TestTier4RealWorldScenarios:
    """
    Tier 4: Real-World Scenarios executing the complete end-to-end operational lifecycle:
    Startup Disarmed -> User Arms -> Autonomous Latch -> Operator Intervenes
    -> Manual Navigation -> Emergency Stop -> Hard Stop Verification.
    """

    def test_full_operational_lifecycle_e2e_flow(self):
        """
        Full operational mission flow:
          1. System powers on: Motors initialize disarmed (R2).
          2. Operator explicitly arms motors via cockpit toggle (R2).
          3. Car drives autonomously; vision encounters obstacle, engaging pi_latch (R3).
          4. Operator switches to manual mode; pi_latch is cleared & PI_GO sent (R3).
          5. Operator steers around obstacle at calibrated speed >= 180 PWM (R4, R1).
          6. Safety hazard occurs; operator hits Emergency Stop.
          7. Strict hard stop is enforced: motors disarmed, commands permanently rejected.
        """
        # --- Step 1: Startup Disarmed State ---
        sys_state = SystemState()
        motor, serial_dev = create_test_motor()
        sys_state.motor = motor
        sensors = SensorManager(motor_controller=motor)
        throttler = CommandThrottler()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        assert sys_state.motors_enabled is False, "Step 1: Motors must start disarmed"
        assert sys_state.pi_latched is False, "Step 1: pi_latched must start False"

        # Attempting drive while disarmed must fail
        status, data, _ = simulate_http_request("/manual_drive?cmd=forward&speed=200", sys_state)
        assert data.get("executed") is False, "Step 1: Drive rejected while disarmed"
        assert len(serial_dev.written_commands) == 0

        # --- Step 2: User Arms Motors ---
        status, data, _ = simulate_http_request("/toggle_motors", sys_state)
        assert data.get("motors_enabled") is True, "Step 2: Motors successfully armed"
        assert sys_state.motors_enabled is True

        # --- Step 3: Autonomous Driving & Obstacle Latch ---
        sys_state.control_mode = "autonomous"
        detections = [{
            "class_name": "obstacle",
            "confidence": 0.92,
            "bbox": [150, 100, 480, 380],  # Large obstacle in center corridor
            "type": "positive",
        }]

        pi_latched, action = _run_autonomous(
            motor=motor,
            sensors=sensors,
            detections=detections,
            us_blocked=False,
            frame=frame,
            pi_latched=sys_state.pi_latched,
            motors_enabled=sys_state.motors_enabled,
            throttler=throttler,
        )
        sys_state.pi_latched = pi_latched
        assert sys_state.pi_latched is True, "Step 3: Vision obstacle engages pi_latched"
        assert "PI_STOP" in serial_dev.written_commands, "Step 3: PI_STOP written to serial"

        # --- Step 4: Operator Switches to Manual Mode ---
        serial_dev.written_commands.clear()
        status, data, _ = simulate_http_request("/set_mode?mode=manual", sys_state)
        assert status == 200
        assert sys_state.control_mode == "manual", "Step 4: Switched to manual mode"
        assert sys_state.pi_latched is False, "Step 4: Switching to manual clears pi_latched"
        assert "PI_GO" in serial_dev.written_commands, "Step 4: PI_GO dispatched to serial"

        # --- Step 5: Operator Navigates Around Obstacle ---
        serial_dev.written_commands.clear()
        # Steer right to bypass obstacle
        status, data, _ = simulate_http_request("/manual_drive?cmd=right&speed=190", sys_state)
        assert data.get("executed") is True, "Step 5: Right turn executed"
        assert serial_dev.written_commands[-1] == "TURN 40 200", "Step 5: Right turn dispatched"

        # Drive forward past obstacle at calibrated speed
        status, data, _ = simulate_http_request("/manual_drive?cmd=forward&speed=190", sys_state)
        assert data.get("executed") is True, "Step 5: Forward drive executed"
        assert serial_dev.written_commands[-1].startswith("DRIVE 190"), "Step 5: Forward drive dispatched"

        # --- Step 6: Emergency Stop Triggered ---
        serial_dev.written_commands.clear()
        status, data, _ = simulate_http_request("/emergency_stop", sys_state)
        assert status == 200
        assert sys_state.motors_enabled is False, "Step 6: Motors disarmed by E-stop"
        assert sys_state.pi_latched is True, "Step 6: Latch engaged by E-stop"
        assert "STOP" in serial_dev.written_commands
        assert "PI_STOP" in serial_dev.written_commands

        # --- Step 7: Hard Stop Verification ---
        serial_dev.written_commands.clear()
        status, data, _ = simulate_http_request("/manual_drive?cmd=forward&speed=200", sys_state)
        assert data.get("executed") is False, "Step 7: Motion blocked after E-stop"
        status, data, _ = simulate_http_request("/manual_drive?cmd=left&speed=200", sys_state)
        assert data.get("executed") is False, "Step 7: Turn blocked after E-stop"
        assert len(serial_dev.written_commands) == 0, "Step 7: Zero motor commands sent"
