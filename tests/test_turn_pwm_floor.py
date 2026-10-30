"""
test_turn_pwm_floor.py — Unit Tests for Minimum Turn PWM Floor (>= 200)
======================================================================
Verifies Requirement 3:
1. MotorController.turn() enforces a minimum PWM floor of 200 for active turn commands.
2. Clamping logic: if 0 < speed < 200, speed is clamped to 200.
3. Speed 0 is strictly preserved for stop/neutral.
4. No turn command with speed between 1 and 199 is ever transmitted over serial.
5. Default parameter for turn speed is 200.
6. Configuration constants FOLLOW_SEARCH_TURN_SPEED and LANE_CURVE_SPEED are >= 200.
7. NavigationSystem.navigate_step invokes turn with speed=200.
8. Protocol documentation in arduino/PROTOCOL.md accurately reflects the 200 floor.
"""

import sys
import threading
from pathlib import Path
from typing import List, Tuple
from unittest.mock import MagicMock, patch

import pytest

# Ensure repo root is on sys.path
REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from car.control.motor_controller import MotorController
from car.control.navigation import NavigationSystem
from car.perception import config
from car.perception.sensors import SensorManager


class MockSerialTransport:
    """Simulates serial connection recording transmitted ASCII command lines."""

    def __init__(self):
        self.is_open = True
        self.written_commands: List[str] = []
        self._lock = threading.Lock()

    def write(self, data: bytes) -> int:
        with self._lock:
            text = data.decode("utf-8").strip()
            if text:
                self.written_commands.append(text)
            return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass

    def close(self):
        self.is_open = False


def create_test_motor() -> Tuple[MotorController, MockSerialTransport]:
    """Factory helper creating a MotorController with simulated serial transport."""
    motor = MotorController(port="TEST_MOCK_PORT", baud=115200)
    mock_serial = MockSerialTransport()
    motor._serial = mock_serial
    return motor, mock_serial


class TestTurnPwmFloorClamping:
    """Verifies turn speed clamping and preservation in MotorController."""

    def test_turn_speed_zero_preserved(self):
        """Speed 0 must be preserved for stop/neutral commands."""
        motor, serial_dev = create_test_motor()
        res = motor.turn(45, speed=0)
        assert res is True
        assert len(serial_dev.written_commands) == 1
        assert serial_dev.written_commands[-1] == "TURN 45 0"

    def test_turn_default_speed_is_200(self):
        """Omitting speed parameter must default to 200 PWM."""
        motor, serial_dev = create_test_motor()
        res = motor.turn(45)
        assert res is True
        assert serial_dev.written_commands[-1] == "TURN 45 200"

    @pytest.mark.parametrize("spd", [1, 25, 50, 100, 150, 180, 185, 190, 199])
    def test_turn_speeds_under_200_clamped_to_200(self, spd: int):
        """Every speed between 1 and 199 must clamp to 200 PWM."""
        motor, serial_dev = create_test_motor()
        res = motor.turn(45, speed=spd)
        assert res is True
        assert serial_dev.written_commands[-1] == "TURN 45 200", (
            f"Input speed {spd} must be clamped to 200, got: {serial_dev.written_commands[-1]}"
        )

    def test_turn_speed_exhaustive_floor_guarantee(self):
        """
        Exhaustively verify all integer speeds from 1 to 199.
        Guarantees that no TURN command with speed in [1, 199] is ever transmitted.
        """
        motor, serial_dev = create_test_motor()
        for spd in range(1, 200):
            motor.turn(30, speed=spd)
            last_cmd = serial_dev.written_commands[-1]
            parts = last_cmd.split()
            assert parts[0] == "TURN"
            cmd_spd = int(parts[2])
            assert cmd_spd == 200, (
                f"Violation: speed={spd} resulted in transmitted PWM {cmd_spd} != 200"
            )

    def test_turn_speed_at_or_above_200_preserved(self):
        """Speeds >= 200 must be preserved up to 255."""
        motor, serial_dev = create_test_motor()

        motor.turn(45, speed=200)
        assert serial_dev.written_commands[-1] == "TURN 45 200"

        motor.turn(45, speed=220)
        assert serial_dev.written_commands[-1] == "TURN 45 220"

        motor.turn(45, speed=255)
        assert serial_dev.written_commands[-1] == "TURN 45 255"

    def test_turn_speed_over_255_clamped_to_255(self):
        """Speeds above 255 must be clamped to 255."""
        motor, serial_dev = create_test_motor()
        motor.turn(45, speed=300)
        assert serial_dev.written_commands[-1] == "TURN 45 255"

    def test_turn_negative_speed_clamped_to_zero(self):
        """Negative speed must be clamped to 0 via max(0, min(255, speed))."""
        motor, serial_dev = create_test_motor()
        motor.turn(45, speed=-15)
        assert serial_dev.written_commands[-1] == "TURN 45 0"

    def test_turn_negative_angles_and_inversion(self):
        """Turn floor >= 200 must function identically for left turns and motor inversion."""
        motor, serial_dev = create_test_motor()

        # Left turn with sub-200 speed
        motor.turn(-60, speed=150)
        assert serial_dev.written_commands[-1] == "TURN -60 200"

        # Inverted motor mode
        motor.invert_motors = True
        motor.turn(45, speed=160)
        assert serial_dev.written_commands[-1] == "TURN -45 200"

        motor.turn(-45, speed=160)
        assert serial_dev.written_commands[-1] == "TURN 45 200"

    def test_straight_drive_floor_remains_180(self):
        """Straight drive (DRIVE, REV) floor must remain 180 while turn floor is 200."""
        motor, serial_dev = create_test_motor()

        # DRIVE clamps to 180
        motor.drive(150, 300)
        assert serial_dev.written_commands[-1] == "DRIVE 180 300"

        # REV clamps to 180
        motor.reverse(150, 300)
        assert serial_dev.written_commands[-1] == "REV 180 300"

        # DRIVE at 180 preserved
        motor.drive(180, 300)
        assert serial_dev.written_commands[-1] == "DRIVE 180 300"


class TestConfigTurnSpeedConstants:
    """Verifies that perception & navigation turn constants are >= 200."""

    def test_follow_search_turn_speed_floor(self):
        """FOLLOW_SEARCH_TURN_SPEED must be >= 200."""
        assert config.FOLLOW_SEARCH_TURN_SPEED >= 200, (
            f"FOLLOW_SEARCH_TURN_SPEED ({config.FOLLOW_SEARCH_TURN_SPEED}) must be >= 200"
        )
        assert config.FOLLOW_SEARCH_TURN_SPEED == 200

    def test_lane_curve_speed_floor(self):
        """LANE_CURVE_SPEED must be >= 200."""
        assert config.LANE_CURVE_SPEED >= 200, (
            f"LANE_CURVE_SPEED ({config.LANE_CURVE_SPEED}) must be >= 200"
        )
        assert config.LANE_CURVE_SPEED == 200


class TestNavigationTurnSpeed:
    """Verifies that NavigationSystem executes turns with speed >= 200."""

    @patch("time.sleep", return_value=None)
    def test_navigate_step_calls_turn_with_speed_200(self, mock_sleep):
        """navigate_step must command turn with speed=200."""
        motor, serial_dev = create_test_motor()
        sensors = MagicMock(spec=SensorManager)
        sensors.is_obstacle_ahead.return_value = False

        nav = NavigationSystem(controller=motor, sensors=sensors)
        success = nav.navigate_step(target_distance_m=0.5, target_heading_deg=45.0)

        assert success is True
        # Verify TURN command was emitted with speed 200
        turn_cmds = [cmd for cmd in serial_dev.written_commands if cmd.startswith("TURN")]
        assert len(turn_cmds) == 1
        assert turn_cmds[0] == "TURN 45 200"

    @patch("time.sleep", return_value=None)
    def test_navigate_waypoints_calls_turn_with_speed_200(self, mock_sleep):
        """navigate_waypoints must command all turns with speed=200."""
        motor, serial_dev = create_test_motor()
        sensors = MagicMock(spec=SensorManager)
        sensors.is_obstacle_ahead.return_value = False

        nav = NavigationSystem(controller=motor, sensors=sensors)
        waypoints = [
            (0.5, 30.0),
            (0.5, -45.0),
            (0.5, 90.0),
        ]
        success = nav.navigate_waypoints(waypoints)
        assert success is True

        turn_cmds = [cmd for cmd in serial_dev.written_commands if cmd.startswith("TURN")]
        assert len(turn_cmds) == 3
        assert turn_cmds[0] == "TURN 30 200"
        assert turn_cmds[1] == "TURN -45 200"
        assert turn_cmds[2] == "TURN 90 200"


class TestProtocolDocumentationParity:
    """Verifies that arduino/PROTOCOL.md accurately specifies the 200 PWM floor for turns."""

    def test_protocol_document_specifies_turn_floor_200(self):
        protocol_path = REPO_ROOT / "arduino" / "PROTOCOL.md"
        assert protocol_path.exists(), f"PROTOCOL.md not found at {protocol_path}"
        content = protocol_path.read_text(encoding="utf-8")

        # Check TURN command in table
        assert "TURN 45 200" in content, "PROTOCOL.md must include TURN 45 200 example"
        assert "clamped to 200 floor" in content, "PROTOCOL.md must mention clamped to 200 floor"

        # Check Tier 3 notes
        assert "minimum motor PWM floor of 200" in content, (
            "PROTOCOL.md must document minimum motor PWM floor of 200 for turns"
        )
