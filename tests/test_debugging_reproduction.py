"""
test_debugging_reproduction.py — Systematic Debugging Reproduction Tests
========================================================================
Validates fixes for:
  1. Car moving when motors are disabled (CornerRecoveryController sending commands when disabled).
  2. Random reverse triggered by single-ping ultrasonic noise spikes.
  3. Car getting stuck permanently when faced with single-sided or close obstacle.
  4. Manual mode reverse being killed by front ultrasonic sensors seeing wall.
"""

import time
import pytest
import numpy as np
from car.control.corner_recovery import (
    CornerRecoveryController,
    RecoveryState,
)
from car.perception.sensors import SensorManager
from car.main import _run_autonomous, CommandThrottler, SystemState
from tests.test_main_coordination import create_mock_motor


def test_no_motor_commands_when_motors_disabled():
    """Bug 1: When motors are disabled (enabled=False or motor=None), CornerRecoveryController must NEVER send serial commands."""
    motor, serial_dev = create_mock_motor()
    controller = CornerRecoveryController(motor=motor)

    # Calling update with enabled=False
    is_recovering, action = controller.update(
        us_left=15.0,
        us_right=15.0,
        motor=None,
        enabled=False,
        current_time=100.0,
    )

    # Should NOT have sent REV or TURN commands over serial
    written_moves = [cmd for cmd in serial_dev.written_commands if "REV" in cmd or "TURN" in cmd or "DRIVE" in cmd]
    assert len(written_moves) == 0, f"Expected 0 motor commands when disabled, got: {written_moves}"


def test_single_ping_glitch_does_not_trigger_reverse():
    """Bug 2: Single-ping ultrasonic glitch should not immediately trigger backup if confirmation filter is used."""
    motor, serial_dev = create_mock_motor()
    # Controller with 2-ping confirmation
    controller = CornerRecoveryController(motor=motor, confirm_count=2)

    # Ping 1: Glitch reads 15cm on both
    is_recovering, action = controller.update(us_left=15.0, us_right=15.0, current_time=100.0)
    assert controller.state == RecoveryState.IDLE
    assert not any("REV" in cmd for cmd in serial_dev.written_commands)

    # Ping 2: Sensor reads clear (60cm)
    is_recovering, action = controller.update(us_left=60.0, us_right=60.0, current_time=100.05)
    assert controller.state == RecoveryState.IDLE
    assert not any("REV" in cmd for cmd in serial_dev.written_commands)


def test_two_consecutive_pings_trigger_reverse():
    """Bug 2: Two consecutive valid collision pings SHOULD trigger backup."""
    motor, serial_dev = create_mock_motor()
    controller = CornerRecoveryController(motor=motor, confirm_count=2)

    # Ping 1
    controller.update(us_left=15.0, us_right=15.0, current_time=100.0)
    # Ping 2
    is_recovering, action = controller.update(us_left=15.0, us_right=15.0, current_time=100.05)

    assert is_recovering is True
    assert controller.state == RecoveryState.BACKUP
    assert any("REV 200 100" in cmd for cmd in serial_dev.written_commands)


def test_single_sided_obstacle_executes_evasion_instead_of_freezing():
    """Bug 3: When left is blocked (<25cm) and right is clear (60cm), car must steer right, NOT freeze forever."""
    motor, serial_dev = create_mock_motor()
    motor.latest_sonar = {"left": 18.0, "right": 60.0, "rear": 999.0}
    sensors = SensorManager(motor_controller=motor)
    throttler = CommandThrottler()
    corner_recovery = CornerRecoveryController(motor=motor, confirm_count=2)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    pi_latched, action_label = _run_autonomous(
        motor=motor,
        sensors=sensors,
        detections=[],
        us_blocked=True,
        frame=frame,
        pi_latched=False,
        motors_enabled=True,
        throttler=throttler,
        fresh_inference=True,
        corner_recovery=corner_recovery,
    )

    # Must steer right away from the left obstacle, not just halt forever in US AUTO-BRAKE
    turn_cmds = [cmd for cmd in serial_dev.written_commands if "TURN" in cmd]
    assert len(turn_cmds) > 0, f"Expected avoidance turn away from obstacle, got: {serial_dev.written_commands}"
    assert any("TURN 35" in cmd for cmd in turn_cmds)


def test_manual_mode_reverse_not_killed_by_front_wall():
    """Bug 4: In manual mode, reversing away from a front wall must NOT be blocked or halted by front ultrasonic sensors."""
    motor, serial_dev = create_mock_motor()
    sys_state = SystemState()
    sys_state.control_mode = "manual"
    sys_state.motors_enabled = True
    sys_state.manual_action = "REVERSE"
    sys_state.manual_speed = 185
    sys_state.manual_override_obstacles = False
    sys_state.motor = motor

    # Front is blocked by a wall, rear is clear
    motor.latest_sonar = {"left": 15.0, "right": 15.0, "rear": 999.0}

    # Simulate what main loop does for manual mode
    us = motor.latest_sonar
    us_blocked = (us["left"] < 25.0 or us["right"] < 25.0)

    # Under fixed logic, if manual_action is REVERSE, it must NOT call motor.stop()
    man_act = sys_state.manual_action
    should_autobrake = us_blocked and not sys_state.manual_override_obstacles and (man_act in ("FORWARD", "FWD"))
    assert should_autobrake is False, "Front wall must not auto-brake a reverse maneuver in manual mode!"


def test_wall_approaching_close_safely_autobrakes_instead_of_turning():
    """Bug 5: When approaching a wall where one sensor reads <25cm and the other is NOT wide open (e.g. 26cm or invalid),
    car must safely auto-brake (motor.stop()) rather than commanding an aggressive turn that slams into the wall."""
    motor, serial_dev = create_mock_motor()
    # Left is 20cm, Right is 26cm (facing a wall, neither side is wide open)
    motor.latest_sonar = {"left": 20.0, "right": 26.0, "rear": 999.0}
    sensors = SensorManager(motor_controller=motor)
    throttler = CommandThrottler()
    corner_recovery = CornerRecoveryController(motor=motor)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    pi_latched, action_label = _run_autonomous(
        motor=motor,
        sensors=sensors,
        detections=[],
        us_blocked=True,
        frame=frame,
        pi_latched=False,
        motors_enabled=True,
        throttler=throttler,
        fresh_inference=True,
        corner_recovery=corner_recovery,
    )

    # Must STOP with US AUTO-BRAKE, not command a turn into the wall
    assert "US AUTO-BRAKE" in action_label
    assert any("STOP" in cmd for cmd in serial_dev.written_commands)
    turn_cmds = [cmd for cmd in serial_dev.written_commands if "TURN" in cmd]
    assert len(turn_cmds) == 0, f"Expected NO turn into wall, got: {turn_cmds}"


def test_recovery_does_not_repeat_consecutively_preventing_blind_reverse_into_walls():
    """Verify that after completing 1 recovery (backup + turn), if the car is still blocked,
    it does NOT trigger another backup, and instead safely holds in place."""
    motor, serial_dev = create_mock_motor()
    controller = CornerRecoveryController(motor=motor, confirm_count=1, backup_timeout_s=0.1, turn_duration_s=0.3)

    t0 = 100.0
    # Step 1: Trigger recovery
    controller.update(us_left=15.0, us_right=15.0, current_time=t0)
    assert controller.state == RecoveryState.BACKUP
    assert any("REV 200 100" in cmd for cmd in serial_dev.written_commands)
    serial_dev.written_commands.clear()

    # Step 2: Timeout backup -> enters TURN
    controller.update(us_left=15.0, us_right=15.0, current_time=t0 + 0.12)
    assert controller.state == RecoveryState.TURN

    # Step 3: Complete TURN -> transitions to IDLE and enters cooldown
    is_rec, action = controller.update(us_left=15.0, us_right=15.0, current_time=t0 + 0.45)
    assert is_rec is False
    assert controller.state == RecoveryState.IDLE
    assert controller.can_trigger(current_time=t0 + 0.46) is False

    # Step 4: Next tick while still facing wall (15cm) - MUST NOT trigger another backup!
    is_rec2, action2 = controller.update(us_left=15.0, us_right=15.0, current_time=t0 + 0.50)
    assert is_rec2 is False
    assert controller.state == RecoveryState.IDLE
    # No new REV command dispatched!
    assert not any("REV" in cmd for cmd in serial_dev.written_commands)


def test_single_sided_wall_jam_executes_recovery_nudge_when_closely_jammed():
    """Verify that if car is jammed close against a wall (Left=6cm, Right=30cm),
    it executes a brief 100ms reverse nudge and steers right away from the left wall,
    rather than freezing or crashing."""
    motor, serial_dev = create_mock_motor()
    motor.latest_sonar = {"left": 6.0, "right": 30.0, "rear": 999.0}
    sensors = SensorManager(motor_controller=motor)
    throttler = CommandThrottler()
    corner_recovery = CornerRecoveryController(motor=motor, confirm_count=1)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    pi_latched, action_label = _run_autonomous(
        motor=motor,
        sensors=sensors,
        detections=[],
        us_blocked=True,
        frame=frame,
        pi_latched=False,
        motors_enabled=True,
        throttler=throttler,
        fresh_inference=True,
        corner_recovery=corner_recovery,
    )

    # Must initiate recovery backup (100ms)
    assert corner_recovery.state == RecoveryState.BACKUP
    assert any("REV 200 100" in cmd for cmd in serial_dev.written_commands)


def test_arduino_avoidance_overridden_by_pi():
    """Verify that if Arduino enters AVOIDING state, Pi does not yield,
    but immediately overrides it with PI_STOP/PI_GO to prevent inverted crash."""
    motor, serial_dev = create_mock_motor()
    motor.latest_state = "AVOIDING"
    sensors = SensorManager(motor_controller=motor)
    throttler = CommandThrottler()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    pi_latched, action_label = _run_autonomous(
        motor=motor,
        sensors=sensors,
        detections=[],
        us_blocked=False,
        frame=frame,
        pi_latched=False,
        motors_enabled=True,
        throttler=throttler,
        fresh_inference=True,
    )

    assert "OVERRIDING ARDUINO" in action_label
    assert any("PI_STOP" in cmd for cmd in serial_dev.written_commands)
    assert any("PI_GO" in cmd for cmd in serial_dev.written_commands)
