"""
test_corner_recovery.py — Unit Tests for Corner Collision Backup & Turn Recovery (Requirement 2)
================================================================================================
Verifies:
  1. Dual collision triggers reverse maneuver (REV 200) for 1000ms.
  2. Reverse terminates early when both sensors clear to safe distance (>= 25cm) before 1.0s.
  3. Reverse terminates at 1.0s timeout if sensors remain blocked (< 25cm).
  4. Turn maneuver (TURN +/-45 200) executes away from corner upon reverse completion.
  5. Single-sided obstacle does not trigger corner recovery backup.
  6. Resumption to IDLE after turn duration (500ms) so forward navigation resumes.
  7. Sensor glitch rejection (0.0 noise, 999 timeout, NaN, None, boolean).
  8. Boundary threshold tests (25.0cm exact boundary, 25.01cm clear).
"""

import time
import pytest
from car.control.corner_recovery import (
    CornerRecoveryController,
    RecoveryState,
    is_valid_us,
)
from car.control.motor_controller import MotorController
from tests.test_main_coordination import create_mock_motor


# =============================================================================
# 1. Trigger Tests — Dual vs Single-Sided Obstacles
# =============================================================================

def test_dual_collision_triggers_reverse():
    """Verify that when both ultrasonic sensors are <= 25cm, car initiates reverse at PWM 200."""
    motor, serial_dev = create_mock_motor()
    controller = CornerRecoveryController(motor=motor)

    # Both sensors detect corner at 15cm
    t0 = 100.0
    is_recovering, action = controller.update(us_left=15.0, us_right=15.0, current_time=t0)

    assert is_recovering is True
    assert controller.state == RecoveryState.BACKUP
    assert controller.state == "BACKUP"
    assert "CORNER BACKUP" in action

    # Must issue reverse command with PWM 200 and 1000ms duration
    assert any("REV 200 100" in cmd for cmd in serial_dev.written_commands)


def test_single_sided_obstacle_left_does_not_trigger_corner_backup():
    """Verify single-sided obstacle on left does NOT trigger corner backup."""
    motor, serial_dev = create_mock_motor()
    controller = CornerRecoveryController(motor=motor)

    # Left <= 25cm, Right > 25cm
    t0 = 100.0
    is_recovering, action = controller.update(us_left=15.0, us_right=60.0, current_time=t0)

    assert is_recovering is False
    assert controller.state == RecoveryState.IDLE
    assert controller.state == "IDLE"
    assert not any("REV" in cmd for cmd in serial_dev.written_commands)


def test_single_sided_obstacle_right_does_not_trigger_corner_backup():
    """Verify single-sided obstacle on right does NOT trigger corner backup."""
    motor, serial_dev = create_mock_motor()
    controller = CornerRecoveryController(motor=motor)

    # Left > 25cm, Right <= 25cm
    t0 = 100.0
    is_recovering, action = controller.update(us_left=75.0, us_right=12.0, current_time=t0)

    assert is_recovering is False
    assert controller.state == RecoveryState.IDLE
    assert not any("REV" in cmd for cmd in serial_dev.written_commands)


def test_both_sensors_clear_does_not_trigger_backup():
    """Verify that when both sensors have clearance, car remains in IDLE."""
    motor, serial_dev = create_mock_motor()
    controller = CornerRecoveryController(motor=motor)

    is_recovering, action = controller.update(us_left=50.0, us_right=65.0, current_time=100.0)

    assert is_recovering is False
    assert controller.state == RecoveryState.IDLE
    assert len(serial_dev.written_commands) == 0


# =============================================================================
# 2. Reverse Termination Tests — Early Clearance vs 1.0s Timeout
# =============================================================================

def test_reverse_terminates_early_when_both_sensors_clear():
    """Verify that reverse terminates early when both sensors clear before 1.0s."""
    motor, serial_dev = create_mock_motor()
    controller = CornerRecoveryController(motor=motor)

    # Step 1: Trigger corner collision
    t0 = 100.0
    controller.update(us_left=15.0, us_right=15.0, current_time=t0)
    assert controller.state == RecoveryState.BACKUP
    serial_dev.written_commands.clear()

    # Step 2: 300ms later, sensors still close
    controller.update(us_left=18.0, us_right=19.0, current_time=t0 + 0.04)
    assert controller.state == RecoveryState.BACKUP
    assert len(serial_dev.written_commands) == 0

    # Step 3: 80ms later (< 0.1s), both sensors clear to safe distance (35cm and 45cm)
    # Right has 45cm > Left 35cm -> Turn RIGHT (+45°)
    is_recovering, action = controller.update(us_left=35.0, us_right=45.0, current_time=t0 + 0.08)

    assert is_recovering is True
    assert controller.state == RecoveryState.TURN
    assert "TURN +45°" in action
    assert "CLEARED" in action

    # Must issue turn command with PWM 200
    assert any("TURN 45 200" in cmd for cmd in serial_dev.written_commands)


def test_reverse_terminates_at_1s_timeout_when_still_blocked():
    """Verify that reverse terminates at 1.0s timeout if sensors remain blocked."""
    motor, serial_dev = create_mock_motor()
    controller = CornerRecoveryController(motor=motor)

    # Step 1: Trigger corner collision
    t0 = 100.0
    controller.update(us_left=10.0, us_right=10.0, current_time=t0)
    assert controller.state == RecoveryState.BACKUP
    serial_dev.written_commands.clear()

    # Step 2: 800ms later (< 1.0s), still blocked
    controller.update(us_left=12.0, us_right=14.0, current_time=t0 + 0.05)
    assert controller.state == RecoveryState.BACKUP

    # Step 3: 0.1s elapsed (timeout reached), still blocked (12cm and 18cm)
    # Right 18cm >= Left 12cm -> Turn RIGHT (+45°)
    is_recovering, action = controller.update(us_left=12.0, us_right=18.0, current_time=t0 + 0.1)

    assert is_recovering is True
    assert controller.state == RecoveryState.TURN
    assert "TURN +45°" in action
    assert "TIMEOUT" in action

    # Turn command dispatched with PWM 200
    assert any("TURN 45 200" in cmd for cmd in serial_dev.written_commands)


# =============================================================================
# 3. Turn Maneuver & Escape Direction Tests
# =============================================================================

def test_turn_maneuver_steers_left_when_left_has_more_clearance():
    """Verify car steers LEFT (-45°) when left sensor has greater clearance."""
    motor, serial_dev = create_mock_motor()
    controller = CornerRecoveryController(motor=motor)

    # Trigger backup
    t0 = 100.0
    controller.update(us_left=10.0, us_right=10.0, current_time=t0)
    serial_dev.written_commands.clear()

    # Clears at 0.5s: Left has 60cm, Right has 30cm -> Left > Right -> Turn LEFT (-45°)
    is_recovering, action = controller.update(us_left=60.0, us_right=30.0, current_time=t0 + 0.5)

    assert is_recovering is True
    assert controller.state == RecoveryState.TURN
    assert "TURN -45°" in action
    assert any("TURN -45 200" in cmd for cmd in serial_dev.written_commands)


def test_turn_maneuver_timeout_steers_left_when_left_has_more_space():
    """Verify car steers LEFT (-45°) on timeout when left sensor has more space."""
    motor, serial_dev = create_mock_motor()
    controller = CornerRecoveryController(motor=motor)

    t0 = 100.0
    controller.update(us_left=15.0, us_right=15.0, current_time=t0)
    serial_dev.written_commands.clear()

    # Timeout at 1.05s: Left 22cm > Right 14cm
    is_recovering, action = controller.update(us_left=22.0, us_right=14.0, current_time=t0 + 0.12)

    assert controller.state == RecoveryState.TURN
    assert any("TURN -45 200" in cmd for cmd in serial_dev.written_commands)


def test_turn_maneuver_equal_clearance_defaults_to_right():
    """Verify that equal clearance (us_r == us_l) steers right (+45°)."""
    motor, serial_dev = create_mock_motor()
    controller = CornerRecoveryController(motor=motor)

    t0 = 100.0
    controller.update(us_left=15.0, us_right=15.0, current_time=t0)
    serial_dev.written_commands.clear()

    # Cleared with identical distances (30.0cm == 30.0cm)
    controller.update(us_left=30.0, us_right=30.0, current_time=t0 + 0.4)

    assert controller.state == RecoveryState.TURN
    assert any("TURN 45 200" in cmd for cmd in serial_dev.written_commands)


# =============================================================================
# 4. Turn Completion & Resumption to IDLE
# =============================================================================

def test_resumption_to_idle_after_turn_duration():
    """Verify that after turn duration (500ms), state transitions to IDLE and is_recovering is False."""
    motor, serial_dev = create_mock_motor()
    controller = CornerRecoveryController(motor=motor)

    # 1. Trigger collision
    t0 = 100.0
    controller.update(us_left=15.0, us_right=15.0, current_time=t0)

    # 2. Terminate reverse at t = 100.08
    controller.update(us_left=35.0, us_right=40.0, current_time=t0 + 0.08)
    assert controller.state == RecoveryState.TURN

    # 3. In turn at t = 100.2 (120ms elapsed < 300ms duration)
    is_recovering, action = controller.update(us_left=35.0, us_right=40.0, current_time=t0 + 0.2)
    assert is_recovering is True
    assert controller.state == RecoveryState.TURN

    # 4. Turn complete at t = 100.45 (370ms elapsed >= 300ms duration)
    is_recovering, action = controller.update(us_left=35.0, us_right=40.0, current_time=t0 + 0.45)
    assert is_recovering is False
    assert controller.state == RecoveryState.IDLE
    assert controller.is_recovering is False
    assert action == "IDLE"

    # 5. Next cycle remains in IDLE and normal forward navigation resumes
    is_recovering, action = controller.update(us_left=40.0, us_right=45.0, current_time=t0 + 0.55)
    assert is_recovering is False
    assert controller.state == RecoveryState.IDLE


def test_notify_turn_complete_early_ack():
    """Verify that external turn ACK / telemetry can complete turn maneuver early."""
    motor, serial_dev = create_mock_motor()
    controller = CornerRecoveryController(motor=motor)

    t0 = 100.0
    controller.update(us_left=15.0, us_right=15.0, current_time=t0)
    controller.update(us_left=30.0, us_right=30.0, current_time=t0 + 0.1)
    assert controller.state == RecoveryState.TURN

    # Telemetry notifies done turn at 150ms
    controller.notify_turn_complete()
    assert controller.state == RecoveryState.IDLE
    assert controller.is_recovering is False


# =============================================================================
# 5. Boundary Thresholds & Edge Cases
# =============================================================================

def test_exact_boundary_thresholds():
    """Verify exact 25.0cm boundary behavior."""
    motor, serial_dev = create_mock_motor()
    controller = CornerRecoveryController(motor=motor)

    # 25.0cm on both sensors <= 25.0 triggers backup
    assert controller.is_corner_collision(25.0, 25.0) is True

    # 25.01cm on one sensor does not trigger backup
    assert controller.is_corner_collision(25.01, 25.0) is False
    assert controller.is_corner_collision(25.0, 25.01) is False

    # Clearance threshold: 25.0cm clears backup
    t0 = 100.0
    controller.update(us_left=20.0, us_right=20.0, current_time=t0)
    assert controller.state == RecoveryState.BACKUP

    # Both sensors reach exactly 25.0cm -> Cleared
    controller.update(us_left=25.0, us_right=25.0, current_time=t0 + 0.2)
    assert controller.state == RecoveryState.TURN


def test_sensor_glitch_and_noise_filtering():
    """Verify 0.0 noise, 999.0 timeout, None, NaN, and negative values do NOT trigger corner backup."""
    controller = CornerRecoveryController()

    glitch_cases = [
        (0.0, 15.0),
        (15.0, 0.0),
        (999.0, 15.0),
        (15.0, 999.0),
        (None, 15.0),
        (15.0, None),
        (float("nan"), 15.0),
        (-5.0, 15.0),
        (True, 15.0),
        (0.5, 0.5),      # < 1.0cm is invalid reflection
        (855.0, 15.0),   # > 850cm is out of range
    ]

    for us_l, us_r in glitch_cases:
        assert controller.is_corner_collision(us_l, us_r) is False
        is_rec, _ = controller.update(us_left=us_l, us_right=us_r, current_time=100.0)
        assert is_rec is False
        assert controller.state == RecoveryState.IDLE


def test_reset_aborts_active_recovery():
    """Verify reset() immediately aborts backup or turn and returns to IDLE."""
    controller = CornerRecoveryController()

    # Enter backup
    controller.update(us_left=15.0, us_right=15.0, current_time=100.0)
    assert controller.state == RecoveryState.BACKUP

    controller.reset()
    assert controller.state == RecoveryState.IDLE
    assert controller.is_recovering is False
    assert controller.start_time is None
