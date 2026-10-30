"""
corner_recovery.py — Dual Ultrasonic Corner Collision Backup and Turn Recovery
=============================================================================
Implements Requirement 2:
When both ultrasonic sensors report an obstacle collision (such as when trapped in a corner),
the car reverses until both sensors report that it is safe or 200ms has elapsed, and then
performs a turn maneuver away from the corner before resuming forward navigation.
Includes multi-ping confirmation to suppress spurious ultrasonic noise spikes.
"""

from enum import Enum
import math
import time
from typing import Any, Optional, Tuple

_SENTINEL = object()


class RecoveryState(str, Enum):
    IDLE = "IDLE"
    BACKUP = "BACKUP"
    TURN = "TURN"


def is_valid_us(dist: Any) -> bool:
    """
    Returns True if reading is a valid obstacle reflection.
    Filters out 0.0 noise, 999 timeout, None, booleans, nan, inf, and spurious glitches.
    Valid range: 1.0 < dist < 850.0 cm.
    """
    if dist is None or isinstance(dist, bool) or not isinstance(dist, (int, float)):
        return False
    if math.isnan(dist) or math.isinf(dist):
        return False
    return 1.0 < dist < 850.0


class CornerRecoveryController:
    """
    State machine managing dual-ultrasonic corner collision detection,
    timed reverse backup, and directional turn escape recovery.

    States:
        - IDLE: Monitoring sensors. Forward navigation active.
        - BACKUP: Reversing at PWM 200 for up to 200ms or until both sensors clear >= 25cm.
        - TURN: Turning +/-45 deg away from corner towards greater clearance at PWM 200.
    """

    def __init__(
        self,
        collision_threshold: float = 25.0,
        safe_threshold: float = 25.0,
        backup_speed: int = 200,
        turn_speed: int = 200,
        backup_timeout_s: float = 0.2,
        turn_duration_s: float = 0.5,
        turn_angle_deg: int = 45,
        confirm_count: int = 1,
        motor: Any = None,
    ):
        self.collision_threshold = float(collision_threshold)
        self.safe_threshold = float(safe_threshold)
        self.backup_speed = int(backup_speed)
        self.turn_speed = int(turn_speed)
        self.backup_timeout_s = float(backup_timeout_s)
        self.turn_duration_s = float(turn_duration_s)
        self.turn_angle_deg = int(turn_angle_deg)
        self.confirm_count = max(1, int(confirm_count))
        self.motor = motor

        self._state: RecoveryState = RecoveryState.IDLE
        self._collision_counter: int = 0
        self.start_time: Optional[float] = None
        self.turn_start_time: Optional[float] = None
        self.last_turn_angle: int = 0
        self.last_us_left: Optional[float] = None
        self.last_us_right: Optional[float] = None

    @property
    def state(self) -> RecoveryState:
        return self._state

    @property
    def is_recovering(self) -> bool:
        """True if currently executing backup or turn escape recovery."""
        return self._state != RecoveryState.IDLE

    def is_corner_collision(self, us_left: Any, us_right: Any) -> bool:
        """
        Corner Trigger: Returns True if and only if BOTH ultrasonic sensors simultaneously
        read within collision threshold (<= 25.0cm) with valid readings (1.0 < d < 850.0).
        Single-sided obstacles do NOT trigger corner collision.
        """
        if not is_valid_us(us_left) or not is_valid_us(us_right):
            return False
        return us_left <= self.collision_threshold and us_right <= self.collision_threshold

    def reset(self) -> None:
        """Resets controller state to IDLE and clears timers and collision counter."""
        self._state = RecoveryState.IDLE
        self._collision_counter = 0
        self.start_time = None
        self.turn_start_time = None

    def notify_turn_complete(self) -> None:
        """Allows external telemetry or turn ACK to complete turn maneuver early."""
        if self._state == RecoveryState.TURN:
            self.reset()

    def update(
        self,
        us_left: Any,
        us_right: Any,
        motor: Any = _SENTINEL,
        enabled: bool = True,
        current_time: Optional[float] = None,
    ) -> Tuple[bool, str]:
        """
        Executes one tick of the corner recovery state machine.

        Args:
            us_left: Distance reading from front-left ultrasonic sensor (cm).
            us_right: Distance reading from front-right ultrasonic sensor (cm).
            motor: Optional MotorController instance (defaults to self.motor).
            enabled: If False, suppresses motor actuation (bench-safe).
            current_time: Optional monotonic timestamp (defaults to time.monotonic()).

        Returns:
            Tuple[bool, str]: (is_recovering, action_description).
            When is_recovering is False, normal forward navigation can resume.
        """
        if current_time is None:
            current_time = time.monotonic()
        if motor is _SENTINEL:
            motor = self.motor
        if not enabled:
            motor = None

        valid_l = is_valid_us(us_left)
        valid_r = is_valid_us(us_right)
        if valid_l:
            self.last_us_left = float(us_left)
        if valid_r:
            self.last_us_right = float(us_right)

        # ── State 1: IDLE ────────────────────────────────────────────────────
        if self._state == RecoveryState.IDLE:
            if self.is_corner_collision(us_left, us_right):
                self._collision_counter += 1
                if self._collision_counter >= self.confirm_count:
                    # Confirmed dual obstacle collision detected! Initiate reverse backup.
                    self._collision_counter = 0
                    self._state = RecoveryState.BACKUP
                    self.start_time = current_time
                    if enabled and motor is not None and hasattr(motor, "reverse"):
                        motor.reverse(speed=self.backup_speed, duration_ms=int(self.backup_timeout_s * 1000))
                    return True, f"🔄 CORNER BACKUP (L:{float(us_left):.1f}cm R:{float(us_right):.1f}cm)"
                return False, f"IDLE (Corner confirming {self._collision_counter}/{self.confirm_count})"
            else:
                self._collision_counter = 0
                return False, "IDLE"

        # ── State 2: BACKUP ──────────────────────────────────────────────────
        if self._state == RecoveryState.BACKUP:
            elapsed = current_time - (self.start_time or current_time)

            # Check termination conditions:
            # 1. Both sensors clear to safe distance (>= 25.0cm)
            # 2. Or backup timeout (>= 1.0s) has elapsed
            both_cleared = (
                valid_l and valid_r and
                us_left >= self.safe_threshold and
                us_right >= self.safe_threshold
            )
            timed_out = elapsed >= self.backup_timeout_s

            if both_cleared or timed_out:
                # Reverse terminates! Transition immediately to TURN.
                self._state = RecoveryState.TURN
                self.turn_start_time = current_time

                # Steer away from the corner towards the side with greater clearance:
                # if us_r >= us_l, steer right (+45°); else steer left (-45°)
                clear_r = float(us_right) if valid_r else (self.last_us_right if self.last_us_right is not None else 0.0)
                clear_l = float(us_left) if valid_l else (self.last_us_left if self.last_us_left is not None else 0.0)

                turn_angle = self.turn_angle_deg if clear_r >= clear_l else -self.turn_angle_deg
                self.last_turn_angle = turn_angle

                if enabled and motor is not None and hasattr(motor, "turn"):
                    motor.turn(turn_angle, speed=self.turn_speed)

                reason = "CLEARED" if both_cleared else f"TIMEOUT {elapsed:.2f}s"
                return True, f"🔄 CORNER TURN {turn_angle:+d}° ({reason}, L:{clear_l:.1f}cm R:{clear_r:.1f}cm)"

            return True, f"🔄 CORNER BACKUP (elapsed={elapsed:.2f}s)"

        # ── State 3: TURN ────────────────────────────────────────────────────
        if self._state == RecoveryState.TURN:
            turn_elapsed = current_time - (self.turn_start_time or current_time)
            if turn_elapsed >= self.turn_duration_s:
                # Turn maneuver completed! Transition to IDLE so forward navigation resumes.
                self.reset()
                return False, "IDLE"

            return True, f"🔄 CORNER TURN {self.last_turn_angle:+d}° (elapsed={turn_elapsed:.2f}s)"

        # Fallback safety reset
        self.reset()
        return False, "IDLE"
