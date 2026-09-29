"""
motor_controller.py — Pi-Side Serial Bridge to Arduino Uno
===========================================================
Owned by integration-agent.
Implements the Pi-side half of the serial handshake defined in arduino/PROTOCOL.md.
Transport (USB-Serial or GPIO-UART) is behind a thin interface so it can be swapped
without touching callers. See config.py for SERIAL_PORT and SERIAL_BAUD.

Command methods mirror the three-tier priority stack in PROTOCOL.md:
  Tier 1: pi_stop(), pi_go(), pi_avoid()          ← Pi camera latch
  Tier 3: drive(), reverse(), turn(), stop(), ping() ← Normal driving

Telemetry is parsed passively in read_telemetry():
  [TELEM] US LEFT=x RIGHT=y  → updates latest_sonar dict
  [TELEM] STATE=...           → updates latest_state string
  [ACK] ...                   → returned as-is for callers that care about ACK
"""

import sys
import time
from typing import Optional, Dict
import serial

from car.perception.config import SERIAL_PORT, SERIAL_BAUD


class MotorController:
    """Manages serial communications and sends commands to the Arduino Uno."""

    def __init__(self, port: str = SERIAL_PORT, baud: int = SERIAL_BAUD, timeout: float = 0.5):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None

        # Latest ultrasonic readings parsed from [TELEM] US LEFT=x RIGHT=y
        # Architecture uses 2 front-facing sensors (LEFT / RIGHT) — no center.
        self.latest_sonar: Dict[str, float] = {"left": 999.0, "right": 999.0}

        # Latest Arduino state string ("IDLE", "DRIVING", "REVERSING", etc.)
        self.latest_state: str = "UNKNOWN"

    # ── Connection management ─────────────────────────────────────────────────

    def connect(self) -> bool:
        """Opens the serial link and waits for Arduino bootloader to settle."""
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
            # ATmega328P resets when DTR toggles on serial open.
            # Allow 2 s for bootloader + setup() to complete before sending commands.
            time.sleep(2.0)
            print(f"  🔌 Serial connected: {self.port} @ {self.baud} baud")
            return True
        except serial.SerialException as e:
            print(f"⚠️  MotorController: Could not open {self.port}: {e}", file=sys.stderr)
            self._serial = None
            return False

    def disconnect(self):
        """Safely stops motors and closes serial link."""
        if self.is_connected():
            self.stop()
            self._serial.close()
            self._serial = None

    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    # ── Low-level send ────────────────────────────────────────────────────────

    def _send(self, command: str) -> bool:
        """
        Sends an ASCII command terminated with newline.
        Returns True on success, False if disconnected or serial error.
        Callers that need ACK confirmation should call read_telemetry() after.
        """
        if not self.is_connected():
            return False
        try:
            line = (command.strip() + "\n").encode("utf-8")
            self._serial.write(line)
            self._serial.flush()
            return True
        except serial.SerialException as e:
            print(f"⚠️  Serial write error: {e}", file=sys.stderr)
            return False

    # ── Tier 1 — Pi Camera Latch Commands (highest priority) ─────────────────

    def pi_stop(self) -> bool:
        """
        Latched emergency stop from Pi camera pipeline.
        Freezes all motors instantly — even mid-avoidance.
        Only pi_go() releases the latch.
        Arduino responds: [ACK] PI_STOP  +  [TELEM] STATE=PI_STOPPED
        """
        return self._send("PI_STOP")

    def pi_go(self) -> bool:
        """
        Releases the Pi camera latch.
        Arduino returns to STATE_IDLE.
        Arduino responds: [ACK] PI_GO
        """
        return self._send("PI_GO")

    def pi_avoid(self, direction: str) -> bool:
        """
        Sends a one-shot directional hint for the Arduino's next Tier-2
        avoidance maneuver. If the hint is set when ultrasonics trigger,
        the Arduino turns in the hinted direction instead of deciding locally.

        Args:
            direction: "LEFT" or "RIGHT" (case-insensitive).
        """
        d = direction.strip().upper()
        if d not in ("LEFT", "RIGHT"):
            raise ValueError(f"pi_avoid: direction must be LEFT or RIGHT, got {direction!r}")
        return self._send(f"PI_AVOID {d}")

    # ── Tier 3 — Normal Driving Commands ─────────────────────────────────────

    def drive(self, speed: int, duration_ms: int) -> bool:
        """
        Drive forward at PWM speed (0–255) for duration_ms.
        A new DRIVE replaces any in-progress timed action (no queue).
        """
        speed = max(0, min(255, speed))
        return self._send(f"DRIVE {speed} {duration_ms}")

    def reverse(self, speed: int, duration_ms: int) -> bool:
        """Drive reverse at PWM speed (0–255) for duration_ms."""
        speed = max(0, min(255, speed))
        return self._send(f"REV {speed} {duration_ms}")

    def turn(self, angle_deg: int, speed: int = 150) -> bool:
        """
        Skid-steer turn by angle_deg.
        Positive = right, negative = left. Range: -180 to +180.
        Duration is computed on the Arduino from TURN_MS_PER_DEGREE.
        """
        angle_deg = max(-180, min(180, angle_deg))
        speed = max(0, min(255, speed))
        return self._send(f"TURN {angle_deg} {speed}")

    def stop(self) -> bool:
        """
        Non-latched immediate stop → Arduino returns to IDLE.
        Does NOT lock out further commands (unlike pi_stop).
        """
        return self._send("STOP")

    def ping(self) -> bool:
        """
        Liveness check. Arduino responds with [ACK] PONG.
        Use to confirm the serial link is alive before starting a run.
        """
        return self._send("PING")

    # ── Telemetry Reader ──────────────────────────────────────────────────────

    def read_telemetry(self) -> Optional[str]:
        """
        Reads and parses one pending telemetry line from the Arduino.

        Side-effects:
          - Parses '[TELEM] US LEFT=x RIGHT=y' → updates self.latest_sonar
          - Parses '[TELEM] STATE=XXX ...'     → updates self.latest_state

        Returns:
          The raw line string (so callers can log / inspect ACKs), or None
          if no data is available or serial is disconnected.
        """
        if not self.is_connected():
            return None
        try:
            if self._serial.in_waiting == 0:
                return None
            line = self._serial.readline().decode("utf-8", errors="replace").strip()
            if not line:
                return None

            # ── Parse ultrasonic telemetry ────────────────────────────────────
            # Format: [TELEM] US LEFT=15.3 RIGHT=42.7
            if "[TELEM] US" in line:
                self._parse_us_telemetry(line)

            # ── Parse state telemetry ─────────────────────────────────────────
            # Format: [TELEM] STATE=DRIVING SPEED=180
            elif "[TELEM] STATE=" in line:
                # Extract state token e.g. "STATE=DRIVING"
                for token in line.split():
                    if token.startswith("STATE="):
                        self.latest_state = token.split("=", 1)[1]
                        break

            return line

        except serial.SerialException:
            return None

    def _parse_us_telemetry(self, line: str):
        """
        Parses '[TELEM] US LEFT=15.3 RIGHT=42.7' and updates latest_sonar.
        Silently skips malformed lines.
        """
        try:
            for token in line.split():
                if token.startswith("LEFT="):
                    self.latest_sonar["left"] = float(token.split("=", 1)[1])
                elif token.startswith("RIGHT="):
                    self.latest_sonar["right"] = float(token.split("=", 1)[1])
        except (ValueError, IndexError):
            pass  # Malformed telemetry — ignore silently

    def drain_telemetry(self, max_lines: int = 20):
        """
        Drains up to max_lines of pending telemetry, updating internal state.
        Useful after sending a command to flush ACKs before the next operation.

        Returns:
            List of all raw lines read.
        """
        lines = []
        for _ in range(max_lines):
            line = self.read_telemetry()
            if line is None:
                break
            lines.append(line)
        return lines

    def wait_for_ack(self, command_prefix: str, timeout_s: float = 2.0) -> Optional[str]:
        """
        Blocks until an [ACK] line matching command_prefix is received,
        or until timeout_s seconds elapse.

        Args:
            command_prefix: e.g. "DRIVE", "PI_STOP", "PONG"
        Returns:
            The matching ACK line, or None on timeout.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            line = self.read_telemetry()
            if line and line.startswith("[ACK]") and command_prefix in line:
                return line
            time.sleep(0.01)
        return None
