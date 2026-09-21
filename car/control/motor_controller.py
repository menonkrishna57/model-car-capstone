"""
motor_controller.py — Pi-side Serial Bridge to Arduino Uno
===========================================================
Owned by integration-agent.
Implements the Pi-side half of the serial handshake defined in arduino/PROTOCOL.md.
Keeps transport behind an interface so USB-serial or GPIO-UART can be swapped cleanly.
"""

import time
import sys
from typing import Optional
import serial

from car.perception.config import SERIAL_PORT, SERIAL_BAUD


class MotorController:
    """Manages serial communications and sends commands to the Arduino Uno."""

    def __init__(self, port: str = SERIAL_PORT, baud: int = SERIAL_BAUD, timeout: float = 0.5):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None
        # Shared state for latest sonar readings parsed from telemetry
        self.latest_sonar = {"left": 100.0, "center": 100.0, "right": 100.0}

    def connect(self) -> bool:
        """Opens the serial link and waits for Arduino bootloader to settle."""
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(2.0)  # ATmega328P resets on serial open; allow setup() to complete
            return True
        except serial.SerialException as e:
            print(f"⚠️ MotorController: Could not open {self.port}: {e}", file=sys.stderr)
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

    def _send(self, command: str) -> bool:
        """Sends an ASCII command line ending with newline."""
        if not self.is_connected():
            return False
        try:
            line = (command.strip() + "\n").encode("utf-8")
            self._serial.write(line)
            self._serial.flush()
            return True
        except serial.SerialException as e:
            print(f"⚠️ Serial write error: {e}", file=sys.stderr)
            return False

    def drive(self, speed: int, duration_ms: int) -> bool:
        """Drive forward at PWM speed (0–255) for duration_ms."""
        speed = max(0, min(255, speed))
        return self._send(f"DRIVE {speed} {duration_ms}")

    def reverse(self, speed: int, duration_ms: int) -> bool:
        """Drive reverse at PWM speed (0–255) for duration_ms."""
        speed = max(0, min(255, speed))
        return self._send(f"REV {speed} {duration_ms}")

    def turn(self, angle_deg: int, speed: int = 150) -> bool:
        """Skid-steer turn by angle_deg (-180 to +180; positive = right, negative = left)."""
        angle_deg = max(-180, min(180, angle_deg))
        speed = max(0, min(255, speed))
        return self._send(f"TURN {angle_deg} {speed}")

    def stop(self) -> bool:
        """Immediate emergency or regular stop."""
        return self._send("STOP")

    def ping(self) -> bool:
        """Liveness ping."""
        return self._send("PING")

    def read_telemetry(self) -> Optional[str]:
        """Reads incoming telemetry lines from the Arduino Uno."""
        if not self.is_connected():
            return None
        try:
            if self._serial.in_waiting > 0:
                line = self._serial.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    return None
                
                # Intercept and parse SONAR messages silently update state
                if line.startswith("[TELEM] SONAR"):
                    parts = line.split()
                    if len(parts) >= 5:
                        try:
                            self.latest_sonar["left"] = float(parts[2])
                            self.latest_sonar["center"] = float(parts[3])
                            self.latest_sonar["right"] = float(parts[4])
                        except ValueError:
                            pass
                
                return line
        except serial.SerialException:
            pass
        return None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
