"""
serial_bridge.py — Pi ↔ Arduino Nano Serial Interface
======================================================
Implements the Pi-side half of the serial handshake.

Transport is behind an interface so either USB-serial (/dev/ttyUSB0)
or GPIO-UART (/dev/ttyAMA0) works — just change config.SERIAL_PORT.

The message protocol is defined by firmware-agent and documented in
/docs/serial_protocol.md.  This module sends commands; it does NOT
define the protocol itself.

NOTE: Unverified / simulated — not yet tested on real hardware.
"""

import serial
import time
from perception.config import SERIAL_PORT, SERIAL_BAUD


class SerialBridge:
    """Manages the serial connection to the Arduino Nano."""

    def __init__(self, port: str = SERIAL_PORT, baud: int = SERIAL_BAUD, timeout: float = 1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._serial = None

    def connect(self):
        """Open serial connection and wait for Arduino to reset."""
        self._serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(2)  # Arduino resets on serial open; wait for bootloader

    def disconnect(self):
        """Close serial connection."""
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None

    def send_command(self, command: str):
        """Send a command string to the Arduino.

        Args:
            command: The command to send (e.g., 'FWD', 'STOP', 'LEFT').
                     Protocol format is defined in /docs/serial_protocol.md.
        """
        if self._serial and self._serial.is_open:
            self._serial.write((command + "\n").encode("utf-8"))
            self._serial.flush()

    def read_telemetry(self) -> str | None:
        """Read one line of telemetry from the Arduino.

        Returns:
            Decoded string line, or None if nothing available.
        """
        if self._serial and self._serial.is_open and self._serial.in_waiting:
            return self._serial.readline().decode("utf-8", errors="replace").strip()
        return None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()
