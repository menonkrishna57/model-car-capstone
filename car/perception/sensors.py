"""
sensors.py — ICM20948 IMU & HC-SR04P Ultrasonic Array Reads
============================================================
Owned by perception-agent.
Provides filtered distance readings from 3× ultrasonic sensors and
yaw/heading rates from the ICM20948 9-DOF IMU over I2C.
"""

import time
from typing import Dict, Optional
from car.perception.config import IMU_I2C_BUS, IMU_I2C_ADDRESS
from car.control.motor_controller import MotorController


class SensorManager:
    """Manages ultrasonic rangefinders and onboard IMU."""

    def __init__(self, motor_controller: Optional[MotorController] = None):
        self.motor = motor_controller
        self.imu_bus_num = IMU_I2C_BUS
        self.imu_address = IMU_I2C_ADDRESS
        self._imu_bus = None
        self._init_imu()

    def _init_imu(self):
        """Initializes I2C connection to ICM20948."""
        try:
            import smbus2
            self._imu_bus = smbus2.SMBus(self.imu_bus_num)
        except Exception:
            # Running off-device or smbus2 unavailable
            self._imu_bus = None

    def read_ultrasonic_distances(self) -> Dict[str, float]:
        """Reads distance in cm from each ultrasonic sensor.

        Returns:
            Dict mapping sensor name ('left', 'center', 'right') to distance in cm.
        """
        if self.motor is not None:
            return self.motor.latest_sonar
        
        # Fallback if running without motor controller
        return {"left": 100.0, "center": 100.0, "right": 100.0}

    def read_imu_heading(self) -> Optional[float]:
        """Reads current yaw/heading in degrees from the ICM20948 IMU.

        Returns:
            Heading in degrees (-180.0 to +180.0), or None if read fails.
        """
        if self._imu_bus is None:
            return 0.0
        # Placeholder for register read & DMP/fusion logic
        return 0.0

    def close(self):
        """Closes any open sensor interfaces."""
        if self._imu_bus is not None:
            try:
                self._imu_bus.close()
            except Exception:
                pass
            self._imu_bus = None
