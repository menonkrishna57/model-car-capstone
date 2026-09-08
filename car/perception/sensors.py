"""
sensors.py — ICM20948 IMU & HC-SR04P Ultrasonic Array Reads
============================================================
Owned by perception-agent.
Provides filtered distance readings from 3× ultrasonic sensors and
yaw/heading rates from the ICM20948 9-DOF IMU over I2C.
"""

import time
from typing import Dict, Optional
from car.perception.config import ULTRASONIC_SENSORS, IMU_I2C_BUS, IMU_I2C_ADDRESS


class SensorManager:
    """Manages ultrasonic rangefinders and onboard IMU."""

    def __init__(self):
        self.sensors = ULTRASONIC_SENSORS
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
        # Skeleton/stub: returns simulated or hardware reads
        readings = {}
        for s in self.sensors:
            # Placeholder: 100.0 cm (clear path)
            readings[s["name"]] = 100.0
        return readings

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
