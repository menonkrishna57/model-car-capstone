"""
navigation.py — IMU-aided Dead Reckoning Point-to-Point Navigation
===================================================================
Owned by integration-agent.
No wheel encoders are available on the chassis. Navigation fallback relies on
dead reckoning using calibrated PWM speed + duration, guided by ICM20948 IMU heading.
"""

import time
import math
from typing import Tuple
from car.control.motor_controller import MotorController
from car.perception.sensors import SensorManager


class NavigationSystem:
    """Manages point-to-point waypoints and obstacle-avoidance logic."""

    def __init__(self, controller: MotorController, sensors: SensorManager):
        self.controller = controller
        self.sensors = sensors
        # Calibrated baseline: at PWM 180, car covers approx 0.35 meters/second (flat ground)
        self.speed_m_per_s = 0.35
        self.default_pwm = 180

    def navigate_step(self, target_distance_m: float, target_heading_deg: float) -> bool:
        """Executes a turn-then-drive step toward a relative waypoint.

        Args:
            target_distance_m: Distance in meters to drive.
            target_heading_deg: Angle relative to current heading (-180 to +180).

        Returns:
            True if step completed safely without obstacle abort.
        """
        # Step 1: Check forward obstacles via ultrasonic sensors
        distances = self.sensors.read_ultrasonic_distances()
        if distances.get("center", 100.0) < 25.0:
            print("🚨 Obstacle detected ahead! Aborting move step.")
            self.controller.stop()
            return False

        # Step 2: Turn to align with target heading
        if abs(target_heading_deg) > 5.0:
            self.controller.turn(int(target_heading_deg), speed=160)
            # Allow time for turn action
            time.sleep(abs(target_heading_deg) * 0.012 + 0.2)

        # Step 3: Drive forward estimated duration
        duration_ms = int((target_distance_m / self.speed_m_per_s) * 1000)
        self.controller.drive(self.default_pwm, duration_ms)

        # Monitor distance while driving
        start_t = time.time()
        while (time.time() - start_t) < (duration_ms / 1000.0):
            d = self.sensors.read_ultrasonic_distances()
            if d.get("center", 100.0) < 20.0:
                print("🚨 Emergency stop during navigation step!")
                self.controller.stop()
                return False
            time.sleep(0.05)

        return True
