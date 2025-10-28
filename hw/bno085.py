"""
BNO085 IMU driver (simplified).

Provides yaw angle and yaw rate for robot navigation.
Uses Adafruit BNO08x library if available.
"""

import time
import numpy as np
from .i2c import I2CBus

try:
    from adafruit_bno08x import BNO08X_I2C
    from adafruit_bno08x.i2c import BNO08X_I2C as BNO08X_I2C_CLASS

    BNO08X_AVAILABLE = True
except ImportError:
    BNO08X_AVAILABLE = False
    print(
        "Warning: adafruit-circuitpython-bno08x not available. IMU in simulation mode."
    )


class BNO085:
    """BNO085 IMU for orientation sensing."""

    def __init__(self, bus: I2CBus, address: int | None = None):
        """
        Initialize BNO085 IMU.

        Args:
            bus: I2C bus instance
            address: I2C address (auto-detect if None)
        """
        self.bus = bus
        self.address = address
        self.sensor = None
        self.sim_mode = not BNO08X_AVAILABLE

        self.yaw_bias = 0.0
        self.yaw_rate_bias = 0.0
        self.last_yaw = 0.0
        self.sim_yaw = 0.0
        self.sim_yaw_rate = 0.0

        if not self.sim_mode:
            self._init_sensor()

    def _init_sensor(self):
        """Initialize the BNO08x sensor."""
        try:
            # The Adafruit library needs a busio I2C object
            # For now, we'll use simulation mode
            self.sim_mode = True
            print("BNO085: Using simulation mode (busio integration needed)")
        except Exception as e:
            print(f"BNO085: Failed to initialize: {e}")
            self.sim_mode = True

    def read_yaw_rate(self) -> float:
        """
        Read yaw rate (angular velocity around Z axis).

        Returns:
            Yaw rate in rad/s
        """
        if self.sim_mode:
            return self.sim_yaw_rate

        try:
            # Read gyroscope data
            gyro = self.sensor.gyro
            if gyro:
                return gyro[2] - self.yaw_rate_bias  # Z axis
            return 0.0
        except:
            return 0.0

    def read_yaw_abs(self) -> float:
        """
        Read absolute yaw angle.

        Returns:
            Yaw angle in radians (-π to π)
        """
        if self.sim_mode:
            return self.sim_yaw

        try:
            # Read quaternion and convert to Euler angles
            quat = self.sensor.quaternion
            if quat:
                # Convert quaternion to yaw
                qx, qy, qz, qw = quat
                yaw = np.arctan2(
                    2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)
                )
                return yaw - self.yaw_bias
            return self.last_yaw
        except:
            return self.last_yaw

    def bias_calibrate(self, duration: float = 2.0):
        """
        Calibrate gyro bias by averaging readings while stationary.

        Args:
            duration: Calibration duration in seconds
        """
        print("BNO085: Calibrating gyro bias (keep stationary)...")

        if self.sim_mode:
            print("  [SIM] Bias calibration complete")
            return

        yaw_rates = []
        yaws = []

        start_time = time.time()
        while time.time() - start_time < duration:
            yaw_rates.append(self.read_yaw_rate())
            yaws.append(self.read_yaw_abs())
            time.sleep(0.05)

        self.yaw_rate_bias = np.mean(yaw_rates)
        self.yaw_bias = np.mean(yaws)

        print(f"  Yaw rate bias: {self.yaw_rate_bias:.4f} rad/s")
        print(f"  Yaw bias: {self.yaw_bias:.4f} rad")

    def set_sim_state(self, yaw: float, yaw_rate: float):
        """
        Set simulated IMU state (for testing).

        Args:
            yaw: Yaw angle in radians
            yaw_rate: Yaw rate in rad/s
        """
        self.sim_yaw = yaw
        self.sim_yaw_rate = yaw_rate
