"""MPU-6050 IMU driver.

Provides yaw rate (gyro Z) and an integrated yaw estimate for compatibility with
the existing navigation stack. Falls back to simulation mode when I2C is
unavailable so the rest of the system can continue to run.
"""

from __future__ import annotations

import math
import time
from statistics import mean

from .i2c import I2CBus


# MPU-6050 register addresses
_PWR_MGMT_1 = 0x6B
_SMPLRT_DIV = 0x19
_CONFIG = 0x1A
_GYRO_CONFIG = 0x1B
_ACCEL_CONFIG = 0x1C
_ACCEL_ZOUT_H = 0x3F
_GYRO_ZOUT_H = 0x47


class MPU6050:
    """Simplified MPU-6050 driver for yaw-only use."""

    def __init__(self, bus: I2CBus, address: int = 0x68):
        self.bus = bus
        self.address = address
        self.sim_mode = bus.sim_mode

        # Scale factors for default ±500 deg/s and ±4 g ranges
        self._gyro_scale = 65.5  # LSB per deg/s when FS_SEL = 1 (±500 deg/s)

        # Integrated state
        self._yaw = 0.0
        self._yaw_bias = 0.0
        self._yaw_rate_bias = 0.0
        self._last_time = time.time()

        # Simulation placeholders
        self._sim_yaw = 0.0
        self._sim_yaw_rate = 0.0

        if not self.sim_mode:
            try:
                self._init_sensor()
            except Exception as exc:  # pragma: no cover - hardware specific
                print(f"MPU6050: Initialization failed ({exc}). Using simulation mode.")
                self.sim_mode = True

    def _init_sensor(self):
        """Configure sensor for gyro data."""

        # Wake up device
        self.bus.write_byte_data(self.address, _PWR_MGMT_1, 0x00)
        time.sleep(0.05)

        # Set sample rate (1 kHz / (1 + divider)). Divider=7 → 125 Hz.
        self.bus.write_byte_data(self.address, _SMPLRT_DIV, 7)

        # Set digital low-pass filter (~10 Hz bandwidth)
        self.bus.write_byte_data(self.address, _CONFIG, 0x05)

        # Gyro full-scale ±500 deg/s (FS_SEL = 1)
        self.bus.write_byte_data(self.address, _GYRO_CONFIG, 0x08)

        # (Optional) Accel full-scale ±4 g (AFS_SEL = 1)
        self.bus.write_byte_data(self.address, _ACCEL_CONFIG, 0x08)

    def _read_word(self, register: int) -> int:
        """Read a signed 16-bit value from the IMU."""

        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        value = (high << 8) | low
        if value & 0x8000:
            value -= 0x10000
        return value

    def _update_integration(self, yaw_rate: float):
        """Integrate yaw rate over time."""

        current = time.time()
        dt = current - self._last_time
        self._last_time = current
        self._yaw += yaw_rate * dt

    def read_yaw_rate(self) -> float:
        """Return yaw rate (rad/s)."""

        if self.sim_mode:
            return self._sim_yaw_rate

        raw = self._read_word(_GYRO_ZOUT_H)
        deg_per_s = raw / self._gyro_scale
        rad_per_s = math.radians(deg_per_s)
        self._update_integration(rad_per_s)
        return rad_per_s - self._yaw_rate_bias

    def read_yaw_abs(self) -> float:
        """Return integrated yaw (rad)."""

        if self.sim_mode:
            return self._sim_yaw

        # Ensure integration is up to date even if caller only asks for yaw
        self.read_yaw_rate()
        return self._yaw - self._yaw_bias

    def bias_calibrate(self, duration: float = 2.0):
        """Estimate gyro bias. Keep robot stationary during calibration."""

        print("MPU6050: Calibrating gyro bias (keep stationary)...")

        if self.sim_mode:
            print("  [SIM] Bias calibration complete")
            return

        yaw_rates = []
        yaws = []
        start = time.time()

        while time.time() - start < duration:
            yaw_rates.append(self.read_yaw_rate())
            yaws.append(self.read_yaw_abs())
            time.sleep(0.02)

        self._yaw_rate_bias = mean(yaw_rates)
        self._yaw_bias = mean(yaws)

        print(f"  Yaw rate bias: {self._yaw_rate_bias:.5f} rad/s")
        print(f"  Yaw bias: {self._yaw_bias:.5f} rad")

    # --- Simulation helpers -------------------------------------------------
    def set_sim_state(self, yaw: float, yaw_rate: float):
        self._sim_yaw = yaw
        self._sim_yaw_rate = yaw_rate


__all__ = ["MPU6050"]
