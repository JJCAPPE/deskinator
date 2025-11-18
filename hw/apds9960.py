"""
APDS9960 proximity sensor driver.

Minimal implementation using adafruit_apds9960 library.
"""

import time
import numpy as np
from .i2c import I2CBus

try:
    import busio
    from adafruit_apds9960.apds9960 import APDS9960 as AdafruitAPDS9960

    ADAFRUIT_AVAILABLE = True
except ImportError:
    ADAFRUIT_AVAILABLE = False


class APDS9960:
    """APDS9960 proximity sensor using adafruit library."""

    def __init__(self, bus: I2CBus, address: int = 0x39):
        """
        Initialize APDS9960 sensor.

        Args:
            bus: I2C bus instance (extracts bus_number from it)
            address: I2C address (default 0x39, ignored - adafruit library handles it)
        """
        self.bus_number = bus.bus_number  # currently informational only
        self.address = address
        self.calibration_offset = 0.0
        self.calibration_scale = 1.0
        self.sensor = None
        self.sim_mode = not ADAFRUIT_AVAILABLE

        if not ADAFRUIT_AVAILABLE:
            print(
                "Warning: adafruit_apds9960 not available. "
                "Install with: pip install adafruit-circuitpython-apds9960"
            )

    def init(self):
        """Initialize sensor for proximity detection."""
        if self.sim_mode:
            print("APDS9960: Running in simulation mode")
            return

        try:
            import board

            # Uses default I2C pins (bus 1 on Raspberry Pi)
            i2c = busio.I2C(board.SCL, board.SDA)

            self.sensor = AdafruitAPDS9960(i2c)
            self.sensor.enable_proximity = True

            time.sleep(0.1)

        except Exception as e:
            print(f"APDS9960: Failed to initialize on bus {self.bus_number}: {e}")
            print("  Falling back to simulation mode")
            import traceback

            traceback.print_exc()
            self.sim_mode = True
            self.sensor = None

    def read_proximity_raw(self) -> int:
        """
        Read raw proximity value.

        Returns:
            Raw proximity value (0-255)
        """
        if self.sim_mode or self.sensor is None:
            return 0

        try:
            return int(self.sensor.proximity)
        except Exception as e:
            print(f"APDS9960: Read error: {e}")
            return 0

    def read_proximity_norm(self) -> float:
        """
        Read normalized proximity value.

        Returns:
            Normalized proximity 0.0-1.0 (0=far, 1=near)
        """
        raw = self.read_proximity_raw()
        if self.calibration_offset == 0.0 and self.calibration_scale == 1.0:
            normalized = raw / 255.0
        else:
            normalized = (raw - self.calibration_offset) * self.calibration_scale
        return float(np.clip(normalized, 0.0, 1.0))

    def calibrate(self, on_table_samples: int = 20, off_table_samples: int = 20):
        """
        Calibrate sensor with on-table and off-table readings.

        This should be called during setup with the sensor first over the table,
        then off the edge.

        Args:
            on_table_samples: Number of samples to take on table
            off_table_samples: Number of samples to take off table
        """
        if self.sim_mode:
            print("APDS9960: Calibration skipped in simulation mode.")
            return

        print(f"APDS9960 @ 0x{self.address:02X}: Calibrating...")
        print("  Place sensor over table surface...")
        time.sleep(2.0)

        on_table_readings = []
        for _ in range(on_table_samples):
            on_table_readings.append(self.read_proximity_raw())
            time.sleep(0.05)

        on_table_mean = np.mean(on_table_readings)

        print("  Move sensor off table edge...")
        time.sleep(2.0)

        off_table_readings = []
        for _ in range(off_table_samples):
            off_table_readings.append(self.read_proximity_raw())
            time.sleep(0.05)

        off_table_mean = np.mean(off_table_readings)

        self.calibration_offset = off_table_mean
        if on_table_mean > off_table_mean:
            self.calibration_scale = 1.0 / (on_table_mean - off_table_mean)
        else:
            print("  WARNING: on-table mean is not greater than off-table mean.")
            print("           Using default scale=1.0 (normalization may be poor).")
            self.calibration_scale = 1.0

        print("  Calibration complete:")
        print(f"    On-table:  {on_table_mean:.1f}")
        print(f"    Off-table: {off_table_mean:.1f}")
        print(f"    Scale:     {self.calibration_scale:.4f}")

        if on_table_mean < 10 or off_table_mean < 10:
            print("  WARNING: Very low raw values detected!")
            print("    This may indicate:")
            print("    - Sensor not properly initialized")
            print("    - Sensor too far from surface")
            print("    - Hardware connection issues")
            print("    - Sensor needs more time to stabilize")

        if abs(on_table_mean - off_table_mean) < 2.0:
            print("  WARNING: Small difference between on-table and off-table!")
            print(f"    Difference: {abs(on_table_mean - off_table_mean):.1f}")
            print("    This may make edge detection unreliable.")
