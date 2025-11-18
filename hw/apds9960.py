"""
APDS9960 proximity sensor driver.

Minimal implementation using direct I2C register access.
"""

import time
import numpy as np
from .i2c import I2CBus

# APDS9960 Registers
APDS9960_ENABLE = 0x80
APDS9960_PDATA = 0x9C
APDS9960_CONTROL = 0x8F
APDS9960_PPULSE = 0x8E

# Enable bits
APDS9960_PON = 0x01  # Power on
APDS9960_PEN = 0x04  # Proximity enable


class APDS9960:
    """APDS9960 proximity sensor."""

    def __init__(self, bus: I2CBus, address: int = 0x39):
        """
        Initialize APDS9960 sensor.

        Args:
            bus: I2C bus instance
            address: I2C address (default 0x39)
        """
        self.bus = bus
        self.address = address
        self.calibration_offset = 0.0
        self.calibration_scale = 1.0
        self.sim_mode = bus.sim_mode

    def init(self):
        """Initialize sensor for proximity detection."""
        if self.sim_mode:
            print("APDS9960: Running in simulation mode")
            return

        try:
            # Power on
            self.bus.write_byte_data(self.address, APDS9960_ENABLE, APDS9960_PON)
            time.sleep(0.05)

            # Configure proximity detection: PDRIVE = 100mA, PGAIN = 8x
            self.bus.write_byte_data(self.address, APDS9960_CONTROL, 0x2C)

            # Proximity pulse: 16 pulses, 16us length
            self.bus.write_byte_data(self.address, APDS9960_PPULSE, 0x8F)

            # Enable proximity detection
            enable = self.bus.read_byte_data(self.address, APDS9960_ENABLE)
            self.bus.write_byte_data(
                self.address, APDS9960_ENABLE, enable | APDS9960_PEN
            )

            time.sleep(0.1)

        except Exception as e:
            print(f"APDS9960: Failed to initialize on bus {self.bus.bus_number}: {e}")
            print("  Falling back to simulation mode")
            self.sim_mode = True

    def read_proximity_raw(self) -> int:
        """
        Read raw proximity value.

        Returns:
            Raw proximity value (0-255)
        """
        if self.sim_mode:
            return 0

        try:
            return int(self.bus.read_byte_data(self.address, APDS9960_PDATA))
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
