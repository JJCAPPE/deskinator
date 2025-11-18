"""
APDS9960 proximity sensor driver.

Simplified driver for proximity sensing only (no gesture/color).
"""

import time
import numpy as np
from .i2c import I2CBus


# APDS9960 Registers
APDS9960_ENABLE = 0x80
APDS9960_PDATA = 0x9C
APDS9960_CONTROL = 0x8F
APDS9960_PPULSE = 0x8E
APDS9960_POFFSET_UR = 0x9D
APDS9960_POFFSET_DL = 0x9E
APDS9960_STATUS = 0x93  # Status register

# Enable bits
APDS9960_PON = 0x01  # Power on
APDS9960_PEN = 0x04  # Proximity enable

# Status register bits
APDS9960_PVALID = 0x02  # Proximity data valid (bit 1)


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

    def init(self):
        """Initialize sensor for proximity detection."""
        # Power on
        self.bus.write_byte_data(self.address, APDS9960_ENABLE, APDS9960_PON)
        time.sleep(0.05)  # Give sensor time to power up

        # Configure proximity detection
        # PDRIVE = 100mA, PGAIN = 8x
        self.bus.write_byte_data(self.address, APDS9960_CONTROL, 0x2C)

        # Proximity pulse: 16 pulses, 16us length (stronger signal)
        self.bus.write_byte_data(self.address, APDS9960_PPULSE, 0x8F)

        # Enable proximity detection
        enable = self.bus.read_byte_data(self.address, APDS9960_ENABLE)
        self.bus.write_byte_data(self.address, APDS9960_ENABLE, enable | APDS9960_PEN)

        # Wait for sensor to stabilize and take first measurement
        time.sleep(0.1)
        
        # Wait for valid data to be available
        self._wait_for_valid_data(timeout=0.5)

    def _wait_for_valid_data(self, timeout: float = 0.5):
        """
        Wait for proximity data to be valid.
        
        Args:
            timeout: Maximum time to wait in seconds
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.bus.read_byte_data(self.address, APDS9960_STATUS)
            if status & APDS9960_PVALID:
                return
            time.sleep(0.01)
        # If timeout, continue anyway - sensor might still work

    def read_proximity_raw(self) -> int:
        """
        Read raw proximity value.

        Returns:
            Raw proximity value (0-255)
        """
        # Wait briefly for valid data (non-blocking check)
        self._wait_for_valid_data(timeout=0.05)
        return self.bus.read_byte_data(self.address, APDS9960_PDATA)

    def read_proximity_norm(self) -> float:
        """
        Read normalized proximity value.

        Returns:
            Normalized proximity 0.0-1.0 (0=far, 1=near)
        """
        raw = self.read_proximity_raw()
        # If not calibrated (default offset=0, scale=1), fall back to raw/255.0
        if self.calibration_offset == 0.0 and self.calibration_scale == 1.0:
            normalized = raw / 255.0
        else:
            normalized = (raw - self.calibration_offset) * self.calibration_scale
        return np.clip(normalized, 0.0, 1.0)

    def calibrate(self, on_table_samples: int = 20, off_table_samples: int = 20):
        """
        Calibrate sensor with on-table and off-table readings.

        This should be called during setup with the sensor first over the table,
        then off the edge.

        Args:
            on_table_samples: Number of samples to take on table
            off_table_samples: Number of samples to take off table
        """
        print(f"APDS9960 @ {self.address:02x}: Calibrating...")
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

        # Set calibration parameters
        # Map off_table -> 0.0, on_table -> 1.0
        self.calibration_offset = off_table_mean
        if on_table_mean > off_table_mean:
            self.calibration_scale = 1.0 / (on_table_mean - off_table_mean)
        else:
            self.calibration_scale = 1.0

        print(f"  Calibration complete:")
        print(f"    On-table: {on_table_mean:.1f}")
        print(f"    Off-table: {off_table_mean:.1f}")
        print(f"    Scale: {self.calibration_scale:.4f}")
        
        # Warn if values seem suspicious
        if on_table_mean < 10 or off_table_mean < 10:
            print(f"  WARNING: Very low raw values detected!")
            print(f"    This may indicate:")
            print(f"    - Sensor not properly initialized")
            print(f"    - Sensor too far from surface")
            print(f"    - Hardware connection issues")
            print(f"    - Sensor needs more time to stabilize")
        
        if abs(on_table_mean - off_table_mean) < 2.0:
            print(f"  WARNING: Small difference between on-table and off-table!")
            print(f"    Difference: {abs(on_table_mean - off_table_mean):.1f}")
            print(f"    This may make edge detection unreliable.")
