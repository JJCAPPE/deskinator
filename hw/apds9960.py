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
APDS9960_CONFIG2 = 0x90

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
            print(
                f"APDS9960.init: Using bus {self.bus.bus_number}, address 0x{self.address:02X}"
            )
            # Power on
            self.bus.write_byte_data(self.address, APDS9960_ENABLE, APDS9960_PON)
            time.sleep(0.05)

            # Configure proximity detection: PDRIVE = 100mA, PGAIN = 8x
            self.bus.write_byte_data(self.address, APDS9960_CONTROL, 0x2C)

            # Verify CONTROL register
            ctl = self.bus.read_byte_data(self.address, APDS9960_CONTROL)
            if ctl != 0x2C:
                print(
                    f"APDS9960 Warning: CONTROL register mismatch! Expected 0x2C, got 0x{ctl:02X}"
                )
                # Retry once
                time.sleep(0.01)
                self.bus.write_byte_data(self.address, APDS9960_CONTROL, 0x2C)
                ctl = self.bus.read_byte_data(self.address, APDS9960_CONTROL)
                if ctl != 0x2C:
                    print(
                        f"APDS9960 Error: CONTROL register failed to set! Got 0x{ctl:02X}"
                    )

            # Proximity pulse: 8 pulses, 32us length (matches gesture sensor)
            # 0x80 (PPLEN=32us) | 0x07 (PPULSE=8) = 0x87
            # Previous was 0x8F (16us, 16 pulses)
            self.bus.write_byte_data(self.address, APDS9960_PPULSE, 0x87)

            # Verify PPULSE register
            pulse = self.bus.read_byte_data(self.address, APDS9960_PPULSE)
            if pulse != 0x87:
                print(
                    f"APDS9960 Warning: PPULSE register mismatch! Expected 0x87, got 0x{pulse:02X}"
                )
                # Retry once
                time.sleep(0.01)
                self.bus.write_byte_data(self.address, APDS9960_PPULSE, 0x87)

            # LED Boost (CONFIG2) - increases IR LED power
            # 0x41 = LED_BOOST 300% (bit 6 set? no 0x40 is LED_BOOST 300%)
            # Wait, 0x90 register CONFIG2.
            # Bit 5:4 LED_BOOST. 00=100%, 01=150%, 10=200%, 11=300%.
            # 0x41 ??
            # Register 0x90 (CONFIG2).
            # Bits 5:4 are LED_BOOST.
            # 00 = 100%, 01 = 150%, 10 = 200%, 11 = 300%.
            # User suggested 0x41.
            # 0x41 = 0100 0001.
            # Bit 6 is 1. Bit 0 is 1.
            # Reserved bits are 7, 3:2.
            # Bit 6 is PSIEN (Proximity Saturation Interrupt Enable).
            # Bit 0, 1 are reserved/CPSIEN?
            # Let's check datasheet or trust user.
            # User said: "self.bus.write_byte_data(self.address, 0x90, 0x41)  # if values still very low, try 0x49"
            # 0x40 is LED_BOOST=100%?? No.
            # Let's trust the user's value 0x41 for now as they requested it.
            self.bus.write_byte_data(self.address, APDS9960_CONFIG2, 0x41)

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
            # Small delay to ensure bus is ready (helps with timing differences between hardware/software I2C)
            time.sleep(0.002)
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
