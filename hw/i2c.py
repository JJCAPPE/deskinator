"""
I2C bus utilities.

Provides abstraction over smbus2 for I2C communication.
"""

import sys
from typing import List

try:
    from smbus2 import SMBus

    I2C_AVAILABLE = True
except ImportError:
    I2C_AVAILABLE = False
    print("Warning: smbus2 not available. Running in simulation mode.")


class I2CBus:
    """Manages I2C bus communication."""

    def __init__(self, bus_number: int = 1):
        """
        Initialize I2C bus.

        Args:
            bus_number: I2C bus number (usually 1 on Raspberry Pi)
        """
        self.bus_number = bus_number
        self.bus = None
        self.sim_mode = not I2C_AVAILABLE

        if not self.sim_mode:
            try:
                self.bus = SMBus(bus_number)
            except Exception as e:
                print(f"Warning: Could not open I2C bus {bus_number}: {e}")
                self.sim_mode = True

    def write_byte(self, addr: int, data: int):
        """Write a single byte to device."""
        if self.sim_mode:
            return
        self.bus.write_byte(addr, data)

    def write_byte_data(self, addr: int, register: int, data: int):
        """Write a byte to a specific register."""
        if self.sim_mode:
            return
        self.bus.write_byte_data(addr, register, data)

    def write_i2c_block_data(self, addr: int, register: int, data: List[int]):
        """Write a block of data to a register."""
        if self.sim_mode:
            return
        self.bus.write_i2c_block_data(addr, register, data)

    def read_byte(self, addr: int) -> int:
        """Read a single byte from device."""
        if self.sim_mode:
            return 0
        return self.bus.read_byte(addr)

    def read_byte_data(self, addr: int, register: int) -> int:
        """Read a byte from a specific register."""
        if self.sim_mode:
            return 0
        return self.bus.read_byte_data(addr, register)

    def read_i2c_block_data(self, addr: int, register: int, length: int) -> List[int]:
        """Read a block of data from a register."""
        if self.sim_mode:
            return [0] * length
        return self.bus.read_i2c_block_data(addr, register, length)

    def scan(self) -> List[int]:
        """
        Scan for I2C devices on the bus.

        Returns:
            List of detected I2C addresses
        """
        if self.sim_mode:
            print("[SIM] I2C scan: simulated devices at 0x29, 0x39, 0x70")
            return [0x29, 0x39, 0x70]

        devices = []
        for addr in range(0x03, 0x78):
            try:
                self.bus.write_quick(addr)
                devices.append(addr)
            except:
                pass
        return devices

    def close(self):
        """Close the I2C bus."""
        if not self.sim_mode and self.bus:
            self.bus.close()
