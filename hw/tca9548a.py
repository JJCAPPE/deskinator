"""
TCA9548A I2C multiplexer driver.

Allows multiple I2C devices with the same address to coexist.
"""

from .i2c import I2CBus


class TCA9548A:
    """TCA9548A 8-channel I2C multiplexer."""

    def __init__(self, bus: I2CBus, address: int = 0x70):
        """
        Initialize TCA9548A.

        Args:
            bus: I2C bus instance
            address: I2C address of the multiplexer (default 0x70)
        """
        self.bus = bus
        self.address = address
        self.current_channel = None

    def select(self, channel: int | None):
        """
        Select a multiplexer channel.

        Args:
            channel: Channel number 0-7, or None to disable all channels
        """
        if channel is None:
            # Disable all channels
            self.bus.write_byte(self.address, 0x00)
            self.current_channel = None
        elif 0 <= channel <= 7:
            # Enable the specified channel
            self.bus.write_byte(self.address, 1 << channel)
            self.current_channel = channel
        else:
            raise ValueError(f"Invalid channel {channel}. Must be 0-7 or None.")

    def get_channel(self) -> int | None:
        """Get currently selected channel."""
        return self.current_channel

    def disable_all(self):
        """Disable all channels."""
        self.select(None)
