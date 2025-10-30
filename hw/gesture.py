"""
Gesture sensor using APDS9960 for start/stop control.

Uses a dedicated I2C bus (software or hardware) on GPIO17/GPIO4 to avoid
conflicts with the 4 proximity sensors on the main I2C bus.
"""

import time
from typing import Literal

try:
    import smbus2

    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False
    print("Warning: smbus2 not available. Gesture sensor in simulation mode.")

try:  # Support both package and script execution
    from ..config import PINS, I2C as I2C_CONFIG  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - fallback when run as script
    from config import PINS, I2C as I2C_CONFIG


# APDS9960 Registers
APDS9960_ENABLE = 0x80
APDS9960_GCONF1 = 0xA2
APDS9960_GCONF2 = 0xA3
APDS9960_GCONF3 = 0xAA
APDS9960_GCONF4 = 0xAB
APDS9960_GSTATUS = 0xAF
APDS9960_GFLVL = 0xAE
APDS9960_GFIFO_U = 0xFC
APDS9960_GFIFO_D = 0xFD
APDS9960_GFIFO_L = 0xFE
APDS9960_GFIFO_R = 0xFF
APDS9960_GPULSE = 0xA6
APDS9960_GOFFSET_U = 0xA9
APDS9960_GOFFSET_D = 0xA7
APDS9960_GOFFSET_L = 0xA8
APDS9960_GOFFSET_R = 0xAA

# Enable bits
APDS9960_PON = 0x01  # Power on
APDS9960_GEN = 0x40  # Gesture enable
APDS9960_PEN = 0x04  # Proximity enable

# Gesture directions
GESTURE_NONE = 0
GESTURE_UP = 1
GESTURE_DOWN = 2
GESTURE_LEFT = 3
GESTURE_RIGHT = 4


GestureType = Literal["none", "up", "down", "left", "right"]


class GestureSensor:
    """
    APDS9960 gesture sensor for touchless start/stop control.

    Uses a separate I2C bus to avoid address conflicts with proximity sensors.
    """

    def __init__(self, bus_number: int = 3, address: int = 0x39):
        """
        Initialize gesture sensor.

        Args:
            bus_number: I2C bus number (3 for software I2C on GPIO17/4)
            address: I2C address (default 0x39)

        Note:
            On Raspberry Pi, you may need to configure software I2C or use
            dtoverlay to create an additional I2C bus on GPIO17/GPIO4.
        """
        self.bus_number = bus_number
        self.address = address
        self.bus = None
        self.sim_mode = not SMBUS_AVAILABLE

        # Gesture state
        self.gesture_ud_delta = 0
        self.gesture_lr_delta = 0
        self.gesture_ud_count = 0
        self.gesture_lr_count = 0
        self.gesture_near_count = 0
        self.gesture_far_count = 0

        if not self.sim_mode:
            try:
                from smbus2 import SMBus

                self.bus = SMBus(bus_number)
                self._init_sensor()
            except Exception as e:
                print(f"Gesture sensor: Failed to initialize on bus {bus_number}: {e}")
                print("  Falling back to simulation mode")
                print("  See HARDWARE_SETUP.md for I2C bus configuration")
                self.sim_mode = True

    def _init_sensor(self):
        """Initialize the APDS9960 for gesture detection."""
        if self.sim_mode:
            return

        try:
            # Power on
            self.bus.write_byte_data(self.address, APDS9960_ENABLE, APDS9960_PON)
            time.sleep(0.01)

            # Configure gesture engine
            # GMODE = 1 (gesture mode)
            self.bus.write_byte_data(self.address, APDS9960_GCONF4, 0x01)

            # Set gesture proximity entry threshold
            self.bus.write_byte_data(self.address, APDS9960_GCONF1, 0x40)

            # Set gesture exit persistence (4 gesture end)
            self.bus.write_byte_data(self.address, APDS9960_GCONF2, 0x01)

            # Set gesture LED drive strength and wait time
            self.bus.write_byte_data(self.address, APDS9960_GPULSE, 0xC9)

            # Enable proximity and gesture
            enable = self.bus.read_byte_data(self.address, APDS9960_ENABLE)
            self.bus.write_byte_data(
                self.address, APDS9960_ENABLE, enable | APDS9960_GEN | APDS9960_PEN
            )

            time.sleep(0.01)

        except Exception as e:
            print(f"Gesture sensor initialization error: {e}")
            self.sim_mode = True

    def is_gesture_available(self) -> bool:
        """
        Check if a gesture is available to read.

        Returns:
            True if gesture data is ready
        """
        if self.sim_mode:
            return False

        try:
            status = self.bus.read_byte_data(self.address, APDS9960_GSTATUS)
            return (status & 0x01) != 0  # GVALID bit
        except:
            return False

    def read_gesture(self) -> GestureType:
        """
        Read and decode gesture.

        Returns:
            Gesture direction: "none", "up", "down", "left", "right"
        """
        if self.sim_mode:
            return "none"

        if not self.is_gesture_available():
            return "none"

        try:
            # Read FIFO level
            fifo_level = self.bus.read_byte_data(self.address, APDS9960_GFLVL)

            if fifo_level == 0:
                return "none"

            # Read gesture data
            gesture_data = []
            for _ in range(fifo_level):
                up = self.bus.read_byte_data(self.address, APDS9960_GFIFO_U)
                down = self.bus.read_byte_data(self.address, APDS9960_GFIFO_D)
                left = self.bus.read_byte_data(self.address, APDS9960_GFIFO_L)
                right = self.bus.read_byte_data(self.address, APDS9960_GFIFO_R)
                gesture_data.append((up, down, left, right))

            # Process gesture data
            return self._process_gesture_data(gesture_data)

        except Exception as e:
            print(f"Gesture read error: {e}")
            return "none"

    def _process_gesture_data(self, data: list) -> GestureType:
        """
        Process raw gesture data to determine direction.

        Args:
            data: List of (up, down, left, right) tuples

        Returns:
            Gesture direction
        """
        if len(data) < 4:
            return "none"

        # Calculate deltas
        ud_delta = 0
        lr_delta = 0

        for up, down, left, right in data:
            ud_delta += up - down
            lr_delta += right - left

        # Determine primary direction
        if abs(ud_delta) > abs(lr_delta):
            if ud_delta > 13:
                return "up"
            elif ud_delta < -13:
                return "down"
        else:
            if lr_delta > 13:
                return "right"
            elif lr_delta < -13:
                return "left"

        return "none"

    def wait_for_gesture(self, timeout: float = 30.0) -> GestureType:
        """
        Wait for a gesture to occur.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            Detected gesture or "none" if timeout
        """
        start_time = time.time()

        while time.time() - start_time < timeout:
            gesture = self.read_gesture()
            if gesture != "none":
                return gesture
            time.sleep(0.05)

        return "none"

    def wait_for_start_gesture(
        self, start_gestures: list[GestureType] = ["up", "right"], timeout: float = 30.0
    ) -> bool:
        """
        Wait for a specific gesture to start the robot.

        Args:
            start_gestures: List of gestures that trigger start (default: up or right)
            timeout: Maximum time to wait

        Returns:
            True if start gesture detected, False if timeout
        """
        print(f"Waiting for gesture ({', '.join(start_gestures)}) to start...")
        print("Wave your hand over the sensor to begin")

        gesture = self.wait_for_gesture(timeout)

        if gesture in start_gestures:
            print(f"✓ Gesture detected: {gesture}")
            return True
        elif gesture != "none":
            print(f"Wrong gesture: {gesture}. Try again.")
            return False
        else:
            print("Timeout waiting for gesture")
            return False

    def cleanup(self):
        """Clean up resources."""
        if not self.sim_mode and self.bus:
            try:
                # Disable gesture engine
                enable = self.bus.read_byte_data(self.address, APDS9960_ENABLE)
                self.bus.write_byte_data(
                    self.address, APDS9960_ENABLE, enable & ~APDS9960_GEN
                )
                self.bus.close()
            except:
                pass


def test_gestures():
    """Test gesture sensor interactively."""
    print("=" * 60)
    print("GESTURE SENSOR TEST")
    print("=" * 60)
    print("\nWave your hand over the sensor:")
    print("  UP    - Start cleaning")
    print("  DOWN  - Stop")
    print("  LEFT  - Previous mode")
    print("  RIGHT - Next mode")
    print("\nPress Ctrl+C to exit\n")

    sensor = GestureSensor(I2C_CONFIG.GESTURE_BUS, I2C_CONFIG.GESTURE_ADDR)

    try:
        while True:
            gesture = sensor.read_gesture()
            if gesture != "none":
                print(f"Gesture detected: {gesture.upper()}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\nTest stopped")
    finally:
        sensor.cleanup()


if __name__ == "__main__":
    test_gestures()
