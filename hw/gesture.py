"""
Gesture sensor using APDS9960 for start/stop control.

Uses a dedicated I2C bus (software or hardware) on GPIO17/GPIO4 to avoid
conflicts with the proximity sensors on other I2C buses.
"""

import time
from typing import Literal, Optional

try:
    import busio
    import board
    from adafruit_apds9960.apds9960 import APDS9960 as AdafruitAPDS9960

    ADAFRUIT_AVAILABLE = True
except ImportError:
    ADAFRUIT_AVAILABLE = False
    print(
        "Warning: adafruit_apds9960 not available. Gesture sensor in simulation mode."
    )

try:  # Support both package and script execution
    from ..config import PINS, I2C as I2C_CONFIG  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - fallback when run as script
    from config import PINS, I2C as I2C_CONFIG


GestureType = Literal["none", "up", "down", "left", "right"]


class GestureSensor:
    """
    APDS9960 gesture/proximity sensor for touchless start control.

    Uses a separate I2C bus to avoid address conflicts with proximity sensors.
    Simplified to use adafruit library like the proximity sensors.
    """

    def __init__(
        self,
        bus_number: int = 3,
        address: int = 0x39,
        trigger_threshold: int = 50,
    ):
        """
        Initialize gesture sensor.

        Args:
            bus_number: I2C bus number (3 for software I2C on GPIO17/4)
            address: I2C address (default 0x39, ignored - adafruit library handles it)
            trigger_threshold: Raw proximity value threshold to trigger start (default 50)
        """
        self.bus_number = bus_number
        self.address = address
        self.trigger_threshold = trigger_threshold
        self.sensor = None
        self.sim_mode = not ADAFRUIT_AVAILABLE
        self.last_proximity_raw = 0

        if not ADAFRUIT_AVAILABLE:
            print(
                "Warning: adafruit_apds9960 not available. "
                "Install with: pip install adafruit-circuitpython-apds9960"
            )

        if not self.sim_mode:
            try:
                self._init_sensor()
            except Exception as e:
                print(f"Gesture sensor: Failed to initialize on bus {bus_number}: {e}")
                print("  Falling back to simulation mode")
                self.sim_mode = True
                self.sensor = None

    def _init_sensor(self):
        """Initialize the APDS9960 for proximity detection."""
        if self.sim_mode:
            return

        try:
            # Uses default I2C pins (bus 1 on Raspberry Pi)
            # Note: For other buses, you may need to configure GPIO pins differently
            i2c = busio.I2C(board.SCL, board.SDA)

            self.sensor = AdafruitAPDS9960(i2c)
            self.sensor.enable_proximity = True

            time.sleep(0.1)

            print(
                f"  Gesture sensor ready (trigger threshold: {self.trigger_threshold})"
            )

        except Exception as e:
            print(f"Gesture sensor initialization error: {e}")
            import traceback

            traceback.print_exc()
            self.sim_mode = True
            self.sensor = None

    def read_proximity_raw(self) -> int:
        """Read raw proximity value (0-255)."""
        if self.sim_mode or self.sensor is None:
            return 0

        try:
            raw = int(self.sensor.proximity)
            self.last_proximity_raw = raw
            return raw
        except Exception as e:
            print(f"Gesture sensor: proximity read error: {e}")
            return 0

    def read_proximity_norm(self) -> float:
        """Read normalized proximity value between 0.0 (far) and 1.0 (near)."""
        raw = self.read_proximity_raw()
        return float(raw / 255.0)

    def is_hand_present(self) -> bool:
        """Return True when raw proximity value exceeds the trigger threshold (> 50)."""
        if self.sim_mode:
            return False

        raw = self.read_proximity_raw()
        return raw > self.trigger_threshold

    def wait_for_hand_presence(
        self, timeout: Optional[float] = 30.0, hold_time: float = 0.2
    ) -> bool:
        """Block until raw proximity value > threshold is detected."""
        if self.sim_mode:
            return False

        start = time.time()
        interval = 0.05
        consecutive = 0
        required = max(1, int(hold_time / interval))

        while True:
            if timeout is not None and time.time() - start > timeout:
                return False

            if self.is_hand_present():
                consecutive += 1
                if consecutive >= required:
                    return True
            else:
                consecutive = 0

            time.sleep(interval)

    def is_gesture_available(self) -> bool:
        """
        Check if a gesture is available to read.

        Returns:
            False (gesture detection not implemented, only proximity)
        """
        return False

    def read_gesture(self) -> GestureType:
        """
        Read and decode gesture.

        Returns:
            "none" (gesture detection not implemented, only proximity)
        """
        return "none"

    def wait_for_gesture(self, timeout: float = 30.0) -> GestureType:
        """
        Wait for a gesture to occur.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            "none" (gesture detection not implemented, only proximity)
        """
        return "none"

    def wait_for_start_gesture(
        self,
        start_gestures: Optional[list[GestureType]] = None,
        timeout: float = 30.0,
        hold_time: float = 0.2,
    ) -> bool:
        """
        Wait for a proximity trigger to start the robot.

        Args:
            start_gestures: Deprecated; retained for API compatibility
            timeout: Maximum time to wait
            hold_time: Minimum duration the hand must remain present

        Returns:
            True if proximity trigger detected, False if timeout
        """
        print("Waiting for hand presence to start...")
        print("Hold your hand over the sensor to begin")

        triggered = self.wait_for_hand_presence(timeout=timeout, hold_time=hold_time)

        if triggered:
            print("✓ Start trigger detected (proximity)")
            return True

        print("Timeout waiting for proximity trigger")
        return False

    def cleanup(self):
        """Clean up resources."""
        # Adafruit library handles cleanup automatically
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
