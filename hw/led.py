"""
LED control for visual feedback.

Provides blink patterns for signaling robot states.
"""

import time
import threading

try:  # Support both package and script execution
    from ..config import PINS  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - fallback when run as script
    from config import PINS

from .gpio import gpio_manager


class LED:
    """LED controller for visual feedback."""

    def __init__(self):
        """Initialize LED controller."""
        self.led_pin = PINS.LED
        self.is_active = False
        self.blink_thread = None
        self._stop_blink = False

        # Setup GPIO
        gpio_manager.setup()
        gpio_manager.setup_output(self.led_pin, 0)

    def on(self):
        """Turn LED on."""
        if self.is_active:
            return
        gpio_manager.output(self.led_pin, 1)
        self.is_active = True

    def off(self):
        """Turn LED off."""
        if not self.is_active:
            return
        gpio_manager.output(self.led_pin, 0)
        self.is_active = False

    def blink(self, duration: float = 0.1):
        """
        Produce a single blink.

        Args:
            duration: Blink duration in seconds
        """
        self.on()
        time.sleep(duration)
        self.off()

    def blink_pattern(
        self,
        count: int = 1,
        duration: float = 0.1,
        pause: float = 0.1,
    ):
        """
        Produce a pattern of blinks.

        Args:
            count: Number of blinks
            duration: Duration of each blink in seconds
            pause: Pause between blinks in seconds
        """
        for i in range(count):
            self.blink(duration)
            if i < count - 1:  # Don't pause after the last blink
                time.sleep(pause)

    def blink_start(self):
        """
        Play startup blink pattern (2 short blinks).
        Signals that the robot is starting a cleaning cycle.
        """
        self.blink_pattern(count=2, duration=0.1, pause=0.1)

    def blink_finish(self):
        """
        Play finish blink pattern (3 blinks).
        Signals completion.
        """
        self.blink_pattern(count=3, duration=0.15, pause=0.1)

    def blink_error(self):
        """
        Play error blink pattern (3 rapid blinks).
        Signals an error condition.
        """
        self.blink_pattern(count=3, duration=0.05, pause=0.05)

    def blink_warning(self):
        """
        Play warning blink pattern (single long blink).
        Signals a warning condition.
        """
        self.blink(duration=0.5)

    def blink_async(
        self,
        count: int = 1,
        duration: float = 0.1,
        pause: float = 0.1,
    ):
        """
        Produce blinks asynchronously in a background thread.

        Args:
            count: Number of blinks
            duration: Duration of each blink in seconds
            pause: Pause between blinks in seconds
        """
        if self.blink_thread and self.blink_thread.is_alive():
            return  # Already blinking

        def blink_worker():
            self.blink_pattern(count, duration, pause)

        self.blink_thread = threading.Thread(target=blink_worker, daemon=True)
        self.blink_thread.start()

    def cleanup(self):
        """Clean up resources."""
        self._stop_blink = True
        self.off()
