"""
Buzzer/Beeper control for audio feedback.

Provides beep patterns for signaling robot states.
Supports both Passive (PWM-driven) and Active (DC-driven) buzzers.
"""

import time
import threading

try:  # Support both package and script execution
    from ..config import PINS  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - fallback when run as script
    from config import PINS

from .gpio import gpio_manager


class Buzzer:
    """Buzzer controller for audio feedback."""

    def __init__(self, frequency: float = 2000.0, active_buzzer: bool = False):
        """
        Initialize buzzer controller.

        Args:
            frequency: PWM frequency in Hz (default 2000 Hz for piezo buzzer).
                       Ignored if active_buzzer is True.
            active_buzzer: Set to True if using an active buzzer (internal oscillator)
                           that just needs High/Low signal.
                           Set to False (default) for passive piezo elements needing PWM.
        """
        self.buzzer_pin = PINS.BUZZER
        self.frequency = frequency
        self.active_buzzer = active_buzzer
        self.pwm = None
        self.is_active = False
        self.beep_thread = None
        
        # Setup GPIO
        gpio_manager.setup()
        gpio_manager.setup_output(self.buzzer_pin, 0)
        
        if not self.active_buzzer:
            # Only setup PWM for passive buzzers
            self.pwm = gpio_manager.pwm(self.buzzer_pin, self.frequency)

    def on(self, duty: float = 0.5):
        """
        Turn buzzer on continuously.

        Args:
            duty: Duty cycle 0.0-1.0 (default 0.5 for square wave).
                  Ignored for active buzzers (always 100% when on).
        """
        if self.is_active:
            if not self.active_buzzer and self.pwm:
                # Update duty cycle for running PWM
                duty = max(0.0, min(1.0, duty))
                self.pwm.ChangeDutyCycle(duty * 100.0)
            return

        # Turn on
        if self.active_buzzer:
            gpio_manager.output(self.buzzer_pin, 1)
        else:
            if self.pwm:
                duty = max(0.0, min(1.0, duty))
                self.pwm.start(duty * 100.0)
        
        self.is_active = True

    def off(self):
        """Turn buzzer off."""
        if not self.is_active:
            return

        if self.active_buzzer:
            gpio_manager.output(self.buzzer_pin, 0)
        else:
            if self.pwm:
                self.pwm.stop()
        
        self.is_active = False

    def beep(self, duration: float = 0.1, duty: float = 0.5):
        """
        Produce a single beep.

        Args:
            duration: Beep duration in seconds
            duty: Duty cycle 0.0-1.0 (PWM only)
        """
        self.on(duty)
        time.sleep(duration)
        self.off()

    def beep_pattern(
        self,
        count: int = 1,
        duration: float = 0.1,
        pause: float = 0.1,
        duty: float = 0.5,
    ):
        """
        Produce a pattern of beeps.

        Args:
            count: Number of beeps
            duration: Duration of each beep in seconds
            pause: Pause between beeps in seconds
            duty: Duty cycle 0.0-1.0
        """
        for i in range(count):
            self.beep(duration, duty)
            if i < count - 1:  # Don't pause after the last beep
                time.sleep(pause)

    def beep_start(self):
        """
        Play startup beep pattern (2 short beeps).
        Signals that the robot is starting a cleaning cycle.
        """
        self.beep_pattern(count=2, duration=0.1, pause=0.1)

    def beep_finish(self):
        """
        Play finish beep pattern.
        
        For passive buzzers: 3 ascending beeps.
        For active buzzers: 3 simple beeps (cannot change pitch).
        """
        if self.active_buzzer or not self.pwm:
            # Simple pattern for active buzzer
            self.beep_pattern(count=3, duration=0.15, pause=0.1)
            return

        # Ascending tones for passive buzzer
        freqs = [2000, 2500, 3000]
        for freq in freqs:
            self.pwm.ChangeFrequency(freq)
            self.beep(duration=0.15)
            time.sleep(0.1)
        
        # Reset to default frequency
        self.pwm.ChangeFrequency(self.frequency)

    def beep_error(self):
        """
        Play error beep pattern (3 rapid beeps).
        Signals an error condition.
        """
        self.beep_pattern(count=3, duration=0.05, pause=0.05)

    def beep_warning(self):
        """
        Play warning beep pattern (single long beep).
        Signals a warning condition.
        """
        self.beep(duration=0.5)

    def beep_async(
        self,
        count: int = 1,
        duration: float = 0.1,
        pause: float = 0.1,
        duty: float = 0.5,
    ):
        """
        Produce beeps asynchronously in a background thread.

        Args:
            count: Number of beeps
            duration: Duration of each beep in seconds
            pause: Pause between beeps in seconds
            duty: Duty cycle 0.0-1.0
        """
        if self.beep_thread and self.beep_thread.is_alive():
            return  # Already beeping

        def beep_worker():
            self.beep_pattern(count, duration, pause, duty)

        self.beep_thread = threading.Thread(target=beep_worker, daemon=True)
        self.beep_thread.start()

    def cleanup(self):
        """Clean up resources."""
        self.off()
