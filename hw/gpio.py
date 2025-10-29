"""
GPIO utilities for Raspberry Pi.

Provides a thin abstraction over RPi.GPIO or lgpio.
"""

import sys
from typing import Literal

# Try to import GPIO library
try:
    import RPi.GPIO as GPIO

    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("Warning: RPi.GPIO not available. Running in simulation mode.")


class GPIOManager:
    """Manages GPIO initialization and cleanup."""

    def __init__(self):
        self.initialized = False
        self.sim_mode = not GPIO_AVAILABLE

    def setup(self):
        """Initialize GPIO in BCM mode."""
        if self.sim_mode:
            print("[SIM] GPIO setup")
            self.initialized = True
            return

        if not self.initialized:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            self.initialized = True

    def cleanup(self):
        """Clean up GPIO resources."""
        if self.sim_mode:
            print("[SIM] GPIO cleanup")
            return

        if self.initialized:
            GPIO.cleanup()
            self.initialized = False

    def setup_output(self, pin: int, initial: int = 0):
        """Configure pin as output."""
        if self.sim_mode:
            print(f"[SIM] Setup pin {pin} as OUTPUT, initial={initial}")
            return

        GPIO.setup(pin, GPIO.OUT, initial=initial)

    def setup_input(self, pin: int, pull_up_down: int | None = None):
        """Configure pin as input."""
        if self.sim_mode:
            print(f"[SIM] Setup pin {pin} as INPUT")
            return

        if pull_up_down is not None:
            GPIO.setup(pin, GPIO.IN, pull_up_down=pull_up_down)
        else:
            GPIO.setup(pin, GPIO.IN)

    def output(self, pin: int, value: int):
        """Set output pin value."""
        if self.sim_mode:
            return

        GPIO.output(pin, value)

    def input(self, pin: int) -> int:
        """Read input pin value."""
        if self.sim_mode:
            return 0

        return GPIO.input(pin)

    def pwm(self, pin: int, frequency: float):
        """Create PWM object for pin."""
        if self.sim_mode:
            return SimPWM(pin, frequency)

        return GPIO.PWM(pin, frequency)


class SimPWM:
    """Simulated PWM for testing without hardware."""

    def __init__(self, pin: int, frequency: float):
        self.pin = pin
        self.frequency = frequency
        self.duty_cycle = 0.0
        self.running = False

    def start(self, duty_cycle: float):
        """Start PWM with given duty cycle."""
        self.duty_cycle = duty_cycle
        self.running = True
        print(f"[SIM] PWM pin {self.pin} start at {duty_cycle}%")

    def ChangeDutyCycle(self, duty_cycle: float):
        """Change duty cycle."""
        self.duty_cycle = duty_cycle

    def ChangeFrequency(self, frequency: float):
        """Change frequency."""
        self.frequency = frequency

    def stop(self):
        """Stop PWM."""
        self.running = False
        print(f"[SIM] PWM pin {self.pin} stop")


# Global GPIO manager instance
gpio_manager = GPIOManager()
