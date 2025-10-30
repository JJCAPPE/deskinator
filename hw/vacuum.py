"""
Vacuum fan control via PWM.

Controls a 12V BLDC vacuum fan with PWM duty cycle.
"""

try:  # Support both package and script execution
    from ..config import PINS  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - fallback when run as script
    from config import PINS

from .gpio import gpio_manager


class Vacuum:
    """Vacuum fan controller."""

    def __init__(self, pwm_frequency: float = 25000.0):
        """
        Initialize vacuum controller.

        Args:
            pwm_frequency: PWM frequency in Hz (default 25 kHz for BLDC)
        """
        self.pwm_pin = PINS.VACUUM_PWM
        self.frequency = pwm_frequency
        self.pwm = None
        self.duty = 0.0
        self.is_running = False

        # Setup GPIO
        gpio_manager.setup()
        gpio_manager.setup_output(self.pwm_pin, 0)
        self.pwm = gpio_manager.pwm(self.pwm_pin, self.frequency)

    def on(self, duty: float = 1.0):
        """
        Turn vacuum on with specified duty cycle.

        Args:
            duty: Duty cycle 0.0-1.0 (default 1.0 = full power)
        """
        duty = max(0.0, min(1.0, duty))  # Clamp to [0, 1]
        self.duty = duty

        if not self.is_running and duty > 0:
            self.pwm.start(duty * 100.0)  # Convert to percentage
            self.is_running = True
        elif self.is_running:
            self.pwm.ChangeDutyCycle(duty * 100.0)

    def off(self):
        """Turn vacuum off."""
        if self.is_running:
            self.pwm.stop()
            self.is_running = False
            self.duty = 0.0

    def set_duty(self, duty: float):
        """
        Set duty cycle.

        Args:
            duty: Duty cycle 0.0-1.0
        """
        self.on(duty)

    def cleanup(self):
        """Clean up resources."""
        self.off()
