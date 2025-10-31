"""
Stepper motor driver for TMC2209 via STEP/DIR control.

Converts velocity commands to wheel velocities, generates step pulses,
and provides odometry feedback.
"""

import time
import threading
import numpy as np

try:  # Support both package and script execution
    from ..config import PINS, GEOM, LIMS  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - fallback when run as script
    from config import PINS, GEOM, LIMS

from .gpio import gpio_manager


class StepperDrive:
    """Differential drive stepper motor controller."""

    _PULSE_WIDTH_S = 2.0e-5  # 20 microseconds high time per STEP pulse
    _LOOP_SLEEP_S = 1.0e-4  # 100 microseconds background loop sleep

    def __init__(self):
        self.left_step_pin = PINS.LEFT_STEP
        self.left_dir_pin = PINS.LEFT_DIR
        self.right_step_pin = PINS.RIGHT_STEP
        self.right_dir_pin = PINS.RIGHT_DIR
        self.left_enable_pin = getattr(PINS, "LEFT_ENABLE", None)
        self.right_enable_pin = getattr(PINS, "RIGHT_ENABLE", None)

        # Velocity state
        self.v_cmd = 0.0  # commanded linear velocity (m/s)
        self.omega_cmd = 0.0  # commanded angular velocity (rad/s)
        self.vL_target = 0.0  # target left wheel velocity
        self.vR_target = 0.0  # target right wheel velocity
        self.vL_current = 0.0  # current left wheel velocity (with limits)
        self.vR_current = 0.0  # current right wheel velocity (with limits)

        # Odometry counters
        self.steps_left = 0
        self.steps_right = 0
        self.last_steps_left = 0
        self.last_steps_right = 0

        # Pulse generation
        self.running = False
        self.pulse_thread = None
        self.lock = threading.Lock()
        # Setup GPIO
        gpio_manager.setup()
        gpio_manager.setup_output(self.left_step_pin, 0)
        gpio_manager.setup_output(self.left_dir_pin, 0)
        gpio_manager.setup_output(self.right_step_pin, 0)
        gpio_manager.setup_output(self.right_dir_pin, 0)
        if self.left_enable_pin is not None:
            gpio_manager.setup_output(self.left_enable_pin, 1)
        if self.right_enable_pin is not None:
            gpio_manager.setup_output(self.right_enable_pin, 1)

        # Start background pulse generation immediately so commands take effect
        self.start_pulse_generation()

    def command(self, v: float, omega: float):
        """Set velocity command (m/s, rad/s)."""
        # Clamp to limits
        v = np.clip(v, -LIMS.V_REV_MAX, LIMS.V_MAX)
        omega = np.clip(omega, -LIMS.OMEGA_MAX, LIMS.OMEGA_MAX)

        with self.lock:
            self.v_cmd = v
            self.omega_cmd = omega

            # Convert to wheel velocities
            self.vL_target = v - 0.5 * omega * GEOM.WHEEL_BASE
            self.vR_target = v + 0.5 * omega * GEOM.WHEEL_BASE

    def update(self, dt: float) -> tuple[float, float]:
        """
        Update velocity with acceleration limits.
        Returns (dSL, dSR) - distance traveled by each wheel.
        """
        with self.lock:
            # Apply acceleration limits
            max_dv = LIMS.A_MAX * dt

            dv_left = self.vL_target - self.vL_current
            dv_right = self.vR_target - self.vR_current

            if abs(dv_left) > max_dv:
                dv_left = np.sign(dv_left) * max_dv
            if abs(dv_right) > max_dv:
                dv_right = np.sign(dv_right) * max_dv

            self.vL_current += dv_left
            self.vR_current += dv_right

            # Set direction pins
            gpio_manager.output(self.left_dir_pin, 1 if self.vL_current >= 0 else 0)
            gpio_manager.output(self.right_dir_pin, 1 if self.vR_current >= 0 else 0)

            # Calculate distances
            dSL = self.vL_current * dt
            dSR = self.vR_current * dt

            # Update step counters
            steps_L = int(abs(dSL) * GEOM.STEPS_PER_M)
            steps_R = int(abs(dSR) * GEOM.STEPS_PER_M)

            if self.vL_current >= 0:
                self.steps_left += steps_L
            else:
                self.steps_left -= steps_L

            if self.vR_current >= 0:
                self.steps_right += steps_R
            else:
                self.steps_right -= steps_R

            return dSL, dSR

    def read_odometry(self) -> tuple[float, float]:
        """
        Read odometry since last call.
        Returns (dSL, dSR) in meters.
        """
        with self.lock:
            dSteps_L = self.steps_left - self.last_steps_left
            dSteps_R = self.steps_right - self.last_steps_right

            self.last_steps_left = self.steps_left
            self.last_steps_right = self.steps_right

            dSL = dSteps_L / GEOM.STEPS_PER_M
            dSR = dSteps_R / GEOM.STEPS_PER_M

            return dSL, dSR

    def start_pulse_generation(self):
        """Start background thread for step pulse generation."""
        if self.running:
            return

        self.enable_drivers()
        self.running = True
        self.pulse_thread = threading.Thread(target=self._pulse_loop, daemon=True)
        self.pulse_thread.start()

    def stop_pulse_generation(self):
        """Stop background pulse generation."""
        self.running = False
        if self.pulse_thread:
            self.pulse_thread.join(timeout=1.0)

    def _pulse_loop(self):
        """Background loop for generating step pulses."""
        next_left = time.perf_counter()
        next_right = next_left

        while self.running:
            with self.lock:
                vL = self.vL_current
                vR = self.vR_current

            rate_L = abs(vL) * GEOM.STEPS_PER_M
            rate_R = abs(vR) * GEOM.STEPS_PER_M

            now = time.perf_counter()

            if rate_L > 0.0:
                period_L = max(1.0 / rate_L, self._PULSE_WIDTH_S * 2.0)
                while now >= next_left and self.running:
                    self._emit_step_pulse(self.left_step_pin)
                    next_left += period_L
                    if next_left - now > period_L:
                        break
                    now = time.perf_counter()
            else:
                next_left = now

            now = time.perf_counter()
            if rate_R > 0.0:
                period_R = max(1.0 / rate_R, self._PULSE_WIDTH_S * 2.0)
                while now >= next_right and self.running:
                    self._emit_step_pulse(self.right_step_pin)
                    next_right += period_R
                    if next_right - now > period_R:
                        break
                    now = time.perf_counter()
            else:
                next_right = now

            time.sleep(self._LOOP_SLEEP_S)

    def stop(self):
        """Emergency stop - zero all velocities."""
        self.command(0.0, 0.0)
        with self.lock:
            self.vL_current = 0.0
            self.vR_current = 0.0

    def _emit_step_pulse(self, pin: int):
        """Toggle the given step pin high/low with the configured pulse width."""
        gpio_manager.output(pin, 1)
        end_time = time.perf_counter() + self._PULSE_WIDTH_S
        while time.perf_counter() < end_time:
            pass
        gpio_manager.output(pin, 0)

    def enable_drivers(self):
        """Assert enable lines (active low) to energize both drivers."""
        if self.left_enable_pin is not None:
            gpio_manager.output(self.left_enable_pin, 0)
        if self.right_enable_pin is not None:
            gpio_manager.output(self.right_enable_pin, 0)

    def disable_drivers(self):
        """De-assert enable lines (active low) to release both drivers."""
        if self.left_enable_pin is not None:
            gpio_manager.output(self.left_enable_pin, 1)
        if self.right_enable_pin is not None:
            gpio_manager.output(self.right_enable_pin, 1)
