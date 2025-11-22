"""
Stepper motor driver for A4988 stepper motor drivers.

Controls two A4988 drivers via GPIO pins for smooth differential drive motion.
Uses microstepping (1/16) for maximum smoothness and implements acceleration
ramping for jerk-free motion.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import numpy as np

try:  # Support both package and script execution
    from ..config import GEOM, LIMS, PINS  # type: ignore[import-not-found]
    from .gpio import gpio_manager
except ImportError:  # pragma: no cover - fallback when run as script
    from config import GEOM, LIMS, PINS
    from hw.gpio import gpio_manager


class StepperDrive:
    """Differential drive stepper motor controller using A4988 drivers."""

    # Timing constants for A4988
    _MIN_STEP_PULSE_WIDTH_US = 1  # Minimum step pulse width (microseconds)
    _MIN_STEP_PULSE_WIDTH_S = _MIN_STEP_PULSE_WIDTH_US / 1e6
    _STEP_PULSE_WIDTH_S = 10e-6  # 10 microseconds - safe pulse width
    _LOOP_SLEEP_S = 0.0001  # 100 µs background loop sleep for precise timing
    _MIN_STEP_PERIOD_S = 0.0001  # Minimum step period (10 kHz max = 0.1 ms)

    @staticmethod
    def _busy_wait(duration_s: float):
        """Busy-wait for precise short delays."""
        if duration_s <= 0:
            return
        end_time = time.perf_counter() + duration_s
        while time.perf_counter() < end_time:
            pass

    # Microstepping: A4988 default is 1/16 microstep mode
    # This means 16 microsteps per full step
    MICROSTEPS_PER_STEP = 16

    def __init__(
        self,
        *,
        microsteps_per_step: int = 16,
        release_on_idle: bool = False,
    ):
        """
        Initialize A4988 stepper motor driver.

        Args:
            microsteps_per_step: Number of microsteps per full step (default 16 for 1/16 microstepping)
            release_on_idle: If True, disable drivers when motors are idle
        """
        self.logger = logging.getLogger(__name__)
        self.microsteps_per_step = microsteps_per_step
        self.release_on_idle = release_on_idle

        # Initialize GPIO
        gpio_manager.setup()

        # Setup GPIO pins for left motor
        gpio_manager.setup_output(PINS.LEFT_STEP, initial=0)
        gpio_manager.setup_output(PINS.LEFT_DIR, initial=0)
        gpio_manager.setup_output(PINS.LEFT_ENABLE, initial=1)  # Start disabled (active low)

        # Setup GPIO pins for right motor
        gpio_manager.setup_output(PINS.RIGHT_STEP, initial=0)
        gpio_manager.setup_output(PINS.RIGHT_DIR, initial=0)
        gpio_manager.setup_output(PINS.RIGHT_ENABLE, initial=1)  # Start disabled (active low)

        # Calculate effective steps-per-meter considering microstepping
        # GEOM.STEPS_PER_M is full steps per meter, multiply by microsteps
        self.steps_per_m_effective = GEOM.STEPS_PER_M * self.microsteps_per_step
        if self.steps_per_m_effective <= 0:
            raise ValueError("GEOM.STEPS_PER_M must be positive")

        # Maximum step rate (steps per second)
        self._max_step_rate = 1.0 / self._MIN_STEP_PERIOD_S
        self._max_wheel_speed = self._max_step_rate / self.steps_per_m_effective

        # Velocity state
        self.v_cmd = 0.0  # commanded linear velocity (m/s)
        self.omega_cmd = 0.0  # commanded angular velocity (rad/s)
        self.vL_target = 0.0  # target left wheel velocity
        self.vR_target = 0.0  # target right wheel velocity
        self.vL_current = 0.0  # current left wheel velocity (with limits)
        self.vR_current = 0.0  # current right wheel velocity (with limits)

        # Odometry counters (in microsteps)
        self.steps_left = 0
        self.steps_right = 0
        self.last_steps_left = 0
        self.last_steps_right = 0

        # Motion scheduling
        self.running = False
        self.pulse_thread = None
        self.lock = threading.Lock()

        # Motor enable state
        self._left_enabled = False
        self._right_enabled = False

        # Start background pulse generation immediately so commands take effect
        self.start_pulse_generation()

    def command(self, v: float, omega: float):
        """Set velocity command (m/s, rad/s)."""
        # Clamp to limits
        v = np.clip(v, -LIMS.V_REV_MAX, LIMS.V_MAX)
        omega = np.clip(omega, -LIMS.OMEGA_MAX, LIMS.OMEGA_MAX)

        with self.lock:
            # Convert to wheel velocities
            vL_target = v - 0.5 * omega * GEOM.WHEEL_BASE
            vR_target = v + 0.5 * omega * GEOM.WHEEL_BASE

            # Invert right wheel direction: RIGHT pins control physical left wheel
            # which has reversed direction
            vR_target = -vR_target

            max_wheel = self._max_wheel_speed
            self.vL_target = np.clip(vL_target, -max_wheel, max_wheel)
            self.vR_target = np.clip(vR_target, -max_wheel, max_wheel)

            # Store the achievable chassis velocities corresponding to the
            # clipped wheel targets. After pin swap and direction inversion:
            # - LEFT pins (vL_target) control physical right wheel
            # - RIGHT pins (vR_target, inverted) control physical left wheel
            # Physical left = -vR_target, Physical right = vL_target
            physical_left = -self.vR_target
            physical_right = self.vL_target
            self.v_cmd = 0.5 * (physical_left + physical_right)
            self.omega_cmd = (physical_right - physical_left) / GEOM.WHEEL_BASE

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

            max_wheel = self._max_wheel_speed
            self.vL_current = np.clip(self.vL_current, -max_wheel, max_wheel)
            self.vR_current = np.clip(self.vR_current, -max_wheel, max_wheel)

            # Calculate distances using the instantaneous velocity estimate.
            # After pin swap and direction inversion, swap and negate to get
            # physical wheel distances
            dSL_physical = -self.vR_current * dt  # Physical left wheel
            dSR_physical = self.vL_current * dt    # Physical right wheel

            return dSL_physical, dSR_physical

    def read_odometry(self) -> tuple[float, float]:
        """
        Read odometry since last call.
        Returns (dSL, dSR) in meters.
        
        Note: After pin swap, LEFT pins control physical right wheel,
        and RIGHT pins control physical left wheel. We swap and negate
        to return correct values for physical left/right wheels.
        """
        with self.lock:
            dSteps_L = self.steps_left - self.last_steps_left
            dSteps_R = self.steps_right - self.last_steps_right

            self.last_steps_left = self.steps_left
            self.last_steps_right = self.steps_right

            # After pin swap: LEFT pins = physical right, RIGHT pins = physical left
            # After direction inversion: RIGHT pins have inverted step counting
            # So we swap and negate RIGHT to get physical left wheel distance
            dSL_physical = -dSteps_R / self.steps_per_m_effective  # Physical left wheel
            dSR_physical = dSteps_L / self.steps_per_m_effective   # Physical right wheel

            return dSL_physical, dSR_physical

    @property
    def step_style_name(self) -> str:
        """Human-readable name for the active step style."""
        return f"MICROSTEP_{self.microsteps_per_step}"

    @property
    def steps_per_meter(self) -> float:
        """Effective number of motor steps per meter of travel."""
        return self.steps_per_m_effective

    @property
    def max_wheel_speed(self) -> float:
        """Maximum achievable wheel surface speed in m/s."""
        return self._max_wheel_speed

    def start_pulse_generation(self):
        """Start background thread for step scheduling."""
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
        """Background loop for generating step pulses via GPIO."""
        next_left = time.perf_counter()
        next_right = next_left

        while self.running:
            with self.lock:
                vL = self.vL_current
                vR = self.vR_current

            now = time.perf_counter()

            # Left motor pulse generation
            rate_L = abs(vL) * self.steps_per_m_effective
            if rate_L > 0.0:
                period_L = max(1.0 / rate_L, self._MIN_STEP_PERIOD_S)
                direction_L = 0 if vL >= 0 else 1

                # Set direction pin
                gpio_manager.output(PINS.LEFT_DIR, direction_L)

                # Generate step pulses
                while now >= next_left and self.running:
                    # Generate step pulse with precise timing
                    gpio_manager.output(PINS.LEFT_STEP, 1)
                    self._busy_wait(self._STEP_PULSE_WIDTH_S)
                    gpio_manager.output(PINS.LEFT_STEP, 0)

                    # Update odometry
                    step_delta = 1 if vL >= 0 else -1
                    with self.lock:
                        self.steps_left += step_delta

                    next_left += period_L
                    now = time.perf_counter()

                    # Don't accumulate too many steps at once
                    if next_left - now > period_L:
                        break
            else:
                next_left = now

            # Right motor pulse generation
            now = time.perf_counter()
            rate_R = abs(vR) * self.steps_per_m_effective
            if rate_R > 0.0:
                period_R = max(1.0 / rate_R, self._MIN_STEP_PERIOD_S)
                direction_R = 0 if vR >= 0 else 1

                # Set direction pin
                gpio_manager.output(PINS.RIGHT_DIR, direction_R)

                # Generate step pulses
                while now >= next_right and self.running:
                    # Generate step pulse with precise timing
                    gpio_manager.output(PINS.RIGHT_STEP, 1)
                    self._busy_wait(self._STEP_PULSE_WIDTH_S)
                    gpio_manager.output(PINS.RIGHT_STEP, 0)

                    # Update odometry
                    step_delta = 1 if vR >= 0 else -1
                    with self.lock:
                        self.steps_right += step_delta

                    next_right += period_R
                    now = time.perf_counter()

                    # Don't accumulate too many steps at once
                    if next_right - now > period_R:
                        break
            else:
                next_right = now

            # Small sleep to prevent CPU spinning
            time.sleep(self._LOOP_SLEEP_S)

            # Handle idle release if enabled
            if self.release_on_idle:
                with self.lock:
                    vL_check = self.vL_current
                    vR_check = self.vR_current
                
                rate_L_check = abs(vL_check) * self.steps_per_m_effective
                rate_R_check = abs(vR_check) * self.steps_per_m_effective
                
                if rate_L_check == 0.0 and self._left_enabled:
                    self._disable_left()
                elif rate_L_check > 0.0 and not self._left_enabled:
                    self._enable_left()

                if rate_R_check == 0.0 and self._right_enabled:
                    self._disable_right()
                elif rate_R_check > 0.0 and not self._right_enabled:
                    self._enable_right()

    def _enable_left(self):
        """Enable left motor driver."""
        gpio_manager.output(PINS.LEFT_ENABLE, 0)  # Active low
        self._left_enabled = True

    def _disable_left(self):
        """Disable left motor driver."""
        gpio_manager.output(PINS.LEFT_ENABLE, 1)  # Active low
        self._left_enabled = False

    def _enable_right(self):
        """Enable right motor driver."""
        gpio_manager.output(PINS.RIGHT_ENABLE, 0)  # Active low
        self._right_enabled = True

    def _disable_right(self):
        """Disable right motor driver."""
        gpio_manager.output(PINS.RIGHT_ENABLE, 1)  # Active low
        self._right_enabled = False

    def stop(self):
        """Emergency stop - zero all velocities."""
        self.command(0.0, 0.0)
        with self.lock:
            self.vL_current = 0.0
            self.vR_current = 0.0
        if self.release_on_idle:
            self.disable_drivers()

    def enable_drivers(self):
        """Enable both stepper drivers."""
        self._enable_left()
        self._enable_right()

    def disable_drivers(self):
        """Disable both stepper drivers to save power."""
        self._disable_left()
        self._disable_right()
