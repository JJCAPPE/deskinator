"""
Stepper motor driver using the Adafruit DC & Stepper Motor HAT (MotorKit API).

Converts velocity commands to wheel velocities, schedules stepper moves,
and provides odometry feedback.
"""

from __future__ import annotations

import logging
import threading
import time
from importlib import import_module
from typing import Optional

import numpy as np

try:  # Support both package and script execution
    from ..config import GEOM, LIMS  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - fallback when run as script
    from config import GEOM, LIMS


class StepperDrive:
    """Differential drive stepper motor controller."""

    _MIN_STEP_PERIOD_S = 0.003  # practical floor given I2C latency (~333 steps/s)
    _LOOP_SLEEP_S = 0.001  # 1 ms background loop sleep to reduce CPU churn
    _I2C_ERROR_SLEEP_S = 0.01  # back-off when the HAT/I2C hiccups

    def __init__(
        self,
        kit: Optional[object] = None,
        *,
        step_style: Optional[int] = None,
        release_on_idle: bool = False,
    ):
        try:
            stepper_module = import_module("adafruit_motor.stepper")
        except ImportError as exc:  # pragma: no cover - hardware dependency
            raise ImportError(
                "Adafruit MotorKit library is required. Install with "
                "'pip3 install adafruit-circuitpython-motorkit'."
            ) from exc

        self.logger = logging.getLogger(__name__)

        if kit is None:
            try:
                motorhat_module = import_module("adafruit_motorkit")
            except ImportError as exc:  # pragma: no cover - hardware dependency
                raise ImportError(
                    "Adafruit MotorKit library is required. Install with "
                    "'pip3 install adafruit-circuitpython-motorkit'."
                ) from exc
            self.kit = motorhat_module.MotorKit()
        else:
            self.kit = kit

        self.left_stepper = getattr(self.kit, "stepper1", None)
        self.right_stepper = getattr(self.kit, "stepper2", None)
        if self.left_stepper is None or self.right_stepper is None:
            raise RuntimeError(
                "MotorKit did not report both stepper1 and stepper2. "
                "Ensure the DC & Stepper Motor HAT is connected."
            )

        self._stepper_module = stepper_module
        self.step_style = (
            step_style if step_style is not None else self._stepper_module.INTERLEAVE
        )
        self.release_on_idle = release_on_idle
        self._left_engaged = False
        self._right_engaged = False

        # Determine effective steps-per-meter considering the selected step style.
        style_multiplier = {
            self._stepper_module.SINGLE: 1.0,
            self._stepper_module.DOUBLE: 1.0,
            self._stepper_module.INTERLEAVE: 2.0,
        }
        if hasattr(self._stepper_module, "MICROSTEP"):
            style_multiplier[self._stepper_module.MICROSTEP] = 8.0

        self._step_style_multiplier = style_multiplier.get(self.step_style, 1.0)
        self.steps_per_m_effective = GEOM.STEPS_PER_M * self._step_style_multiplier
        if self.steps_per_m_effective <= 0:
            raise ValueError("GEOM.STEPS_PER_M must be positive")

        self._max_step_rate = 1.0 / self._MIN_STEP_PERIOD_S
        self._max_wheel_speed = self._max_step_rate / self.steps_per_m_effective

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

        # Motion scheduling
        self.running = False
        self.pulse_thread = None
        self.lock = threading.Lock()

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

            max_wheel = self._max_wheel_speed
            self.vL_target = np.clip(vL_target, -max_wheel, max_wheel)
            self.vR_target = np.clip(vR_target, -max_wheel, max_wheel)

            # Store the achievable chassis velocities corresponding to the
            # clipped wheel targets. This keeps downstream consumers aware of
            # any saturation we applied.
            self.v_cmd = 0.5 * (self.vL_target + self.vR_target)
            self.omega_cmd = (self.vR_target - self.vL_target) / GEOM.WHEEL_BASE

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
            dSL = self.vL_current * dt
            dSR = self.vR_current * dt

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

            dSL = dSteps_L / self.steps_per_m_effective
            dSR = dSteps_R / self.steps_per_m_effective

            return dSL, dSR

    @property
    def step_style_name(self) -> str:
        """Human-readable name for the active step style."""
        mapping = {
            getattr(self._stepper_module, "SINGLE", None): "SINGLE",
            getattr(self._stepper_module, "DOUBLE", None): "DOUBLE",
            getattr(self._stepper_module, "INTERLEAVE", None): "INTERLEAVE",
            getattr(self._stepper_module, "MICROSTEP", None): "MICROSTEP",
        }
        return mapping.get(self.step_style, f"STYLE_{self.step_style}")

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
        """Background loop for scheduling stepper moves via MotorKit."""
        next_left = time.perf_counter()
        next_right = next_left

        while self.running:
            with self.lock:
                vL = self.vL_current
                vR = self.vR_current

            rate_L = abs(vL) * self.steps_per_m_effective
            rate_R = abs(vR) * self.steps_per_m_effective

            now = time.perf_counter()

            if rate_L > 0.0:
                period_L = max(1.0 / rate_L, self._MIN_STEP_PERIOD_S)
                direction_L = (
                    self._stepper_module.BACKWARD
                    if vL >= 0
                    else self._stepper_module.FORWARD
                )
                step_delta_L = 1 if direction_L == self._stepper_module.FORWARD else -1
                while now >= next_left and self.running:
                    try:
                        self.left_stepper.onestep(
                            direction=direction_L, style=self.step_style
                        )
                        with self.lock:
                            self.steps_left += step_delta_L
                            self._left_engaged = True
                        next_left += period_L
                    except Exception as exc:  # pragma: no cover - hardware path
                        self.logger.warning(
                            "Left stepper onestep failed: %s", exc, exc_info=False
                        )
                        next_left = time.perf_counter() + self._I2C_ERROR_SLEEP_S
                        time.sleep(self._I2C_ERROR_SLEEP_S)
                        break
                    now = time.perf_counter()
                    if next_left - now > period_L:
                        break
            else:
                next_left = now
                if self.release_on_idle:
                    with self.lock:
                        if self._left_engaged:
                            self.left_stepper.release()
                            self._left_engaged = False

            now = time.perf_counter()
            if rate_R > 0.0:
                period_R = max(1.0 / rate_R, self._MIN_STEP_PERIOD_S)
                direction_R = (
                    self._stepper_module.BACKWARD
                    if vR >= 0
                    else self._stepper_module.FORWARD
                )
                step_delta_R = 1 if direction_R == self._stepper_module.FORWARD else -1
                while now >= next_right and self.running:
                    try:
                        self.right_stepper.onestep(
                            direction=direction_R, style=self.step_style
                        )
                        with self.lock:
                            self.steps_right += step_delta_R
                            self._right_engaged = True
                        next_right += period_R
                    except Exception as exc:  # pragma: no cover - hardware path
                        self.logger.warning(
                            "Right stepper onestep failed: %s", exc, exc_info=False
                        )
                        next_right = time.perf_counter() + self._I2C_ERROR_SLEEP_S
                        time.sleep(self._I2C_ERROR_SLEEP_S)
                        break
                    now = time.perf_counter()
                    if next_right - now > period_R:
                        break
            else:
                next_right = now
                if self.release_on_idle:
                    with self.lock:
                        if self._right_engaged:
                            self.right_stepper.release()
                            self._right_engaged = False

            time.sleep(self._LOOP_SLEEP_S)

    def stop(self):
        """Emergency stop - zero all velocities."""
        self.command(0.0, 0.0)
        with self.lock:
            self.vL_current = 0.0
            self.vR_current = 0.0
        if self.release_on_idle:
            self.disable_drivers()

    def enable_drivers(self):
        """Ensure both stepper channels are ready to energize."""
        # MotorKit energizes coils on demand; no explicit action required.
        self._left_engaged = False
        self._right_engaged = False

    def disable_drivers(self):
        """Release both motors to save power."""
        self.left_stepper.release()
        self.right_stepper.release()
        self._left_engaged = False
        self._right_engaged = False
