"""
Stepper motor driver using the Adafruit DC & Stepper Motor HAT (MotorKit API).

Converts velocity commands to wheel velocities, schedules stepper moves,
and provides odometry feedback.
"""

from __future__ import annotations

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

    _MIN_STEP_PERIOD_S = 4.0e-5  # minimum time between steps (40 µs)
    _LOOP_SLEEP_S = 1.0e-4  # 100 microseconds background loop sleep

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

            rate_L = abs(vL) * GEOM.STEPS_PER_M
            rate_R = abs(vR) * GEOM.STEPS_PER_M

            now = time.perf_counter()

            if rate_L > 0.0:
                period_L = max(1.0 / rate_L, self._MIN_STEP_PERIOD_S)
                direction_L = (
                    self._stepper_module.FORWARD
                    if vL >= 0
                    else self._stepper_module.BACKWARD
                )
                while now >= next_left and self.running:
                    self.left_stepper.onestep(
                        direction=direction_L, style=self.step_style
                    )
                    self._left_engaged = True
                    next_left += period_L
                    if next_left - now > period_L:
                        break
                    now = time.perf_counter()
            else:
                next_left = now
                if self.release_on_idle and self._left_engaged:
                    self.left_stepper.release()
                    self._left_engaged = False

            now = time.perf_counter()
            if rate_R > 0.0:
                period_R = max(1.0 / rate_R, self._MIN_STEP_PERIOD_S)
                direction_R = (
                    self._stepper_module.FORWARD
                    if vR >= 0
                    else self._stepper_module.BACKWARD
                )
                while now >= next_right and self.running:
                    self.right_stepper.onestep(
                        direction=direction_R, style=self.step_style
                    )
                    self._right_engaged = True
                    next_right += period_R
                    if next_right - now > period_R:
                        break
                    now = time.perf_counter()
            else:
                next_right = now
                if self.release_on_idle and self._right_engaged:
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
