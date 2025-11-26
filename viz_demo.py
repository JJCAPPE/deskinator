"""
Demo visualization script for Deskinator.

Demonstrates the visualization capabilities without requiring hardware.
Generates synthetic data matching the robot's data structures and displays
it using the existing Visualizer class.

Usage:
    python viz_demo.py [--error 0.0-1.0]
"""

import argparse
import time
import math
import random
import sys
import threading
from dataclasses import dataclass
from typing import Tuple, Optional

import numpy as np

# Import main to patch classes
import main
from config import GEOM, I2C, ALG, PINS
from slam.frames import wrap_angle

# --- Simulation State ---


@dataclass
class SimState:
    """Ground truth simulation state."""

    x: float
    y: float
    theta: float

    # Table definition (centered at 0,0)
    table_width: float = 2.0
    table_length: float = 3.0

    # Robot physical properties
    wheel_base: float = GEOM.WHEEL_BASE

    # Current velocity (ground truth)
    v: float = 0.0
    omega: float = 0.0

    def __post_init__(self):
        self.min_x = -self.table_width / 2.0
        self.max_x = self.table_width / 2.0
        self.min_y = -self.table_length / 2.0
        self.max_y = self.table_length / 2.0

    def update(
        self, v_left: float, v_right: float, dt: float, error_factor: float = 0.0
    ):
        """Update robot pose based on wheel velocities."""
        # Kinematics
        self.v = (v_left + v_right) / 2.0
        self.omega = (v_right - v_left) / self.wheel_base

        # Add process noise (slip/drift)
        if error_factor > 0:
            self.v *= 1.0 + random.gauss(0, 0.02 * error_factor)
            self.omega *= 1.0 + random.gauss(0, 0.02 * error_factor)

        # Simple integration
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt
        self.theta = wrap_angle(self.theta)

    def is_on_table(self, x: float, y: float) -> bool:
        return (self.min_x <= x <= self.max_x) and (self.min_y <= y <= self.max_y)


# Global simulation state
SIM_STATE: Optional[SimState] = None
ERROR_FACTOR: float = 0.0


# --- Mock Hardware Classes ---


class SimI2CBus:
    """Mock I2C bus."""

    def __init__(self, bus_number: int):
        self.bus_number = bus_number
        self.sim_mode = True

    def scan(self):
        return [0x39]  # Default APDS addr

    def write_byte_data(self, addr, reg, val):
        pass

    def read_byte_data(self, addr, reg):
        return 0

    def close(self):
        pass


class SimStepper:
    """Mock StepperDrive."""

    # Interface constants matching hw.stepper
    MICROSTEPS_PER_STEP = 16

    def __init__(self, microsteps_per_step=16, release_on_idle=False):
        self.microsteps_per_step = microsteps_per_step
        self.steps_per_m_effective = GEOM.STEPS_PER_M * self.microsteps_per_step

        self.vL_target = 0.0
        self.vR_target = 0.0
        self.vL_current = 0.0
        self.vR_current = 0.0

        self.steps_left = 0
        self.steps_right = 0
        self.last_steps_left = 0
        self.last_steps_right = 0

        self.lock = threading.Lock()

    def command(self, v: float, omega: float):
        """Set target velocities."""
        w = GEOM.WHEEL_BASE
        # Calculate wheel velocities
        # vL = v - w*omega/2
        # vR = v + w*omega/2
        # BUT: StepperDrive implementation accounts for pin swap/inversion logic.
        # Let's simulate the *physical* wheel velocities directly.
        # Physical Left = v - w*omega/2
        # Physical Right = v + w*omega/2

        physical_left = v - 0.5 * omega * w
        physical_right = v + 0.5 * omega * w

        self.vL_target = physical_left
        self.vR_target = physical_right

    def update(self, dt: float):
        """Update simulation state."""
        # Instant acceleration for simplicity (or we could limit it)
        self.vL_current = self.vL_target
        self.vR_current = self.vR_target

        # Update ground truth
        if SIM_STATE:
            SIM_STATE.update(self.vL_current, self.vR_current, dt, ERROR_FACTOR)

        # Update steps (Odometry simulation)
        # Calculate distance traveled
        dL = self.vL_current * dt
        dR = self.vR_current * dt

        # Add encoder noise/error
        if ERROR_FACTOR > 0:
            dL *= 1.0 + random.gauss(0, 0.01 * ERROR_FACTOR)
            dR *= 1.0 + random.gauss(0, 0.01 * ERROR_FACTOR)

        # Convert to steps
        self.steps_left += int(dL * self.steps_per_m_effective)
        self.steps_right += int(dR * self.steps_per_m_effective)

    def read_odometry(self) -> Tuple[float, float]:
        """Return distance traveled by each wheel since last call."""
        dSteps_L = self.steps_left - self.last_steps_left
        dSteps_R = self.steps_right - self.last_steps_right

        self.last_steps_left = self.steps_left
        self.last_steps_right = self.steps_right

        dSL = dSteps_L / self.steps_per_m_effective
        dSR = dSteps_R / self.steps_per_m_effective

        return dSL, dSR

    def stop(self):
        pass

    def stop_pulse_generation(self):
        pass

    def disable_drivers(self):
        pass


class SimAPDS9960:
    """Mock APDS9960."""

    def __init__(self, bus, address=0x39):
        self.bus = bus
        self.bus_wrapper = bus
        self.sim_mode = True

        # Determine sensor position based on bus
        # See main.py for bus mapping
        self.mount_pos = (0.0, 0.0)  # (x, y) in robot frame

        if bus.bus_number == I2C.LEFT_SENSOR_BUS:
            # Left sensor
            # Typically slightly forward and to the left
            # Using approx from GEOM if available or config
            lat = GEOM.SENSOR_LAT[0] if len(GEOM.SENSOR_LAT) > 0 else 0.1
            self.mount_pos = (GEOM.SENSOR_FWD, lat)
            self.is_gesture = False
        elif bus.bus_number == I2C.RIGHT_SENSOR_BUS:
            # Right sensor
            lat = GEOM.SENSOR_LAT[-1] if len(GEOM.SENSOR_LAT) > 0 else -0.1
            self.mount_pos = (GEOM.SENSOR_FWD, lat)
            self.is_gesture = False
        elif bus.bus_number == I2C.GESTURE_BUS:
            self.is_gesture = True
        else:
            self.is_gesture = False

    def init(self):
        pass

    def calibrate(self, on_table_samples=10, off_table_samples=10):
        pass

    def read_proximity_raw(self) -> int:
        if self.is_gesture:
            # Gesture sensor always triggered in sim to start
            return 255

        if not SIM_STATE:
            return 0

        # Calculate sensor world position
        rx, ry, rtheta = SIM_STATE.x, SIM_STATE.y, SIM_STATE.theta
        mx, my = self.mount_pos

        # Transform mount point to world
        sx = rx + mx * math.cos(rtheta) - my * math.sin(rtheta)
        sy = ry + mx * math.sin(rtheta) + my * math.cos(rtheta)

        # Check if on table
        if SIM_STATE.is_on_table(sx, sy):
            val = 200  # On table
        else:
            val = 10  # Off table

        # Add noise
        if ERROR_FACTOR > 0:
            val += int(random.gauss(0, 10 * ERROR_FACTOR))
            val = max(0, min(255, val))

        return val


class SimMPU6050:
    """Mock MPU6050."""

    def __init__(
        self, bus, address=0x68, auto_calibrate=True, calibration_duration=2.5
    ):
        self.sim_mode = True

    def bias_calibrate(self, duration=2.0):
        pass

    def read_yaw_rate(self) -> float:
        if not SIM_STATE:
            return 0.0

        rate = SIM_STATE.omega

        # Add noise
        if ERROR_FACTOR > 0:
            rate += random.gauss(0, 0.05 * ERROR_FACTOR)
            # Bias drift?
            rate += 0.01 * ERROR_FACTOR

        return rate


class SimVacuum:
    def on(self, duty=0.8):
        pass

    def off(self):
        pass


class SimBuzzer:
    def __init__(self, frequency=2000.0, active_buzzer=False):
        pass

    def beep_pattern(self, count, duration, pause):
        pass

    def beep_async(self, count, duration, pause):
        pass

    def cleanup(self):
        pass


# --- Patching ---


def apply_patches():
    """Patch main module to use mock hardware."""
    main.I2CBus = SimI2CBus
    main.StepperDrive = SimStepper
    main.APDS9960 = SimAPDS9960
    main.MPU6050 = SimMPU6050
    main.Vacuum = SimVacuum
    main.Buzzer = SimBuzzer

    # Patch gpio_manager to avoid hardware calls in other places
    class MockGPIO:
        def setup(self):
            pass

        def setup_output(self, pin, initial):
            pass

        def output(self, pin, val):
            pass

        def cleanup(self):
            pass

        def pwm(self, pin, freq):
            return None

    main.gpio_manager = MockGPIO()


def main_viz():
    global SIM_STATE, ERROR_FACTOR

    parser = argparse.ArgumentParser(description="Deskinator Simulation Demo")
    parser.add_argument(
        "--error", type=float, default=0.0, help="Error factor (0.0-1.0)"
    )
    args = parser.parse_args()

    ERROR_FACTOR = max(0.0, min(1.0, args.error))

    # Initialize Simulation State
    # Table: 2m x 3m (centered at 0,0 -> -1..1, -1.5..1.5)
    # Robot: Random position at least 20cm from edge
    margin = 0.2
    w = 2.0
    l = 3.0

    rx = random.uniform(-w / 2 + margin, w / 2 - margin)
    ry = random.uniform(-l / 2 + margin, l / 2 - margin)
    rtheta = random.uniform(-math.pi, math.pi)

    SIM_STATE = SimState(x=rx, y=ry, theta=rtheta, table_width=w, table_length=l)

    print("=" * 60)
    print(f"Deskinator Simulation (Error Factor: {ERROR_FACTOR})")
    print(f"Table Size: {w}m x {l}m")
    print(f"Start Pose: ({rx:.2f}, {ry:.2f}, {rtheta:.2f})")
    print("=" * 60)

    # Apply Patches
    apply_patches()

    # Initialize Robot (forces enable_viz=True)
    robot = main.Deskinator(enable_viz=True)

    # Run Robot
    # The robot.run() loop is blocking and runs the async loop.
    # It uses RateTimer which sleeps. This works fine for real-time simulation.

    # Manually trigger start signal
    print("[Sim] Auto-starting robot...")
    robot.start_signal = True

    robot.run()


if __name__ == "__main__":
    main_viz()
