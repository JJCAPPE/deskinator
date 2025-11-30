"""
Demo visualization script for Deskinator.

Demonstrates the visualization capabilities without requiring hardware.
Generates synthetic data matching the robot's data structures and displays
it using the existing Visualizer class.

Usage:
    python viz_demo.py [--error 0.0-1.0] [--speed 1.0-10.0]
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
    table_length: float = 2.0

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

        # Calculate proposed new position
        new_x = self.x + self.v * math.cos(self.theta) * dt
        new_y = self.y + self.v * math.sin(self.theta) * dt

        # Safety: Don't let robot center go more than 10cm off table (would fall)
        max_overhang = 0.10
        new_x = max(self.min_x - max_overhang, min(self.max_x + max_overhang, new_x))
        new_y = max(self.min_y - max_overhang, min(self.max_y + max_overhang, new_y))

        self.x = new_x
        self.y = new_y
        self.theta += self.omega * dt
        self.theta = wrap_angle(self.theta)

    def is_on_table(self, x: float, y: float) -> bool:
        return (self.min_x <= x <= self.max_x) and (self.min_y <= y <= self.max_y)


# Global simulation state
SIM_STATE: Optional[SimState] = None
ERROR_FACTOR: float = 0.0
SPEED_FACTOR: float = 1.0


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
    """Mock StepperDrive with continuous background simulation."""

    # Interface constants matching hw.stepper
    MICROSTEPS_PER_STEP = 16

    # Simulation rate (Hz) - how often the background thread updates
    SIM_RATE = 200  # 200 Hz for smooth simulation

    def __init__(self, microsteps_per_step=16, release_on_idle=False):
        self.microsteps_per_step = microsteps_per_step
        self.steps_per_m_effective = GEOM.STEPS_PER_M * self.microsteps_per_step

        self.vL_target = 0.0
        self.vR_target = 0.0
        self.vL_current = 0.0
        self.vR_current = 0.0

        # Use floating point for sub-step precision
        self.steps_left = 0.0
        self.steps_right = 0.0
        self.last_steps_left = 0.0
        self.last_steps_right = 0.0

        # Time tracking for continuous simulation
        self.last_update_time = time.time()

        self.lock = threading.Lock()

        # Background simulation thread
        self._running = True
        self._sim_thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self._sim_thread.start()

    def _simulation_loop(self):
        """Background thread that continuously updates simulation state."""
        sim_period = 1.0 / self.SIM_RATE

        while self._running:
            current_time = time.time()

            with self.lock:
                dt = current_time - self.last_update_time
                self.last_update_time = current_time

                # Apply speed factor to simulation time
                dt_scaled = dt * SPEED_FACTOR

                # Update current velocities (instant for simplicity)
                self.vL_current = self.vL_target
                self.vR_current = self.vR_target

                # Update ground truth physics
                if SIM_STATE:
                    SIM_STATE.update(
                        self.vL_current, self.vR_current, dt_scaled, ERROR_FACTOR
                    )

                # Calculate distance traveled
                dL = self.vL_current * dt_scaled
                dR = self.vR_current * dt_scaled

                # Add encoder noise/error
                if ERROR_FACTOR > 0:
                    dL *= 1.0 + random.gauss(0, 0.01 * ERROR_FACTOR)
                    dR *= 1.0 + random.gauss(0, 0.01 * ERROR_FACTOR)

                # Accumulate steps (floating point for precision)
                self.steps_left += dL * self.steps_per_m_effective
                self.steps_right += dR * self.steps_per_m_effective

            # Sleep to maintain simulation rate
            time.sleep(sim_period / SPEED_FACTOR)

    def command(self, v: float, omega: float):
        """Set target velocities."""
        w = GEOM.WHEEL_BASE

        # Physical wheel velocities
        physical_left = v - 0.5 * omega * w
        physical_right = v + 0.5 * omega * w

        with self.lock:
            self.vL_target = physical_left
            self.vR_target = physical_right

    def update(self, dt: float):
        """Called from control loop - no-op since background thread handles updates."""
        # Background thread handles continuous simulation
        pass

    def read_odometry(self) -> Tuple[float, float]:
        """Return distance traveled by each wheel since last call."""
        with self.lock:
            dSteps_L = self.steps_left - self.last_steps_left
            dSteps_R = self.steps_right - self.last_steps_right

            self.last_steps_left = self.steps_left
            self.last_steps_right = self.steps_right

            dSL = dSteps_L / self.steps_per_m_effective
            dSR = dSteps_R / self.steps_per_m_effective

            return dSL, dSR

    def stop(self):
        with self.lock:
            self.vL_target = 0.0
            self.vR_target = 0.0
            self.vL_current = 0.0
            self.vR_current = 0.0

    def stop_pulse_generation(self):
        pass

    def disable_drivers(self):
        self._running = False
        if self._sim_thread.is_alive():
            self._sim_thread.join(timeout=0.5)


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

        # Store for debug access
        self._last_world_pos = (sx, sy)
        self._last_robot_pos = (rx, ry, rtheta)

        # Calculate distance to nearest edge (negative = outside table)
        dist_to_left = sx - SIM_STATE.min_x
        dist_to_right = SIM_STATE.max_x - sx
        dist_to_bottom = sy - SIM_STATE.min_y
        dist_to_top = SIM_STATE.max_y - sy

        # Minimum distance to any edge (negative if outside)
        min_dist = min(dist_to_left, dist_to_right, dist_to_bottom, dist_to_top)
        self._last_min_dist = min_dist

        # Smooth falloff: sensor detects table surface via IR reflection
        # - Far from edge (>5cm): strong signal ~200
        # - At edge (0cm): weak signal ~20 (threshold)
        # - Over edge (<0cm): very weak ~5
        edge_falloff_dist = 0.05  # 5cm falloff zone

        if min_dist > edge_falloff_dist:
            # Solidly on table
            val = 200
        elif min_dist > 0:
            # In falloff zone - linear interpolation
            t = min_dist / edge_falloff_dist
            val = int(20 + t * 180)  # 20 at edge, 200 at 5cm in
        else:
            # Off table - very low value
            val = max(5, int(15 + min_dist * 100))  # Drops further as you go over

        # Add noise
        if ERROR_FACTOR > 0:
            val += int(random.gauss(0, 8 * ERROR_FACTOR))

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


# --- Patched Visualizer ---

# Import the real Visualizer
from utils.viz import Visualizer as RealVisualizer


class SimVisualizer(RealVisualizer):
    """Visualizer that injects ground truth table bounds from simulation."""

    def update(self, *args, **kwargs):
        # Inject ground truth bounds from SIM_STATE
        if SIM_STATE:
            kwargs["ground_truth_bounds"] = (
                SIM_STATE.min_x,
                SIM_STATE.max_x,
                SIM_STATE.min_y,
                SIM_STATE.max_y,
            )
        super().update(*args, **kwargs)


# --- Fast RateTimer ---


class FastRateTimer:
    """RateTimer with speed multiplier for faster simulation."""

    def __init__(self, hz: float):
        self.hz = hz
        self.period = 1.0 / hz
        self.last_time = time.time()

    def sleep(self):
        current_time = time.time()
        elapsed = current_time - self.last_time
        scaled_period = self.period / SPEED_FACTOR
        if elapsed < scaled_period:
            time.sleep(scaled_period - elapsed)
        self.last_time = time.time()

    async def sleep_async(self):
        import asyncio

        current_time = time.time()
        elapsed = current_time - self.last_time
        scaled_period = self.period / SPEED_FACTOR
        if elapsed < scaled_period:
            await asyncio.sleep(scaled_period - elapsed)
        self.last_time = time.time()

    def elapsed(self) -> float:
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        return dt

    def reset(self):
        self.last_time = time.time()


# --- Patching ---


def apply_patches():
    """Patch main module to use mock hardware."""
    main.I2CBus = SimI2CBus
    main.StepperDrive = SimStepper
    main.APDS9960 = SimAPDS9960
    main.MPU6050 = SimMPU6050
    main.Vacuum = SimVacuum
    main.Buzzer = SimBuzzer
    main.RateTimer = FastRateTimer
    main.Visualizer = SimVisualizer

    # Patch EKF to initialize with ground truth starting pose
    from slam.ekf import EKF as RealEKF

    class SimEKF(RealEKF):
        def __init__(self, *args, **kwargs):
            # Override initial pose with ground truth from simulation
            if SIM_STATE is not None:
                kwargs["x0"] = SIM_STATE.x
                kwargs["y0"] = SIM_STATE.y
                kwargs["theta0"] = SIM_STATE.theta
            super().__init__(*args, **kwargs)

    main.EKF = SimEKF

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
    global SIM_STATE, ERROR_FACTOR, SPEED_FACTOR

    parser = argparse.ArgumentParser(description="Deskinator Simulation Demo")
    parser.add_argument(
        "--error", type=float, default=0.0, help="Error factor (0.0-1.0)"
    )
    parser.add_argument(
        "--speed", type=float, default=1.0, help="Speed multiplier (1.0-10.0)"
    )
    args = parser.parse_args()

    ERROR_FACTOR = max(0.0, min(1.0, args.error))
    SPEED_FACTOR = max(1.0, min(10.0, args.speed))

    # Initialize Simulation State
    # Table: 2m x 2m square (centered at 0,0 -> -1..1, -1..1)
    # Robot: Random position at least 20cm from edge
    margin = 0.2
    w = 2.0
    l = 2.0

    rx = random.uniform(-w / 2 + margin, w / 2 - margin)
    ry = random.uniform(-l / 2 + margin, l / 2 - margin)
    rtheta = random.uniform(-math.pi, math.pi)

    SIM_STATE = SimState(x=rx, y=ry, theta=rtheta, table_width=w, table_length=l)

    print("=" * 60)
    print(f"Deskinator Simulation (Error: {ERROR_FACTOR}, Speed: {SPEED_FACTOR}x)")
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
