#!/usr/bin/env python3
"""Simple standalone test to exercise the drive motors."""

import time

from config import LIMS
from hw.stepper import StepperDrive
from hw.gpio import gpio_manager


TEST_SEQUENCE = [
    ("Forward crawl", 0.05, 0.0, 3.0),
    ("Forward faster", 0.12, 0.0, 3.0),
    ("Spin in place left", 0.0, 0.8, 2.5),
    ("Spin in place right", 0.0, -0.8, 2.5),
    ("Reverse", -0.04, 0.0, 2.0),
    ("Stop", 0.0, 0.0, 1.0),
]


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def run_sequence(sequence: list[tuple[str, float, float, float]], dt: float = 0.02):
    """Run a scripted set of velocity commands."""
    drive = StepperDrive()

    try:
        for label, v_cmd, omega_cmd, duration in sequence:
            v = clamp(v_cmd, -LIMS.V_REV_MAX, LIMS.V_MAX)
            omega = clamp(omega_cmd, -LIMS.OMEGA_MAX, LIMS.OMEGA_MAX)

            print(f"\n=== {label} ===")
            print(
                f"Commanding v={v:.3f} m/s, omega={omega:.3f} rad/s for {duration:.1f}s"
            )

            drive.command(v, omega)

            end_time = time.time() + duration
            while time.time() < end_time:
                drive.update(dt)
                time.sleep(dt)

        print("\nSequence complete")

    except KeyboardInterrupt:
        print("\nInterrupted, stopping motors...")
    finally:
        drive.stop()
        drive.stop_pulse_generation()
        drive.disable_drivers()
        gpio_manager.cleanup()


if __name__ == "__main__":
    run_sequence(TEST_SEQUENCE)
