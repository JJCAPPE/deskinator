#!/usr/bin/env python3
"""Wheel direction calibration utility for A4988 stepper drivers.

This script helps you identify which physical wheel corresponds to which
GPIO pin configuration and verify forward/backward directions.

Controls:
  LEFT/RIGHT arrows : Select which wheel to test (left or right)
  UP arrow          : Move selected wheel forward
  DOWN arrow        : Move selected wheel backward
  SPACE             : Stop selected wheel
  X                 : Exit

Example usage:

    python3 calibrate_wheels.py --speed 0.05

"""

from __future__ import annotations

import argparse
import sys
import termios
import time
import tty
from select import select
from typing import Optional

try:
    from hw.stepper import StepperDrive
    from config import GEOM, LIMS
except ImportError as exc:
    raise SystemExit(
        "Failed to import required modules. Make sure you're running from "
        "the project root directory."
    ) from exc


class RawTerminal:
    """Context manager to put the terminal into raw, non-blocking mode."""

    def __init__(self, stream):
        self._stream = stream
        self._fd = stream.fileno()
        self._old_attrs: Optional[list[int]] = None

    def __enter__(self):
        self._old_attrs = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._old_attrs is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_attrs)


class WheelCalibrator:
    """Interactive wheel calibration controller."""

    PROMPT = """
Controls:
  ← / → : Select wheel to test (left or right)
  ↑     : Move selected wheel FORWARD
  ↓     : Move selected wheel BACKWARD
  SPACE : Stop selected wheel
  X     : Exit

Watch the wheels and identify:
  1. Which physical wheel moves when LEFT is selected
  2. Which physical wheel moves when RIGHT is selected
  3. Which direction is FORWARD for each wheel
"""

    def __init__(self, stepper: StepperDrive, *, speed: float):
        self._stepper = stepper
        self._speed = speed
        self._selected_wheel: Optional[str] = None  # "left" or "right"
        self._wheel_direction: int = 0  # -1 backward, 0 stop, +1 forward

        # Control loop timing
        self._dt = 0.02  # 50 Hz update rate

    def run(self):
        """Main interactive loop."""
        self._print_banner()

        try:
            with RawTerminal(sys.stdin):
                last_update = time.monotonic()
                while True:
                    # Check for keyboard input (non-blocking)
                    timeout = max(0.0, self._dt - (time.monotonic() - last_update))
                    key = self._read_key(timeout)

                    if key:
                        self._handle_key(key)

                    # Update stepper at fixed rate
                    now = time.monotonic()
                    if now - last_update >= self._dt:
                        self._update_stepper()
                        last_update = now

        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self._stepper.stop()
            print("\nMotors stopped.")

    def _read_key(self, timeout: float) -> Optional[str]:
        """Read a single keypress, handling escape sequences for arrows."""
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if not rlist:
            return None

        char = sys.stdin.read(1)
        if char == "\x1b":  # ESC sequence
            # Check for arrow keys
            char2 = sys.stdin.read(1)
            if char2 == "[":
                char3 = sys.stdin.read(1)
                if char3 == "A":
                    return "UP"
                elif char3 == "B":
                    return "DOWN"
                elif char3 == "C":
                    return "RIGHT"
                elif char3 == "D":
                    return "LEFT"
        elif char.lower() == "x":
            return "X"
        elif char == " ":
            return "SPACE"
        return char

    def _handle_key(self, key: str):
        """Handle keyboard input."""
        key_upper = key.upper()

        if key_upper == "X":
            print("\nExiting...")
            sys.exit(0)

        if key_upper == "LEFT":
            self._selected_wheel = "left"
            self._wheel_direction = 0  # Stop when switching
            self._update_display()
            return

        if key_upper == "RIGHT":
            self._selected_wheel = "right"
            self._wheel_direction = 0  # Stop when switching
            self._update_display()
            return

        if not self._selected_wheel:
            print("\nPlease select a wheel first (LEFT or RIGHT arrow)")
            return

        if key_upper == "UP":
            self._wheel_direction = 1  # Forward
            self._update_display()
            return

        if key_upper == "DOWN":
            self._wheel_direction = -1  # Backward
            self._update_display()
            return

        if key_upper == "SPACE":
            self._wheel_direction = 0  # Stop
            self._update_display()
            return

    def _update_stepper(self):
        """Update stepper drive with current wheel command."""
        if not self._selected_wheel or self._wheel_direction == 0:
            self._stepper.command(0.0, 0.0)
            self._stepper.update(self._dt)
            return

        # Calculate wheel velocities
        # For left wheel only: vL = speed, vR = 0
        #   v = (vL + vR) / 2 = speed / 2
        #   omega = (vR - vL) / wheel_base = -speed / wheel_base
        # For right wheel only: vL = 0, vR = speed
        #   v = speed / 2
        #   omega = speed / wheel_base

        wheel_speed = self._wheel_direction * self._speed

        if self._selected_wheel == "left":
            v = wheel_speed / 2.0
            omega = -wheel_speed / GEOM.WHEEL_BASE
        else:  # right
            v = wheel_speed / 2.0
            omega = wheel_speed / GEOM.WHEEL_BASE

        self._stepper.command(v, omega)
        self._stepper.update(self._dt)

    def _update_display(self):
        """Update the status display."""
        wheel_str = self._selected_wheel.upper() if self._selected_wheel else "NONE"
        if self._wheel_direction > 0:
            dir_str = "FORWARD"
        elif self._wheel_direction < 0:
            dir_str = "BACKWARD"
        else:
            dir_str = "STOPPED"

        print(
            f"\rSelected: {wheel_str:5s} | Direction: {dir_str:8s} | Speed: {self._speed:.3f} m/s",
            end="",
            flush=True,
        )

    def _print_banner(self):
        """Print startup banner."""
        print("=" * 70)
        print("  Wheel Direction Calibration")
        print("=" * 70)
        print(self.PROMPT)
        print("\nWaiting for wheel selection...")


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--speed",
        type=float,
        default=0.05,
        help="Wheel speed in m/s (default: 0.05).",
    )
    return parser.parse_args()


def main():
    """Main entry point."""
    args = parse_args()

    speed = max(0.01, min(LIMS.V_MAX, args.speed))

    print("Initializing stepper drivers...")
    try:
        stepper = StepperDrive()
    except Exception as exc:
        raise SystemExit(f"Failed to initialize stepper drivers: {exc}") from exc

    calibrator = WheelCalibrator(stepper, speed=speed)
    calibrator.run()


if __name__ == "__main__":
    main()

