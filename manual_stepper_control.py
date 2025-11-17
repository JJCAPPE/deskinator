#!/usr/bin/env python3
"""Interactive keyboard drive utility for A4988 stepper motor drivers.

This script provides manual control of the robot using keyboard input.
Use the WASD keys (plus Q/E/Z/C) to jog the robot and SPACE to stop.
Press X or CTRL+C to exit.

The robot uses A4988 stepper drivers controlled via GPIO pins with
microstepping for smooth motion.

Example usage:

    python3 manual_stepper_control.py --speed 0.10 --max-speed 0.18

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


class KeyboardStepper:
    """Minimal keyboard-based jog controller using velocity commands."""

    PROMPT = """
Controls:
  w : both wheels forward
  s : both wheels backward
  a : pivot left (left back, right forward)
  d : pivot right (left forward, right back)
  q : left wheel forward
  z : left wheel backward
  e : right wheel forward
  c : right wheel backward
  space : stop (zero velocity)
  x : exit

Speed tweaks:
  + / = : increase speed
  - / _ : decrease speed
  ? / h : show this help

Press keys repeatedly or hold them; the last pressed command continues
until another command is given or STOP (space) is pressed.
"""

    COMMANDS = {
        "w": (+1, 0),  # Forward (v > 0, omega = 0)
        "s": (-1, 0),  # Backward (v < 0, omega = 0)
        "a": (0, +1),  # Pivot left (v = 0, omega > 0)
        "d": (0, -1),  # Pivot right (v = 0, omega < 0)
        "q": (+1, -1),  # Left forward (turn right)
        "z": (-1, +1),  # Left backward (turn left)
        "e": (+1, +1),  # Right forward (turn left)
        "c": (-1, -1),  # Right backward (turn right)
    }

    def __init__(
        self,
        stepper: StepperDrive,
        *,
        base_speed: float,
        speed_step: float,
        max_speed: float,
    ):
        self._stepper = stepper
        self._base_speed = base_speed
        self._speed_step = speed_step
        self._max_speed = max_speed
        self._min_speed = 0.01

        # Angular speed for pivoting (fixed, independent of linear speed)
        self._omega_speed = LIMS.OMEGA_MAX * 0.5  # Use 50% of max for pivoting

        # Current command multipliers (-1, 0, or +1 for v and omega)
        self._v_mult = 0
        self._omega_mult = 0

        # Current speed setting (0 to max_speed)
        self._speed = self._clip_speed(base_speed)

    def run(self):
        """Main interactive loop."""

        self._print_banner()

        # Control loop runs at ~50 Hz for smooth motion
        dt = 0.02
        next_update = time.monotonic()

        try:
            with RawTerminal(sys.stdin):
                while True:
                    timeout = max(0.0, next_update - time.monotonic())
                    key = self._read_key(timeout)

                    if key:
                        key = key.lower()
                        if key == "x":
                            print("\nExiting...")
                            break
                        if key == " ":
                            self._set_command(0, 0)
                            continue
                        if key in {"+", "="}:
                            self._adjust_speed(self._speed_step)
                            continue
                        if key in {"-", "_"}:
                            self._adjust_speed(-self._speed_step)
                            continue
                        if key in {"?", "h"}:
                            self._show_help()
                            continue
                        cmd = self.COMMANDS.get(key)
                        if cmd:
                            self._set_command(*cmd)
                        else:
                            print(f"\nUnmapped key: {repr(key)}")

                    # Update stepper control
                    now = time.monotonic()
                    if now >= next_update:
                        self._update_stepper()
                        next_update = now + dt

        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self._stepper.stop()
            self._stepper.stop_pulse_generation()

    def _read_key(self, timeout: float) -> Optional[str]:
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None

    def _set_command(self, v_mult: int, omega_mult: int):
        """Set velocity command multipliers."""
        self._v_mult = int(v_mult)
        self._omega_mult = int(omega_mult)
        print(
            f"\rV: {self._dir_label(self._v_mult)} | "
            f"Omega: {self._dir_label(self._omega_mult)} | "
            f"Speed: {self._speed:.3f} m/s",
            end="",
            flush=True,
        )

    def _dir_label(self, multiplier: int) -> str:
        if multiplier > 0:
            return "forward"
        if multiplier < 0:
            return "backward"
        return "stop"

    def _adjust_speed(self, delta: float):
        """Adjust base speed setting."""
        if delta == 0.0:
            return
        new_speed = self._clip_speed(self._speed + delta)
        if abs(new_speed - self._speed) < 1e-6:
            print(
                f"\nSpeed already at limit ({self._speed:.3f} m/s)."
            )
            return
        self._speed = new_speed
        print(f"\nSpeed -> {self._speed:.3f} m/s (max: {self._max_speed:.3f} m/s)")

    def _clip_speed(self, speed: float) -> float:
        """Clip speed to valid range."""
        return max(self._min_speed, min(self._max_speed, speed))

    def _update_stepper(self):
        """Update stepper drive with current velocity command."""
        # Calculate commanded velocities
        v_cmd = self._v_mult * self._speed
        # Use fixed angular speed for pivoting (independent of linear speed)
        omega_cmd = self._omega_mult * self._omega_speed

        # Send command to stepper
        self._stepper.command(v_cmd, omega_cmd)

        # Update internal state (needed for acceleration ramping)
        self._stepper.update(0.02)

    def _show_help(self):
        print("\n" + self.PROMPT + "\n")

    def _print_banner(self):
        print("=" * 60)
        print("  A4988 Stepper Driver keyboard controller")
        print(f"  Step style: {self._stepper.step_style_name}")
        print(
            f"  Base speed: {self._speed:.3f} m/s "
            f"(limits {self._min_speed:.3f}-{self._max_speed:.3f})"
        )
        print(f"  Max linear velocity: {LIMS.V_MAX:.3f} m/s")
        print(f"  Max angular velocity: {LIMS.OMEGA_MAX:.3f} rad/s")
        print("  Press X or CTRL+C to exit.")
        print("=" * 60)
        print(self.PROMPT)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--speed",
        type=float,
        default=0.10,
        help="Initial linear speed in m/s (default: 0.10).",
    )
    parser.add_argument(
        "--max-speed",
        type=float,
        default=None,
        help=f"Maximum linear speed in m/s (default: {LIMS.V_MAX:.3f} from config).",
    )
    parser.add_argument(
        "--speed-step",
        type=float,
        default=0.02,
        help="Speed increment/decrement for +/- keys in m/s (default: 0.02).",
    )
    parser.add_argument(
        "--hold",
        action="store_true",
        help="Keep drivers enabled on exit (default disables).",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    max_speed = args.max_speed if args.max_speed is not None else LIMS.V_MAX
    max_speed = max(0.01, min(max_speed, LIMS.V_MAX))
    speed_step = max(0.01, args.speed_step)
    base_speed = max(0.01, min(max_speed, args.speed))

    # Initialize stepper drive
    stepper = StepperDrive(release_on_idle=not args.hold)

    try:
        controller = KeyboardStepper(
            stepper,
            base_speed=base_speed,
            speed_step=speed_step,
            max_speed=max_speed,
        )
        controller.run()
    finally:
        # Ensure cleanup
        stepper.stop()
        stepper.stop_pulse_generation()


if __name__ == "__main__":
    main()
