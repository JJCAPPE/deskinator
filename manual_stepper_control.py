#!/usr/bin/env python3
"""Interactive keyboard drive utility for the Adafruit Stepper Motor HAT.

This script is intentionally lightweight so you can confirm the hardware
stack works end-to-end.  Use the WASD keys (plus Q/E) to jog the robot and
SPACE to hold position. Press X or CTRL+C to exit.  The default stepping
rate is conservative (100 steps/s) to stay within what the HAT can deliver
reliably over I2C while running from Python.

Example usage:

    python3 manual_stepper_control.py --rate 120 --style interleave

"""

from __future__ import annotations

import argparse
import sys
import termios
import time
import tty
from select import select
from typing import Optional


def _load_hat():
    """Import MotorKit + stepper modules with a nice error if missing."""

    try:
        import adafruit_motor.stepper as stepper_mod  # type: ignore
        from adafruit_motorkit import MotorKit  # type: ignore
    except ImportError as exc:  # pragma: no cover - hardware dependency
        raise SystemExit(
            "This utility requires the Adafruit MotorKit libraries. "
            "Install with 'pip3 install adafruit-circuitpython-motorkit'."
        ) from exc

    return MotorKit, stepper_mod


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
    """Minimal keyboard-based jog controller."""

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
  space : hold position (stop stepping)
  x : exit

Speed tweaks:
  + / = : increase step rate
  - / _ : decrease step rate
  ? / h : show this help

Press keys repeatedly or hold them; the last pressed command continues
until another command is given or STOP (space) is pressed.
"""

    COMMANDS = {
        "w": (+1, +1),
        "s": (-1, -1),
        "a": (-1, +1),
        "d": (+1, -1),
        "q": (+1, 0),
        "z": (-1, 0),
        "e": (0, +1),
        "c": (0, -1),
    }

    def __init__(
        self,
        kit,
        stepper_module,
        *,
        step_style,
        base_rate: float,
        rate_step: float,
        max_rate: float,
        release_on_exit: bool,
    ):
        self._kit = kit
        self._stepper_module = stepper_module
        self._step_style = step_style
        self._release_on_exit = release_on_exit

        self._left = getattr(kit, "stepper1", None)
        self._right = getattr(kit, "stepper2", None)
        if self._left is None or self._right is None:
            raise SystemExit(
                "MotorKit did not report both stepper1 and stepper2. "
                "Check hardware wiring."
            )

        self._min_rate = 30.0
        self._max_rate = max(self._min_rate, max_rate)
        self._rate_step = max(1.0, rate_step)
        self._rate = self._clip_rate(base_rate)
        self._period = self._rate_to_period(self._rate)

        self._left_dir = 0
        self._right_dir = 0

    def run(self):
        """Main interactive loop."""

        self._print_banner()

        next_step = time.monotonic()
        try:
            with RawTerminal(sys.stdin):
                while True:
                    timeout = max(0.0, next_step - time.monotonic())
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
                            if self._adjust_rate(self._rate_step):
                                next_step = time.monotonic() + self._period
                            continue
                        if key in {"-", "_"}:
                            if self._adjust_rate(-self._rate_step):
                                next_step = time.monotonic() + self._period
                            continue
                        if key in {"?", "h"}:
                            self._show_help()
                            continue
                        dirs = self.COMMANDS.get(key)
                        if dirs:
                            self._set_command(*dirs)
                        else:
                            print(f"\nUnmapped key: {repr(key)}")

                    now = time.monotonic()
                    while now >= next_step:
                        self._step_once()
                        next_step += self._period
                        now = time.monotonic()
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            if self._release_on_exit:
                self._release()

    def _rate_to_period(self, rate: float) -> float:
        return 1.0 / rate if rate > 0 else 0.01

    def _read_key(self, timeout: float) -> Optional[str]:
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None

    def _set_command(self, left_dir: int, right_dir: int):
        self._left_dir = int(left_dir)
        self._right_dir = int(right_dir)
        print(
            f"\rLeft: {self._dir_label(self._left_dir)} | "
            f"Right: {self._dir_label(self._right_dir)}",
            end="",
            flush=True,
        )

    def _dir_label(self, direction: int) -> str:
        if direction > 0:
            return "forward"
        if direction < 0:
            return "backward"
        return "stop"

    def _adjust_rate(self, delta: float) -> bool:
        if delta == 0.0:
            return False
        new_rate = self._clip_rate(self._rate + delta)
        if abs(new_rate - self._rate) < 1e-6:
            print(f"\nStep rate already at limit ({self._rate:.0f} steps/s).")
            return False
        self._rate = new_rate
        self._period = self._rate_to_period(self._rate)
        print(f"\nStep rate -> {self._rate:.0f} steps/s")
        return True

    def _clip_rate(self, rate: float) -> float:
        return max(self._min_rate, min(self._max_rate, rate))

    def _show_help(self):
        print("\n" + self.PROMPT + "\n")

    def _step_once(self):
        if self._left_dir:
            self._do_step(self._left, self._left_dir)
        if self._right_dir:
            self._do_step(self._right, self._right_dir)

    def _do_step(self, motor, direction: int):
        try:
            motor.onestep(
                direction=(
                    self._stepper_module.FORWARD
                    if direction > 0
                    else self._stepper_module.BACKWARD
                ),
                style=self._step_style,
            )
        except Exception as exc:  # pragma: no cover - hardware path
            print(f"\nStepper I2C error: {exc}")
            time.sleep(0.05)

    def _release(self):
        try:
            self._left.release()
        except Exception:
            pass
        try:
            self._right.release()
        except Exception:
            pass

    def _print_banner(self):
        style_name = self._style_name(self._step_style)
        print("=" * 60)
        print("  Adafruit Stepper HAT keyboard controller")
        print(f"  Step style: {style_name}")
        print(
            f"  Step rate: {self._rate:.0f} steps/s per motor "
            f"(limits {self._min_rate:.0f}-{self._max_rate:.0f})"
        )
        print("  Press X or CTRL+C to exit.")
        print("=" * 60)
        print(self.PROMPT)

    def _style_name(self, style_code: int) -> str:
        mapping = {
            getattr(self._stepper_module, "SINGLE", None): "SINGLE",
            getattr(self._stepper_module, "DOUBLE", None): "DOUBLE",
            getattr(self._stepper_module, "INTERLEAVE", None): "INTERLEAVE",
            getattr(self._stepper_module, "MICROSTEP", None): "MICROSTEP",
        }
        return mapping.get(style_code, f"0x{style_code:02X}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--rate",
        type=float,
        default=320.0,
        help="Initial steps per second per motor (default: 320).",
    )
    parser.add_argument(
        "--max-rate",
        type=float,
        default=720.0,
        help="Maximum allowable step rate (default: 720).",
    )
    parser.add_argument(
        "--rate-step",
        type=float,
        default=40.0,
        help="Step rate increment/decrement for +/- keys (default: 40).",
    )
    parser.add_argument(
        "--style",
        choices=["single", "double", "interleave", "microstep"],
        default="interleave",
        help="Stepper style to use (default: interleave).",
    )
    parser.add_argument(
        "--hold",
        action="store_true",
        help="Keep coils energized on exit (default releases).",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    MotorKit, stepper_module = _load_hat()

    style_lookup = {
        "single": stepper_module.SINGLE,
        "double": stepper_module.DOUBLE,
        "interleave": stepper_module.INTERLEAVE,
    }
    if hasattr(stepper_module, "MICROSTEP"):
        style_lookup["microstep"] = stepper_module.MICROSTEP
    else:
        style_lookup.pop("microstep", None)
        if args.style == "microstep":
            raise SystemExit("Microstepping not supported by this library build.")

    try:
        step_style = style_lookup[args.style]
    except KeyError as exc:  # pragma: no cover - should not happen with choices
        raise SystemExit(f"Unsupported style: {args.style}") from exc

    max_rate = max(60.0, args.max_rate)
    rate_step = max(5.0, args.rate_step)
    base_rate = max(30.0, min(max_rate, args.rate))

    kit = MotorKit()
    controller = KeyboardStepper(
        kit,
        stepper_module,
        step_style=step_style,
        base_rate=base_rate,
        rate_step=rate_step,
        max_rate=max_rate,
        release_on_exit=not args.hold,
    )
    controller.run()


if __name__ == "__main__":
    main()
