#!/usr/bin/env python3
"""Interactive keyboard drive utility for the Adafruit Stepper Motor HAT.

Use this when you just need to move the robot manually and verify wiring.
Keys map directly to wheel directions (WASD + QE/ZC) and the utility now
supports smooth velocity ramps, dynamic speed presets, and on-the-fly rate
adjustment so the motion feels far less choppy.

Example usage:

    python3 manual_stepper_control.py --style auto --rate 150 --max-rate 400

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
    """Keyboard-driven jog controller with smooth ramps and speed presets."""

    PROMPT = """
Controls:
  w : both wheels forward          s : both wheels backward
  a : pivot left (left back)       d : pivot right (right back)
  q : left wheel forward           z : left wheel backward
  e : right wheel forward          c : right wheel backward
  space : hold position (stop stepping)
  x : exit controller

Speed tweaks:
  -/+ : decrease/increase base step rate by the configured step size
  1-4 : preset speeds (25%, 50%, 75%, 100% of max rate)
  ?   : show this help again
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

    SPEED_LEVELS = {
        "1": 0.25,
        "2": 0.50,
        "3": 0.75,
        "4": 1.00,
    }

    def __init__(
        self,
        kit,
        stepper_module,
        *,
        step_style,
        base_rate: float,
        rate_step: float,
        min_rate: float,
        max_rate: float,
        accel: float,
        tick_interval: float,
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

        self._min_rate = max(1.0, min_rate)
        self._max_rate = max(self._min_rate, max_rate)
        self._rate_step = max(1.0, abs(rate_step))
        self._base_rate = self._clip_rate(base_rate)
        self._slew_per_sec = max(0.0, accel)
        self._tick_interval = max(1e-3, tick_interval)

        self._cmd_left = 0
        self._cmd_right = 0
        self._target_left = 0.0
        self._target_right = 0.0
        self._left_rate = 0.0
        self._right_rate = 0.0
        self._left_phase = 0.0
        self._right_phase = 0.0

        self._status_interval = 0.2
        self._next_status = 0.0

    def run(self):
        """Main interactive loop."""

        self._print_banner()
        self._show_help()
        self._print_status_line(newline=True)

        last_tick = time.monotonic()
        try:
            with RawTerminal(sys.stdin):
                while True:
                    key = self._read_key(self._tick_interval)

                    now = time.monotonic()
                    dt = now - last_tick
                    last_tick = now
                    if dt <= 0.0:
                        dt = 1e-6
                    elif dt > 0.2:
                        dt = 0.2

                    if key:
                        key = key.lower()
                        if not self._handle_key(key):
                            break

                    self._update_rates(dt)
                    self._service_steps(dt)
                    self._maybe_refresh_status(now)
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            if self._release_on_exit:
                self._release()

    def _read_key(self, timeout: float) -> Optional[str]:
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None

    def _handle_key(self, key: str) -> bool:
        if key == "x":
            print("\nExiting...")
            return False
        if key == " ":
            self._set_command(0, 0)
            return True
        if key in self.COMMANDS:
            self._set_command(*self.COMMANDS[key])
            return True
        if key in {"+", "="}:
            self._change_base_rate(self._rate_step)
            return True
        if key in {"-", "_"}:
            self._change_base_rate(-self._rate_step)
            return True
        if key in self.SPEED_LEVELS:
            self._set_speed_level(self.SPEED_LEVELS[key])
            return True
        if key in {"?", "h"}:
            self._show_help()
            self._print_status_line(newline=True)
            return True
        if key == "\x03":  # Ctrl+C
            raise KeyboardInterrupt

        print(f"\nUnmapped key: {repr(key)}")
        return True

    def _set_command(self, left_dir: int, right_dir: int):
        self._cmd_left = int(left_dir)
        self._cmd_right = int(right_dir)
        self._sync_targets()
        self._print_status_line(newline=True)

    def _sync_targets(self):
        self._target_left = self._cmd_left * self._base_rate
        self._target_right = self._cmd_right * self._base_rate

    def _change_base_rate(self, delta: float):
        if delta == 0.0:
            return
        new_rate = self._clip_rate(self._base_rate + delta)
        if abs(new_rate - self._base_rate) < 1e-6:
            print("\nBase rate already at limit.")
            return
        self._base_rate = new_rate
        self._sync_targets()
        print(
            f"\nBase rate -> {self._base_rate:.0f} steps/s "
            f"(limits {self._min_rate:.0f}-{self._max_rate:.0f})"
        )
        self._print_status_line()

    def _set_speed_level(self, fraction: float):
        fraction = max(0.0, min(1.0, fraction))
        new_rate = max(self._min_rate, self._max_rate * fraction)
        new_rate = self._clip_rate(new_rate)
        self._base_rate = new_rate
        self._sync_targets()
        print(f"\nSpeed preset {fraction * 100:.0f}% -> {self._base_rate:.0f} steps/s")
        self._print_status_line()

    def _clip_rate(self, rate: float) -> float:
        return max(self._min_rate, min(self._max_rate, rate))

    def _update_rates(self, dt: float):
        if dt <= 0.0:
            return
        max_delta = self._slew_per_sec * dt if self._slew_per_sec > 0 else float("inf")
        self._left_rate = self._approach(self._left_rate, self._target_left, max_delta)
        self._right_rate = self._approach(
            self._right_rate, self._target_right, max_delta
        )

    @staticmethod
    def _approach(value: float, target: float, max_delta: float) -> float:
        delta = target - value
        if abs(delta) <= max_delta:
            return target
        step = max_delta if delta > 0 else -max_delta
        return value + step

    def _service_steps(self, dt: float):
        self._service_motor(dt, self._left_rate, self._left, is_left=True)
        self._service_motor(dt, self._right_rate, self._right, is_left=False)

    def _service_motor(self, dt: float, rate: float, motor, *, is_left: bool):
        if abs(rate) < 1e-6:
            if is_left:
                self._left_phase = 0.0
            else:
                self._right_phase = 0.0
            return

        phase_attr = "_left_phase" if is_left else "_right_phase"
        phase = getattr(self, phase_attr)
        phase += abs(rate) * dt
        direction = 1 if rate > 0 else -1

        while phase >= 1.0:
            self._do_step(motor, direction)
            phase -= 1.0

        setattr(self, phase_attr, phase)

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

    def _maybe_refresh_status(self, now: float):
        if now >= self._next_status:
            self._print_status_line()
            self._next_status = now + self._status_interval

    def _print_status_line(self, newline: bool = False):
        status = (
            f"Base {self._base_rate:.0f} sps | "
            f"L {self._status_label(self._left_rate)} | "
            f"R {self._status_label(self._right_rate)}"
        )
        prefix = "\n" if newline else "\r"
        print(f"{prefix}{status:<80}", end="", flush=True)

    def _status_label(self, rate: float) -> str:
        if abs(rate) < 1e-2:
            return "stop"
        direction = "fwd" if rate > 0 else "rev"
        return f"{direction} @ {abs(rate):.0f}sps"

    def _print_banner(self):
        style_name = self._style_name(self._step_style)
        print("=" * 60)
        print("  Adafruit Stepper HAT keyboard controller")
        print(f"  Step style: {style_name}")
        print(
            f"  Rate limits: {self._min_rate:.0f}-{self._max_rate:.0f} steps/s, "
            f"step increment {self._rate_step:.0f}"
        )
        print(
            f"  Slew limit: {self._slew_per_sec:.0f} steps/s^2, tick {self._tick_interval*1000:.1f} ms"
        )
        print("  Press '?' for help, 'x' to exit, CTRL+C to abort.")
        print("=" * 60)

    def _style_name(self, style_code: int) -> str:
        mapping = {
            getattr(self._stepper_module, "SINGLE", None): "SINGLE",
            getattr(self._stepper_module, "DOUBLE", None): "DOUBLE",
            getattr(self._stepper_module, "INTERLEAVE", None): "INTERLEAVE",
            getattr(self._stepper_module, "MICROSTEP", None): "MICROSTEP",
        }
        return mapping.get(style_code, f"0x{style_code:02X}")

    def _show_help(self):
        print(self.PROMPT)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--rate",
        type=float,
        default=150.0,
        help="Initial steps per second per motor (default: 150).",
    )
    parser.add_argument(
        "--min-rate",
        type=float,
        default=20.0,
        help="Minimum allowable step rate (default: 20).",
    )
    parser.add_argument(
        "--max-rate",
        type=float,
        default=400.0,
        help="Maximum allowable step rate (default: 400).",
    )
    parser.add_argument(
        "--rate-step",
        type=float,
        default=20.0,
        help="Rate increment/decrement when pressing +/- (default: 20).",
    )
    parser.add_argument(
        "--accel",
        type=float,
        default=600.0,
        help="Slew rate in steps/s^2 for smoothing (default: 600).",
    )
    parser.add_argument(
        "--tick-hz",
        type=float,
        default=400.0,
        help="Scheduler update frequency in Hz (default: 400).",
    )
    parser.add_argument(
        "--style",
        choices=["auto", "single", "double", "interleave", "microstep"],
        default="auto",
        help="Stepper style (auto prefers microstep if available).",
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
    microstep_available = hasattr(stepper_module, "MICROSTEP")
    if microstep_available:
        style_lookup["microstep"] = stepper_module.MICROSTEP

    if args.style == "auto":
        step_style = (
            style_lookup["microstep"]
            if microstep_available
            else style_lookup["interleave"]
        )
    elif args.style == "microstep":
        if not microstep_available:
            raise SystemExit("Microstepping not supported by this library build.")
        step_style = style_lookup["microstep"]
    else:
        try:
            step_style = style_lookup[args.style]
        except KeyError as exc:  # pragma: no cover - should not happen with choices
            raise SystemExit(f"Unsupported style: {args.style}") from exc

    min_rate = max(1.0, args.min_rate)
    max_rate = max(min_rate, args.max_rate)
    base_rate = max(min_rate, min(max_rate, args.rate))
    rate_step = max(1.0, args.rate_step)
    accel = max(0.0, args.accel)
    tick_hz = max(10.0, args.tick_hz)
    tick_interval = 1.0 / tick_hz

    kit = MotorKit()
    controller = KeyboardStepper(
        kit,
        stepper_module,
        step_style=step_style,
        base_rate=base_rate,
        rate_step=rate_step,
        min_rate=min_rate,
        max_rate=max_rate,
        accel=accel,
        tick_interval=tick_interval,
        release_on_exit=not args.hold,
    )
    controller.run()


if __name__ == "__main__":
    main()
