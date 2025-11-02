#!/usr/bin/env python3
"""Interactive keyboard drive utility for the Adafruit Stepper Motor HAT.

This script keeps things simple while still giving you smoother motion and
adjustable speed. Use the WASD keys (plus Q/E/Z/C) to jog the robot, press
SPACE to hold position, and X or CTRL+C to exit.  Hit +/- to change the step
rate on the fly or F to jump to the configured max rate.  A modest acceleration
limit blends rate changes so the motors feel less choppy.

Example usage:

    python3 manual_stepper_control.py --rate 240 --max-rate 500 --style interleave

"""

from __future__ import annotations

import argparse
import sys
import termios
import time
import tty
from math import inf
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
    """Keyboard-driven jog controller with adjustable rate and smoothing."""

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
  x : exit controller

Speed tweaks:
  + / = : increase base step rate
  - / _ : decrease base step rate
  f     : toggle fast mode (max rate)
  ? / h : show this help again

Press keys repeatedly or hold them; the last pressed command continues until
another command is given or STOP (space) is pressed.
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
        max_rate: float,
        rate_step: float,
        accel: float,
        tick_interval: float,
        release_on_exit: bool,
    ):
        self._kit = kit
        self._stepper_module = stepper_module
        self._step_style = step_style
        self._release_on_exit = release_on_exit

        # Hardware wiring swap: the physical right wheel is on MotorKit.stepper1
        # and the physical left wheel is on MotorKit.stepper2. Capture that here
        # so the rest of the code can speak in physical terms.
        self._right = getattr(kit, "stepper1", None)
        self._left = getattr(kit, "stepper2", None)
        if self._left is None or self._right is None:
            raise SystemExit(
                "MotorKit did not report both stepper1 and stepper2. "
                "Check hardware wiring."
            )

        self._min_rate = 1.0
        self._max_rate = max(self._min_rate, max_rate)
        self._base_rate = self._clip_rate(base_rate)
        self._saved_base_rate = self._base_rate
        self._rate_step = max(1.0, abs(rate_step))
        self._accel = max(0.0, accel)
        self._tick_interval = max(1e-3, tick_interval)
        self._fast_mode = False

        self._cmd_left = 0
        self._cmd_right = 0
        self._target_left = 0.0
        self._target_right = 0.0
        self._left_rate = 0.0
        self._right_rate = 0.0
        self._left_phase = 0.0
        self._right_phase = 0.0

        self._status_interval = 0.3
        self._next_status = time.monotonic()

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
        print()

    def _read_key(self, timeout: float) -> Optional[str]:
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None

    def _handle_key(self, key: str) -> bool:
        if key == "\x03":  # Ctrl+C
            raise KeyboardInterrupt
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
        if key in {"f"}:
            self._toggle_fast_mode()
            return True
        if key in {"?", "h"}:
            self._show_help()
            self._print_status_line(newline=True)
            return True

        print(f"\nUnmapped key: {repr(key)}")
        return True

    def _set_command(self, left_dir: int, right_dir: int):
        self._cmd_left = int(left_dir)
        self._cmd_right = int(right_dir)
        self._fast_mode = False
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
        self._saved_base_rate = self._base_rate
        self._fast_mode = False
        self._sync_targets()
        print(
            f"\nBase rate -> {self._base_rate:.0f} steps/s "
            f"(limits {self._min_rate:.0f}-{self._max_rate:.0f})"
        )
        self._print_status_line()

    def _toggle_fast_mode(self):
        if not self._fast_mode:
            self._saved_base_rate = self._base_rate
            self._base_rate = self._max_rate
            self._fast_mode = True
            msg = "Fast mode ON"
        else:
            self._base_rate = self._clip_rate(self._saved_base_rate)
            self._fast_mode = False
            msg = "Fast mode OFF"
        self._sync_targets()
        print(f"\n{msg} -> base {self._base_rate:.0f} steps/s")
        self._print_status_line()

    def _clip_rate(self, rate: float) -> float:
        return max(self._min_rate, min(self._max_rate, rate))

    def _update_rates(self, dt: float):
        if dt <= 0.0:
            return
        max_delta = self._accel * dt if self._accel > 0 else float("inf")
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
        self._service_motor(
            dt,
            self._right_rate,
            self._right,
            is_left=False,
            invert_direction=True,
        )

    def _service_motor(
        self,
        dt: float,
        rate: float,
        motor,
        *,
        is_left: bool,
        invert_direction: bool = False,
    ):
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
        if invert_direction:
            direction *= -1

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
        print(f"  Base rate: {self._base_rate:.0f} steps/s (max {self._max_rate:.0f})")
        print(
            f"  Accel limit: {self._accel:.0f} steps/s², tick {self._tick_interval*1000:.1f} ms"
        )
        print("  Press X or CTRL+C to exit.")
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
        default=240.0,
        help="Initial steps per second per motor (default: 240).",
    )
    parser.add_argument(
        "--max-rate",
        type=float,
        default=600.0,
        help="Maximum allowable step rate (default: 600).",
    )
    parser.add_argument(
        "--rate-step",
        type=float,
        default=40.0,
        help="Rate increment/decrement when pressing +/- (default: 40).",
    )
    parser.add_argument(
        "--accel",
        type=float,
        default=800.0,
        help="Slew rate in steps/s^2 for smoothing (default: 800).",
    )
    parser.add_argument(
        "--tick-hz",
        type=float,
        default=250.0,
        help="Scheduler update frequency in Hz (default: 250).",
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

    max_rate = max(1.0, args.max_rate)
    base_rate = max(1.0, min(max_rate, args.rate))
    rate_step = max(1.0, args.rate_step)
    accel = max(0.0, args.accel)
    tick_hz = max(20.0, args.tick_hz)
    tick_interval = 1.0 / tick_hz

    kit = MotorKit()
    controller = KeyboardStepper(
        kit,
        stepper_module,
        step_style=step_style,
        base_rate=base_rate,
        max_rate=max_rate,
        rate_step=rate_step,
        accel=accel,
        tick_interval=tick_interval,
        release_on_exit=not args.hold,
    )
    controller.run()


if __name__ == "__main__":
    main()
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
import contextlib
import sys
import termios
import time
import tty
from math import inf
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
        step_period: float,
        release_on_exit: bool,
    ):
        self._kit = kit
        self._stepper_module = stepper_module
        self._step_style = step_style
        self._period = max(step_period, 1e-3)
        self._release_on_exit = release_on_exit

        self._left = getattr(kit, "stepper1", None)
        self._right = getattr(kit, "stepper2", None)
        if self._left is None or self._right is None:
            raise SystemExit(
                "MotorKit did not report both stepper1 and stepper2. "
                "Check hardware wiring."
            )

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
                        dirs = self.COMMANDS.get(key)
                        if dirs:
                            self._set_command(*dirs)
                        else:
                            print(f"\nUnmapped key: {repr(key)}")

                    now = time.monotonic()
                    if now >= next_step:
                        self._step_once()
                        next_step = now + self._period
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
        rate = 1.0 / self._period if self._period > 0 else inf
        print("=" * 60)
        print("  Adafruit Stepper HAT keyboard controller")
        print(f"  Step style: {style_name}")
        print(f"  Target step rate: {rate:.1f} steps/s per motor")
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
        default=100.0,
        help="Target steps per second per motor (default: 100).",
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

    period = 1.0 / args.rate if args.rate > 0 else 0.01

    kit = MotorKit()
    controller = KeyboardStepper(
        kit,
        stepper_module,
        step_style=step_style,
        step_period=period,
        release_on_exit=not args.hold,
    )
    controller.run()


if __name__ == "__main__":
    main()
