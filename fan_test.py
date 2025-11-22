#!/usr/bin/env python3
"""Simple standalone test to run the vacuum fan at different speeds."""

import sys
import time
from hw.vacuum import Vacuum
from hw.gpio import gpio_manager


def getch():
    """Read a single character from stdin."""
    try:
        # Windows
        import msvcrt

        return msvcrt.getch().decode("utf-8")
    except ImportError:
        # Unix
        import tty
        import termios

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


def main():
    vacuum = Vacuum()
    print("\nVacuum Fan Control")
    print("------------------")
    print("0-9: Set duty cycle 0% - 90%")
    print("f:   Set 100% (Full speed)")
    print("q:   Quit")
    print("\nPress keys to control...")

    current_duty = 0.0
    try:
        vacuum.on(current_duty)
        while True:
            ch = getch()

            if ch == "\x03" or ch.lower() == "q":  # Ctrl+C (ETX) or q
                break

            new_duty = None
            if ch.isdigit():
                val = int(ch)
                new_duty = val / 10.0
            elif ch.lower() == "f":
                new_duty = 1.0

            if new_duty is not None:
                current_duty = new_duty
                vacuum.on(current_duty)
                # Use \r to overwrite the line
                print(f"\rDuty Cycle: {current_duty*100:>3.0f}% ", end="", flush=True)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        print("\nStopping fan...")
        vacuum.cleanup()
        gpio_manager.cleanup()


if __name__ == "__main__":
    main()
