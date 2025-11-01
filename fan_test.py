#!/usr/bin/env python3
"""Simple standalone test to run the vacuum fan at full power."""

import time

from hw.vacuum import Vacuum
from hw.gpio import gpio_manager


def main():
    vacuum = Vacuum()
    print("Running vacuum fan at 100% duty cycle. Press Ctrl+C to stop.")

    try:
        vacuum.on(1.0)
        while True:
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nStopping fan...")

    finally:
        vacuum.cleanup()
        gpio_manager.cleanup()


if __name__ == "__main__":
    main()
