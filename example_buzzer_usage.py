#!/usr/bin/env python3
"""
Example: Using the buzzer in your cleaning routine.

This shows how to integrate the buzzer into your main cleaning code
to signal start and finish of cleaning cycles.
"""

import time
from hw.buzzer import Buzzer
from hw.gpio import gpio_manager


def main():
    """Example cleaning cycle with buzzer feedback."""

    # Initialize buzzer
    buzzer = Buzzer()

    print("=" * 60)
    print("DESKINATOR CLEANING CYCLE WITH AUDIO FEEDBACK")
    print("=" * 60)

    try:
        # Signal start of cleaning
        print("\n🔊 Signaling START of cleaning cycle...")
        buzzer.beep_start()  # 2 short beeps
        time.sleep(1.0)

        # Simulate cleaning process
        print("🧹 Cleaning in progress...")
        print("   (Mapping desk, sweeping surface, etc.)")
        for i in range(5):
            print(f"   Progress: {(i+1)*20}%")
            time.sleep(1.0)

        # Signal completion
        print("\n✅ Cleaning complete!")
        print("🔊 Signaling FINISH...")
        buzzer.beep_finish()  # 3 ascending beeps
        time.sleep(1.0)

        print("\n" + "=" * 60)
        print("Cleaning cycle complete!")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        buzzer.beep_error()

    except Exception as e:
        print(f"\n\n❌ Error occurred: {e}")
        buzzer.beep_error()  # 3 rapid beeps for errors

    finally:
        # Cleanup
        buzzer.cleanup()
        gpio_manager.cleanup()


def demo_all_patterns():
    """Demonstrate all buzzer patterns."""

    buzzer = Buzzer()

    print("\n" + "=" * 60)
    print("BUZZER PATTERN DEMONSTRATION")
    print("=" * 60 + "\n")

    patterns = [
        ("Start Pattern (2 beeps)", buzzer.beep_start),
        ("Finish Pattern (3 ascending beeps)", buzzer.beep_finish),
        ("Error Pattern (3 rapid beeps)", buzzer.beep_error),
        ("Warning Pattern (1 long beep)", buzzer.beep_warning),
    ]

    for name, func in patterns:
        print(f"Playing: {name}")
        func()
        time.sleep(1.5)

    print("\nAll patterns demonstrated!")
    buzzer.cleanup()
    gpio_manager.cleanup()


if __name__ == "__main__":
    # Run the main example
    main()

    # Uncomment to demo all patterns instead:
    # demo_all_patterns()
