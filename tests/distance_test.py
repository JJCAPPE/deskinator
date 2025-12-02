#!/usr/bin/env python3
"""Test script to verify motor distance accuracy based on input distance."""

import argparse
import sys
import time
from pathlib import Path

# Add parent directory to path for imports when running from tests/
parent_dir = Path(__file__).parent.parent
if str(parent_dir) not in sys.path:
    sys.path.insert(0, str(parent_dir))

from config import LIMS
from hw.stepper import StepperDrive


def test_distance(
    target_distance: float,
    velocity: float = None,
    rate: float = None,
    max_rate: float = None,
    dt: float = 0.02,
    tolerance: float = 0.01,
):
    """
    Test motor distance accuracy by commanding a target distance and measuring actual.

    Args:
        target_distance: Target distance to travel in meters (forward is positive)
        velocity: Commanded velocity in m/s (if None, calculated from rate)
        rate: Step rate in steps/s (default: 150)
        max_rate: Maximum step rate in steps/s (default: 200)
        dt: Update period in seconds (default: 0.02 s = 50 Hz)
        tolerance: Distance tolerance in meters for completion (default: 0.01 m)

    Returns:
        tuple: (actual_distance_left, actual_distance_right, elapsed_time)
    """
    drive = StepperDrive(release_on_idle=True)

    # Calculate velocity from rate if rate is specified
    if rate is not None:
        # velocity = rate / steps_per_meter
        calculated_velocity = rate / drive.steps_per_meter
        if velocity is None:
            velocity = calculated_velocity
        else:
            # Use the smaller of user-specified velocity or rate-based velocity
            velocity = min(abs(velocity), abs(calculated_velocity)) * (
                1 if velocity >= 0 else -1
            )

    # Apply max_rate limit if specified
    if max_rate is not None:
        max_velocity_from_rate = max_rate / drive.steps_per_meter
        if velocity is None:
            velocity = max_velocity_from_rate
        else:
            # Clamp velocity to max_rate
            if abs(velocity) > max_velocity_from_rate:
                velocity = max_velocity_from_rate * (1 if velocity >= 0 else -1)

    # Default velocity if neither velocity nor rate specified
    if velocity is None:
        velocity = 0.10

    # Determine direction based on target distance sign
    is_forward = target_distance >= 0
    abs_target = abs(target_distance)

    # Ensure velocity sign matches target direction
    if is_forward and velocity < 0:
        velocity = abs(velocity)
    elif not is_forward and velocity > 0:
        velocity = -abs(velocity)

    print("\n" + "=" * 60)
    print("Motor Distance Test")
    print("=" * 60)
    print(f"Step style: {drive.step_style_name}")
    print(f"Steps/m (effective): {drive.steps_per_meter:.1f}")
    print(f"Max wheel speed: {drive.max_wheel_speed:.4f} m/s")

    # Calculate actual step rate
    actual_step_rate = abs(velocity) * drive.steps_per_meter
    if rate is not None:
        print(f"Requested step rate: {rate:.1f} steps/s")
    if max_rate is not None:
        print(f"Max step rate limit: {max_rate:.1f} steps/s")
    print(f"Actual step rate: {actual_step_rate:.1f} steps/s")

    print(
        f"\nTarget distance: {target_distance:.4f} m ({'forward' if is_forward else 'reverse'})"
    )
    print(f"Commanded velocity: {velocity:.4f} m/s")
    print("-" * 60)

    # Clamp velocity to limits
    v_cmd = max(-LIMS.V_REV_MAX, min(velocity, LIMS.V_MAX))
    if v_cmd != velocity:
        print(f"Warning: Velocity clamped from {velocity:.4f} to {v_cmd:.4f} m/s")

    # Reset odometry
    drive.read_odometry()  # Initialize last_steps counters

    # Start motion
    drive.command(v_cmd, 0.0)  # No rotation

    start_time = time.time()
    total_distance_left = 0.0
    total_distance_right = 0.0
    elapsed_time = 0.0

    print("\nMoving...")
    print("Time (s) | Distance L (m) | Distance R (m) | Avg (m)")
    print("-" * 60)

    try:
        while True:
            # Update motion
            drive.update(dt)

            # Read odometry
            dSL, dSR = drive.read_odometry()
            total_distance_left += dSL
            total_distance_right += dSR
            elapsed_time = time.time() - start_time

            # Average distance (signed: positive = forward, negative = reverse)
            avg_distance = (total_distance_left + total_distance_right) / 2.0

            # Print progress every 0.5 seconds
            if int(elapsed_time * 2) != int((elapsed_time - dt) * 2):
                print(
                    f"{elapsed_time:7.2f} | "
                    f"{total_distance_left:14.4f} | "
                    f"{total_distance_right:14.4f} | "
                    f"{avg_distance:7.4f}"
                )

            # Check if we've reached the target
            if is_forward:
                if avg_distance >= abs_target:
                    break
            else:
                if avg_distance <= -abs_target:
                    break

            time.sleep(dt)

        # Stop motors
        drive.stop()

        # Final reading
        dSL, dSR = drive.read_odometry()
        total_distance_left += dSL
        total_distance_right += dSR

        # Brief pause to ensure motors stop
        time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        drive.stop()
    finally:
        drive.stop_pulse_generation()
        drive.disable_drivers()

    # Calculate results
    avg_distance = (total_distance_left + total_distance_right) / 2.0
    error = avg_distance - target_distance
    error_percent = (error / target_distance * 100) if target_distance != 0 else 0.0

    print("\n" + "=" * 60)
    print("Results:")
    print("=" * 60)
    print(f"Target distance:     {target_distance:10.4f} m")
    print(f"Left wheel distance: {total_distance_left:10.4f} m")
    print(f"Right wheel distance:{total_distance_right:10.4f} m")
    print(f"Average distance:    {avg_distance:10.4f} m")
    print(f"Error:               {error:10.4f} m ({error_percent:+.2f}%)")
    print(f"Elapsed time:        {elapsed_time:10.2f} s")
    print(
        f"Average speed:       {avg_distance/elapsed_time if elapsed_time > 0 else 0:10.4f} m/s"
    )
    print("=" * 60)

    return total_distance_left, total_distance_right, elapsed_time


def main():
    parser = argparse.ArgumentParser(
        description="Test motor distance accuracy based on input distance",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Test 0.5 meters forward (uses --rate 150, --max-rate 200)
  python3 distance_test.py 0.5
  
  # Test 0.2 meters with custom step rate
  python3 distance_test.py 0.2 --rate 100 --max-rate 150
  
  # Test with explicit velocity (overrides rate)
  python3 distance_test.py 0.3 --velocity 0.15
  
  # Test reverse (negative distance)
  python3 distance_test.py -0.1
        """,
    )
    parser.add_argument(
        "distance",
        type=float,
        help="Target distance in meters (positive = forward, negative = reverse)",
    )
    parser.add_argument(
        "--velocity",
        type=float,
        default=None,
        help="Commanded velocity in m/s (if not specified, calculated from --rate)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=150.0,
        help="Step rate in steps/s (default: 150)",
    )
    parser.add_argument(
        "--max-rate",
        type=float,
        default=200.0,
        help="Maximum step rate in steps/s (default: 200)",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.02,
        help="Update period in seconds (default: 0.02)",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.01,
        help="Distance tolerance in meters (default: 0.01)",
    )

    args = parser.parse_args()

    if args.distance == 0:
        print("Error: Distance cannot be zero")
        return 1

    try:
        test_distance(
            target_distance=args.distance,
            velocity=args.velocity,
            rate=args.rate,
            max_rate=args.max_rate,
            dt=args.dt,
            tolerance=args.tolerance,
        )
        return 0
    except Exception as e:
        print(f"\nError during test: {e}")
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    exit(main())
