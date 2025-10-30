#!/usr/bin/env python3
"""
Hardware test script for Deskinator robot.

Tests all sensors and actuators to verify wiring:
- I2C bus and device detection
- TCA9548A I2C multiplexer
- 4x APDS9960 proximity sensors (through multiplexer)
- MPU-6050 IMU
- Left and right stepper motors
- Vacuum motor PWM

Run this after completing wiring to verify all connections.
"""

import time
import sys
from typing import List, Tuple

from config import PINS, I2C as I2C_CONFIG
from hw.i2c import I2CBus
from hw.gpio import gpio_manager
from hw.tca9548a import TCA9548A
from hw.apds9960 import APDS9960
from hw.mpu6050 import MPU6050
from hw.stepper import StepperDrive
from hw.vacuum import Vacuum
from hw.buzzer import Buzzer
from hw.gesture import GestureSensor


class Colors:
    """ANSI color codes for terminal output."""

    GREEN = "\033[92m"
    RED = "\033[91m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    BOLD = "\033[1m"
    END = "\033[0m"


def print_header(text: str):
    """Print a section header."""
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'='*60}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.BLUE}{text}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'='*60}{Colors.END}\n")


def print_test(name: str):
    """Print a test name."""
    print(f"{Colors.BOLD}[TEST]{Colors.END} {name}...", end=" ", flush=True)


def print_pass(msg: str = "PASS"):
    """Print pass message."""
    print(f"{Colors.GREEN}✓ {msg}{Colors.END}")


def print_fail(msg: str = "FAIL"):
    """Print fail message."""
    print(f"{Colors.RED}✗ {msg}{Colors.END}")


def print_warn(msg: str):
    """Print warning message."""
    print(f"{Colors.YELLOW}⚠ {msg}{Colors.END}")


def print_info(msg: str):
    """Print info message."""
    print(f"  {msg}")


def test_i2c_bus() -> Tuple[bool, I2CBus]:
    """Test I2C bus and scan for devices."""
    print_header("I2C BUS TEST")

    print_test("Initializing I2C bus")
    try:
        bus = I2CBus(I2C_CONFIG.BUS)
        print_pass()
    except Exception as e:
        print_fail(f"Error: {e}")
        return False, None

    print_test("Scanning I2C bus")
    try:
        devices = bus.scan()
        if devices:
            print_pass(f"Found {len(devices)} device(s)")
            for addr in devices:
                print_info(f"Device at address: 0x{addr:02X}")
        else:
            print_warn("No devices found")

        # Check for expected devices
        expected_mux = I2C_CONFIG.ADDR_MUX
        if expected_mux in devices:
            print_info(f"✓ TCA9548A multiplexer detected at 0x{expected_mux:02X}")
        else:
            print_warn(f"TCA9548A not found at expected address 0x{expected_mux:02X}")

        if I2C_CONFIG.ADDR_IMU and I2C_CONFIG.ADDR_IMU in devices:
            print_info(f"✓ MPU-6050 IMU detected at 0x{I2C_CONFIG.ADDR_IMU:02X}")

        return True, bus
    except Exception as e:
        print_fail(f"Error: {e}")
        return False, None


def test_multiplexer(bus: I2CBus) -> Tuple[bool, TCA9548A]:
    """Test TCA9548A I2C multiplexer."""
    print_header("TCA9548A MULTIPLEXER TEST")

    print_test("Initializing TCA9548A")
    try:
        mux = TCA9548A(bus, I2C_CONFIG.ADDR_MUX)
        print_pass()
    except Exception as e:
        print_fail(f"Error: {e}")
        return False, None

    print_test("Testing channel selection")
    try:
        # Test each channel
        for ch in range(8):
            mux.select(ch)
            if mux.get_channel() == ch:
                print_info(f"  Channel {ch}: OK")
            else:
                print_warn(f"  Channel {ch}: Failed to select")

        # Disable all channels
        mux.disable_all()
        print_pass("All channels tested")
        return True, mux
    except Exception as e:
        print_fail(f"Error: {e}")
        return False, None


def test_proximity_sensors(bus: I2CBus, mux: TCA9548A) -> bool:
    """Test all 4 APDS9960 proximity sensors through multiplexer."""
    print_header("APDS9960 PROXIMITY SENSORS TEST")

    channels = I2C_CONFIG.MUX_CHANS
    sensors = []

    for i, ch in enumerate(channels):
        print_test(f"Sensor {i} on channel {ch}")
        try:
            # Select multiplexer channel
            mux.select(ch)
            time.sleep(0.05)

            # Initialize sensor
            sensor = APDS9960(bus, I2C_CONFIG.APDS_ADDR)
            sensor.init()

            # Read proximity value
            prox_raw = sensor.read_proximity_raw()
            prox_norm = sensor.read_proximity_norm()

            print_pass(f"Raw: {prox_raw}, Normalized: {prox_norm:.3f}")
            sensors.append(sensor)

        except Exception as e:
            print_fail(f"Error: {e}")
            print_warn(f"  Check wiring to sensor {i} on MUX channel {ch}")
            sensors.append(None)

    # Test real-time readings
    if any(sensors):
        print("\n" + Colors.BOLD + "Live proximity readings (5 seconds):" + Colors.END)
        print("(Move your hand near each sensor to test)")
        print(
            "Sensor order: 0=left-outer, 1=left-inner, 2=right-inner, 3=right-outer\n"
        )

        start_time = time.time()
        while time.time() - start_time < 5.0:
            readings = []
            for i, (sensor, ch) in enumerate(zip(sensors, channels)):
                if sensor:
                    mux.select(ch)
                    time.sleep(0.01)
                    prox = sensor.read_proximity_raw()
                    readings.append(f"S{i}:{prox:3d}")
                else:
                    readings.append(f"S{i}:---")

            print(f"\r{' | '.join(readings)}", end="", flush=True)
            time.sleep(0.1)

        print("\n")

    mux.disable_all()
    return any(sensors)


def test_imu(bus: I2CBus) -> bool:
    """Test MPU-6050 IMU."""
    print_header("MPU-6050 IMU TEST")

    print_test("Initializing MPU-6050")
    try:
        imu = MPU6050(bus, I2C_CONFIG.ADDR_IMU)
        print_pass()
    except Exception as e:
        print_fail(f"Error: {e}")
        return False

    print_test("Reading IMU data")
    try:
        yaw = imu.read_yaw_abs()
        yaw_rate = imu.read_yaw_rate()

        print_pass(f"Yaw: {yaw:.3f} rad, Yaw rate: {yaw_rate:.3f} rad/s")

        # Live readings
        print("\n" + Colors.BOLD + "Live IMU readings (5 seconds):" + Colors.END)
        print("(Rotate the robot to test)\n")

        start_time = time.time()
        while time.time() - start_time < 5.0:
            yaw = imu.read_yaw_abs()
            yaw_rate = imu.read_yaw_rate()
            yaw_deg = yaw * 180.0 / 3.14159

            print(
                f"\rYaw: {yaw_deg:7.2f}° | Rate: {yaw_rate:6.3f} rad/s",
                end="",
                flush=True,
            )
            time.sleep(0.05)

        print("\n")
        return True

    except Exception as e:
        print_fail(f"Error: {e}")
        return False


def test_stepper_motors() -> bool:
    """Test stepper motor drivers."""
    print_header("STEPPER MOTOR TEST")

    print_test("Initializing stepper driver")
    try:
        drive = StepperDrive()
        print_pass()
    except Exception as e:
        print_fail(f"Error: {e}")
        return False

    print_test("Testing GPIO pins")
    try:
        print_pass(f"Left: STEP={PINS.LEFT_STEP}, DIR={PINS.LEFT_DIR}")
        print_info(f"Right: STEP={PINS.RIGHT_STEP}, DIR={PINS.RIGHT_DIR}")
    except Exception as e:
        print_fail(f"Error: {e}")
        return False

    print("\n" + Colors.BOLD + "Direction Pin Test:" + Colors.END)
    print_info("Testing direction pins (you should see DIR pins toggle)")
    print_info("Use a multimeter or LED to verify 3.3V on DIR pins")

    # Test direction pins
    tests = [
        ("Both forward", 0.1, 0.0),
        ("Both reverse", -0.05, 0.0),
        ("Turn right", 0.1, 0.5),
        ("Turn left", 0.1, -0.5),
        ("Stop", 0.0, 0.0),
    ]

    for name, v, omega in tests:
        print(f"\n  {name}: v={v:.2f} m/s, ω={omega:.2f} rad/s")
        drive.command(v, omega)
        drive.update(0.1)  # Update to set direction pins

        # Calculate expected wheel velocities
        vL = v - 0.5 * omega * 0.170  # WHEEL_BASE = 0.170m
        vR = v + 0.5 * omega * 0.170

        dir_left = "FWD" if vL >= 0 else "REV"
        dir_right = "FWD" if vR >= 0 else "REV"

        print_info(
            f"    Left: {dir_left} ({vL:+.3f} m/s), Right: {dir_right} ({vR:+.3f} m/s)"
        )
        print_info(
            f"    DIR pins - Left: GPIO{PINS.LEFT_DIR}, Right: GPIO{PINS.RIGHT_DIR}"
        )
        time.sleep(2.0)

    print_pass("Direction test complete")

    print(
        "\n"
        + Colors.YELLOW
        + "NOTE:"
        + Colors.END
        + " Full step pulse generation requires:"
    )
    print("  - Power to motor drivers (12V)")
    print("  - Enable pins pulled low (if using)")
    print("  - Motors connected")
    print("  You should hear motors step during normal operation\n")

    return True


def test_vacuum() -> bool:
    """Test vacuum motor PWM."""
    print_header("VACUUM MOTOR TEST")

    print_test("Initializing vacuum controller")
    try:
        vacuum = Vacuum()
        print_pass(f"PWM pin: GPIO{PINS.VACUUM_PWM}")
    except Exception as e:
        print_fail(f"Error: {e}")
        return False

    print("\n" + Colors.BOLD + "PWM Duty Cycle Test:" + Colors.END)
    print_info("Testing vacuum motor at different speeds")
    print_info("Use oscilloscope or multimeter to verify PWM signal")
    print_info("(If motor is connected, you should hear speed changes)\n")

    duty_cycles = [0.25, 0.50, 0.75, 1.00]

    for duty in duty_cycles:
        print(f"  Duty cycle: {duty*100:.0f}%")
        vacuum.on(duty)
        time.sleep(2.0)

    print("\n  Turning off...")
    vacuum.off()
    time.sleep(0.5)

    print_pass("PWM test complete")

    print(
        "\n"
        + Colors.YELLOW
        + "NOTE:"
        + Colors.END
        + " Full vacuum motor operation requires:"
    )
    print("  - 12V power supply connected")
    print("  - MOSFET gate connected to PWM pin")
    print("  - Vacuum motor connected to MOSFET drain")
    print("  - Proper heatsinking on MOSFET\n")

    vacuum.cleanup()
    return True


def test_buzzer() -> bool:
    """Test buzzer for audio feedback."""
    print_header("BUZZER TEST")

    print_test("Initializing buzzer")
    try:
        buzzer = Buzzer()
        print_pass(f"Buzzer pin: GPIO{PINS.BUZZER}")
    except Exception as e:
        print_fail(f"Error: {e}")
        return False

    print("\n" + Colors.BOLD + "Buzzer Pattern Test:" + Colors.END)
    print_info("Testing various beep patterns")
    print_info("(You should hear different beep patterns)\n")

    try:
        print("  Single beep...")
        buzzer.beep()
        time.sleep(0.5)

        print("  Double beep (Start pattern)...")
        buzzer.beep_start()
        time.sleep(0.5)

        print("  Triple beep (Error pattern)...")
        buzzer.beep_error()
        time.sleep(0.5)

        print("  Long beep (Warning pattern)...")
        buzzer.beep_warning()
        time.sleep(0.5)

        print("  Ascending beeps (Finish pattern)...")
        buzzer.beep_finish()
        time.sleep(0.5)

        print_pass("Buzzer test complete")

        print("\n" + Colors.YELLOW + "NOTE:" + Colors.END + " Buzzer usage:")
        print("  - Start cleaning: 2 short beeps")
        print("  - Finish cleaning: 3 ascending beeps")
        print("  - Error: 3 rapid beeps")
        print("  - Warning: 1 long beep\n")

        buzzer.cleanup()
        return True

    except Exception as e:
        print_fail(f"Error during beep test: {e}")
        buzzer.cleanup()
        return False


def test_gesture_sensor() -> bool:
    """Test gesture sensor for touchless control."""
    print_header("GESTURE SENSOR TEST")

    print_test("Initializing gesture sensor")
    try:
        gesture = GestureSensor(I2C_CONFIG.GESTURE_BUS, I2C_CONFIG.GESTURE_ADDR)
        print_pass(f"I2C pins: SDA=GPIO{PINS.GESTURE_SDA}, SCL=GPIO{PINS.GESTURE_SCL}")
    except Exception as e:
        print_fail(f"Error: {e}")
        return False

    print("\n" + Colors.BOLD + "Gesture Detection Test:" + Colors.END)
    print_info("Hold your hand near the sensor to trigger start")
    print_info("Testing for 10 seconds - move hand in and out a few times\n")

    trigger_count = 0
    hand_active = False

    try:
        start_time = time.time()

        while time.time() - start_time < 10.0:
            present = gesture.is_hand_present()
            raw = gesture.last_proximity_raw

            if present and not hand_active:
                trigger_count += 1
                print(
                    f"  {Colors.GREEN}✓{Colors.END} Proximity trigger detected (raw={raw})"
                )
                hand_active = True
            elif not present and hand_active:
                print("  Hand removed")
                hand_active = False

            time.sleep(0.05)

        print()
        if trigger_count > 0:
            print_pass(f"Proximity triggers detected: {trigger_count}x")
        else:
            print_warn("No proximity trigger detected - move hand closer to sensor")

        print("\n" + Colors.YELLOW + "NOTE:" + Colors.END + " Gesture sensor usage:")
        print("  - Any hand presence starts cleaning")
        print("  - No cancel gesture; use hardware stop or UI to end")
        print("  - Requires software I2C on GPIO17/GPIO4")
        print("  - See HARDWARE_SETUP.md for I2C configuration\n")

        gesture.cleanup()
        return True

    except Exception as e:
        print_fail(f"Error during gesture test: {e}")
        gesture.cleanup()
        return False


def test_summary(results: dict):
    """Print test summary."""
    print_header("TEST SUMMARY")

    total = len(results)
    passed = sum(results.values())

    for test_name, result in results.items():
        status = (
            f"{Colors.GREEN}PASS{Colors.END}"
            if result
            else f"{Colors.RED}FAIL{Colors.END}"
        )
        print(f"  {test_name:.<40} {status}")

    print(f"\n{Colors.BOLD}Total: {passed}/{total} tests passed{Colors.END}")

    if passed == total:
        print(
            f"\n{Colors.GREEN}{Colors.BOLD}✓ ALL TESTS PASSED! Wiring looks good!{Colors.END}\n"
        )
    else:
        print(
            f"\n{Colors.RED}{Colors.BOLD}✗ Some tests failed. Check wiring and connections.{Colors.END}\n"
        )


def main():
    """Main test sequence."""
    print(f"\n{Colors.BOLD}{Colors.BLUE}")
    print("╔══════════════════════════════════════════════════════════╗")
    print("║         DESKINATOR HARDWARE TEST SUITE                   ║")
    print("║         Testing all sensors and motors                   ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print(Colors.END)

    results = {}

    # Test I2C bus
    i2c_ok, bus = test_i2c_bus()
    results["I2C Bus"] = i2c_ok

    if not i2c_ok or bus is None:
        print_warn("I2C bus failed, skipping I2C device tests")
        test_summary(results)
        return 1

    # Test multiplexer
    mux_ok, mux = test_multiplexer(bus)
    results["TCA9548A Multiplexer"] = mux_ok

    # Test proximity sensors (only if multiplexer works)
    if mux_ok and mux:
        prox_ok = test_proximity_sensors(bus, mux)
        results["APDS9960 Sensors"] = prox_ok
    else:
        print_warn("Skipping proximity sensor tests (multiplexer failed)")
        results["APDS9960 Sensors"] = False

    # Test IMU
    imu_ok = test_imu(bus)
    results["MPU-6050 IMU"] = imu_ok

    # Test stepper motors
    stepper_ok = test_stepper_motors()
    results["Stepper Motors"] = stepper_ok

    # Test vacuum
    vacuum_ok = test_vacuum()
    results["Vacuum Motor"] = vacuum_ok

    # Test buzzer
    buzzer_ok = test_buzzer()
    results["Buzzer"] = buzzer_ok

    # Test gesture sensor
    gesture_ok = test_gesture_sensor()
    results["Gesture Sensor"] = gesture_ok

    # Print summary
    test_summary(results)

    # Cleanup
    print("Cleaning up...")
    gpio_manager.cleanup()
    if bus:
        bus.close()

    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print(f"\n\n{Colors.YELLOW}Test interrupted by user{Colors.END}")
        gpio_manager.cleanup()
        sys.exit(1)
    except Exception as e:
        print(f"\n\n{Colors.RED}Unexpected error: {e}{Colors.END}")
        import traceback

        traceback.print_exc()
        gpio_manager.cleanup()
        sys.exit(1)
