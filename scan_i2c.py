#!/usr/bin/env python3
"""
I2C Address Scanner for Deskinator

This script scans the I2C bus to find all connected devices and helps
you determine the correct addresses for config.py.

Run this after wiring your I2C devices to find:
- MPU-6050 IMU address
- TCA9548A multiplexer address (should be 0x70)
- APDS9960 proximity sensors (2 front-facing sensors)
"""

import sys
import time
from typing import List, Dict

from hw.i2c import I2CBus
from config import I2C as I2C_CONFIG


class Colors:
    """ANSI color codes for terminal output."""

    GREEN = "\033[92m"
    RED = "\033[91m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    BOLD = "\033[1m"
    END = "\033[0m"


def print_header(text: str):
    """Print a section header."""
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'='*70}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.BLUE}{text:^70}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'='*70}{Colors.END}\n")


def print_device(addr: int, name: str = "Unknown", note: str = ""):
    """Print a detected device."""
    addr_str = f"0x{addr:02X}"
    if note:
        print(
            f"  {Colors.GREEN}✓{Colors.END} {addr_str:8} - {Colors.BOLD}{name}{Colors.END} {Colors.CYAN}({note}){Colors.END}"
        )
    else:
        print(
            f"  {Colors.GREEN}✓{Colors.END} {addr_str:8} - {Colors.BOLD}{name}{Colors.END}"
        )


def identify_device(addr: int) -> tuple[str, str]:
    """
    Identify common I2C devices by address.

    Returns:
        (device_name, note)
    """
    common_devices = {
        0x68: ("MPU-6050 IMU", "Default address"),
        0x69: ("MPU-6050 IMU", "AD0 pulled high"),
        0x39: ("APDS9960", "Proximity/Gesture sensor"),
        0x70: ("TCA9548A", "I2C Multiplexer"),
        0x71: ("TCA9548A", "I2C Multiplexer - Alt addr"),
        0x72: ("TCA9548A", "I2C Multiplexer - Alt addr"),
        0x73: ("TCA9548A", "I2C Multiplexer - Alt addr"),
        0x74: ("TCA9548A", "I2C Multiplexer - Alt addr"),
        0x75: ("TCA9548A", "I2C Multiplexer - Alt addr"),
        0x76: ("TCA9548A", "I2C Multiplexer - Alt addr"),
        0x77: ("TCA9548A", "I2C Multiplexer - Alt addr"),
    }

    if addr in common_devices:
        return common_devices[addr]
    else:
        return ("Unknown device", "")


def scan_main_bus(bus: I2CBus) -> List[int]:
    """Scan the main I2C bus for devices."""
    print_header("MAIN I2C BUS SCAN")

    print("Scanning I2C bus 1 for devices...\n")

    devices = bus.scan()

    if not devices:
        print(f"{Colors.RED}✗ No devices found!{Colors.END}")
        print(f"\n{Colors.YELLOW}Troubleshooting:{Colors.END}")
        print(
            "  1. Check I2C is enabled: sudo raspi-config -> Interface Options -> I2C"
        )
        print("  2. Verify wiring: SDA, SCL, VCC, GND")
        print("  3. Check pull-up resistors (usually 4.7kΩ)")
        print("  4. Run: i2cdetect -y 1\n")
        return []

    print(f"Found {Colors.BOLD}{len(devices)}{Colors.END} device(s):\n")

    for addr in devices:
        name, note = identify_device(addr)
        print_device(addr, name, note)

    return devices


def scan_multiplexer_channels(bus: I2CBus, mux_addr: int) -> Dict[int, List[int]]:
    """
    Scan all channels of the TCA9548A multiplexer.

    Returns:
        Dictionary mapping channel number to list of device addresses
    """
    print_header("TCA9548A MULTIPLEXER CHANNEL SCAN")

    print(f"Scanning multiplexer at 0x{mux_addr:02X}...\n")

    channel_devices = {}

    for channel in range(8):
        print(f"{Colors.BOLD}Channel {channel}:{Colors.END}")

        try:
            # Select the channel
            bus.write_byte(mux_addr, 1 << channel)
            time.sleep(0.05)  # Wait for channel to stabilize

            # Scan for devices
            devices = []
            for addr in range(0x03, 0x78):
                if addr == mux_addr:  # Skip the multiplexer itself
                    continue
                try:
                    bus.read_byte(addr)
                    devices.append(addr)
                except:
                    pass

            if devices:
                for addr in devices:
                    name, note = identify_device(addr)
                    print_device(addr, name, note)
                channel_devices[channel] = devices
            else:
                print(f"  {Colors.YELLOW}○{Colors.END} No devices detected\n")

        except Exception as e:
            print(f"  {Colors.RED}✗ Error scanning channel: {e}{Colors.END}\n")

    # Disable all channels
    try:
        bus.write_byte(mux_addr, 0x00)
    except:
        pass

    return channel_devices


def generate_config_suggestions(
    main_devices: List[int], mux_addr: int | None, channel_devices: Dict[int, List[int]]
):
    """Generate configuration suggestions for config.py."""
    print_header("CONFIGURATION SUGGESTIONS")

    # Find IMU
    imu_candidates = [addr for addr in main_devices if addr in [0x68, 0x69]]

    # Find APDS9960 sensors
    apds_channels = []
    for ch, devices in channel_devices.items():
        if 0x39 in devices:
            apds_channels.append(ch)

    print(f"{Colors.BOLD}Update your config.py with these values:{Colors.END}\n")
    print(f"{Colors.CYAN}# In config.py, I2CParams class:{Colors.END}\n")

    # IMU configuration
    if imu_candidates:
        imu_addr = imu_candidates[0]
        print(
            f"{Colors.GREEN}ADDR_IMU: int = 0x{imu_addr:02X}{Colors.END}  # MPU-6050 detected"
        )
        if len(imu_candidates) > 1:
            print(
                f"{Colors.YELLOW}# Note: Multiple IMU addresses detected: {[f'0x{a:02X}' for a in imu_candidates]}{Colors.END}"
            )
    else:
        print(
            f"{Colors.YELLOW}ADDR_IMU: int | None = None{Colors.END}  # No IMU detected - check wiring"
        )

    # MUX configuration
    if mux_addr:
        print(
            f"{Colors.GREEN}ADDR_MUX: int = 0x{mux_addr:02X}{Colors.END}  # TCA9548A detected"
        )
    else:
        print(
            f"{Colors.RED}ADDR_MUX: int = 0x70{Colors.END}  # No multiplexer detected - check wiring"
        )

    # APDS configuration
    print(f"APDS_ADDR: int = 0x39  # APDS9960 standard address")

    if apds_channels:
        if len(apds_channels) == 2:
            print(
                f"{Colors.GREEN}MUX_CHANS: tuple[int, int] = {tuple(apds_channels)}{Colors.END}  # Both sensors detected"
            )
        else:
            print(
                f"{Colors.YELLOW}MUX_CHANS: tuple[int, int] = {tuple(apds_channels[:2] if len(apds_channels) >= 2 else apds_channels + [0] * (2 - len(apds_channels)))}{Colors.END}  # Only {len(apds_channels)}/2 sensors detected"
            )
    else:
        print(
            f"{Colors.RED}MUX_CHANS: tuple[int, int] = (0, 1){Colors.END}  # No APDS sensors detected"
        )

    # Summary
    print(f"\n{Colors.BOLD}Summary:{Colors.END}")
    print(
        f"  IMU (MPU-6050):     {Colors.GREEN if imu_candidates else Colors.RED}{'✓ Found' if imu_candidates else '✗ Not found'}{Colors.END}"
    )
    print(
        f"  Multiplexer (TCA):   {Colors.GREEN if mux_addr else Colors.RED}{'✓ Found' if mux_addr else '✗ Not found'}{Colors.END}"
    )
    print(
        f"  Proximity sensors:   {Colors.GREEN if len(apds_channels) == 2 else Colors.YELLOW if apds_channels else Colors.RED}{len(apds_channels)}/2 detected{Colors.END}"
    )

    # Sensor mapping
    if apds_channels:
        print(f"\n{Colors.BOLD}Sensor Channel Mapping:{Colors.END}")
        sensor_labels = ["Left", "Right"]
        print("  (Verify this matches your physical layout)")
        for i, ch in enumerate(apds_channels):
            label = sensor_labels[i] if i < len(sensor_labels) else f"Sensor {i}"
            print(f"  Channel {ch} → {label}")

    # Warnings
    if not imu_candidates:
        print(f"\n{Colors.YELLOW}⚠ Warning: No IMU detected{Colors.END}")
        print("  - Check MPU-6050 wiring (SDA, SCL, VCC, GND, AD0)")
        print("  - Ensure MPU-6050 has 3.3V power and common ground")
        print("  - Expected addresses: 0x68 (AD0 low) or 0x69 (AD0 high)")

    if not mux_addr:
        print(f"\n{Colors.YELLOW}⚠ Warning: No multiplexer detected{Colors.END}")
        print("  - Check TCA9548A wiring")
        print("  - Expected address: 0x70 (default)")

    if len(apds_channels) < 2:
        print(
            f"\n{Colors.YELLOW}⚠ Warning: Only {len(apds_channels)}/2 proximity sensors detected{Colors.END}"
        )
        print("  - Check APDS9960 wiring on each MUX channel")
        print("  - All APDS9960 sensors use address 0x39")
        print("  - Each sensor must be on a different MUX channel")
        print("  - Expected: 2 front-facing sensors (left and right)")


def main():
    """Main scanning routine."""
    print(f"\n{Colors.BOLD}{Colors.CYAN}")
    print("╔══════════════════════════════════════════════════════════════════╗")
    print("║              DESKINATOR I2C ADDRESS SCANNER                      ║")
    print("║         Detecting all I2C devices and addresses                 ║")
    print("╚══════════════════════════════════════════════════════════════════╝")
    print(Colors.END)

    print(f"\n{Colors.BOLD}Prerequisites:{Colors.END}")
    print("  • I2C enabled on Raspberry Pi")
    print("  • All devices wired and powered")
    print("  • Pull-up resistors installed (4.7kΩ)\n")

    input(f"Press {Colors.BOLD}Enter{Colors.END} to start scanning...")

    # Initialize I2C bus
    try:
        bus = I2CBus(I2C_CONFIG.BUS)
    except Exception as e:
        print(f"\n{Colors.RED}✗ Failed to initialize I2C bus: {e}{Colors.END}")
        print("\nMake sure I2C is enabled:")
        print("  sudo raspi-config → Interface Options → I2C → Enable")
        return 1

    # Scan main bus
    main_devices = scan_main_bus(bus)

    if not main_devices:
        bus.close()
        return 1

    # Look for multiplexer
    mux_addr = None
    for addr in main_devices:
        if 0x70 <= addr <= 0x77:  # TCA9548A address range
            mux_addr = addr
            break

    # Scan multiplexer channels if found
    channel_devices = {}
    if mux_addr:
        channel_devices = scan_multiplexer_channels(bus, mux_addr)
    else:
        print(
            f"\n{Colors.YELLOW}⚠ No multiplexer detected - skipping channel scan{Colors.END}"
        )
        print("  If you have a TCA9548A, check its wiring\n")

    # Generate configuration suggestions
    generate_config_suggestions(main_devices, mux_addr, channel_devices)

    # Cleanup
    bus.close()

    print(f"\n{Colors.GREEN}{Colors.BOLD}✓ Scan complete!{Colors.END}\n")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print(f"\n\n{Colors.YELLOW}Scan interrupted by user{Colors.END}\n")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n{Colors.RED}✗ Unexpected error: {e}{Colors.END}")
        import traceback

        traceback.print_exc()
        sys.exit(1)
