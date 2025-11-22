import time
import sys

try:
    from adafruit_extended_bus import ExtendedI2C as I2C
except ImportError:
    print("Error: adafruit-circuitpython-extended-bus (or adafruit-extended-bus) is not installed.")
    print("Please run: pip3 install adafruit-circuitpython-extended-bus")
    sys.exit(1)

try:
    from adafruit_apds9960.apds9960 import APDS9960
except ImportError:
    print("Error: adafruit-circuitpython-apds9960 is not installed.")
    print("Please run: pip install adafruit-circuitpython-apds9960")
    sys.exit(1)

def main():
    print("Initializing sensors using Adafruit library...")
    print("Note: This script uses adafruit-circuitpython-extended-bus to access")
    print("      the software I2C buses defined in /boot/config.txt")

    # Configuration from user query/config.py
    # Right sensor is on Bus 3
    # Left sensor is on Bus 7
    
    sensors = {}
    
    # Initialize Right Sensor (Bus 3)
    try:
        i2c3 = I2C(3)
        apds_right = APDS9960(i2c3)
        apds_right.enable_proximity = True
        
        # Optional: Configure similar to our custom driver if needed
        # The Adafruit lib defaults are usually fine, but we can check
        # apds_right.proximity_gain
        
        sensors['RIGHT (Bus 3)'] = apds_right
        print("✓ Right sensor (Bus 3) initialized")
    except Exception as e:
        print(f"✗ Failed to init Right sensor (Bus 3): {e}")

    # Initialize Left Sensor (Bus 7)
    try:
        i2c7 = I2C(7)
        apds_left = APDS9960(i2c7)
        apds_left.enable_proximity = True
        sensors['LEFT (Bus 7)'] = apds_left
        print("✓ Left sensor (Bus 7) initialized")
    except Exception as e:
        print(f"✗ Failed to init Left sensor (Bus 7): {e}")

    if not sensors:
        print("No sensors initialized. Exiting.")
        return

    print("\nReading Proximity Values (0-255)... Press Ctrl+C to stop")
    print("-" * 50)
    
    try:
        while True:
            outputs = []
            for name, sensor in sensors.items():
                try:
                    val = sensor.proximity
                    outputs.append(f"{name}: {val:>3}")
                except Exception as e:
                    outputs.append(f"{name}: ERR")
            
            print(" | ".join(outputs), end='\r')
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()

