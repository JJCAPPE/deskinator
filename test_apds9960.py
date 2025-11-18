"""Super basic test for APDS9960 sensors - prints raw values every 0.5 seconds."""

import time
from config import I2C
from hw.apds9960 import APDS9960
from hw.i2c import I2CBus

print("Initializing APDS9960 sensors...")

# Left sensor
left_sensor = None
try:
    left_i2c = I2CBus(I2C.LEFT_SENSOR_BUS)
    left_sensor = APDS9960(left_i2c, I2C.APDS_ADDR)
    left_sensor.init()
    print(f"Left sensor OK (bus {I2C.LEFT_SENSOR_BUS})")
except Exception as e:
    print(f"Left sensor FAILED: {e}")

# Right sensor
right_sensor = None
try:
    right_i2c = I2CBus(I2C.RIGHT_SENSOR_BUS)
    right_sensor = APDS9960(right_i2c, I2C.APDS_ADDR)
    right_sensor.init()
    print(f"Right sensor OK (bus {I2C.RIGHT_SENSOR_BUS})")
except Exception as e:
    print(f"Right sensor FAILED: {e}")

print("\nReading sensors every 0.5 seconds... (Ctrl+C to stop)\n")

try:
    while True:
        left_raw = left_sensor.read_proximity_raw() if left_sensor else None
        right_raw = right_sensor.read_proximity_raw() if right_sensor else None
        
        print(f"Left: {left_raw:3d}  |  Right: {right_raw:3d}")
        
        time.sleep(0.5)
except KeyboardInterrupt:
    print("\nStopped.")

