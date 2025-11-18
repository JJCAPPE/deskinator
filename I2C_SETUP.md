# I2C Bus Configuration

## Sensor Setup

The Deskinator now uses **two separate I2C buses** for the front-facing APDS9960 proximity sensors:

- **Right sensor**: Bus 1 (hardware I2C on GPIO2/GPIO3)
- **Left sensor**: Bus 7 (software I2C on GPIO19/GPIO26)

## Raspberry Pi Configuration

Add the following to `/boot/firmware/config.txt` (or `/boot/config.txt` on older Raspberry Pi OS):

```
# Enable hardware I2C bus 1 (GPIO2/GPIO3) for right sensor
# Set to 100kHz to match software I2C buses for consistency
dtparam=i2c_arm=on,i2c_arm_baudrate=100000

# Left sensor I2C bus (GPIO19/GPIO26)
dtoverlay=i2c-gpio,bus=7,i2c_gpio_sda=19,i2c_gpio_scl=26
```

**I2C Speed Settings:**
- **All buses set to 100kHz for consistency**
  - Hardware I2C (bus 1): Explicitly set to 100kHz via `i2c_arm_baudrate=100000`
  - Software I2C buses (3, 5, 7): Default to ~100kHz
  - Matching speeds ensures consistent timing between front sensors (one on hardware, one on software)
  - 100kHz is more than sufficient for sensor reading at 50Hz update rate
  - Both APDS9960 and MPU6050 support 100kHz (standard mode)

**Why match speeds:**
- Front sensors are read sequentially and used together for edge detection
- Consistent timing avoids potential synchronization issues
- Software I2C speed is not easily configurable, so matching hardware to software default is the practical approach

**Note**: The right sensor uses GPIO2/GPIO3, which are the hardware I2C pins. The `dtparam=i2c_arm=on` enables this bus (usually enabled by default, but explicit is better).

## After Configuration

1. Reboot the Raspberry Pi: `sudo reboot`
2. Verify the buses are available:
   ```bash
   ls /dev/i2c-*
   ```
   You should see `/dev/i2c-1` and `/dev/i2c-7`
3. Test the sensors:
   ```bash
   python -m deskinator --scan-i2c
   ```

## Configuration in Code

The bus numbers are configured in `config.py`:
- `RIGHT_SENSOR_BUS = 1` (hardware I2C)
- `LEFT_SENSOR_BUS = 7` (software I2C)

Both sensors use address `0x39` (APDS9960 default), but since they're on separate buses, there's no address conflict.

