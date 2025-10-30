# Hardware Setup Guide

Quick reference for finding I2C addresses and testing hardware wiring.

## 🔍 Step 1: Find I2C Addresses

After wiring all your I2C devices, run the scanner to find their addresses:

```bash
python3 scan_i2c.py
```

This script will:
- ✅ Scan the main I2C bus for all devices
- ✅ Identify your BNO085 IMU address
- ✅ Detect the TCA9548A multiplexer
- ✅ Scan all 8 multiplexer channels
- ✅ Find all 4 APDS9960 proximity sensors
- ✅ **Generate exact config.py values** for you to copy/paste

### Expected Devices

| Device                    | Expected Address | I2C Bus| Notes |
|---------------------------|------------------|--------|-------|
| **BNO085 IMU**            | `0x4A` or `0x4B` | Bus 1  | Depends on SA0 pin |
| **TCA9548A Multiplexer**  | `0x70`           | Bus 1  | Default address |
| **APDS9960 Sensors (4x)** | `0x39`           | Bus 1  | Each on different MUX channel |
| **APDS9960 Gesture**      | `0x39`           | Bus 3  | Separate bus (GPIO17/GPIO4) |

### Example Output

```
Main I2C Bus:
  0x4B    - BNO085 IMU
  0x70    - TCA9548A Multiplexer

Multiplexer Channels:
  Channel 0: 0x39 - APDS9960
  Channel 1: 0x39 - APDS9960
  Channel 2: 0x39 - APDS9960
  Channel 3: 0x39 - APDS9960

✓ Update config.py with:
  ADDR_IMU: int = 0x4B
  ADDR_MUX: int = 0x70
  MUX_CHANS: tuple[int, int, int, int] = (0, 1, 2, 3)
```

---

## 🧪 Step 2: Test All Hardware

After updating `config.py` with the correct addresses, test everything:

```bash
python3 test_hardware.py
```

This comprehensive test will verify:
1. ✅ **I2C Bus** - Connectivity and device detection
2. ✅ **TCA9548A Multiplexer** - All 8 channels
3. ✅ **APDS9960 Sensors** - All 4 sensors with live readings
4. ✅ **BNO085 IMU** - Yaw angle and rotation rate
5. ✅ **Stepper Motors** - Direction pins for left/right motors
6. ✅ **Vacuum Motor** - PWM at various duty cycles
7. ✅ **Buzzer** - All beep patterns

### What to Check

#### I2C Devices
- **All devices detected** at expected addresses
- **Live sensor readings** respond to movement/proximity

#### Stepper Motors
- **Direction pins toggle** - Use multimeter to verify 3.3V
- Need 12V power connected to see actual motion
- Listen for stepping sound when powered

#### Vacuum Motor
- **PWM signal** on GPIO5 - Use oscilloscope to verify
- Should hear speed changes when motor connected
- Requires 12V power and MOSFET driver

#### Buzzer
- **Different beep patterns** are audible
- Start: 2 short beeps
- Finish: 3 ascending beeps
- Error: 3 rapid beeps

---

## 🎮 Configuring the Gesture Sensor (5th APDS9960)

The gesture sensor uses a **separate I2C bus** to avoid address conflicts with the 4 proximity sensors.

### Enable Software I2C on GPIO17/GPIO4

Edit `/boot/config.txt`:
```bash
sudo nano /boot/config.txt
```

Add this line at the end:
```bash
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=17,i2c_gpio_scl=4
```

**Reboot to apply:**
```bash
sudo reboot
```

### Verify the Bus

After reboot, check that `/dev/i2c-3` exists:
```bash
ls -l /dev/i2c-*
```

You should see:
```
/dev/i2c-1  (main bus for sensors/IMU)
/dev/i2c-3  (gesture sensor on GPIO17/GPIO4)
```

### Test the Gesture Sensor

```bash
# Scan the gesture sensor bus
i2cdetect -y 3

# Should show 0x39 (APDS9960)
```

### Hardware Connection

| Gesture Sensor Pin | Raspberry Pi Pin|
|-------------------|------------------|
| VCC               | 3.3V             |
| GND               | GND              |
| SDA               | GPIO17 (Pin 11)  |
| SCL               | GPIO4 (Pin 7)    |
| INT               | Not connected    |

**Note:** Software I2C is slower than hardware I2C but works fine for gesture detection.

---

## 🔧 Alternative: Manual I2C Detection

If you prefer the traditional approach:

### Install i2c-tools
```bash
sudo apt-get install i2c-tools
```

### Scan Main Bus
```bash
i2cdetect -y 1
```

### Scan Through Multiplexer

For each channel (0-7), manually enable it then scan:

```bash
# Select channel 0
i2cset -y 1 0x70 0x01

# Scan for devices
i2cdetect -y 1

# Select channel 1
i2cset -y 1 0x70 0x02

# Scan again
i2cdetect -y 1

# ... repeat for channels 2-7 ...

# Disable all channels
i2cset -y 1 0x70 0x00
```

**Note:** The `scan_i2c.py` script does all of this automatically!

---

## 📝 Updating config.py

After scanning, update `/config.py`:

```python
@dataclass
class I2CParams:
    """I2C bus configuration and device addresses."""

    BUS: int = 1
    ADDR_IMU: int = 0x4B  # ← Update with your IMU address
    ADDR_MUX: int = 0x70  # ← Should be 0x70 (TCA9548A default)
    APDS_ADDR: int = 0x39  # ← Fixed for all APDS9960
    MUX_CHANS: tuple[int, int, int, int] = (0, 1, 2, 3)  # ← Update with your channels
    LEFT_PAIR: tuple[int, int] = (0, 1)
    RIGHT_PAIR: tuple[int, int] = (2, 3)
```

---

## 🎯 Pin Summary

### GPIO Pins (BCM Numbering)
| Component | Pin | Type |
|-----------|-----|------|
| Right Stepper STEP | GPIO22 | Digital Out |
| Right Stepper DIR | GPIO23 | Digital Out |
| Left Stepper DIR | GPIO21 | Digital Out |
| Left Stepper STEP | GPIO20 | Digital Out |
| Vacuum Motor | GPIO5 | PWM Out |
| Buzzer | GPIO12 | PWM Out |

### I2C Buses

#### Main I2C Bus (Hardware)
| Pin | Function | Connected Devices |
|-----|----------|-------------------|
| GPIO2 (Pin 3) | SDA | BNO085, TCA9548A, 4x APDS9960 (via MUX) |
| GPIO3 (Pin 5) | SCL | BNO085, TCA9548A, 4x APDS9960 (via MUX) |

#### Gesture Sensor I2C Bus (Software I2C)
| Pin | Function | Connected Devices |
|-----|----------|-------------------|
| GPIO17 (Pin 11) | SDA | APDS9960 Gesture Sensor |
| GPIO4 (Pin 7) | SCL | APDS9960 Gesture Sensor |

**Don't forget 4.7kΩ pull-up resistors on both I2C buses (SDA and SCL)!**

---

## 🐛 Troubleshooting

### No I2C Devices Found
1. Enable I2C:
   ```bash
   sudo raspi-config
   # → Interface Options → I2C → Enable
   ```
2. Check wiring: SDA, SCL, VCC (3.3V), GND
3. Verify pull-up resistors (4.7kΩ on SDA and SCL)
4. Check device power supplies

### BNO085 Not Detected
- May need RESET pin pulled HIGH via 10kΩ resistor
- Address depends on SA0 pin: LOW = 0x4A, HIGH = 0x4B
- Some modules have different default addresses (check datasheet)

### APDS9960 Not on All Channels
- Each sensor MUST be on a different multiplexer channel
- All sensors use the same address (0x39)
- Check VCC (3.3V) and GND on each sensor

### Stepper Motors Not Moving
- Check 12V power supply to motor drivers
- Verify ENABLE pin is pulled LOW (if present)
- Listen for coil energizing sound
- Check STEP/DIR connections

### Vacuum Motor Not Running
- Requires 12V power supply
- MOSFET gate should be connected to PWM pin
- Motor connects to drain of MOSFET
- Check MOSFET heatsinking

---

## 📚 Next Steps

After hardware is verified:
1. ✅ Run `scan_i2c.py` to find addresses
2. ✅ Update `config.py` with detected addresses
3. ✅ Run `test_hardware.py` to verify everything works
4. ✅ Proceed to testing the main robot code!

Good luck with your build! 🤖

