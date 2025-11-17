# Quick Start Guide - Running Deskinator on Raspberry Pi

## Prerequisites

1. **Hardware Setup**: All hardware must be wired and connected (see `HARDWARE_SETUP.md`)
2. **Python 3.10+**: Raspberry Pi OS typically includes Python 3.11+
3. **Dependencies**: Install required packages

## Initial Setup (First Time Only)

### 1. Install Dependencies

```bash
# Install system dependencies
sudo apt-get update
sudo apt-get install -y python3-pip python3-venv i2c-tools

# Enable I2C (if not already enabled)
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable

# Install Python packages
pip3 install -r requirements.txt

# Or install as a package (optional)
pip3 install -e .
```

### 2. Configure I2C Addresses

First, scan for I2C devices to find their addresses:

```bash
python3 scan_i2c.py
```

This will show you all detected devices and suggest values for `config.py`. Update `config.py` with the detected addresses if they differ from defaults.

### 3. Test Hardware (Recommended)

Before running the full cleaning cycle, test all hardware:

```bash
python3 test_hardware.py
```

This verifies:
- I2C bus connectivity
- Multiplexer channels
- Proximity sensors
- IMU
- Stepper motors
- Vacuum fan
- Buzzer

### 4. Calibrate Sensors (Recommended)

Calibrate the IMU and proximity sensors for best performance:

```bash
python3 -m main --calibrate
```

This will:
- Calibrate IMU bias (takes ~2 seconds)
- Calibrate each proximity sensor (takes ~5 seconds per sensor)

**Note**: For proximity sensor calibration, you'll need to:
- Place sensors on the table (for "on-table" samples)
- Lift sensors off the table (for "off-table" samples)

## Running the Cleaning Process

### Basic Usage

The code uses relative imports, so it must be run as a Python module:

```bash
# From the project root directory
python3 -m main
```

**OR** install as a package first, then run:

```bash
# Install the package (from project root)
pip3 install -e .

# Then run using the console script
deskinator
```

**Note**: Running `python3 main.py` directly will fail due to relative imports. Always use `python3 -m main` or install the package.

### What Happens

1. **Initialization**: Robot initializes all hardware (steppers, vacuum, sensors, IMU, etc.)
2. **Wait for Gesture**: Robot waits for you to place your hand near the gesture sensor
3. **Start Beep**: Robot plays 3 beeps to indicate start
4. **Vacuum Starts**: Vacuum fan turns on at 80% duty cycle
5. **Boundary Discovery**: Robot explores the table edge to map the boundary
6. **Coverage**: Robot follows boustrophedon (lawn-mower) pattern to clean
7. **Completion**: Robot plays 2 beeps and stops when done

### Command-Line Options

```bash
# Basic run (default)
python3 -m main

# Enable real-time visualization
python3 -m main --viz

# Scan I2C bus only (doesn't start cleaning)
python3 -m main --scan-i2c

# Run calibration only (doesn't start cleaning)
python3 -m main --calibrate

# Combine options
python3 -m main --viz --calibrate

# Or if installed as package:
deskinator --viz
deskinator --calibrate
```

### Stopping the Robot

- **Normal Stop**: Press `Ctrl+C` - robot will safely shutdown (stops motors, turns off vacuum, saves logs)
- **Emergency**: If needed, press `Ctrl+C` multiple times or disconnect power

## Troubleshooting

### Robot Doesn't Start

- Check that gesture sensor is connected and working
- Verify I2C devices are detected: `python3 scan_i2c.py`
- Check hardware connections: `python3 test_hardware.py`

### Robot Doesn't Detect Edges

- Calibrate sensors: `python3 main.py --calibrate`
- Check sensor readings: `python3 proximity_viewer.py`
- Verify sensor positions and wiring

### Robot Gets Stuck

- Robot has watchdog recovery that triggers after 0.6 seconds without progress
- Check logs in `logs/` directory for telemetry data
- View trajectory: `python3 pose_viewer.py`

## Logs and Data

All telemetry is logged to CSV files in the `logs/` directory:

- `telemetry_*.csv`: Full robot state (pose, velocity, sensors, FSM state)
- `edges_*.csv`: Edge detection events

View logs in real-time:

```bash
# View pose trajectory
python3 pose_viewer.py

# View proximity sensors
python3 proximity_viewer.py
```

## Running as a Service (Optional)

To run Deskinator automatically on boot, create a systemd service:

```bash
sudo nano /etc/systemd/system/deskinator.service
```

Add (adjust paths as needed):

```ini
[Unit]
Description=Deskinator Cleaning Robot
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/deskinator-code
ExecStart=/usr/bin/python3 -m main
Restart=on-failure
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

**OR** if installed as a package:

```ini
[Unit]
Description=Deskinator Cleaning Robot
After=network.target

[Service]
Type=simple
User=pi
ExecStart=/usr/local/bin/deskinator
Restart=on-failure
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

Then:

```bash
sudo systemctl enable deskinator
sudo systemctl start deskinator
sudo systemctl status deskinator
```

## Safety Notes

⚠️ **Important**:
- Always test on a small area first
- Ensure table surface is clear of obstacles
- Keep hands away from robot during operation
- Monitor first few runs closely
- Robot will stop automatically if it detects edges or timeouts occur

## Next Steps

- Review `config.py` to adjust motion limits, sensor thresholds, and algorithm parameters
- Check `HARDWARE_SETUP.md` for wiring details
- See `README.md` for detailed algorithm descriptions

