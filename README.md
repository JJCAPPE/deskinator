# Deskinator - Autonomous Desk Cleaning Robot

An autonomous desk-cleaning robot that uses SLAM (Simultaneous Localization and Mapping) to map and clean flat tabletops. The robot employs edge detection for safety, builds a pose graph for localization, infers rectangular boundaries, and executes boustrophedon (lawn-mower) coverage patterns.

## Hardware

- **Compute**: Raspberry Pi 4B/5 (5V via XL4015 buck converter)
- **Drive**: 2× NEMA17 stepper motors with TMC2209 drivers
- **IMU**: BNO085 9-DOF for yaw sensing (I²C)
- **Proximity**: 4× APDS-9960 sensors via TCA9548A I²C multiplexer
- **Vacuum**: 12V BLDC fan (Neato-compatible)
- **Power**: 4S Li-ion battery (14.8V nominal)

## Software Architecture

### Modules

```
deskinator/
├── config.py          # Central configuration (pins, parameters)
├── hw/                # Hardware abstraction layer
│   ├── gpio.py        # GPIO manager
│   ├── stepper.py     # Stepper motor control
│   ├── vacuum.py      # Vacuum fan control
│   ├── i2c.py         # I²C bus utilities
│   ├── tca9548a.py    # I²C multiplexer driver
│   ├── apds9960.py    # Proximity sensor driver
│   └── bno085.py      # IMU driver
├── slam/              # SLAM algorithms
│   ├── ekf.py         # Extended Kalman Filter
│   ├── posegraph.py   # Pose graph optimization
│   ├── rect_fit.py    # Rectangle boundary inference
│   └── frames.py      # SE(2) transformations
├── planning/          # Path planning
│   ├── coverage.py    # Boustrophedon coverage planner
│   └── map2d.py       # 2D swept coverage map
├── control/           # Control algorithms
│   ├── fsm.py         # Supervisor state machine
│   ├── motion.py      # Motion controllers
│   └── limits.py      # Velocity/acceleration limits
├── utils/             # Utilities
│   ├── logs.py        # CSV telemetry logging
│   ├── viz.py         # Real-time visualization
│   └── timing.py      # Loop timing utilities
└── main.py            # Main orchestration (asyncio)
```

## Installation

### On Raspberry Pi

```bash
# Clone repository
cd deskinator/code

# Install in development mode
pip install -e ".[rpi]"
```

### For Development (without RPi hardware)

```bash
pip install -e .
```

The code includes simulation modes for GPIO and I²C when hardware is not available.

## Usage

### I²C Device Scanning

```bash
deskinator --scan-i2c
```

### Sensor Calibration

Before first use, calibrate the proximity sensors and IMU:

```bash
deskinator --calibrate
```

Follow the on-screen prompts to:
1. Place sensors over table surface
2. Move sensors off table edge
3. Keep robot stationary for IMU bias calibration

### Run Robot

```bash
# Run with live visualization
deskinator --viz

# Run without visualization
deskinator
```

The robot will:
1. Wait for start signal (auto-starts in this version)
2. Discover boundary by moving forward and detecting edges
3. Fit a rectangle to the observed boundary
4. Generate coverage lanes
5. Execute cleaning coverage
6. Stop when 97% coverage is reached or timeout occurs

### Logs

Telemetry and edge detections are logged to `logs/` directory with timestamps:
- `telemetry_YYYYMMDD_HHMMSS.csv` - pose, velocities, sensor readings
- `edges_YYYYMMDD_HHMMSS.csv` - edge detection points

## Configuration

Edit `deskinator/config.py` to adjust:

### GPIO Pins
```python
PINS.RIGHT_STEP = 22
PINS.LEFT_STEP = 20
# ... etc
```

### I²C Addresses
```python
I2C.ADDR_MUX = 0x70      # TCA9548A multiplexer
I2C.APDS_ADDR = 0x39     # APDS9960 sensors
```

### Robot Geometry
```python
GEOM.WHEEL_BASE = 0.170       # m
GEOM.STEPS_PER_M = 6400.0     # steps/meter
GEOM.VAC_WIDTH = 0.160        # m
```

### Motion Limits
```python
LIMS.V_MAX = 0.18        # m/s
LIMS.OMEGA_MAX = 1.8     # rad/s
LIMS.A_MAX = 0.60        # m/s²
```

### Algorithm Parameters
```python
ALG.EDGE_THRESH = 0.5         # Proximity threshold for edge
ALG.EDGE_DEBOUNCE = 0.06      # s
ALG.NODE_SPACING = 0.05       # m (pose graph nodes)
ALG.SWEEP_OVERLAP = 0.02      # m
```

## Safety Features

1. **Edge Detection**: Dual-sensor confirmation with debouncing prevents false triggers
2. **Emergency Brake**: Edge events trigger immediate braking
3. **Backoff Sequence**: 3cm reverse after edge detection
4. **Velocity Limits**: Acceleration and jerk limits ensure safe motion
5. **Timeouts**: 2-minute boundary discovery, 3-minute coverage timeouts

## Algorithms

### SLAM
- **EKF**: Fuses wheel odometry with IMU gyro for pose estimation
- **Pose Graph**: Nodes every 5cm with odometry and loop-closure constraints
- **Optimization**: SciPy least-squares (Levenberg-Marquardt)

### Rectangle Inference
- **RANSAC Line Fitting**: Extracts 4 dominant lines from edge points
- **Orthogonality Check**: Validates ±8° tolerance
- **Confidence**: Requires 0.6m of perimeter observed

### Coverage Planning
- **Boustrophedon Pattern**: Alternating parallel lanes
- **Lane Orientation**: Aligned to longest desk dimension
- **Overlap**: 2cm overlap between lanes for complete coverage

## Testing

### Bench Tests
```bash
# 1. I²C scan
deskinator --scan-i2c

# 2. Sensor calibration
deskinator --calibrate

# 3. Check GPIO
python -c "from deskinator.hw.gpio import gpio_manager; gpio_manager.setup(); print('OK')"
```

### Validation Criteria
- Rectangle orthogonality ≤ 8°
- Pose drift < 3cm after boundary loop (with optimization)
- Coverage ≥ 97% of inferred rectangle
- No forward overrun (edge safety)

## Troubleshooting

### I²C Devices Not Found
```bash
# Check I²C is enabled
sudo raspi-config
# Interface Options → I²C → Enable

# Scan manually
i2cdetect -y 1
```

### Steppers Not Moving
- Check TMC2209 VM power (should be 12-16V from 4S battery)
- Verify DIR/STEP pin connections
- Check motor current setting (should be ~1A for NEMA17)

### Sensors Give Wrong Readings
- Re-run calibration: `deskinator --calibrate`
- Check sensor mounting (should face downward)
- Verify I²C multiplexer channels match `I2C.MUX_CHANS`

## Architecture Notes

- **Asyncio**: Three concurrent loops (sense, map, control) at 50/20/50 Hz
- **Simulation Mode**: Automatically activates when hardware unavailable
- **Modular Design**: Each subsystem (HW, SLAM, planning, control) is independently testable

## Future Enhancements

- [ ] Web UI for remote monitoring/control
- [ ] Hall encoders for improved odometry
- [ ] Occupancy grid mapping
- [ ] Multi-room support
- [ ] Battery voltage monitoring

## License

Educational project for EK210 - Engineering Design

## References

- TMC2209 Datasheet: https://www.trinamic.com/products/integrated-circuits/details/tmc2209-la/
- APDS-9960: https://www.broadcom.com/products/optical-sensors/integrated-ambient-light-and-proximity-sensors/apds-9960
- BNO085: https://www.ceva-ip.com/product/bno080-085/
- Pose Graph SLAM: Grisetti et al., "A Tutorial on Graph-Based SLAM" (2010)

