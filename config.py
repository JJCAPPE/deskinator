"""
Configuration module for Deskinator.

All hardware pins, I2C addresses, geometry, limits, and algorithm parameters.
"""

from dataclasses import dataclass


@dataclass
class Pins:
    """GPIO pin assignments for Raspberry Pi."""

    RIGHT_STEP: int = 20  # Driver Right Step GPIO20
    RIGHT_DIR: int = 21  # Driver Right Dir  GPIO21
    RIGHT_ENABLE: int = 16  # Driver Right Enable GPIO16
    LEFT_DIR: int = 23  # Driver Left  Dir  GPIO23
    LEFT_STEP: int = 22  # Driver Left  Step GPIO22
    LEFT_ENABLE: int = 27  # Driver Left  Enable GPIO27
    VACUUM_PWM: int = 5  # Vacuum MOSFET     GPIO5
    BUZZER: int = 12  # Buzzer/Beeper      GPIO12
    # Gesture sensor I2C (separate bus or software I2C)
    GESTURE_SDA: int = 15  # Gesture APDS9960 SDA GPIO17
    GESTURE_SCL: int = 14  # Gesture APDS9960 SCL GPIO4


@dataclass
class I2CParams:
    """I2C bus configuration and device addresses."""

    BUS: int = 1  # Main I2C bus (hardware I2C on GPIO2/GPIO3)
    IMU_BUS: int = 5  # I2C bus for IMU (software I2C on GPIO 6/13)
    GESTURE_BUS: int = 3  # I2C bus for gesture sensor (software I2C on GPIO 15/14)
    RIGHT_SENSOR_BUS: int = 1  # Right APDS9960 sensor (hardware I2C on GPIO2/GPIO3)
    LEFT_SENSOR_BUS: int = 7  # Left APDS9960 sensor (software I2C on GPIO19/GPIO26)
    ADDR_IMU: int | None = 0x68  # MPU-6050 detected (AD0 low)
    APDS_ADDR: int = 0x39  # APDS9960 default
    GESTURE_ADDR: int = 0x39  # Gesture APDS9960 (on separate bus)
    # Sensor indices: 0 = left sensor, 1 = right sensor
    LEFT_SENSOR_IDX: int = 0
    RIGHT_SENSOR_IDX: int = 1


@dataclass
class Geometry:
    """Robot physical dimensions and sensor positions.

    Robot frame convention: +x forward, +y left, origin at axle midpoint.
    """

    WHEEL_BASE: float = 0.170  # m (update from CAD)
    STEPS_PER_M: float = (
        1212.1  # calibrated from physical test: 0.33m actual / 0.0625m odometry = 5.28x correction
    )
    SENSOR_FWD: float = 0.080  # m; sensors lead axle
    SENSOR_LAT: tuple[float, ...] = (+0.08484, +0.05444, -0.0544, -0.08484)  # m
    VAC_WIDTH: float = 0.198  # m; effective cleaned width
    SENSOR_TO_VAC: float = 0.03949  # m; vacuum ahead of sensors


@dataclass
class Limits:
    """Motion limits for NEMA17 + A4988 @ 12V, 1A — desk-safe."""

    V_MAX: float = 0.18  # m/s
    OMEGA_MAX: float = 1.8  # rad/s
    A_MAX: float = 0.60  # m/s^2
    ALPHA_MAX: float = 4.0  # rad/s^2
    J_MAX: float = 4.0  # m/s^3
    V_REV_MAX: float = 0.06  # m/s (reverse is minimal)


@dataclass
class Algo:
    """Algorithm parameters for SLAM, edge detection, and coverage."""

    FUSE_HZ: int = 50  # EKF update rate
    EDGE_THRESH: float = 0.5  # APDS off-table threshold (norm.)
    EDGE_DEBOUNCE: float = 0.06  # s
    EDGE_EWMA_ALPHA: float = 0.35  # smoothing factor for proximity filtering
    EDGE_RAW_HYST: float = 1.15  # multiplier for raw readings hysteresis
    NODE_SPACING: float = 0.05  # m
    LOOP_RADIUS: float = 0.06  # m
    SWEEP_OVERLAP: float = 0.02  # m
    GRID_RES: float = 0.02  # m (2 cm raster)
    RECT_CONF_MIN_LEN: float = 0.6  # m of perimeter observed
    RECT_ORTHTOL_DEG: float = 8.0  # deg
    STOP_DELAY_AFTER_EDGE: float = 0.00  # s
    POST_EDGE_BACKOFF: float = 0.03  # m
    POST_EDGE_SIDE_STEP: float = 0.05  # m
    BOUNDARY_SPEED: float = 0.10  # m/s baseline during boundary sweep
    BOUNDARY_MIN_SPEED: float = 0.04  # m/s minimum crawl near edge
    BOUNDARY_EDGE_GAIN: float = 2.2  # rad/s per normalized edge delta
    BOUNDARY_SWAY_GAIN: float = 0.35  # rad/s sinusoidal dither gain
    BOUNDARY_SWAY_RATE: float = 0.7  # Hz equivalent angular rate for dither
    WATCHDOG_TIMEOUT: float = 0.6  # s without progress before recovery
    WATCHDOG_COOLDOWN: float = 2.0  # s between recovery attempts
    RECOVERY_BACKOFF: float = 0.04  # m reverse distance during recovery
    RECOVERY_TURN_ANGLE: float = 0.5  # rad turn away during recovery


PINS = Pins()
I2C = I2CParams()
GEOM = Geometry()
LIMS = Limits()
ALG = Algo()
