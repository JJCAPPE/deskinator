"""
Configuration module for Deskinator.

All hardware pins, I2C addresses, geometry, limits, and algorithm parameters.
"""

from dataclasses import dataclass


@dataclass
class Pins:
    """GPIO pin assignments for Raspberry Pi."""

    RIGHT_STEP: int = 22  # Driver Right Step GPIO22
    RIGHT_DIR: int = 23  # Driver Right Dir  GPIO23
    LEFT_DIR: int = 21  # Driver Left  Dir  GPIO21
    LEFT_STEP: int = 20  # Driver Left  Step GPIO20
    VACUUM_PWM: int = 5  # Vacuum MOSFET     GPIO5
    START_IN: int | None = None  # Optional start/gesture input


@dataclass
class I2CParams:
    """I2C bus configuration and device addresses."""

    BUS: int = 1
    ADDR_IMU: int | None = None  # set via i2cdetect
    ADDR_MUX: int = 0x70  # TCA9548A default
    APDS_ADDR: int = 0x39  # APDS9960 default
    MUX_CHANS: tuple[int, int, int, int] = (0, 1, 2, 3)
    LEFT_PAIR: tuple[int, int] = (0, 1)  # define exact channels later
    RIGHT_PAIR: tuple[int, int] = (2, 3)


@dataclass
class Geometry:
    """Robot physical dimensions and sensor positions.

    Robot frame convention: +x forward, +y left, origin at axle midpoint.
    """

    WHEEL_BASE: float = 0.170  # m (update from CAD)
    STEPS_PER_M: float = 6400.0  # set from microstepping/drive
    SENSOR_FWD: float = 0.080  # m; sensors lead axle
    SENSOR_LAT: tuple[float, ...] = (+0.040, +0.015, -0.015, -0.040)  # m
    VAC_WIDTH: float = 0.160  # m; effective cleaned width
    SENSOR_TO_VAC: float = 0.015  # m; vacuum ahead of sensors


@dataclass
class Limits:
    """Motion limits for NEMA17 + TMC2209 @ 12V, 1A — desk-safe."""

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
    NODE_SPACING: float = 0.05  # m
    LOOP_RADIUS: float = 0.06  # m
    SWEEP_OVERLAP: float = 0.02  # m
    GRID_RES: float = 0.02  # m (2 cm raster)
    RECT_CONF_MIN_LEN: float = 0.6  # m of perimeter observed
    RECT_ORTHTOL_DEG: float = 8.0  # deg
    STOP_DELAY_AFTER_EDGE: float = 0.00  # s
    POST_EDGE_BACKOFF: float = 0.03  # m
    POST_EDGE_SIDE_STEP: float = 0.05  # m


# Global configuration instances
PINS = Pins()
I2C = I2CParams()
GEOM = Geometry()
LIMS = Limits()
ALG = Algo()
