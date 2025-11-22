"""Minimal live viewer for Deskinator proximity sensors.

Displays the normalized distance (0.0 = far, 1.0 = near) measured by the
two front APDS9960 sensors (left and right) plus the separate start-gesture sensor.
Designed to resemble the lightweight approach in ``pose_viewer.py`` while staying
simple enough to run directly on the robot.

Usage examples::

    python3 proximity_viewer.py --RLGI  # All sensors
    python3 proximity_viewer.py --RL    # Just front sensors (Right, Left)
    python3 proximity_viewer.py --I     # Just IMU
    python3 proximity_viewer.py --interval 0.2
"""

# TODO: The sensors are fucked

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import List, Optional, Sequence

import numpy as np

try:  # Prefer PyQt5 when available.
    from PyQt5 import QtCore, QtWidgets

    def _exec_app(app: QtWidgets.QApplication) -> int:
        return app.exec_()

    QT_BACKEND = "PyQt5"
except ImportError:  # Fall back to PySide6.
    from PySide6 import QtCore, QtWidgets  # type: ignore

    def _exec_app(app: QtWidgets.QApplication) -> int:  # type: ignore
        return app.exec()

    QT_BACKEND = "PySide6"

import pyqtgraph as pg

from config import I2C
from hw.apds9960 import APDS9960
from hw.gesture import GestureSensor
from hw.i2c import I2CBus
from hw.mpu6050 import MPU6050


def _parse_optional_hex(value: str) -> Optional[int]:
    value = value.strip().lower()
    if value in {"none", "null", "off", "disable", "disabled"}:
        return None
    return int(value, 0)


class ProximityRig:
    """Handle hardware access for the proximity sensors."""

    def __init__(
        self,
        left_bus: int,
        right_bus: int,
        sensor_address: int,
        gesture_bus: int,
        gesture_address: int,
        imu_address: Optional[int] = None,
        imu_bus: Optional[int] = None,
        active_sensors: str = "RLGI",
    ) -> None:
        self.channel_names = ["left", "right"]
        self.active_sensors = active_sensors.upper()

        self.sensors: List[Optional[APDS9960]] = [None, None]  # [Left, Right]

        # Left sensor
        if "L" in self.active_sensors:
            try:
                left_i2c = I2CBus(left_bus)
                print(
                    f"Initializing LEFT sensor on bus {left_bus} (I2CBus bus_number={left_i2c.bus_number})"
                )
                left_sensor = APDS9960(left_i2c, sensor_address)
                left_sensor.init()
                time.sleep(0.010)
                self.sensors[0] = left_sensor
                print(f"  LEFT sensor initialized successfully")
            except Exception as exc:
                print(
                    f"Warning: failed to init left proximity sensor on bus {left_bus}: {exc}"
                )
        else:
            print("Skipping LEFT sensor (flag L not set)")

        # Right sensor
        if "R" in self.active_sensors:
            try:
                right_i2c = I2CBus(right_bus)
                print(
                    f"Initializing RIGHT sensor on bus {right_bus} (I2CBus bus_number={right_i2c.bus_number})"
                )
                right_sensor = APDS9960(right_i2c, sensor_address)
                right_sensor.init()
                time.sleep(0.010)
                self.sensors[1] = right_sensor
                print(f"  RIGHT sensor initialized successfully")
            except Exception as exc:
                print(
                    f"Warning: failed to init right proximity sensor on bus {right_bus}: {exc}"
                )
        else:
            print("Skipping RIGHT sensor (flag R not set)")

        if (
            right_bus == gesture_bus
            and "R" in self.active_sensors
            and "G" in self.active_sensors
        ):
            print(
                f"Warning: Right sensor and Gesture sensor share bus {right_bus}. "
                "They may conflict if they have the same address!"
            )

        self.gesture: Optional[GestureSensor] = None
        if "G" in self.active_sensors:
            try:
                print(f"Initializing GESTURE sensor on bus {gesture_bus}")
                gesture = GestureSensor(bus_number=gesture_bus, address=gesture_address)
                print(
                    f"  GESTURE sensor initialized successfully (bus_number={gesture.bus_number})"
                )
                self.gesture = gesture
            except Exception as exc:
                print(f"Warning: failed to init gesture sensor: {exc}")
        else:
            print("Skipping GESTURE sensor (flag G not set)")

        self._imu_warning_printed = False
        self.imu: Optional[MPU6050] = None

        if "I" in self.active_sensors and imu_address is not None:
            try:
                # Use separate bus for IMU if provided, otherwise fallback to default bus 1
                bus_num = imu_bus if imu_bus is not None else 1
                imu_bus_instance = I2CBus(bus_num)
                self.imu = MPU6050(imu_bus_instance, imu_address)
                if getattr(self.imu, "sim_mode", False):
                    print("Info: IMU running in simulation mode")
                else:
                    print(f"  IMU initialized successfully on bus {bus_num}")
            except Exception as exc:
                print(
                    f"Warning: failed to init IMU at address 0x{imu_address:02x}: {exc}"
                )
        else:
            print("Skipping IMU (flag I not set or no address)")

    def read(self) -> tuple[List[Optional[float]], List[Optional[int]]]:
        """Read normalized proximity values and raw values for all sensors.

        Returns:
            Tuple of (normalized_values, raw_values)
        """

        values: List[Optional[float]] = []
        raw_values: List[Optional[int]] = []

        # Read left sensor (bus 7 - software I2C)
        if self.sensors[0] is not None:
            try:
                raw = self.sensors[0].read_proximity_raw()
                time.sleep(0.010)  # Delay for bus timing
                reading = self.sensors[0].read_proximity_norm()
                raw_values.append(raw)
                values.append(reading)
            except Exception as exc:
                print(f"Warning: read error on left sensor: {exc}")
                values.append(None)
                raw_values.append(None)
        else:
            values.append(None)
            raw_values.append(None)

        # Small delay when switching between buses if both active
        if self.sensors[0] is not None and self.sensors[1] is not None:
            time.sleep(0.010)

        # Read right sensor (bus 1 - hardware I2C)
        if self.sensors[1] is not None:
            try:
                raw = self.sensors[1].read_proximity_raw()
                time.sleep(0.010)  # Delay for bus timing
                reading = self.sensors[1].read_proximity_norm()
                raw_values.append(raw)
                values.append(reading)
            except Exception as exc:
                print(f"Warning: read error on right sensor: {exc}")
                values.append(None)
                raw_values.append(None)
        else:
            values.append(None)
            raw_values.append(None)

        # Small delay when switching to gesture bus
        if (
            self.sensors[0] is not None or self.sensors[1] is not None
        ) and self.gesture is not None:
            time.sleep(0.010)

        # Read gesture sensor (bus 3 - software I2C)
        if self.gesture is not None:
            try:
                gesture_raw = self.gesture.read_proximity_raw()
                time.sleep(0.010)  # Delay for bus timing
                gesture_value = self.gesture.read_proximity_norm()
                raw_values.append(gesture_raw)
            except Exception as exc:
                print(f"Warning: gesture sensor read error: {exc}")
                gesture_value = None
                raw_values.append(None)
        else:
            gesture_value = None
            raw_values.append(None)

        values.append(gesture_value)
        return values, raw_values

    def read_imu(self) -> tuple[Optional[float], Optional[float]]:
        """Read yaw (rad) and yaw rate (rad/s) from the IMU, if available."""

        if self.imu is None:
            return None, None

        try:
            yaw, yaw_rate = self.imu.read_yaw_and_rate()
        except Exception as exc:
            if not self._imu_warning_printed:
                print(f"Warning: IMU read error: {exc}")
                self._imu_warning_printed = True
            return None, None

        self._imu_warning_printed = False
        return yaw, yaw_rate

    def close(self) -> None:
        """Release I2C resources."""

        if hasattr(self.gesture, "bus") and getattr(self.gesture, "bus", None):
            try:
                self.gesture.bus.close()  # type: ignore[attr-defined]
            except Exception:
                pass

        # Close I2C buses for sensors
        for sensor in self.sensors:
            if sensor is not None and hasattr(sensor, "bus"):
                try:
                    sensor.bus.close()  # type: ignore[attr-defined]
                except Exception:
                    pass


class ProximityViewer(QtWidgets.QMainWindow):
    """Simple bar-chart viewer for live proximity readings."""

    def __init__(self, rig: ProximityRig, interval_s: float) -> None:
        super().__init__()

        self.rig = rig
        self.interval_s = max(0.01, interval_s)

        self.sensor_labels = list(rig.channel_names) + ["start_gesture"]
        self.has_imu = self.rig.imu is not None

        self.setWindowTitle("Deskinator Proximity Viewer")
        self.resize(760, 520)

        pg.setConfigOptions(antialias=True)

        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        self.setCentralWidget(central)

        info_text = f"Polling proximity sensors... (Qt backend: {QT_BACKEND})"
        self.status_label = QtWidgets.QLabel(info_text)
        self.status_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.status_label)

        self.plot = pg.PlotWidget()
        self.plot.setYRange(0.0, 1.05, padding=0)
        self.plot.showGrid(x=True, y=True, alpha=0.2)
        self.plot.setLabel("left", "Raw proximity (scaled)", units="arb")
        self.plot.setLabel("bottom", "Sensors")
        self.plot.setTitle("Live raw proximity (1.0 = near)")
        layout.addWidget(self.plot, stretch=3)

        ticks = [(i, label) for i, label in enumerate(self.sensor_labels)]
        self.plot.getAxis("bottom").setTicks([ticks])

        self.bars = pg.BarGraphItem(
            x=list(range(len(self.sensor_labels))),
            height=[0.0] * len(self.sensor_labels),
            width=0.6,
            brush=pg.mkBrush(80, 200, 255),
            pen=pg.mkPen(30, 120, 180, width=1),
        )
        self.plot.addItem(self.bars)

        imu_frame = QtWidgets.QFrame()
        imu_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        imu_layout = QtWidgets.QVBoxLayout(imu_frame)
        imu_layout.setContentsMargins(8, 8, 8, 8)
        imu_layout.setSpacing(6)

        imu_title = QtWidgets.QLabel("IMU Orientation")
        imu_title.setAlignment(QtCore.Qt.AlignCenter)
        imu_layout.addWidget(imu_title)

        self.imu_plot = pg.PlotWidget()
        self.imu_plot.setAspectLocked(True)
        self.imu_plot.setRange(xRange=(-1.1, 1.1), yRange=(-1.1, 1.1))
        self.imu_plot.showGrid(x=True, y=True, alpha=0.1)
        self.imu_plot.hideAxis("left")
        self.imu_plot.hideAxis("bottom")
        imu_layout.addWidget(self.imu_plot, stretch=1)

        theta = np.linspace(0.0, 2.0 * math.pi, 256)
        self.imu_plot.plot(
            np.cos(theta),
            np.sin(theta),
            pen=pg.mkPen(150, 150, 150, width=1),
        )
        self.imu_plot.addLine(
            x=0.0, pen=pg.mkPen(170, 170, 170, style=QtCore.Qt.DashLine)
        )
        self.imu_plot.addLine(
            y=0.0, pen=pg.mkPen(170, 170, 170, style=QtCore.Qt.DashLine)
        )

        self.imu_vector = self.imu_plot.plot(
            [0.0, 0.0],
            [0.0, 0.0],
            pen=pg.mkPen(255, 120, 0, width=4),
        )

        self.imu_label = QtWidgets.QLabel("IMU not configured")
        self.imu_label.setAlignment(QtCore.Qt.AlignCenter)
        imu_layout.addWidget(self.imu_label)

        layout.addWidget(imu_frame, stretch=2)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(int(self.interval_s * 1000))

        self.update_plot()

    def update_plot(self) -> None:
        self.readings, self.raw_readings = self.rig.read()

        heights: List[float] = []
        parts = []
        for label, value, raw_value in zip(
            self.sensor_labels, self.readings, self.raw_readings
        ):
            if raw_value is None or (
                isinstance(raw_value, float) and not math.isfinite(raw_value)
            ):
                heights.append(0.0)
                # Show -- if active but failed, or Disabled if disabled via flags
                is_active = False
                if label == "left" and "L" in self.rig.active_sensors:
                    is_active = True
                elif label == "right" and "R" in self.rig.active_sensors:
                    is_active = True
                elif label == "start_gesture" and "G" in self.rig.active_sensors:
                    is_active = True

                status_str = "ERR" if is_active else "OFF"
                parts.append(f"{label}={status_str}")
            else:
                clamped = max(0.0, min(1.0, float(raw_value) / 255.0))
                heights.append(clamped)
                parts.append(f"{label}=raw:{int(raw_value)}")

        self.bars.setOpts(height=heights)
        self.status_label.setText("  |  ".join(parts))

        self.update_imu_display()

    def update_imu_display(self) -> None:
        self.has_imu = self.rig.imu is not None
        if not self.has_imu:
            self.imu_label.setText("IMU not configured (or disabled)")
            self.imu_vector.setData([0.0, 0.0], [0.0, 0.0])
            return

        yaw, yaw_rate = self.rig.read_imu()
        if yaw is None or yaw_rate is None:
            self.imu_label.setText("IMU data unavailable")
            self.imu_vector.setData([0.0, 0.0], [0.0, 0.0])
            return

        vector_length = 0.95
        x = vector_length * math.sin(yaw)
        y = vector_length * math.cos(yaw)
        self.imu_vector.setData([0.0, x], [0.0, y])

        yaw_deg = math.degrees(yaw)
        yaw_deg = ((yaw_deg + 180.0) % 360.0) - 180.0
        yaw_rate_deg = math.degrees(yaw_rate)
        mode_text = "SIM" if getattr(self.rig.imu, "sim_mode", False) else "HW"
        self.imu_label.setText(
            f"Yaw: {yaw_deg:6.1f}°   Rate: {yaw_rate_deg:6.1f}°/s   ({mode_text})"
        )


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live proximity sensor viewer")
    parser.add_argument(
        "active_sensors",
        nargs="?",
        default="RLGI",
        help="Sensors to enable (RLGI): R=Right, L=Left, G=Gesture, I=IMU. Default: RLGI",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="Refresh interval in seconds (default: 0.1)",
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Calibrate APDS sensors (interactive) and exit",
    )
    parser.add_argument(
        "--left-bus",
        type=int,
        default=I2C.LEFT_SENSOR_BUS,
        help="I2C bus number for the left front sensor (default from config)",
    )
    parser.add_argument(
        "--right-bus",
        type=int,
        default=I2C.RIGHT_SENSOR_BUS,
        help="I2C bus number for the right front sensor (default from config)",
    )
    parser.add_argument(
        "--sensor-addr",
        type=lambda x: int(x, 0),
        default=I2C.APDS_ADDR,
        help="APDS9960 sensor address (default from config)",
    )
    parser.add_argument(
        "--gesture-bus",
        type=int,
        default=I2C.GESTURE_BUS,
        help="I2C bus for the start gesture sensor (default from config)",
    )
    parser.add_argument(
        "--gesture-addr",
        type=lambda x: int(x, 0),
        default=I2C.GESTURE_ADDR,
        help="APDS9960 gesture sensor address (default from config)",
    )
    parser.add_argument(
        "--imu-addr",
        type=_parse_optional_hex,
        default=I2C.ADDR_IMU,
        help="IMU I2C address (default from config; 'none' to disable)",
    )

    # Handle the positional arg if it looks like a flag or isn't provided
    # This is a bit of a hack to allow flags without a value if the user provides "RLG"
    args, unknown = parser.parse_known_args(argv)

    # If active_sensors starts with '-', it might have been parsed as a flag if we defined it differently.
    # But here we made it a positional arg.
    # If the user typed `python script.py --interval 0.1`, `active_sensors` takes "RLGI" (default)
    # If the user typed `python script.py RL`, `active_sensors` takes "RL"

    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    # Clean up the active_sensors string (remove dashes if user provided them like -RLGI)
    active_flags = args.active_sensors.replace("-", "")

    # Optional one-shot calibration path
    if args.calibrate:
        rig = ProximityRig(
            left_bus=args.left_bus,
            right_bus=args.right_bus,
            sensor_address=args.sensor_addr,
            gesture_bus=args.gesture_bus,
            gesture_address=args.gesture_addr,
            imu_address=args.imu_addr,
            imu_bus=I2C.IMU_BUS,
            active_sensors=active_flags,
        )
        print(
            "\nStarting APDS calibration (place over table, then off edge when prompted)..."
        )
        sensor_names = ["left", "right"]
        for name, sensor in zip(sensor_names, rig.sensors):
            if sensor is None:
                print(
                    f"- {name.capitalize()} sensor: not present (or disabled), skipping"
                )
                continue
            try:
                print(f"\nCalibrating {name} sensor:")
                sensor.calibrate(on_table_samples=10, off_table_samples=10)
            except Exception as exc:
                print(f"  Calibration error on {name} sensor: {exc}")
        rig.close()
        print("\nCalibration complete.")
        return 0

    rig = ProximityRig(
        left_bus=args.left_bus,
        right_bus=args.right_bus,
        sensor_address=args.sensor_addr,
        gesture_bus=args.gesture_bus,
        gesture_address=args.gesture_addr,
        imu_address=args.imu_addr,
        imu_bus=I2C.IMU_BUS,
        active_sensors=active_flags,
    )

    app = QtWidgets.QApplication(sys.argv)
    app.aboutToQuit.connect(rig.close)

    viewer = ProximityViewer(rig, args.interval)
    viewer.show()

    return _exec_app(app)


if __name__ == "__main__":
    sys.exit(main())
