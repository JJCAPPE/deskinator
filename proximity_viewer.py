"""Minimal live viewer for Deskinator proximity sensors.

Displays the normalized distance (0.0 = far, 1.0 = near) measured by the
four front APDS9960 sensors plus the separate start-gesture sensor. Designed
to resemble the lightweight approach in ``pose_viewer.py`` while staying
simple enough to run directly on the robot.

Usage examples::

    python3 proximity_viewer.py
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
from hw.tca9548a import TCA9548A


def _parse_optional_hex(value: str) -> Optional[int]:
    value = value.strip().lower()
    if value in {"none", "null", "off", "disable", "disabled"}:
        return None
    return int(value, 0)


class ProximityRig:
    """Handle hardware access for the proximity sensors."""

    def __init__(
        self,
        bus_number: int,
        sensor_address: int,
        mux_address: int,
        mux_channels: Sequence[int],
        gesture_bus: int,
        gesture_address: int,
        imu_address: Optional[int] = None,
        imu_bus: Optional[int] = None,
    ) -> None:
        self.bus = I2CBus(bus_number)
        self.mux = TCA9548A(self.bus, mux_address)
        self.mux_channels = list(mux_channels)
        default_names = ("left_outer", "left_inner", "right_inner", "right_outer")
        self.channel_names = [
            default_names[i] if i < len(default_names) else f"sensor_{i}"
            for i, _ in enumerate(self.mux_channels)
        ]

        self.sensors: List[Optional[APDS9960]] = []
        for ch in self.mux_channels:
            sensor: Optional[APDS9960]
            try:
                self.mux.select(ch)
                # Conservative settle time after MUX channel switch (20ms)
                time.sleep(0.020)
                sensor = APDS9960(self.bus, sensor_address)
                sensor.init()
                # Brief delay after initialization
                time.sleep(0.010)
            except Exception as exc:
                print(
                    f"Warning: failed to init proximity sensor on channel {ch}: {exc}"
                )
                sensor = None
            self.sensors.append(sensor)

        self.mux.select(None)

        gesture: Optional[GestureSensor]
        try:
            gesture = GestureSensor(bus_number=gesture_bus, address=gesture_address)
        except Exception as exc:
            print(f"Warning: failed to init gesture sensor: {exc}")
            gesture = None
        self.gesture = gesture

        self._imu_warning_printed = False
        if imu_address is None:
            self.imu: Optional[MPU6050] = None
        else:
            try:
                # Use separate bus for IMU if provided, otherwise use main bus (backward compat)
                imu_bus_instance = I2CBus(imu_bus) if imu_bus is not None else self.bus
                self.imu = MPU6050(imu_bus_instance, imu_address)
            except Exception as exc:
                print(
                    f"Warning: failed to init IMU at address 0x{imu_address:02x}: {exc}"
                )
                self.imu = None
        if self.imu is not None and getattr(self.imu, "sim_mode", False):
            print("Info: IMU running in simulation mode")

    def read(self) -> tuple[List[Optional[float]], List[Optional[int]]]:
        """Read normalized proximity values and raw values for all sensors.

        Returns:
            Tuple of (normalized_values, raw_values)
        """

        values: List[Optional[float]] = []
        raw_values: List[Optional[int]] = []

        for ch, sensor in zip(self.mux_channels, self.sensors):
            if sensor is None:
                values.append(None)
                raw_values.append(None)
                continue

            try:
                self.mux.select(ch)
                # Conservative settle time after MUX channel switch (20ms)
                time.sleep(0.020)
                raw = sensor.read_proximity_raw()
                # Small delay between raw and normalized reads
                time.sleep(0.005)
                reading = sensor.read_proximity_norm()
                raw_values.append(raw)
            except Exception as exc:
                print(f"Warning: read error on channel {ch}: {exc}")
                reading = None
                raw_values.append(None)

            values.append(reading)

        self.mux.select(None)

        if self.gesture is not None:
            try:
                gesture_raw = self.gesture.read_proximity_raw()
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
            self.mux.select(None)
        except Exception:
            pass

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

        try:
            self.mux.select(None)
        except Exception:
            pass

        if hasattr(self.gesture, "bus") and getattr(self.gesture, "bus", None):
            try:
                self.gesture.bus.close()  # type: ignore[attr-defined]
            except Exception:
                pass

        if hasattr(self.bus, "close"):
            try:
                self.bus.close()
            except Exception:
                pass


class ProximityViewer(QtWidgets.QMainWindow):
    """Simple bar-chart viewer for live proximity readings."""

    def __init__(self, rig: ProximityRig, interval_s: float) -> None:
        super().__init__()

        self.rig = rig
        self.interval_s = max(0.05, interval_s)

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
        readings, raw_readings = self.rig.read()

        heights: List[float] = []
        parts = []
        for label, value, raw_value in zip(self.sensor_labels, readings, raw_readings):
            if raw_value is None or (isinstance(raw_value, float) and not math.isfinite(raw_value)):
                heights.append(0.0)
                parts.append(f"{label}=-- (raw:--)")
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
            self.imu_label.setText("IMU not configured")
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
        "--bus",
        type=int,
        default=I2C.BUS,
        help="I2C bus number for the front sensors (default from config)",
    )
    parser.add_argument(
        "--mux-addr",
        type=lambda x: int(x, 0),
        default=I2C.ADDR_MUX,
        help="TCA9548A multiplexer address (default from config)",
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
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    # Optional one-shot calibration path
    if args.calibrate:
        rig = ProximityRig(
            bus_number=args.bus,
            sensor_address=args.sensor_addr,
            mux_address=args.mux_addr,
            mux_channels=I2C.MUX_CHANS,
            gesture_bus=args.gesture_bus,
            gesture_address=args.gesture_addr,
            imu_address=args.imu_addr,
            imu_bus=I2C.IMU_BUS,
        )
        print("\nStarting APDS calibration (place over table, then off edge when prompted)...")
        for ch, sensor in zip(rig.mux_channels, rig.sensors):
            if sensor is None:
                print(f"- Channel {ch}: not present, skipping")
                continue
            try:
                rig.mux.select(ch)
                # Conservative settle time before calibration (20ms)
                time.sleep(0.020)
                print(f"\nCalibrating sensor on channel {ch}:")
                sensor.calibrate(on_table_samples=10, off_table_samples=10)
            except Exception as exc:
                print(f"  Calibration error on channel {ch}: {exc}")
        rig.mux.select(None)
        rig.close()
        print("\nCalibration complete.")
        return 0

    rig = ProximityRig(
        bus_number=args.bus,
        sensor_address=args.sensor_addr,
        mux_address=args.mux_addr,
        mux_channels=I2C.MUX_CHANS,
        gesture_bus=args.gesture_bus,
        gesture_address=args.gesture_addr,
        imu_address=args.imu_addr,
    )

    app = QtWidgets.QApplication(sys.argv)
    app.aboutToQuit.connect(rig.close)

    viewer = ProximityViewer(rig, args.interval)
    viewer.show()

    return _exec_app(app)


if __name__ == "__main__":
    sys.exit(main())
