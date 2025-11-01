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
from hw.tca9548a import TCA9548A


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
                time.sleep(0.01)
                sensor = APDS9960(self.bus, sensor_address)
                sensor.init()
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

    def read(self) -> List[Optional[float]]:
        """Read normalized proximity values for all sensors."""

        values: List[Optional[float]] = []

        for ch, sensor in zip(self.mux_channels, self.sensors):
            if sensor is None:
                values.append(None)
                continue

            try:
                self.mux.select(ch)
                time.sleep(0.002)
                reading = sensor.read_proximity_norm()
            except Exception as exc:
                print(f"Warning: read error on channel {ch}: {exc}")
                reading = None

            values.append(reading)

        self.mux.select(None)

        if self.gesture is not None:
            try:
                gesture_value = self.gesture.read_proximity_norm()
            except Exception as exc:
                print(f"Warning: gesture sensor read error: {exc}")
                gesture_value = None
        else:
            gesture_value = None

        values.append(gesture_value)
        return values

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

        self.setWindowTitle("Deskinator Proximity Viewer")
        self.resize(600, 360)

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
        self.plot.setLabel("left", "Normalized proximity", units="arb")
        self.plot.setLabel("bottom", "Sensors")
        self.plot.setTitle("Live proximity (1.0 = near)")
        layout.addWidget(self.plot, stretch=1)

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

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(int(self.interval_s * 1000))

        self.update_plot()

    def update_plot(self) -> None:
        readings = self.rig.read()

        heights: List[float] = []
        parts = []
        for label, value in zip(self.sensor_labels, readings):
            if value is None or not math.isfinite(value):
                heights.append(0.0)
                parts.append(f"{label}=--")
            else:
                clamped = max(0.0, min(1.0, float(value)))
                heights.append(clamped)
                parts.append(f"{label}={clamped:0.2f}")

        self.bars.setOpts(height=heights)
        self.status_label.setText("  |  ".join(parts))


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live proximity sensor viewer")
    parser.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="Refresh interval in seconds (default: 0.1)",
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
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    rig = ProximityRig(
        bus_number=args.bus,
        sensor_address=args.sensor_addr,
        mux_address=args.mux_addr,
        mux_channels=I2C.MUX_CHANS,
        gesture_bus=args.gesture_bus,
        gesture_address=args.gesture_addr,
    )

    app = QtWidgets.QApplication(sys.argv)
    app.aboutToQuit.connect(rig.close)

    viewer = ProximityViewer(rig, args.interval)
    viewer.show()

    return _exec_app(app)


if __name__ == "__main__":
    sys.exit(main())
