import sys
import time
import numpy as np
from typing import List, Optional

# Try to import PyQt/PySide for GUI
try:
    from PyQt5 import QtCore, QtWidgets

    QT_BACKEND = "PyQt5"

    def _exec_app(app: QtWidgets.QApplication) -> int:
        return app.exec_()

except ImportError:
    try:
        from PySide6 import QtCore, QtWidgets

        QT_BACKEND = "PySide6"

        def _exec_app(app: QtWidgets.QApplication) -> int:
            return app.exec()

    except ImportError:
        print("Error: PyQt5 or PySide6 is required for the viewer.")
        sys.exit(1)

import pyqtgraph as pg

try:
    from adafruit_extended_bus import ExtendedI2C as I2C
except ImportError:
    print("Error: adafruit-circuitpython-extended-bus is not installed.")
    sys.exit(1)

try:
    from adafruit_apds9960.apds9960 import APDS9960
except ImportError:
    print("Error: adafruit-circuitpython-apds9960 is not installed.")
    sys.exit(1)


class AdafruitProximityRig:
    def __init__(self):
        self.channel_names = ["Left (Bus 7)", "Right (Bus 3)"]
        self.sensors: List[Optional[APDS9960]] = []

        # Initialize Left Sensor (Bus 7)
        try:
            print("Initializing Left Sensor on Bus 7...")
            i2c7 = I2C(7)
            left_sensor = APDS9960(i2c7)
            left_sensor.enable_proximity = True
            self.sensors.append(left_sensor)
            print("✓ Left Sensor Initialized")
        except Exception as e:
            print(f"✗ Failed Left Sensor: {e}")
            self.sensors.append(None)

        # Initialize Right Sensor (Bus 3)
        try:
            print("Initializing Right Sensor on Bus 3...")
            i2c3 = I2C(3)
            right_sensor = APDS9960(i2c3)
            right_sensor.enable_proximity = True
            self.sensors.append(right_sensor)
            print("✓ Right Sensor Initialized")
        except Exception as e:
            print(f"✗ Failed Right Sensor: {e}")
            self.sensors.append(None)

    def read(self) -> tuple[List[Optional[float]], List[Optional[int]]]:
        norm_values = []
        raw_values = []

        for sensor in self.sensors:
            if sensor is None:
                norm_values.append(None)
                raw_values.append(None)
                continue

            try:
                raw = int(sensor.proximity)
                # Normalize 0-255 to 0.0-1.0
                norm = float(np.clip(raw / 255.0, 0.0, 1.0))
                raw_values.append(raw)
                norm_values.append(norm)
            except Exception as e:
                print(f"Read Error: {e}")
                norm_values.append(None)
                raw_values.append(None)

        return norm_values, raw_values

    def close(self):
        pass


class ProximityViewer(QtWidgets.QMainWindow):
    """Simple bar-chart viewer for live proximity readings (Adafruit Backend)."""

    def __init__(self, rig: AdafruitProximityRig, interval_s: float = 0.1) -> None:
        super().__init__()

        self.rig = rig
        self.interval_s = max(0.05, interval_s)
        self.sensor_labels = rig.channel_names

        self.setWindowTitle("Deskinator Proximity Viewer (Adafruit Lib)")
        self.resize(600, 400)

        pg.setConfigOptions(antialias=True)

        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        self.setCentralWidget(central)

        info_text = f"Polling sensors via Adafruit Lib... (Qt backend: {QT_BACKEND})"
        self.status_label = QtWidgets.QLabel(info_text)
        self.status_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.status_label)

        self.plot = pg.PlotWidget()
        self.plot.setYRange(0.0, 1.05, padding=0)
        self.plot.showGrid(x=True, y=True, alpha=0.2)
        self.plot.setLabel("left", "Proximity (Normalized)", units="")
        self.plot.setLabel("bottom", "Sensors")
        self.plot.setTitle("Live Proximity (1.0 = Near)")
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
        readings, raw_readings = self.rig.read()

        heights: List[float] = []
        parts = []
        for label, value, raw_value in zip(self.sensor_labels, readings, raw_readings):
            if raw_value is None:
                heights.append(0.0)
                parts.append(f"{label}: ERR")
            else:
                heights.append(value)
                parts.append(f"{label}: {int(raw_value)}")

        self.bars.setOpts(height=heights)
        self.status_label.setText("  |  ".join(parts))


def main():
    app = QtWidgets.QApplication(sys.argv)

    print("Initializing Adafruit Proximity Rig...")
    rig = AdafruitProximityRig()

    viewer = ProximityViewer(rig)
    viewer.show()

    app.aboutToQuit.connect(rig.close)
    return _exec_app(app)


if __name__ == "__main__":
    sys.exit(main())
