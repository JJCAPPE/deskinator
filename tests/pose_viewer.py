"""Lightweight PyQtGraph viewer for Deskinator pose telemetry.

Run this on the Raspberry Pi (or any host that can reach the log files)
while the main robot program is active. It tails the most recent
``telemetry_*.csv`` file produced by ``TelemetryLogger`` and displays a
2D trajectory with start and current pose markers. Designed to be
minimal so it performs well over VNC.

Usage examples::

    # Install dependencies first (once):
    #   sudo apt install python3-pyqt5
    #   python3 -m pip install pyqtgraph

    # Then run the viewer (will wait for a telemetry file):
    python3 pose_viewer.py

    # Or point to a specific file / adjust refresh:
    python3 pose_viewer.py --file logs/telemetry_20250101_120000.csv --interval 0.2
"""

from __future__ import annotations

import argparse
import csv
import sys
import math
from pathlib import Path
from typing import Any, Dict, List, Optional

try:  # Prefer PyQt5 on Raspberry Pi for availability.
    from PyQt5 import QtCore, QtWidgets

    def _exec_app(app: QtWidgets.QApplication) -> int:
        return app.exec_()

    QT_BACKEND = "PyQt5"
except ImportError:  # Fallback to PySide6 if installed instead.
    from PySide6 import QtCore, QtWidgets  # type: ignore

    def _exec_app(app: QtWidgets.QApplication) -> int:  # type: ignore
        return app.exec()

    QT_BACKEND = "PySide6"

import pyqtgraph as pg


class TelemetryTail:
    """Incrementally read pose samples from the newest telemetry log."""

    def __init__(
        self,
        log_dir: Path,
        file_path: Optional[Path] = None,
        max_samples: int = 2500,
    ) -> None:
        self.log_dir = log_dir
        self.file_path = file_path
        self.max_samples = max_samples

        self._file = None
        self._fieldnames: List[str] = []
        self.samples: List[Dict[str, Any]] = []

    def _find_latest_file(self) -> Optional[Path]:
        if not self.log_dir.exists():
            return None

        matches = sorted(
            self.log_dir.glob("telemetry_*.csv"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )

        return matches[0] if matches else None

    def _ensure_open(self) -> bool:
        if self._file and not self._file.closed:
            return True

        if self.file_path is None:
            self.file_path = self._find_latest_file()

        if not self.file_path or not self.file_path.exists():
            return False

        self._file = self.file_path.open("r", newline="")
        reader = csv.reader(self._file)

        try:
            self._fieldnames = next(reader)
        except StopIteration:
            self._fieldnames = []
            return True

        for row in reader:
            self._process_row(row)

        return True

    def _process_row(self, row: List[str]) -> None:
        if not self._fieldnames:
            return

        record = dict(zip(self._fieldnames, row))

        try:
            x = float(record.get("x", "nan"))
            y = float(record.get("y", "nan"))
        except ValueError:
            return

        if not (math.isfinite(x) and math.isfinite(y)):
            return

        edge_flag = False
        edge_raw = record.get("edge_event")
        if edge_raw not in (None, "", "nan"):
            try:
                edge_flag = bool(int(float(edge_raw)))
            except ValueError:
                edge_flag = False

        sample = {
            "x": x,
            "y": y,
            "timestamp": _as_float(record.get("timestamp")),
            "theta": _as_float(record.get("theta")),
            "state": record.get("state", ""),
            "edge_event": edge_flag,
        }

        self.samples.append(sample)
        if len(self.samples) > self.max_samples:
            self.samples = self.samples[-self.max_samples :]

    def poll(self) -> bool:
        """Read any newly appended lines. Returns True when data changed."""

        before_count = len(self.samples)

        if not self._ensure_open():
            return False

        if not self._file:
            return False

        while True:
            pos = self._file.tell()
            line = self._file.readline()
            if not line:
                self._file.seek(pos)
                break

            line = line.strip()
            if not line:
                continue

            parsed = next(csv.reader([line]))
            self._process_row(parsed)
        return len(self.samples) != before_count


class PoseViewer(QtWidgets.QMainWindow):
    """Main window for the live pose visualization."""

    def __init__(self, tail: TelemetryTail, interval_s: float) -> None:
        super().__init__()

        self.tail = tail
        self.interval_s = max(0.05, interval_s)

        self.setWindowTitle("Deskinator Pose Viewer")
        self.resize(640, 640)

        pg.setConfigOptions(antialias=True)

        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        self.setCentralWidget(central)

        wait_text = f"Waiting for telemetry... (Qt backend: {QT_BACKEND})"
        self.status_label = QtWidgets.QLabel(wait_text)
        self.status_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.status_label)

        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.2)
        self.plot.setAspectLocked(True)
        self.plot.setLabel("left", "Y", units="m")
        self.plot.setLabel("bottom", "X", units="m")
        self.plot.setTitle("Deskinator pose graph (live)")
        layout.addWidget(self.plot, stretch=1)

        pen = pg.mkPen(color=(80, 200, 255), width=2)
        self.curve = self.plot.plot([], [], pen=pen)

        self.start_marker = pg.ScatterPlotItem(
            size=12, brush=pg.mkBrush(50, 220, 70), pen=pg.mkPen(None)
        )
        self.end_marker = pg.ScatterPlotItem(
            size=14,
            brush=pg.mkBrush(255, 80, 80),
            pen=pg.mkPen(255, 255, 255, 120),
            symbol="o",
        )

        self.plot.addItem(self.start_marker)
        self.plot.addItem(self.end_marker)

        self.edge_points = pg.ScatterPlotItem(
            size=10,
            brush=pg.mkBrush(255, 200, 0, 160),
            pen=pg.mkPen(None),
            symbol="t",
        )
        self.plot.addItem(self.edge_points)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(int(self.interval_s * 1000))

        # Perform an initial refresh so we display data immediately if available.
        self.update_plot()

    def update_plot(self) -> None:
        changed = self.tail.poll()

        if not self.tail.samples:
            if self.tail.file_path:
                self.status_label.setText(
                    f"Waiting for telemetry file: {self.tail.file_path}"
                )
            else:
                self.status_label.setText(
                    f"Looking for telemetry_*.csv in {self.tail.log_dir}"
                )
            return

        if not changed:
            # Still update label with most recent state even if no new points.
            last = self.tail.samples[-1]
            self.status_label.setText(_format_status(last, self.tail.file_path))
            return

        xs = [sample["x"] for sample in self.tail.samples]
        ys = [sample["y"] for sample in self.tail.samples]

        self.curve.setData(xs, ys)

        self.start_marker.setData(xs[:1], ys[:1])
        self.end_marker.setData(xs[-1:], ys[-1:])

        edge_points = [s for s in self.tail.samples if s.get("edge_event")]
        if edge_points:
            ex = [s["x"] for s in edge_points]
            ey = [s["y"] for s in edge_points]
            self.edge_points.setData(ex, ey)
        else:
            self.edge_points.setData([], [])

        self.status_label.setText(
            _format_status(self.tail.samples[-1], self.tail.file_path)
        )

        # Keep the viewport slightly padded around the path for clarity.
        padding = 0.05
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)

        if xmin == xmax:
            xmin -= padding
            xmax += padding
        if ymin == ymax:
            ymin -= padding
            ymax += padding

        self.plot.setXRange(xmin - padding, xmax + padding, padding=0)
        self.plot.setYRange(ymin - padding, ymax + padding, padding=0)


def _as_float(value: Optional[str]) -> Optional[float]:
    if value in (None, "", "nan"):
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _format_status(sample: Dict[str, Any], path: Optional[Path]) -> str:
    timestamp = sample.get("timestamp")
    state = sample.get("state") or ""

    ts_text = f"t={timestamp:.3f}s" if isinstance(timestamp, float) else "t=--"
    state_text = f"state={state}" if state else "state=--"

    source = path.name if path else "(no file)"
    return f"{ts_text}  |  {state_text}  |  source={source}"


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live PyQtGraph pose viewer")
    parser.add_argument(
        "--log-dir",
        default="logs",
        type=Path,
        help="Directory containing telemetry_*.csv files (default: logs)",
    )
    parser.add_argument(
        "--file",
        type=Path,
        help="Specific telemetry CSV file to tail. By default the newest file in --log-dir is used.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="Refresh interval in seconds (default: 0.1)",
    )
    parser.add_argument(
        "--max-samples",
        type=int,
        default=2500,
        help="Maximum number of samples to retain in memory (default: 2500)",
    )

    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)

    tail = TelemetryTail(args.log_dir, args.file, max_samples=args.max_samples)

    app = QtWidgets.QApplication(sys.argv)
    viewer = PoseViewer(tail, args.interval)
    viewer.show()

    return _exec_app(app)


if __name__ == "__main__":
    sys.exit(main())
