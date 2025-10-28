"""
Data logging utilities.

Logs telemetry to CSV files for offline analysis.
"""

import csv
import time
from pathlib import Path
from typing import List, Dict, Any


class CSVLogger:
    """Logs data to CSV file."""

    def __init__(self, filename: str, fields: List[str]):
        """
        Initialize CSV logger.

        Args:
            filename: Output filename
            fields: List of field names
        """
        self.filename = filename
        self.fields = fields
        self.file = None
        self.writer = None

        # Ensure directory exists
        Path(filename).parent.mkdir(parents=True, exist_ok=True)

    def open(self):
        """Open log file for writing."""
        self.file = open(self.filename, "w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=self.fields)
        self.writer.writeheader()

    def log(self, data: Dict[str, Any]):
        """
        Log a data row.

        Args:
            data: Dictionary with field values
        """
        if self.writer is None:
            return

        # Filter to only include defined fields
        filtered_data = {k: v for k, v in data.items() if k in self.fields}
        self.writer.writerow(filtered_data)

    def close(self):
        """Close log file."""
        if self.file:
            self.file.close()
            self.file = None
            self.writer = None


class TelemetryLogger:
    """Logs robot telemetry."""

    def __init__(self, log_dir: str = "logs"):
        """
        Initialize telemetry logger.

        Args:
            log_dir: Directory for log files
        """
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        # Create timestamped log file
        timestamp = time.strftime("%Y%m%d_%H%M%S")

        # Main telemetry log
        self.telemetry_logger = CSVLogger(
            str(self.log_dir / f"telemetry_{timestamp}.csv"),
            fields=[
                "timestamp",
                "x",
                "y",
                "theta",
                "v",
                "omega",
                "sensor_0",
                "sensor_1",
                "sensor_2",
                "sensor_3",
                "edge_event",
                "state",
            ],
        )

        # Edge detections log
        self.edge_logger = CSVLogger(
            str(self.log_dir / f"edges_{timestamp}.csv"),
            fields=["timestamp", "x", "y", "robot_x", "robot_y", "robot_theta"],
        )

        # Open logs
        self.telemetry_logger.open()
        self.edge_logger.open()

        print(f"[Logger] Logging to {self.log_dir}")

    def log_telemetry(
        self,
        timestamp: float,
        pose: tuple,
        velocities: tuple,
        sensors: list,
        edge_event: bool,
        state: str,
    ):
        """Log telemetry data."""
        self.telemetry_logger.log(
            {
                "timestamp": timestamp,
                "x": pose[0],
                "y": pose[1],
                "theta": pose[2],
                "v": velocities[0],
                "omega": velocities[1],
                "sensor_0": sensors[0] if len(sensors) > 0 else 0.0,
                "sensor_1": sensors[1] if len(sensors) > 1 else 0.0,
                "sensor_2": sensors[2] if len(sensors) > 2 else 0.0,
                "sensor_3": sensors[3] if len(sensors) > 3 else 0.0,
                "edge_event": int(edge_event),
                "state": state,
            }
        )

    def log_edge(self, timestamp: float, edge_point: tuple, robot_pose: tuple):
        """Log edge detection."""
        self.edge_logger.log(
            {
                "timestamp": timestamp,
                "x": edge_point[0],
                "y": edge_point[1],
                "robot_x": robot_pose[0],
                "robot_y": robot_pose[1],
                "robot_theta": robot_pose[2],
            }
        )

    def close(self):
        """Close all log files."""
        self.telemetry_logger.close()
        self.edge_logger.close()
        print("[Logger] Logs closed")
