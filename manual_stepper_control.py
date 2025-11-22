#!/usr/bin/env python3
"""
Visual Control Center for Deskinator.

Combines manual stepper control with live sensor visualization.
Provides a GUI for driving the robot while monitoring proximity sensors
and IMU data to prevent driving off the table.
"""

import argparse
import sys
import time
from typing import Optional

# Attempt to import Qt (prefer PyQt5 to match proximity_viewer, but handle both)
try:
    from PyQt5 import QtCore, QtWidgets, QtGui

    QT_BACKEND = "PyQt5"
except ImportError:
    from PySide6 import QtCore, QtWidgets, QtGui

    QT_BACKEND = "PySide6"

from config import LIMS, I2C
from hw.stepper import StepperDrive
from proximity_viewer import ProximityRig, ProximityViewer, _exec_app


class RobotControlCenter(ProximityViewer):
    """
    GUI Control Center combining sensor viewer and manual drive control.
    """

    def __init__(
        self,
        rig: ProximityRig,
        stepper: StepperDrive,
        base_speed: float,
        max_speed: float,
        speed_step: float,
        interval_s: float = 0.1,
    ) -> None:
        super().__init__(rig, interval_s)
        self.stepper = stepper
        self.setWindowTitle("Deskinator Control Center")

        # Control Parameters
        self._base_speed = base_speed
        self._max_speed = max_speed
        self._speed_step = speed_step
        self._min_speed = 0.01
        self._speed = max(self._min_speed, min(self._max_speed, base_speed))

        # Motion State
        self._v_mult = 0
        self._omega_mult = 0
        self._omega_speed = LIMS.OMEGA_MAX * 0.5

        # Safety State
        self._edge_detected = False
        self._safety_override = False

        # Enhance UI
        self._init_control_ui()

        # Stepper Control Loop Timer (50 Hz)
        self.control_timer = QtCore.QTimer(self)
        self.control_timer.timeout.connect(self._update_control_loop)
        self.control_timer.start(20)  # 20ms = 50Hz

    def _init_control_ui(self):
        """Insert drive control widgets into the existing layout."""
        layout = self.centralWidget().layout()

        # Container for control status
        self.control_frame = QtWidgets.QFrame()
        self.control_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.control_frame.setStyleSheet(
            "QFrame { background-color: #2b2b2b; border-radius: 5px; }"
        )

        h_layout = QtWidgets.QHBoxLayout(self.control_frame)
        h_layout.setContentsMargins(10, 10, 10, 10)

        # Direction Indicator / Status
        self.dir_label = QtWidgets.QLabel("STOP")
        self.dir_label.setFont(QtGui.QFont("Arial", 24, QtGui.QFont.Bold))
        self.dir_label.setStyleSheet("color: #eeeeee;")
        self.dir_label.setAlignment(QtCore.Qt.AlignCenter)
        h_layout.addWidget(self.dir_label, stretch=2)

        # Speed Indicator
        self.speed_label = QtWidgets.QLabel(f"{self._speed:.2f} m/s")
        self.speed_label.setFont(QtGui.QFont("Monospace", 18))
        self.speed_label.setStyleSheet("color: #00ccff;")
        self.speed_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        h_layout.addWidget(self.speed_label, stretch=1)

        # Safety Indicator
        self.safety_label = QtWidgets.QLabel("SAFE")
        self.safety_label.setFont(QtGui.QFont("Arial", 14, QtGui.QFont.Bold))
        self.safety_label.setStyleSheet(
            "color: #00ff00; background-color: #1a441a; padding: 5px;"
        )
        self.safety_label.setAlignment(QtCore.Qt.AlignCenter)
        h_layout.addWidget(self.safety_label, stretch=1)

        # Insert at the top of the main VBoxLayout
        layout.insertWidget(0, self.control_frame)

        # Add help text at the bottom
        help_text = (
            "Controls: WASD (Move) | Q/E/Z/C (Turn) | SPACE (Stop)\n"
            "+/- (Speed) | Safety: Stops FWD if sensor < 30"
        )
        self.help_label = QtWidgets.QLabel(help_text)
        self.help_label.setAlignment(QtCore.Qt.AlignCenter)
        self.help_label.setStyleSheet("color: #888888;")
        layout.addWidget(self.help_label)

    def keyPressEvent(self, event: QtGui.QKeyEvent):
        key = event.key()
        text = event.text().lower()

        # Speed control
        if text in {"+", "="}:
            self._adjust_speed(self._speed_step)
            return
        if text in {"-", "_"}:
            self._adjust_speed(-self._speed_step)
            return

        # Motion commands
        # Map keys to (v_mult, omega_mult)
        cmd = None
        if key == QtCore.Qt.Key_W:
            cmd = (1, 0)
        elif key == QtCore.Qt.Key_S:
            cmd = (-1, 0)
        elif key == QtCore.Qt.Key_A:
            cmd = (0, 1)
        elif key == QtCore.Qt.Key_D:
            cmd = (0, -1)
        elif key == QtCore.Qt.Key_Q:
            cmd = (1, -1)
        elif key == QtCore.Qt.Key_E:
            cmd = (1, 1)
        elif key == QtCore.Qt.Key_Z:
            cmd = (-1, 1)
        elif key == QtCore.Qt.Key_C:
            cmd = (-1, -1)
        elif key == QtCore.Qt.Key_Space:
            cmd = (0, 0)

        if cmd is not None:
            v_req, omega_req = cmd

            # Pre-check safety for new command
            if v_req > 0 and self._edge_detected:
                # Flash warning, refuse to set forward command
                self.safety_label.setStyleSheet(
                    "color: white; background-color: red; padding: 5px;"
                )
                return

            self._set_command(v_req, omega_req)

    def _set_command(self, v_mult, omega_mult):
        # Update state
        self._v_mult = v_mult
        self._omega_mult = omega_mult

        # Update UI Text
        if v_mult == 0 and omega_mult == 0:
            text = "STOP"
        elif v_mult > 0:
            text = (
                "FORWARD"
                if omega_mult == 0
                else ("FWD RIGHT" if omega_mult < 0 else "FWD LEFT")
            )
        elif v_mult < 0:
            text = (
                "BACKWARD"
                if omega_mult == 0
                else ("BACK RIGHT" if omega_mult < 0 else "BACK LEFT")
            )
        else:  # v=0
            text = "PIVOT RIGHT" if omega_mult < 0 else "PIVOT LEFT"

        self.dir_label.setText(text)

    def _adjust_speed(self, delta: float):
        new_speed = max(self._min_speed, min(self._max_speed, self._speed + delta))
        self._speed = new_speed
        self.speed_label.setText(f"{self._speed:.2f} m/s")

    def _update_control_loop(self):
        """Runs at 50Hz to update stepper and check safety."""

        # Check safety using cached readings from ProximityViewer (updated at ~10Hz)
        self._check_safety()

        # Enforce safety on current command
        if self._v_mult > 0 and self._edge_detected:
            # Emergency Stop for forward motion
            self._v_mult = 0
            self._omega_mult = 0
            self.dir_label.setText("SAFETY STOP")
            self.dir_label.setStyleSheet("color: red;")
        elif not self._edge_detected:
            self.dir_label.setStyleSheet("color: #eeeeee;")

        # Compute commands
        v_cmd = self._v_mult * self._speed
        omega_cmd = self._omega_mult * self._omega_speed

        # Send to stepper
        self.stepper.command(v_cmd, omega_cmd)
        self.stepper.update(0.02)

    def _check_safety(self):
        """Analyze latest sensor data for edges."""
        # self.last_raw_readings is maintained by ProximityViewer.update_plot()
        if self.last_raw_readings is None:
            return

        limit = 30
        is_unsafe = False

        # ProximityRig.read() returns [Left, Right, Gesture] (if active)
        # Check Left (Index 0)
        if len(self.last_raw_readings) > 0:
            val = self.last_raw_readings[0]
            # val is None if sensor disabled/error
            if val is not None and val < limit:
                is_unsafe = True

        # Check Right (Index 1)
        if len(self.last_raw_readings) > 1:
            val = self.last_raw_readings[1]
            if val is not None and val < limit:
                is_unsafe = True

        self._edge_detected = is_unsafe

        if is_unsafe:
            self.safety_label.setText("EDGE DETECTED")
            self.safety_label.setStyleSheet(
                "color: white; background-color: red; padding: 5px;"
            )
        else:
            self.safety_label.setText("SAFE")
            self.safety_label.setStyleSheet(
                "color: #00ff00; background-color: #1a441a; padding: 5px;"
            )

    def closeEvent(self, event):
        """Stop motors when window is closed."""
        self.stepper.stop()
        self.stepper.stop_pulse_generation()
        event.accept()


def parse_args():
    parser = argparse.ArgumentParser(description="Deskinator Control Center")

    # Drive Args
    parser.add_argument("--speed", type=float, default=0.10, help="Base speed m/s")
    parser.add_argument(
        "--max-speed", type=float, default=LIMS.V_MAX, help="Max speed m/s"
    )
    parser.add_argument(
        "--speed-step", type=float, default=0.02, help="Speed increment"
    )
    parser.add_argument(
        "--hold", action="store_true", help="Keep motors enabled on exit"
    )

    # Sensor Args (Mirroring proximity_viewer defaults)
    parser.add_argument(
        "--active-sensors", default="RLGI", help="Sensors to enable (RLGI)"
    )
    parser.add_argument("--left-bus", type=int, default=I2C.LEFT_SENSOR_BUS)
    parser.add_argument("--right-bus", type=int, default=I2C.RIGHT_SENSOR_BUS)
    parser.add_argument("--sensor-addr", type=int, default=I2C.APDS_ADDR)
    parser.add_argument("--gesture-bus", type=int, default=I2C.GESTURE_BUS)
    parser.add_argument("--gesture-addr", type=int, default=I2C.GESTURE_ADDR)
    parser.add_argument(
        "--imu-addr",
        type=lambda x: int(x, 0) if x != "none" else None,
        default=I2C.ADDR_IMU,
    )
    parser.add_argument(
        "--interval", type=float, default=0.1, help="Sensor update interval"
    )

    return parser.parse_args()


def main():
    args = parse_args()

    # 1. Initialize Sensors
    print("Initializing sensors...")
    active_flags = args.active_sensors.replace("-", "").upper()
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

    # 2. Initialize Stepper
    print("Initializing stepper drive...")
    stepper = StepperDrive(release_on_idle=not args.hold)

    # 3. Start GUI
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")  # Dark mode friendly usually

    # Close rig on exit
    app.aboutToQuit.connect(rig.close)

    viewer = RobotControlCenter(
        rig=rig,
        stepper=stepper,
        base_speed=args.speed,
        max_speed=args.max_speed,
        speed_step=args.speed_step,
        interval_s=args.interval,
    )
    viewer.show()

    try:
        return _exec_app(app)
    finally:
        print("Cleaning up...")
        stepper.stop()
        stepper.stop_pulse_generation()


if __name__ == "__main__":
    sys.exit(main())
