#!/usr/bin/env python3
"""
Visual Control Center for Deskinator.

Combines manual stepper control with live sensor visualization.
Provides a GUI for driving the robot while monitoring proximity sensors
and IMU data to prevent driving off the table.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional

# Attempt to import Qt (prefer PyQt5 to match proximity_viewer, but handle both)
try:
    from proximity_viewer import ProximityRig, ProximityViewer, _exec_app, QT_BACKEND

    if QT_BACKEND == "PyQt5":
        from PyQt5 import QtCore, QtWidgets, QtGui
    else:
        from PySide6 import QtCore, QtWidgets, QtGui
except ImportError as exc:
    raise SystemExit(
        "Failed to import required modules. Ensure proximity_viewer.py is in the path "
        "and dependencies are installed."
    ) from exc

from config import LIMS, I2C, ALG
from hw.stepper import StepperDrive


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
        interval_s: float = 0.05,  # 20Hz refresh for responsiveness
    ) -> None:
        # Initialize all state variables BEFORE super().__init__ because super().__init__
        # calls update_plot(), which relies on these variables being present.

        self.stepper = stepper
        self._base_speed = base_speed
        self._max_speed = max_speed
        self._speed_step = speed_step
        self._min_speed = 0.01
        self._speed = max(self._min_speed, min(self._max_speed, base_speed))

        # Motion State
        self._v_mult = 0
        self._omega_mult = 0
        self._omega_speed = LIMS.OMEGA_MAX * 0.5
        self.last_update_time = time.monotonic()

        # Safety State
        self._edge_detected = False
        self._safety_override = False

        # Gesture Pause State
        self.is_paused = False
        self.gesture_debounce_counter = 0
        self.gesture_active_prev = False

        # Now initialize the viewer part (which triggers the first update_plot)
        super().__init__(rig, interval_s)

        self.setWindowTitle("Deskinator Control Center")

        # Enhance UI
        self._init_control_ui()

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
            "+/- (Speed) | Safety: Stops FWD if sensor < 30 | Gesture to Pause/Resume"
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

            # If paused, do not accept motion commands (except stop, implicitly handled by update_plot overriding)
            # Actually, we let the command be set, but update_plot will override it if paused.
            # This allows "pre-selecting" a direction, or resuming immediately if keys are held?
            # Better to block if paused to avoid surprises.
            if self.is_paused and (v_req != 0 or omega_req != 0):
                return

            self._set_command(v_req, omega_req)

    def _set_command(self, v_mult, omega_mult):
        # Detect mode transitions (logic from original script)
        prev_mode = self._get_mode_type(self._v_mult, self._omega_mult)
        new_mode = self._get_mode_type(v_mult, omega_mult)

        if prev_mode in (1, 2) and new_mode in (1, 2) and prev_mode != new_mode:
            with self.stepper.lock:
                self.stepper.vL_current = 0.0
                self.stepper.vR_current = 0.0

        self._v_mult = v_mult
        self._omega_mult = omega_mult

        self._update_dir_label()

    def _update_dir_label(self):
        if self.is_paused:
            text = "PAUSED"
        elif self._v_mult == 0 and self._omega_mult == 0:
            text = "STOP"
        elif self._v_mult > 0:
            text = (
                "FORWARD"
                if self._omega_mult == 0
                else ("FWD RIGHT" if self._omega_mult < 0 else "FWD LEFT")
            )
        elif self._v_mult < 0:
            text = (
                "BACKWARD"
                if self._omega_mult == 0
                else ("BACK RIGHT" if self._omega_mult < 0 else "BACK LEFT")
            )
        else:  # v=0
            text = "PIVOT RIGHT" if self._omega_mult < 0 else "PIVOT LEFT"

        if hasattr(self, "dir_label"):
            self.dir_label.setText(text)
            if self.is_paused:
                self.dir_label.setStyleSheet("color: #ffaa00;")
            else:
                self.dir_label.setStyleSheet("color: #eeeeee;")

    def _get_mode_type(self, v_mult: int, omega_mult: int) -> int:
        if v_mult == 0 and omega_mult == 0:
            return 0
        elif v_mult != 0 and omega_mult == 0:
            return 1
        elif v_mult == 0 and omega_mult != 0:
            return 2
        else:
            return 3

    def _adjust_speed(self, delta: float):
        new_speed = max(self._min_speed, min(self._max_speed, self._speed + delta))
        self._speed = new_speed
        self.speed_label.setText(f"{self._speed:.2f} m/s")

    def update_plot(self) -> None:
        # 1. Call super to read sensors and update graph
        super().update_plot()

        # 2. Check gesture and safety
        self._check_gesture_toggle()
        self._check_safety()

        # 3. Update Stepper
        now = time.monotonic()
        dt = now - self.last_update_time
        self.last_update_time = now

        # Clamp dt to avoid huge jumps if lag
        if dt > 0.1:
            dt = 0.05

        # Override commands if paused
        if self.is_paused:
            self.stepper.command(0, 0)
            self.stepper.update(dt)
            return

        # Enforce safety on current command
        if self._v_mult > 0 and self._edge_detected:
            # Emergency Stop for forward motion
            self._v_mult = 0
            self._omega_mult = 0
            if hasattr(self, "dir_label"):
                self.dir_label.setText("SAFETY STOP")
                self.dir_label.setStyleSheet("color: red;")
            self.stepper.command(0, 0)
        elif (
            not self._edge_detected
            and hasattr(self, "dir_label")
            and "SAFETY" in self.dir_label.text()
        ):
            # Reset label color if safe again (but stay stopped until key press)
            if self._v_mult == 0 and self._omega_mult == 0:
                self._update_dir_label()

        # Compute commands
        v_cmd = self._v_mult * self._speed
        omega_cmd = self._omega_mult * self._omega_speed

        # Send to stepper
        self.stepper.command(v_cmd, omega_cmd)
        self.stepper.update(dt)

    def _check_gesture_toggle(self):
        """Check for gesture sensor toggle event."""
        if not hasattr(self, "raw_readings") or self.raw_readings is None:
            return

        # Find gesture sensor index
        # ProximityViewer labels: ["left", "right", "start_gesture"] usually
        gesture_val = None
        for label, val in zip(self.sensor_labels, self.raw_readings):
            if label == "start_gesture":
                gesture_val = val
                break

        if gesture_val is None:
            return

        # Check threshold (raw value)
        # Ensure it's a number
        if not isinstance(gesture_val, (int, float)):
            return

        is_active = gesture_val > ALG.GESTURE_RAW_THRESH

        # Debounce logic
        if is_active:
            self.gesture_debounce_counter += 1
        else:
            self.gesture_debounce_counter = 0
            self.gesture_active_prev = False

        if (
            self.gesture_debounce_counter > 2 and not self.gesture_active_prev
        ):  # Require ~3 frames of active (~150ms)
            # Toggle state
            self.is_paused = not self.is_paused
            print(f"[Control] Gesture detected! Paused: {self.is_paused}")

            # Latch to prevent multiple toggles while hand is held
            self.gesture_active_prev = True

            # Stop robot if pausing
            if self.is_paused:
                self._v_mult = 0
                self._omega_mult = 0
                self.stepper.command(0, 0)

            self._update_dir_label()

    def _check_safety(self):
        """Analyze latest sensor data for edges."""
        # self.raw_readings is populated by super().update_plot()
        if not hasattr(self, "raw_readings") or self.raw_readings is None:
            return

        limit = ALG.EDGE_RAW_THRESH
        is_unsafe = False

        # Check Left (Index 0) and Right (Index 1)
        for label, val in zip(self.sensor_labels, self.raw_readings):
            if label in ["left", "right"]:
                # val is None or int
                if val is not None and isinstance(val, (int, float)) and val < limit:
                    is_unsafe = True

        self._edge_detected = is_unsafe

        if not hasattr(self, "safety_label"):
            return

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
        super().closeEvent(
            event
        )  # This will call rig.close() via the app connection usually, but good to be safe


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
        "--interval", type=float, default=0.05, help="Update interval (default 0.05s)"
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
        # rig.close() is handled by app.aboutToQuit or rig destructor


if __name__ == "__main__":
    sys.exit(main())
