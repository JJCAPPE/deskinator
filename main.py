"""
Simplified Deskinator main controller for real hardware.

Replicates viz_demo.py's control architecture on real Raspberry Pi hardware.
Uses simple odometry (no EKF) and proven simulation controllers.
"""

import time
import signal
import sys
import numpy as np
from typing import Optional

try:
    from .config import PINS, I2C, GEOM, LIMS, ALG
    from .hw.gpio import gpio_manager
    from .hw.stepper import StepperDrive
    from .hw.vacuum import Vacuum
    from .hw.led import LED
    from .hw.i2c import I2CBus
    from .hw.apds9960 import APDS9960
    from .slam.simple_odom import SimpleOdometry
    from .planning import SimpleWallFollower, SimpleRectangleFit, CoveragePlanner, SweptMap
    from .utils.logs import TelemetryLogger
    from .utils.viz import Visualizer
    from .hw.table_detector import compute_sensor_world_position
    from .hw.mpu6050 import MPU6050
    from .slam.ekf_fusion import EKFFusion
except ImportError:
    from config import PINS, I2C, GEOM, LIMS, ALG
    from hw.gpio import gpio_manager
    from hw.stepper import StepperDrive
    from hw.vacuum import Vacuum
    from hw.led import LED
    from hw.i2c import I2CBus
    from hw.apds9960 import APDS9960
    from slam.simple_odom import SimpleOdometry
    from planning import SimpleWallFollower, SimpleRectangleFit, CoveragePlanner, SweptMap
    from utils.logs import TelemetryLogger
    from utils.viz import Visualizer
    from hw.table_detector import compute_sensor_world_position
    from hw.mpu6050 import MPU6050
    from slam.ekf_fusion import EKFFusion


class DeskinatorSimple:
    """Simplified robot controller matching viz_demo.py architecture.
    
    Uses real hardware but simple control logic:
    - Simple odometry (no EKF sensor fusion)
    - SimpleWallFollower (same as simulation)
    - Direct sensor boolean interface
    - Single-threaded synchronous loop
    """

    def __init__(self, enable_viz: bool = False, enable_imu: bool = False):
        """Initialize simplified Deskinator.
        
        Args:
            enable_viz: Enable real-time visualization
            enable_imu: Enable IMU sensor fusion
        """
        print("=" * 60)
        print("Deskinator - Simplified Hardware Controller")
        print("(Replicates viz_demo.py on real robot)")
        print("=" * 60)

        # Hardware initialization
        print("[Init] Initializing hardware...")
        self.stepper = StepperDrive()
        self.vacuum = Vacuum()

        # LED for status indication
        self.led = None
        try:
            self.led = LED()
            print(f"  LED ready on GPIO{PINS.LED}")
        except Exception as e:
            print(f"[Init] Warning: LED unavailable ({e})")

        # Proximity sensors (left and right)
        self.sensors = []
        
        # Left sensor on bus 7
        try:
            left_i2c = I2CBus(I2C.LEFT_SENSOR_BUS)
            left_sensor = APDS9960(left_i2c, I2C.APDS_ADDR)
            left_sensor.init()
            self.sensors.append(left_sensor)
            print(f"  Left APDS9960 on bus {I2C.LEFT_SENSOR_BUS} @ 0x{I2C.APDS_ADDR:02x}")
        except Exception as e:
            print(f"[Init] Warning: Left sensor unavailable ({e})")
            self.sensors.append(None)

        # Right sensor on bus 3
        try:
            right_i2c = I2CBus(I2C.RIGHT_SENSOR_BUS)
            right_sensor = APDS9960(right_i2c, I2C.APDS_ADDR)
            right_sensor.init()
            self.sensors.append(right_sensor)
            print(f"  Right APDS9960 on bus {I2C.RIGHT_SENSOR_BUS} @ 0x{I2C.APDS_ADDR:02x}")
        except Exception as e:
            print(f"[Init] Warning: Right sensor unavailable ({e})")
            self.sensors.append(None)

        # Gesture sensor for start/stop control
        self.gesture_sensor = None
        try:
            gesture_i2c = I2CBus(I2C.GESTURE_BUS)
            gesture_sensor = APDS9960(gesture_i2c, I2C.GESTURE_ADDR)
            gesture_sensor.init()
            self.gesture_sensor = gesture_sensor
            print(f"  Gesture APDS9960 on bus {I2C.GESTURE_BUS} @ 0x{I2C.GESTURE_ADDR:02x}")
        except Exception as e:
            print(f"[Init] Warning: Gesture sensor unavailable ({e})")

        # IMU (Optional)
        self.imu = None
        if enable_imu:
            print("[Init] Initializing IMU...")
            try:
                imu_i2c = I2CBus(I2C.IMU_BUS)
                self.imu = MPU6050(imu_i2c, I2C.ADDR_IMU)
                print(f"  MPU-6050 IMU on bus {I2C.IMU_BUS} @ 0x{I2C.ADDR_IMU:02x}")
            except Exception as e:
                print(f"[Init] Warning: IMU unavailable ({e})")
                self.imu = None

        # Simple pose tracking (replaces EKF)
        # Simple pose tracking (replaces EKF)
        if self.imu is not None:
            print("[Init] Initializing EKF Fusion (IMU-aided)...")
            self.odom = EKFFusion(initial_pose=(0.0, 0.0, 0.0))
        else:
            print("[Init] Initializing simple odometry...")
            self.odom = SimpleOdometry(initial_pose=(0.0, 0.0, 0.0))

        # Control (same as viz_demo)
        print("[Init] Initializing control...")
        self.wall_follower = SimpleWallFollower(
            forward_speed=ALG.BOUNDARY_SPEED,
            turn_speed=LIMS.OMEGA_MAX
        )
        
        # Planning (same as viz_demo)
        self.rect_fit = SimpleRectangleFit()
        self.coverage_planner = CoveragePlanner()
        self.swept_map = SweptMap()

        # Logging and visualization
        self.logger = TelemetryLogger()
        self.visualizer = Visualizer() if enable_viz else None

        # State tracking
        self.running = False
        self.state = "WAIT_START"  # WAIT_START, BOUNDARY_DISCOVERY, COVERAGE, DONE
        self.trajectory = []
        
        # Debouncing for edge detection
        self.edge_drop_counts = {"left": 0, "right": 0}
        self.edge_debounce_cycles = max(1, int(ALG.EDGE_DEBOUNCE * 50))  # 50 Hz loop

        # Gesture sensor state for start/stop control
        self.gesture_debounce_counter = 0
        self.gesture_active_prev = False
        self.cleaning_active = False  # True when cleaning is running

        print("[Init] Initialization complete")
        print("=" * 60)

    def calibrate_sensors(self):
        """Calibrate proximity sensors."""
        print("[Calibrate] Calibrating proximity sensors...")
        print("  This will take about 5 seconds per sensor")

        sensor_names = ["Left", "Right"]
        for i, sensor in enumerate(self.sensors):
            if sensor is not None:
                print(f"\n  {sensor_names[i]} sensor:")
                sensor.calibrate(on_table_samples=10, off_table_samples=10)
            else:
                print(f"\n  {sensor_names[i]} sensor: Not available")

        print("\n[Calibrate] Calibration complete")

    def calibrate_imu(self):
        """Calibrate IMU gyro bias."""
        if self.imu is None:
            print("[Calibrate] IMU not enabled or unavailable")
            return

        print("[Calibrate] Calibrating IMU...")
        print("  Keep robot stationary for 3 seconds")
        self.imu.bias_calibrate(duration=3.0)
        print("[Calibrate] IMU calibration complete")

    def _notify_start_blink(self):
        """Play start notification on LED."""
        if self.led:
            try:
                self.led.blink_pattern(count=3, duration=0.1, pause=0.1)
            except Exception as e:
                print(f"[Start] LED error: {e}")

    def _notify_finish_blink(self):
        """Play finish notification on LED."""
        if self.led:
            try:
                self.led.blink_pattern(count=5, duration=0.1, pause=0.1)
            except Exception as e:
                print(f"[Finish] LED error: {e}")

    def _read_gesture_sensor(self) -> Optional[int]:
        """Read gesture sensor raw proximity value.
        
        Returns:
            Raw proximity value, or None if sensor unavailable
        """
        if self.gesture_sensor is None:
            return None
        
        try:
            return self.gesture_sensor.read_proximity()
            return self.gesture_sensor.read_proximity_raw()
        except Exception as e:
            print(f"[Gesture] Read error: {e}")
            return None

    def _check_gesture_toggle(self) -> bool:
        """Check for gesture sensor toggle event.
        
        Returns:
            True if gesture detected and state toggled
        """
        gesture_val = self._read_gesture_sensor()
        
        if gesture_val is None:
            return False
        
        # Check threshold (same as manual_stepper_control.py)
        is_active = gesture_val > ALG.GESTURE_RAW_THRESH
        
        # Debounce logic (same pattern as manual control)
        if is_active:
            self.gesture_debounce_counter += 1
        else:
            self.gesture_debounce_counter = 0
            self.gesture_active_prev = False
        
        # Require ~3 frames of active (~150ms at 50Hz)
        if self.gesture_debounce_counter > 2 and not self.gesture_active_prev:
            # Toggle cleaning state
            self.cleaning_active = not self.cleaning_active
            print(f"[Gesture] Detected! Cleaning active: {self.cleaning_active}")
            
            # Latch to prevent multiple toggles while hand is held
            self.gesture_active_prev = True
            
            # Blink LED on toggle
            if self.cleaning_active:
                self._notify_start_blink()
            else:
                self._notify_finish_blink()
            
            return True
        
        return False

    def _await_user_start(self) -> bool:
        """Wait for user to start via gesture sensor."""
        print("[Start] Wave hand over gesture sensor to begin cleaning...")
        print("[Start] Press Ctrl+C to exit")
        
        try:
            while not self.cleaning_active:
                # Check gesture sensor
                self._check_gesture_toggle()
                time.sleep(0.02)  # 50Hz polling
            
            return True
        except KeyboardInterrupt:
            print("\n[Start] Cancelled by user")
            return False

    def _read_sensors(self) -> tuple:
        """Read proximity sensors.
        
        Returns:
            (left_on, right_on) - True if sensor detects table
        """
        left_on = True  # Default to on-table
        right_on = True
        
        if self.sensors[0] is not None:
            left_on = self.sensors[0].is_on_table()
        
        if self.sensors[1] is not None:
            right_on = self.sensors[1].is_on_table()
            
        return left_on, right_on

    def _add_edge_point(self, pose: tuple, side: str):
        """Add edge detection point to rectangle fitter.
        
        Args:
            pose: Current robot pose (x, y, theta)
            side: "left" or "right"
        """
        # Compute sensor position in world frame
        world_x, world_y = compute_sensor_world_position(pose, side)
        
        # Add to rectangle fitter
        self.rect_fit.add_edge_point((world_x, world_y))
        
        # Log
        self.logger.log_edge(time.time(), (world_x, world_y), pose)

    def _check_edge_events(self, left_on: bool, right_on: bool, pose: tuple):
        """Check for edge detection and add points with debouncing.
        
        Args:
            left_on: True if left sensor on table
            right_on: True if right sensor on table
            pose: Current robot pose
        """
        # Update drop counters
        if not left_on:
            self.edge_drop_counts["left"] += 1
        else:
            self.edge_drop_counts["left"] = 0

        if not right_on:
            self.edge_drop_counts["right"] += 1
        else:
            self.edge_drop_counts["right"] = 0

        # Trigger edge event after debounce cycles
        if self.edge_drop_counts["left"] >= self.edge_debounce_cycles:
            self.edge_drop_counts["left"] = 0
            self._add_edge_point(pose, "left")

        if self.edge_drop_counts["right"] >= self.edge_debounce_cycles:
            self.edge_drop_counts["right"] = 0
            self._add_edge_point(pose, "right")

    def _drive_then_turn_controller(self, pose: tuple, waypoint: tuple) -> tuple:
        """Drive-then-turn controller matching viz_demo.py logic.
        
        Args:
            pose: Current pose (x, y, theta)
            waypoint: Target waypoint (wx, wy, wtheta)
            
        Returns:
            (v, omega) velocity commands
        """
        if waypoint is None:
            return 0.0, 0.0
            
        x, y, theta = pose
        wx, wy, wtheta = waypoint

        # Compute position error
        dx = wx - x
        dy = wy - y
        dist_to_waypoint = np.sqrt(dx*dx + dy*dy)

        # Compute orientation error
        dtheta = wtheta - theta
        dtheta = ((dtheta + np.pi) % (2*np.pi)) - np.pi  # Wrap to [-pi, pi]

        # Tolerances
        POSITION_TOLERANCE = 0.05  # 5cm
        ORIENTATION_TOLERANCE = np.deg2rad(8)  # 8 degrees

        # Check if reached
        position_reached = dist_to_waypoint < POSITION_TOLERANCE
        orientation_reached = abs(dtheta) < ORIENTATION_TOLERANCE

        # Phase 1: Drive to position (if far)
        if not position_reached:
            # Target heading is bearing to waypoint
            target_heading = np.arctan2(dy, dx)
            heading_error = target_heading - theta
            heading_error = ((heading_error + np.pi) % (2*np.pi)) - np.pi

            # Turn in place threshold
            turn_in_place_threshold = np.deg2rad(30)

            if abs(heading_error) > turn_in_place_threshold:
                # Large heading error - turn in place first
                v = 0.0
                omega = 3.0 * heading_error
                omega = np.clip(omega, -LIMS.OMEGA_MAX * 2.0, LIMS.OMEGA_MAX * 2.0)
            else:
                # Move forward with heading correction
                forward_speed_base = LIMS.V_BASE
                heading_scale = 1.0 - (abs(heading_error) / turn_in_place_threshold) * 0.5
                v = forward_speed_base * max(0.5, heading_scale)

                # Slow down when close
                if dist_to_waypoint < 0.1:
                    v *= 0.5

                # Angular velocity to correct heading
                omega = 2.0 * heading_error
                omega = np.clip(omega, -LIMS.OMEGA_MAX, LIMS.OMEGA_MAX)

        # Phase 2: Turn to final orientation (if close)
        elif not orientation_reached:
            v = 0.0  # Stop forward motion
            omega = 3.0 * dtheta
            omega = np.clip(omega, -LIMS.OMEGA_MAX * 2.0, LIMS.OMEGA_MAX * 2.0)
        else:
            # Both reached
            v = 0.0
            omega = 0  
            
        return v, omega

    def run(self):
        """Main control loop - single threaded like viz_demo."""
        
        # Setup signal handler
        signal.signal(signal.SIGINT, self._signal_handler)

        # Wait for start gesture
        if not self._await_user_start():
            self.shutdown()
            return

        # Start vacuum
        print("[Main] Starting vacuum")
        self.vacuum.on(duty=0.8)

        # Main loop parameters
        dt = 0.02  # 50 Hz (20ms)
        self.running = True
        self.state = "BOUNDARY_DISCOVERY"
        
        print("[Main] Starting main loop...")
        print("  State: BOUNDARY_DISCOVERY")
        print("  Wave hand over gesture sensor to stop")

        try:
            t = 0.0
            last_viz_update = 0.0
            last_advance_time = -1.0  # For coverage waypoint debouncing

            while self.running:
                loop_start = time.time()

                # 1. Check gesture sensor for stop command
                if self._check_gesture_toggle():
                    if not self.cleaning_active:
                        print("[Main] Stop gesture detected")
                        self.running = False
                        break

                # 2. Read sensors (hardware)
                left_on, right_on = self._read_sensors()

                # 3. Get current pose (simple odometry)
                pose = self.odom.pose()
                self.trajectory.append(pose)

                # 4. Check for edge events (add points to rect_fit)
                if self.state == "BOUNDARY_DISCOVERY":
                    self._check_edge_events(left_on, right_on, pose)

                # 5. State machine and control
                v_cmd = 0.0
                omega_cmd = 0.0

                if self.state == "BOUNDARY_DISCOVERY":
                    # Use SimpleWallFollower (same as viz_demo)
                    v_cmd, omega_cmd = self.wall_follower.update(pose, left_on, right_on, dt)

                    # Fit rectangle periodically
                    if len(self.rect_fit.edge_points) > 4 and len(self.rect_fit.edge_points) % 5 == 0:
                        self.rect_fit.fit()

                    # Check completion (rotation-based)
                    if self.wall_follower.is_complete():
                        print(f"\n[Main] Wall following complete!")
                        print(f"  Total rotation: {self.wall_follower.get_total_rotation_degrees():.1f}°")
                        print(f"  Collected {len(self.rect_fit.edge_points)} edge points")

                        # Final rectangle fit
                        self.rect_fit.fit()
                        rectangle = self.rect_fit.get_rectangle()

                        if rectangle:
                            cx, cy, heading, w, h = rectangle
                            print(f"  Rectangle: {w:.2f} x {h:.2f} m at ({cx:.2f}, {cy:.2f})")
                            
                            # Transition to coverage
                            print("[Main] Transitioning to COVERAGE")
                            self.state = "COVERAGE"
                            
                            # Build coverage lanes
                            self.coverage_planner.set_rectangle(rectangle)
                            lanes = self.coverage_planner.build_lanes(start_pose=pose)
                            print(f"  Generated {len(lanes)} coverage lanes")
                        else:
                            print("  ERROR: Failed to fit rectangle")
                            self.state = "DONE"

                elif self.state == "COVERAGE":
                    # Get current waypoint
                    waypoint = self.coverage_planner.get_current_waypoint()
                    
                    if waypoint:
                        wx, wy, wtheta = waypoint
                        x, y, theta = pose

                        # Check if waypoint reached
                        dx = wx - x
                        dy = wy - y
                        dist = np.sqrt(dx*dx + dy*dy)
                        dtheta = wtheta - theta
                        dtheta = ((dtheta + np.pi) % (2*np.pi)) - np.pi

                        POSITION_TOLERANCE = 0.05
                        ORIENTATION_TOLERANCE = np.deg2rad(8)

                        if (dist < POSITION_TOLERANCE and 
                            abs(dtheta) < ORIENTATION_TOLERANCE and
                            (t - last_advance_time) > 0.1):
                            
                            self.coverage_planner.advance_waypoint()
                            last_advance_time = t

                            if self.coverage_planner.is_complete():
                                print("\n[Main] Coverage complete!")
                                self.state = "DONE"
                        else:
                            # Drive-then-turn controller
                            v_cmd, omega_cmd = self._drive_then_turn_controller(pose, waypoint)
                    else:
                        v_cmd, omega_cmd = 0.0, 0.0
                        if self.coverage_planner.is_complete():
                            print("\n[Main] Coverage path complete")
                            self.state = "DONE"

                elif self.state == "DONE":
                    v_cmd, omega_cmd = 0.0, 0.0
                    self.running = False

                # 6. Command motors (hardware)
                self.stepper.command(v_cmd, omega_cmd)

                # 7. Update odometry (hardware)
                dSL, dSR = self.stepper.update(dt)
                self.odom.predict(dSL, dSR, dt)

                # Update from IMU gyroscope if available
                if self.imu is not None and hasattr(self.odom, 'update_gyro'):
                    try:
                        yaw_rate = self.imu.read_yaw_rate()
                        self.odom.update_gyro(yaw_rate, dt)
                    except Exception as e:
                        print(f"[IMU] Read error: {e}")

                # 8. Update swept map
                if v_cmd > 0:
                    self.swept_map.add_forward_sweep(pose, v_cmd * dt)

                # 9. Logging
                self.logger.log_telemetry(
                    time.time(),
                    pose,
                    (v_cmd, omega_cmd),
                    [left_on, right_on],
                    False,  # recovery_active
                    self.state
                )

                # 10. Visualization (update at 5 Hz to reduce overhead)
                if self.visualizer and (t - last_viz_update) > 0.2:
                    coverage_grid = self.swept_map.get_grid()
                    bounds = (
                        self.swept_map.min_x,
                        self.swept_map.max_x,
                        self.swept_map.min_y,
                        self.swept_map.max_y
                    )
                    rectangle = self.rect_fit.get_rectangle()
                    
                    # Build status text
                    status = f"State: {self.state}\n"
                    status += f"Time: {t:.1f}s\n"
                    if self.state == "BOUNDARY_DISCOVERY":
                        rotation = self.wall_follower.get_total_rotation_degrees()
                        status += f"Rotation: {rotation:.1f}°\n"
                        status += f"Edge points: {len(self.rect_fit.edge_points)}"
                    elif self.state == "COVERAGE" and rectangle:
                        cov_ratio = self.swept_map.coverage_ratio(rectangle)
                        status += f"Coverage: {cov_ratio:.1%}\n"
                        status += f"Waypoint: {self.coverage_planner.current_waypoint_idx + 1}/{len(self.coverage_planner.path)}"

                    self.visualizer.update(
                        poses=self.trajectory,
                        edge_points=self.rect_fit.edge_points,
                        rectangle=rectangle,
                        coverage_grid=coverage_grid,
                        swept_map_bounds=bounds,
                        text_info=status,
                        robot_state=self.state
                    )
                    last_viz_update = t

                # 11. Sleep to maintain loop rate
                elapsed = time.time() - loop_start
                sleep_time = max(0, dt - elapsed)
                time.sleep(sleep_time)
                t += dt

        except KeyboardInterrupt:
            print("\n[Main] Interrupted by user")
        finally:
            self.shutdown()

    def _signal_handler(self, signum, frame):
        """Handle interrupt signal."""
        print("\n[Main] Stopping...")
        self.running = False

    def shutdown(self):
        """Shutdown robot safely."""
        print("[Shutdown] Stopping robot...")

        self.running = False

        # Stop motors
        self.stepper.stop()
        self.stepper.stop_pulse_generation()
        self.stepper.disable_drivers()

        # Stop vacuum
        self.vacuum.off()

        # Close logs
        self.logger.close()

        # Save visualization
        if self.visualizer:
            self.visualizer.save("output_simple.png")
            self.visualizer.close()

        # LED cleanup
        if self.led:
            self._notify_finish_blink()
            time.sleep(0.5)
            self.led.cleanup()

        # GPIO cleanup
        gpio_manager.cleanup()

        print("[Shutdown] Complete")


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="Deskinator Simple Controller")
    parser.add_argument("--viz", action="store_true", help="Enable visualization")
    parser.add_argument("--calibrate", action="store_true", help="Run sensor calibration")
    parser.add_argument("--imu", action="store_true", help="Enable IMU sensor fusion")

    args = parser.parse_args()

    robot = DeskinatorSimple(enable_viz=args.viz, enable_imu=args.imu)

    if args.calibrate:
        robot.calibrate_sensors()
        if args.imu:
            robot.calibrate_imu()
        return

    # Run main loop
    robot.run()


if __name__ == "__main__":
    main()
