"""
Main orchestration for Deskinator robot.

Coordinates sensing, mapping, and control loops using asyncio.
"""

import asyncio
import time
import signal
import sys
from typing import List, Tuple

from .config import PINS, I2C, GEOM, LIMS, ALG
from .hw.gpio import gpio_manager
from .hw.stepper import StepperDrive
from .hw.vacuum import Vacuum
from .hw.buzzer import Buzzer
from .hw.gesture import GestureSensor
from .hw.i2c import I2CBus
from .hw.apds9960 import APDS9960
from .hw.mpu6050 import MPU6050
from .slam.ekf import EKF
from .slam.posegraph import PoseGraph
from .slam.rect_fit import RectangleFit
from .slam.frames import transform_point
from .planning.coverage import CoveragePlanner
from .planning.map2d import SweptMap
from .control.fsm import SupervisorFSM, RobotState
from .control.motion import MotionController
from .control.limits import VelocityLimiter
from .utils.logs import TelemetryLogger
from .utils.viz import Visualizer
from .utils.timing import RateTimer, LoopTimer


class Deskinator:
    """Main robot controller."""

    def __init__(self, enable_viz: bool = False):
        """
        Initialize Deskinator.

        Args:
            enable_viz: Enable real-time visualization
        """
        print("=" * 60)
        print("Deskinator - Autonomous Desk Cleaning Robot")
        print("=" * 60)

        # Hardware
        print("[Init] Initializing hardware...")
        self.stepper = StepperDrive()
        self.vacuum = Vacuum()

        self.buzzer = None
        try:
            self.buzzer = Buzzer()
            print(f"  Buzzer ready on GPIO{PINS.BUZZER}")
        except Exception as e:
            print(f"[Init] Warning: Buzzer unavailable ({e})")

        self.gesture = None
        try:
            self.gesture = GestureSensor(I2C.GESTURE_BUS, I2C.GESTURE_ADDR)
            if getattr(self.gesture, "sim_mode", False):
                print("  Gesture sensor in simulation mode")
            else:
                print(
                    "  Gesture sensor ready on bus "
                    f"{I2C.GESTURE_BUS} @ 0x{I2C.GESTURE_ADDR:02x}"
                )
        except Exception as e:
            print(f"[Init] Warning: Gesture sensor unavailable ({e})")

        # I2C devices
        # Proximity sensors on separate buses
        self.sensors = []
        
        # Left sensor on bus 7 (GPIO19/GPIO26)
        try:
            left_i2c = I2CBus(I2C.LEFT_SENSOR_BUS)
            left_sensor = APDS9960(left_i2c, I2C.APDS_ADDR)
            left_sensor.init()
            self.sensors.append(left_sensor)
            print(f"  Left APDS9960 on bus {I2C.LEFT_SENSOR_BUS} @ 0x{I2C.APDS_ADDR:02x}")
        except Exception as e:
            print(f"[Init] Warning: Left sensor unavailable ({e})")
            self.sensors.append(None)
        
        # Right sensor on bus 1 (GPIO2/GPIO3 - hardware I2C)
        try:
            right_i2c = I2CBus(I2C.RIGHT_SENSOR_BUS)
            right_sensor = APDS9960(right_i2c, I2C.APDS_ADDR)
            right_sensor.init()
            self.sensors.append(right_sensor)
            print(f"  Right APDS9960 on bus {I2C.RIGHT_SENSOR_BUS} @ 0x{I2C.APDS_ADDR:02x}")
        except Exception as e:
            print(f"[Init] Warning: Right sensor unavailable ({e})")
            self.sensors.append(None)

        # IMU on separate bus (bus 5 - software I2C)
        self.imu_i2c = I2CBus(I2C.IMU_BUS)
        self.imu = MPU6050(self.imu_i2c, I2C.ADDR_IMU or 0x68)
        if getattr(self.imu, "sim_mode", False):
            print("  MPU-6050 IMU in simulation mode")
        else:
            print("  MPU-6050 IMU initialized")

        # State estimation
        print("[Init] Initializing SLAM...")
        self.ekf = EKF()
        self.pose_graph = PoseGraph()
        self.rect_fit = RectangleFit()

        # Planning
        self.coverage_planner = CoveragePlanner()
        self.swept_map = SweptMap()

        # Control
        print("[Init] Initializing control...")
        self.fsm = SupervisorFSM()
        self.motion = MotionController()
        self.limiter = VelocityLimiter()

        # Logging and visualization
        self.logger = TelemetryLogger()
        self.visualizer = Visualizer() if enable_viz else None

        # State
        self.running = False
        self.start_signal = False
        self.finish_alerted = False
        self.edge_filters = [1.0 for _ in self.sensors]
        self.edge_drop_counts = {"left": 0, "right": 0}
        self.edge_debounce_cycles = max(1, int(ALG.EDGE_DEBOUNCE * ALG.FUSE_HZ))
        self.last_node_pose = (0.0, 0.0, 0.0)
        self.sensor_context = {
            "sensors": [],
            "filtered": list(self.edge_filters),
            "timestamp": time.time(),
        }
        self.last_progress_time = time.time()
        self.last_recovery_time = 0.0

        # Timers
        self.sense_timer = LoopTimer("Sense")
        self.map_timer = LoopTimer("Map")
        self.ctrl_timer = LoopTimer("Control")

        print("[Init] Initialization complete")
        print("=" * 60)

    def scan_i2c(self):
        """Scan I2C buses and print devices."""
        print("[I2C] Scanning buses...")
        
        print(f"\n  Bus {I2C.LEFT_SENSOR_BUS} (Left sensor):")
        try:
            left_bus = I2CBus(I2C.LEFT_SENSOR_BUS)
            devices = left_bus.scan()
            print(f"    Found {len(devices)} device(s):")
            for addr in devices:
                print(f"      0x{addr:02x}")
        except Exception as e:
            print(f"    Error: {e}")
        
        print(f"\n  Bus {I2C.RIGHT_SENSOR_BUS} (Right sensor):")
        try:
            right_bus = I2CBus(I2C.RIGHT_SENSOR_BUS)
            devices = right_bus.scan()
            print(f"    Found {len(devices)} device(s):")
            for addr in devices:
                print(f"      0x{addr:02x}")
        except Exception as e:
            print(f"    Error: {e}")

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
        """Calibrate IMU bias."""
        print("[Calibrate] Calibrating IMU...")
        self.imu.bias_calibrate(duration=2.0)
        print("[Calibrate] IMU calibration complete")

    def _notify_start_beep(self):
        """Play start notification on buzzer."""
        if not self.buzzer:
            return

        try:
            self.buzzer.beep_pattern(count=3, duration=0.1, pause=0.1)
        except Exception as e:
            print(f"[Start] Buzzer error: {e}")

    def _notify_finish_beep(self):
        """Play finish notification on buzzer asynchronously."""
        if not self.buzzer:
            return

        try:
            self.buzzer.beep_async(count=2, duration=0.1, pause=0.1)
        except Exception as e:
            print(f"[Shutdown] Buzzer error: {e}")

    def _await_gesture_start(self) -> bool:
        """Block until a gesture is detected to start cleaning."""
        self.start_signal = False
        self.finish_alerted = False

        if not self.gesture:
            print("[Start] Gesture sensor unavailable; auto-starting")
            self.start_signal = True
            self._notify_start_beep()
            return True

        if getattr(self.gesture, "sim_mode", False):
            print("[Start] Gesture sensor simulation mode; auto-starting")
            self.start_signal = True
            self._notify_start_beep()
            return True

        print("[Start] Hold your hand near the gesture sensor to begin...")

        try:
            triggered = self.gesture.wait_for_hand_presence(timeout=None)
        except KeyboardInterrupt:
            print("\n[Start] Gesture wait cancelled by user")
            return False

        if triggered:
            print("[Start] Proximity trigger detected")
            self.start_signal = True
            self._notify_start_beep()
            return True

        print("[Start] Gesture sensor did not trigger")
        return False

    async def loop_sense(self):
        """Sensing loop @ 50 Hz."""
        rate = RateTimer(ALG.FUSE_HZ)

        while self.running:
            self.sense_timer.start()

            # Read odometry
            dSL, dSR = self.stepper.read_odometry()
            dt = 1.0 / ALG.FUSE_HZ

            # Read IMU
            yaw_rate = self.imu.read_yaw_rate()

            # Update EKF
            self.ekf.predict(dSL, dSR, dt)
            self.ekf.update_gyro(yaw_rate, dt)

            # Read proximity sensors
            sensor_readings = []
            for sensor in self.sensors:
                if sensor is not None:
                    reading = sensor.read_proximity_norm()
                    sensor_readings.append(reading)
                else:
                    sensor_readings.append(1.0)  # Default to "on table" if sensor unavailable

            # Check for edge events (pair-based rule)
            self._check_edge_events(sensor_readings)

            # Store sensor context
            now = time.time()
            self.sensor_context = {
                "sensors": sensor_readings,
                "filtered": list(self.edge_filters),
                "timestamp": now,
            }

            self.sense_timer.stop()
            rate.sleep()

    def _check_edge_events(self, sensors: List[float]):
        """Check for edge detection events."""
        if not sensors:
            return

        if len(self.edge_filters) != len(sensors):
            self.edge_filters = [1.0 for _ in sensors]

        alpha = ALG.EDGE_EWMA_ALPHA
        for i, reading in enumerate(sensors):
            prev = self.edge_filters[i]
            self.edge_filters[i] = alpha * reading + (1.0 - alpha) * prev

        def pair_values(indices: tuple[int, int]):
            filtered = []
            raw = []
            for idx in indices:
                if idx < len(sensors):
                    filtered.append(self.edge_filters[idx])
                    raw.append(sensors[idx])
            return filtered, raw

        # Single sensor per side now
        left_filtered = [self.edge_filters[I2C.LEFT_SENSOR_IDX]] if I2C.LEFT_SENSOR_IDX < len(self.edge_filters) else []
        left_raw = [sensors[I2C.LEFT_SENSOR_IDX]] if I2C.LEFT_SENSOR_IDX < len(sensors) else []
        right_filtered = [self.edge_filters[I2C.RIGHT_SENSOR_IDX]] if I2C.RIGHT_SENSOR_IDX < len(self.edge_filters) else []
        right_raw = [sensors[I2C.RIGHT_SENSOR_IDX]] if I2C.RIGHT_SENSOR_IDX < len(sensors) else []

        def is_off(filtered: List[float], raw: List[float]) -> bool:
            if not filtered or not raw:
                return False
            avg = sum(filtered) / len(filtered)
            min_raw = min(raw)
            return (
                avg <= ALG.EDGE_THRESH
                and min_raw <= ALG.EDGE_THRESH * ALG.EDGE_RAW_HYST
            )

        left_off = is_off(left_filtered, left_raw)
        right_off = is_off(right_filtered, right_raw)

        if left_off:
            self.edge_drop_counts["left"] += 1
        else:
            self.edge_drop_counts["left"] = 0

        if right_off:
            self.edge_drop_counts["right"] += 1
        else:
            self.edge_drop_counts["right"] = 0

        if not self.motion.edge_event_active:
            if self.edge_drop_counts["left"] >= self.edge_debounce_cycles:
                print(f"[Edge] Left edge detected")
                self.edge_drop_counts["left"] = 0
                self.motion.handle_edge_event("left")
                self._add_edge_points("left")

            if self.edge_drop_counts["right"] >= self.edge_debounce_cycles:
                print(f"[Edge] Right edge detected")
                self.edge_drop_counts["right"] = 0
                self.motion.handle_edge_event("right")
                self._add_edge_points("right")

    def _add_edge_points(self, side: str):
        """Add edge detection points to map."""
        pose = self.ekf.pose()

        # Add point for the triggered sensor
        if side == "left":
            sensor_idx = I2C.LEFT_SENSOR_IDX
        else:
            sensor_idx = I2C.RIGHT_SENSOR_IDX

        # Sensor position in robot frame
        # Use first sensor lateral position for left, last for right
        if sensor_idx < len(GEOM.SENSOR_LAT):
            sensor_lat = GEOM.SENSOR_LAT[sensor_idx]
        else:
            # Fallback: use average of left/right positions
            sensor_lat = GEOM.SENSOR_LAT[0] if side == "left" else GEOM.SENSOR_LAT[-1]
        
        sensor_pos = (GEOM.SENSOR_FWD, sensor_lat)

        # Transform to world frame
        world_pos = transform_point(pose, sensor_pos)

        # Add to rectangle fit
        self.rect_fit.add_edge_point(world_pos)

        # Log
        self.logger.log_edge(time.time(), world_pos, pose)

    async def loop_map(self):
        """Mapping loop @ 20 Hz."""
        rate = RateTimer(20)
        optimize_counter = 0

        while self.running:
            self.map_timer.start()

            pose = self.ekf.pose()

            # Add pose graph nodes at regular intervals
            dx = pose[0] - self.last_node_pose[0]
            dy = pose[1] - self.last_node_pose[1]
            dist = (dx * dx + dy * dy) ** 0.5

            if dist >= ALG.NODE_SPACING:
                node_id = self.pose_graph.add_node(time.time(), pose)

                # Add odometry edge
                if node_id > 0:
                    # Compute relative pose
                    from .slam.frames import pose_difference

                    z_ij = pose_difference(self.last_node_pose, pose)

                    import numpy as np

                    Info = np.diag([100.0, 100.0, 50.0])
                    self.pose_graph.add_edge_odom(node_id - 1, node_id, z_ij, Info)

                self.last_node_pose = pose

            # Optimize periodically
            optimize_counter += 1
            if optimize_counter >= 100:  # Every 5 seconds
                self.pose_graph.optimize()
                optimize_counter = 0

            # Try rectangle fit
            if self.rect_fit.fit():
                if not self.fsm.rectangle_confident:
                    rect = self.rect_fit.get_rectangle()
                    if rect:
                        print(
                            f"[Map] Rectangle confident: {rect[3]:.2f} x {rect[4]:.2f} m"
                        )
                        self.fsm.rectangle_confident = True

                        # Build coverage lanes
                        self.coverage_planner.set_rectangle(rect)
                        lanes = self.coverage_planner.build_lanes()
                        print(f"[Map] Generated {len(lanes)} coverage lanes")

            self.map_timer.stop()
            rate.sleep()

    async def loop_ctrl(self):
        """Control loop @ 50 Hz."""
        rate = RateTimer(ALG.FUSE_HZ)
        last_v, last_omega = 0.0, 0.0

        while self.running:
            self.ctrl_timer.start()

            pose = self.ekf.pose()
            dt = 1.0 / ALG.FUSE_HZ

            # Handle edge events if active
            if self.motion.edge_event_active:
                v_cmd, omega_cmd = self.motion.update_edge_event(pose, dt)
            elif getattr(self.motion, "recovery_active", False):
                v_cmd, omega_cmd = self.motion.update_recovery(pose, dt)
            else:
                # Update FSM
                context = {
                    "start_signal": self.start_signal,
                    "rectangle_confident": self.fsm.rectangle_confident,
                    "coverage_ratio": (
                        self.swept_map.coverage_ratio(self.rect_fit.get_rectangle())
                        if self.rect_fit.is_confident
                        else 0.0
                    ),
                    "coverage_planner_complete": self.coverage_planner.is_complete(),
                    "error": False,
                }

                state = self.fsm.update(context)

                # Generate motion commands based on state
                if state == RobotState.WAIT_START:
                    v_cmd, omega_cmd = 0.0, 0.0

                elif state == RobotState.BOUNDARY_DISCOVERY:
                    v_cmd, omega_cmd = self.motion.cmd_boundary(
                        pose, self.sensor_context
                    )

                elif state == RobotState.COVERAGE:
                    # Follow coverage path
                    lane = self.coverage_planner.get_current_lane()
                    if lane:
                        v_cmd, omega_cmd = self.motion.cmd_follow_path(
                            pose, lane, self.swept_map
                        )

                        # Check if current lane is complete
                        if self.motion.is_path_complete():
                            # Advance to next waypoint (which may advance to next lane)
                            self.coverage_planner.advance_waypoint()
                            self.motion.reset_path_progress()
                            
                            # Check if all lanes are complete
                            if self.coverage_planner.is_complete():
                                print("[Coverage] All lanes complete")
                    else:
                        # No more lanes available
                        v_cmd, omega_cmd = 0.0, 0.0
                        self.motion.reset_path_progress()
                        if self.coverage_planner.is_complete():
                            print("[Coverage] Coverage planner reports complete")

                elif state == RobotState.DONE:
                    v_cmd, omega_cmd = 0.0, 0.0

                else:
                    v_cmd, omega_cmd = 0.0, 0.0

            # Apply velocity limits
            v_limited, omega_limited = self.limiter.limit(v_cmd, omega_cmd, dt)

            # Command steppers
            self.stepper.command(v_limited, omega_limited)
            self.stepper.update(dt)

            now = time.time()

            # Update swept map (only for forward motion)
            if v_limited > 0:
                ds = v_limited * dt
                self.swept_map.add_forward_sweep(pose, ds)
                self.last_progress_time = now
            elif abs(omega_limited) > 0.4:
                self.last_progress_time = now

            if (
                self.fsm.is_active()
                and not self.motion.edge_event_active
                and not getattr(self.motion, "recovery_active", False)
                and now - self.last_progress_time > ALG.WATCHDOG_TIMEOUT
                and now - self.last_recovery_time > ALG.WATCHDOG_COOLDOWN
            ):
                print("[Watchdog] Recovery maneuver triggered")
                self.motion.start_watchdog_recovery()
                self.last_recovery_time = now

            # Logging
            self.logger.log_telemetry(
                now,
                pose,
                (v_limited, omega_limited),
                self.sensor_context.get("sensors", []),
                self.motion.edge_event_active
                or getattr(self.motion, "recovery_active", False),
                self.fsm.get_state().name,
            )

            # Visualization
            if self.visualizer:
                poses = self.pose_graph.get_all_poses()
                edge_points = self.rect_fit.edge_points
                rectangle = self.rect_fit.get_rectangle()
                coverage_grid = self.swept_map.get_grid()
                bounds = (
                    self.swept_map.min_x,
                    self.swept_map.max_x,
                    self.swept_map.min_y,
                    self.swept_map.max_y,
                )

                # Update every 10th iteration to reduce overhead
                if rate._last_time % 0.2 < 0.02:  # ~Every 0.2 seconds
                    self.visualizer.update(
                        poses, edge_points, rectangle, coverage_grid, bounds
                    )

            last_v, last_omega = v_limited, omega_limited

            self.ctrl_timer.stop()
            rate.sleep()

            # Check if done
            if self.fsm.is_done():
                if not self.finish_alerted:
                    self._notify_finish_beep()
                    self.finish_alerted = True
                print("[Main] Mission complete!")
                self.running = False

    async def run_async(self):
        """Run main control loops."""
        self.running = True

        # Turn on vacuum after gesture start
        print("[Main] Starting vacuum")
        self.vacuum.on(duty=0.8)

        # Run loops concurrently
        try:
            await asyncio.gather(self.loop_sense(), self.loop_map(), self.loop_ctrl())
        except KeyboardInterrupt:
            print("\n[Main] Interrupted by user")
        finally:
            self.shutdown()

    def run(self):
        """Run robot (blocking)."""
        # Setup signal handler
        signal.signal(signal.SIGINT, self._signal_handler)

        print("[Main] Starting main loops...")
        print("  Press Ctrl+C to stop")

        if not self._await_gesture_start():
            self.shutdown()
            return

        # Run async event loop
        asyncio.run(self.run_async())

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

        # Print timing stats
        print("\n[Timing] Loop statistics:")
        self.sense_timer.print_stats()
        self.map_timer.print_stats()
        self.ctrl_timer.print_stats()

        # Save visualization
        if self.visualizer:
            self.visualizer.save("output_map.png")
            self.visualizer.close()

        # Cleanup peripherals
        if self.buzzer:
            self.buzzer.cleanup()
        if self.gesture:
            self.gesture.cleanup()

        # Cleanup GPIO
        gpio_manager.cleanup()

        print("[Shutdown] Complete")


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="Deskinator Robot Controller")
    parser.add_argument("--viz", action="store_true", help="Enable visualization")
    parser.add_argument("--calibrate", action="store_true", help="Run calibration")
    parser.add_argument("--scan-i2c", action="store_true", help="Scan I2C bus")

    args = parser.parse_args()

    robot = Deskinator(enable_viz=args.viz)

    if args.scan_i2c:
        robot.scan_i2c()
        return

    if args.calibrate:
        robot.calibrate_imu()
        robot.calibrate_sensors()
        return

    # Run main loop
    robot.run()


if __name__ == "__main__":
    main()
