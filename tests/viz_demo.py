"""
Visualization demo for wall-following and coverage algorithm.

Simulates a robot with two front-facing sensors following walls
and generating coverage paths.
"""

import numpy as np
import time
from typing import List, Tuple, Optional

import sys
from pathlib import Path

# Add parent directory to path for imports when running from tests/
parent_dir = Path(__file__).parent.parent
if str(parent_dir) not in sys.path:
    sys.path.insert(0, str(parent_dir))

from planning.coverage import (
    SimpleWallFollower,
    SimpleRectangleFit,
    CoveragePlanner,
)
from planning.map2d import SweptMap
from utils.viz import Visualizer
from config import GEOM, ALG, LIMS

# Speed multiplier for simulation (1.0 = real-time, >1.0 = faster, <1.0 = slower)
SPEED_MULTIPLIER = 1.0


class SimulatedRobot:
    """Simulated robot with sensors and motion."""

    def __init__(
        self,
        start_pose: Tuple[float, float, float],
        table_bounds: Tuple[float, float, float, float],
    ):
        """
        Initialize simulated robot.

        Args:
            start_pose: Initial pose (x, y, theta)
            table_bounds: (min_x, max_x, min_y, max_y) table boundaries
        """
        self.pose = np.array(start_pose, dtype=float)
        self.table_bounds = table_bounds
        self.min_x, self.max_x, self.min_y, self.max_y = table_bounds

        # Trajectory history
        self.trajectory: List[Tuple[float, float, float]] = [tuple(self.pose)]

    def update(self, v: float, omega: float, dt: float):
        """
        Update robot pose based on velocity commands.

        Args:
            v: Forward velocity (m/s)
            omega: Angular velocity (rad/s)
            dt: Time step (s)
        """
        x, y, theta = self.pose

        # Simple unicycle model
        if abs(omega) < 1e-6:
            # Straight line
            dx = v * np.cos(theta) * dt
            dy = v * np.sin(theta) * dt
            dtheta = 0.0
        else:
            # Arc motion
            radius = v / omega
            dtheta = omega * dt
            dx = radius * (np.sin(theta + dtheta) - np.sin(theta))
            dy = radius * (-np.cos(theta + dtheta) + np.cos(theta))

        self.pose[0] += dx
        self.pose[1] += dy
        self.pose[2] = (self.pose[2] + dtheta) % (2 * np.pi)

        self.trajectory.append(tuple(self.pose))

    def read_sensors(self) -> Tuple[bool, bool]:
        """
        Read proximity sensors.

        Returns:
            (left_sensor_on, right_sensor_on) - True if sensor detects table
        """
        x, y, theta = self.pose

        # Sensor positions in world frame
        sensor_fwd = GEOM.SENSOR_FWD

        # Left sensor
        if len(GEOM.SENSOR_LAT) > 0:
            left_lat = GEOM.SENSOR_LAT[0]
        else:
            left_lat = 0.1

        left_x = x + sensor_fwd * np.cos(theta) - left_lat * np.sin(theta)
        left_y = y + sensor_fwd * np.sin(theta) + left_lat * np.cos(theta)

        # Right sensor
        if len(GEOM.SENSOR_LAT) > 1:
            right_lat = GEOM.SENSOR_LAT[1]
        else:
            right_lat = -0.1

        right_x = x + sensor_fwd * np.cos(theta) - right_lat * np.sin(theta)
        right_y = y + sensor_fwd * np.sin(theta) + right_lat * np.cos(theta)

        # Check if sensors are within table bounds
        left_on = (
            self.min_x <= left_x <= self.max_x and self.min_y <= left_y <= self.max_y
        )
        right_on = (
            self.min_x <= right_x <= self.max_x and self.min_y <= right_y <= self.max_y
        )

        return left_on, right_on

    def get_pose(self) -> Tuple[float, float, float]:
        """Get current pose."""
        return tuple(self.pose)


class TableSimulator:
    """Simulates a rectangular table with boundaries."""

    def __init__(
        self,
        center: Tuple[float, float],
        width: float,
        height: float,
        heading: float = 0.0,
    ):
        """
        Initialize table simulator.

        Args:
            center: Table center (x, y)
            width: Table width (m)
            height: Table height (m)
            heading: Table rotation (rad)
        """
        self.center = np.array(center)
        self.width = width
        self.height = height
        self.heading = heading

        # Compute bounds
        c, s = np.cos(heading), np.sin(heading)
        R = np.array([[c, -s], [s, c]])

        corners_local = np.array(
            [
                [-width / 2, -height / 2],
                [width / 2, -height / 2],
                [width / 2, height / 2],
                [-width / 2, height / 2],
            ]
        )

        corners_world = self.center + corners_local @ R.T

        self.min_x = corners_world[:, 0].min()
        self.max_x = corners_world[:, 0].max()
        self.min_y = corners_world[:, 1].min()
        self.max_y = corners_world[:, 1].max()

        self.bounds = (self.min_x, self.max_x, self.min_y, self.max_y)

    def get_bounds(self) -> Tuple[float, float, float, float]:
        """Get table bounds."""
        return self.bounds

    def get_ground_truth_rect(self) -> Tuple[float, float, float, float, float]:
        """Get ground truth rectangle for visualization."""
        return (self.center[0], self.center[1], self.heading, self.width, self.height)


def simulate_wall_following(
    speed_multiplier: float = SPEED_MULTIPLIER,
    viz: Optional[Visualizer] = None,
    swept_map: Optional[SweptMap] = None,
    verbose: bool = True,
) -> Tuple[SimulatedRobot, SimpleWallFollower, SimpleRectangleFit, TableSimulator, Optional[Visualizer], SweptMap, float]:
    """Simulate wall-following algorithm.

    Args:
        speed_multiplier: Multiplier for all speeds (default: SPEED_MULTIPLIER)
        viz: Optional Visualizer instance to reuse (creates new if None)
        swept_map: Optional SweptMap instance to reuse (creates new if None)
        verbose: Whether to print progress messages

    Returns:
        Tuple containing:
        - robot: SimulatedRobot instance
        - wall_follower: SimpleWallFollower instance
        - rect_fit: SimpleRectangleFit instance
        - table: TableSimulator instance
        - viz: Visualizer instance (or None)
        - swept_map: SweptMap instance
        - sim_time: Total simulation time in seconds
    """
    if verbose:
        print("=" * 60)
        print("Wall-Following Simulation")
        print("=" * 60)

    # Create table
    table = TableSimulator(center=(0.0, 0.0), width=2.0, height=2.0, heading=0.0)

    # Start robot at random point inside table
    start_x = np.random.uniform(table.min_x + 0.2, table.max_x - 0.2)
    start_y = np.random.uniform(table.min_y + 0.2, table.max_y - 0.2)
    start_theta = np.random.uniform(0, 2 * np.pi)
    start_pose = (start_x, start_y, start_theta)

    start_pose = (start_x, start_y, start_theta)

    if verbose:
        print(
            f"Starting pose: ({start_x:.2f}, {start_y:.2f}, {np.rad2deg(start_theta):.1f}°)"
        )

    # Create robot
    robot = SimulatedRobot(start_pose, table.get_bounds())

    # Create wall follower
    wall_follower = SimpleWallFollower(
        forward_speed=ALG.BOUNDARY_SPEED * speed_multiplier,
        turn_speed=LIMS.OMEGA_MAX * speed_multiplier,
    )

    # Create rectangle fitter
    rect_fit = SimpleRectangleFit()

    # Create or reuse visualizer
    if viz is None:
        viz = Visualizer(figsize=(14, 8))

    # Create or reuse swept map
    if swept_map is None:
        swept_map = SweptMap()

    # Simulation parameters
    dt = 0.05  # 20 Hz
    max_time = 300.0  # Safety timeout (5 minutes) - should not be reached with rotation-based completion
    t = 0.0

    if verbose:
        print("\nStarting simulation...")
        print("Boundary discovery will complete after 360° of total rotation")
        print("Press Ctrl+C to stop early")

    try:
        while t < max_time and not wall_follower.is_complete():
            # Read sensors
            left_on, right_on = robot.read_sensors()

            # Update wall follower
            pose = robot.get_pose()
            v, omega = wall_follower.update(pose, left_on, right_on, dt)

            # Update robot
            robot.update(v, omega, dt)

            # Update swept map during boundary search (track swept area)
            if v > 0:
                swept_map.add_forward_sweep(pose, v * dt)

            # Add edge points to rectangle fitter
            edge_points = wall_follower.get_edge_points()
            if edge_points:
                # Only add new points
                current_count = len(rect_fit.edge_points)
                if len(edge_points) > current_count:
                    for point in edge_points[current_count:]:
                        rect_fit.add_edge_point(point)

            # Fit rectangle periodically
            if len(rect_fit.edge_points) > 4 and len(rect_fit.edge_points) % 5 == 0:
                rect_fit.fit()

            # Update visualization every 5 steps (4 Hz)
            if int(t / dt) % 5 == 0:
                poses = robot.trajectory
                edge_points_list = rect_fit.edge_points
                rectangle = rect_fit.get_rectangle()
                ground_truth = table.get_ground_truth_rect()

                # Get coverage grid from swept map
                coverage_grid = swept_map.get_grid()
                bounds = (
                    swept_map.min_x,
                    swept_map.max_x,
                    swept_map.min_y,
                    swept_map.max_y,
                )

                total_rotation_deg = wall_follower.get_total_rotation_degrees()
                rotation_progress = min(100.0, (total_rotation_deg / 360.0) * 100.0)

                status = f"State: {wall_follower.state.value}\n"
                status += f"Time: {t:.1f}s\n"
                status += (
                    f"Rotation: {total_rotation_deg:.1f}° ({rotation_progress:.1f}%)\n"
                )
                status += f"Edge points: {len(edge_points_list)}\n"
                if rectangle:
                    status += f"Rectangle: {rectangle[3]:.2f} x {rectangle[4]:.2f} m"

                viz.update(
                    poses=poses,
                    edge_points=edge_points_list,
                    rectangle=rectangle,
                    coverage_grid=coverage_grid,
                    swept_map_bounds=bounds,
                    text_info=status,
                    robot_state=wall_follower.state.value,
                    ground_truth_bounds=(
                        table.min_x,
                        table.max_x,
                        table.min_y,
                        table.max_y,
                    ),
                )

            t += dt

        total_rotation_deg = wall_follower.get_total_rotation_degrees()
        if verbose:
            print(f"\nWall-following complete after {t:.1f} seconds")
            print(
                f"Total rotation: {total_rotation_deg:.1f}° ({total_rotation_deg/360.0*100:.1f}% of full loop)"
            )
            print(f"Collected {len(rect_fit.edge_points)} edge points")

        # Final rectangle fit
        rect_fit.fit()
        rectangle = rect_fit.get_rectangle()

        if rectangle:
            if verbose:
                print(f"Fitted rectangle: {rectangle[3]:.2f} x {rectangle[4]:.2f} m")
                print(f"  Center: ({rectangle[0]:.2f}, {rectangle[1]:.2f})")
                print(f"  Heading: {np.rad2deg(rectangle[2]):.1f}°")
        else:
            if verbose:
                print("Failed to fit rectangle")

        # Final visualization update
        coverage_grid = swept_map.get_grid()
        bounds = (
            swept_map.min_x,
            swept_map.max_x,
            swept_map.min_y,
            swept_map.max_y,
        )
        viz.update(
            poses=robot.trajectory,
            edge_points=rect_fit.edge_points,
            rectangle=rectangle,
            coverage_grid=coverage_grid,
            swept_map_bounds=bounds,
            text_info=f"Complete!\nEdge points: {len(rect_fit.edge_points)}\nRectangle fitted: {rectangle is not None}",
            robot_state="COMPLETE",
            ground_truth_bounds=(table.min_x, table.max_x, table.min_y, table.max_y),
        )

        # Wait a bit to see final state
        time.sleep(2.0)

        return robot, wall_follower, rect_fit, table, viz, swept_map, t

    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
        return robot, wall_follower, rect_fit, table, viz, swept_map, t


def simulate_coverage(
    robot,
    rect_fit,
    table,
    speed_multiplier: float = SPEED_MULTIPLIER,
    viz: Optional[Visualizer] = None,
    swept_map: Optional[SweptMap] = None,
    verbose: bool = True,
) -> Tuple[bool, float, Optional[CoveragePlanner]]:
    """Simulate coverage phase with boustrophedon paths.

    Args:
        robot: Simulated robot instance
        rect_fit: Rectangle fitter instance
        table: Table simulator instance
        speed_multiplier: Multiplier for all speeds (default: SPEED_MULTIPLIER)
        viz: Optional Visualizer instance to reuse (creates new if None)
        swept_map: Optional SweptMap instance to reuse (creates new if None)
        verbose: Whether to print progress messages

    Returns:
        Tuple containing:
        - success: True if coverage completed successfully
        - sim_time: Total simulation time in seconds
        - planner: CoveragePlanner instance (or None if failed to start)
    """
    if verbose:
        print("\n" + "=" * 60)
        print("Coverage Simulation")
        print("=" * 60)

    rectangle = rect_fit.get_rectangle()
    if not rectangle:
        if verbose:
            print("No rectangle available for coverage planning")
        return False, 0.0, None

    if verbose:
        print(f"Planning coverage for rectangle: {rectangle[3]:.2f} x {rectangle[4]:.2f} m")

    # Create coverage planner
    planner = CoveragePlanner()
    planner.set_rectangle(rectangle)
    
    # Pass current robot pose to optimize start point
    lanes = planner.build_lanes(start_pose=robot.get_pose())

    if verbose:
        print(f"Generated {len(lanes)} coverage lanes")

    # Reuse or create swept map
    if swept_map is None:
        swept_map = SweptMap()

    # Reuse or create visualizer
    if viz is None:
        viz = Visualizer(figsize=(14, 8))

    # Do NOT reset robot pose - continue from where wall following left off
    # This ensures smooth transition without teleportation
    
    # Add a transition waypoint to the start of the first lane if needed
    if lanes:
        first_waypoint_data = planner.get_current_waypoint()
        if first_waypoint_data:
            # Check if we need to add explicit path to start
            # The planner path already includes transitions, but we might be far from the first point
            pass

    # Simple pure pursuit controller
    def pure_pursuit(pose, waypoint, lookahead=0.15):
        """Simple pure pursuit controller with free rotation when stationary."""
        x, y, theta = pose
        wx, wy = waypoint

        dx = wx - x
        dy = wy - y
        dist = np.sqrt(dx * dx + dy * dy)

        # Desired heading
        desired_theta = np.arctan2(dy, dx)

        # Heading error
        dtheta = desired_theta - theta
        dtheta = ((dtheta + np.pi) % (2 * np.pi)) - np.pi  # Wrap to [-pi, pi]

        # Base parameters
        v_base = LIMS.V_BASE * speed_multiplier
        omega_max = LIMS.OMEGA_MAX * speed_multiplier
        
        # Threshold for turning in place vs moving
        # If heading error is large, turn in place first (free rotation)
        turn_in_place_threshold = np.deg2rad(30)
        
        if abs(dtheta) > turn_in_place_threshold:
            # Turn in place - NO constraints on omega when stationary
            v = 0.0
            # Use larger gain for faster turning when stationary
            kp_omega_stationary = 3.0
            omega = kp_omega_stationary * dtheta
            # Clamp omega to max
            omega = np.clip(omega, -omega_max, omega_max)
        else:
            # Moving forward - apply velocity scaling based on remaining heading error
            # Scale velocity smoothly: 1.0 at 0 error, 0.3 at threshold
            min_scale = 0.3
            scale = min_scale + (1.0 - min_scale) * (1.0 - abs(dtheta) / turn_in_place_threshold)
            v = v_base * scale
            
            # If we are very close to waypoint, slow down further
            if dist < 0.1:
                v *= 0.5
            
            # Angular velocity control while moving (gentler)
            kp_omega_moving = 2.0
            omega = kp_omega_moving * dtheta
            omega = np.clip(omega, -omega_max, omega_max)
        
        return v, omega

    # Simulation parameters
    dt = 0.05
    max_time = 600.0  # Large safety timeout (10 minutes) - should not be reached
    t = 0.0

    print("\nStarting coverage simulation...")
    print(f"Total waypoints in path: {len(planner.path)}")

    try:
        last_advance_time = (
            -1.0
        )  # Track when we last advanced to prevent rapid advances

        # Check if robot starts at first waypoint - if so, advance if oriented correctly
        initial_pose = robot.get_pose()
        first_waypoint = planner.get_current_waypoint()
        if first_waypoint:
            wx, wy, wtheta = first_waypoint
            dx = wx - initial_pose[0]
            dy = wy - initial_pose[1]
            dist = np.sqrt(dx * dx + dy * dy)
            dtheta = wtheta - initial_pose[2]
            dtheta = ((dtheta + np.pi) % (2 * np.pi)) - np.pi
            if dist < 0.1 and abs(dtheta) < np.deg2rad(
                15
            ):  # Close in position and roughly oriented
                print(f"[Coverage] Starting at waypoint, advancing if needed...")
                # Will be handled in main loop

        # Continue until path is complete (with safety timeout)
        while not planner.is_complete() and t < max_time:
            pose = robot.get_pose()

            # Get current waypoint
            waypoint_data = planner.get_current_waypoint()
            if not waypoint_data:
                break

            wx, wy, wtheta = waypoint_data
            waypoint = (wx, wy)

            # Compute distance and orientation error
            dx = waypoint[0] - pose[0]
            dy = waypoint[1] - pose[1]
            dist = np.sqrt(dx * dx + dy * dy)

            dtheta = wtheta - pose[2]
            dtheta = ((dtheta + np.pi) % (2 * np.pi)) - np.pi
            abs_dtheta = abs(dtheta)

            POSITION_TOLERANCE = 0.05  # 5cm
            ORIENTATION_TOLERANCE = np.deg2rad(8)  # 8 degrees

            # Check if waypoint reached (position and orientation)
            position_reached = dist < POSITION_TOLERANCE
            orientation_reached = abs_dtheta < ORIENTATION_TOLERANCE

            # Advance waypoint if both position and orientation are reached
            if (
                position_reached
                and orientation_reached
                and (t - last_advance_time) > 0.1
            ):
                planner.advance_waypoint()
                last_advance_time = t

                # Check if we've completed all waypoints
                if planner.is_complete():
                    if verbose:
                        print(f"\n✓ Reached final waypoint! Path complete.")
                    break

                # Skip control update this iteration since we advanced
                continue

            # Control logic: prioritize orientation when close to waypoint
            # Control logic: Drive-Then-Turn
            # 1. If far from waypoint, drive towards it (heading = bearing)
            # 2. If close to waypoint, turn to desired orientation (heading = wtheta)
            
            target_heading = wtheta
            is_approach_phase = not position_reached
            
            if is_approach_phase:
                # We are approaching the waypoint - target heading is the bearing to it
                target_heading = np.arctan2(dy, dx)
            
            # Compute heading error relative to CURRENT target
            heading_error = target_heading - pose[2]
            heading_error = ((heading_error + np.pi) % (2*np.pi)) - np.pi
            
            if not is_approach_phase:
                # ARRIVAL PHASE: We are at the position, now align to final orientation
                if not orientation_reached:
                    v = 0.0  # Stop forward motion
                    # Use higher gain and allow full omega range when v=0
                    turn_gain = 3.0
                    omega = turn_gain * heading_error
                    omega_max_stationary = LIMS.OMEGA_MAX * 2.0
                    omega = np.clip(omega, -omega_max_stationary, omega_max_stationary)
                else:
                    # Both reached but not advanced yet
                    v = 0.0
                    omega = 0.0
            else:
                # APPROACH PHASE: Drive towards the waypoint
                # Use better strategy: if heading error is large, turn in place first
                turn_in_place_threshold = np.deg2rad(30)  # 30 degrees
                
                if abs(heading_error) > turn_in_place_threshold:
                    # Large heading error - turn in place first
                    v = 0.0
                    turn_gain = 3.0
                    omega = turn_gain * heading_error
                    omega_max_stationary = LIMS.OMEGA_MAX * 2.0
                    omega = np.clip(omega, -omega_max_stationary, omega_max_stationary)
                else:
                    # Small heading error - move forward with heading correction
                    
                    # Forward speed (reduce if heading error is large)
                    forward_speed_base = LIMS.V_BASE * speed_multiplier
                    # Scale speed based on heading error
                    heading_scale = 1.0 - (abs(heading_error) / turn_in_place_threshold) * 0.5
                    v = forward_speed_base * max(0.5, heading_scale)
                    
                    # If we are very close to waypoint, slow down further
                    if dist < 0.1:
                        v *= 0.5
                    
                    # Angular velocity to correct heading toward desired orientation
                    omega = 2.0 * heading_error  # Proportional control
                    omega = np.clip(omega, -LIMS.OMEGA_MAX * speed_multiplier, LIMS.OMEGA_MAX * speed_multiplier)

            # Update robot
            robot.update(v, omega, dt)

            # Update swept map
            if v > 0:
                swept_map.add_forward_sweep(pose, v * dt)

            # Update visualization every 5 steps
            if int(t / dt) % 5 == 0:
                coverage_grid = swept_map.get_grid()
                bounds = (
                    swept_map.min_x,
                    swept_map.max_x,
                    swept_map.min_y,
                    swept_map.max_y,
                )

                # Get current lane index
                current_lane_idx = planner.get_current_lane_index()
                if current_lane_idx is None:
                    current_lane_idx = 0
                else:
                    current_lane_idx += 1  # 1-indexed for display

                status = f"Coverage Mode\n"
                status += f"Time: {t:.1f}s\n"
                status += f"Waypoint: {planner.current_waypoint_idx + 1}/{len(planner.path)}\n"
                status += f"Lane: {current_lane_idx}/{len(lanes)}\n"
                status += f"Coverage: {swept_map.coverage_ratio(rectangle):.1%}"

                # Get inset rectangle for coverage area visualization
                coverage_area_rect = planner.get_inset_rectangle()

                viz.update(
                    poses=robot.trajectory,
                    edge_points=rect_fit.edge_points,
                    rectangle=rectangle,
                    coverage_grid=coverage_grid,
                    swept_map_bounds=bounds,
                    text_info=status,
                    robot_state="COVERAGE",
                    ground_truth_bounds=(
                        table.min_x,
                        table.max_x,
                        table.min_y,
                        table.max_y,
                    ),
                    coverage_lanes=lanes,
                    coverage_area_rect=coverage_area_rect,
                )

            t += dt

        # Check completion reason
        if planner.is_complete():
            if verbose:
                print(f"\n✓ Coverage path completed successfully after {t:.1f} seconds")
                print(f"  Completed all {len(planner.path)} waypoints")
        else:
            if verbose:
                print(f"\n⚠ Coverage simulation stopped after {t:.1f} seconds (timeout)")
                print(
                    f"  Completed {planner.current_waypoint_idx}/{len(planner.path)} waypoints"
                )

        coverage_ratio = swept_map.coverage_ratio(rectangle)
        if verbose:
            print(f"Coverage ratio: {coverage_ratio:.1%}")

        # Get inset rectangle for coverage area visualization
        coverage_area_rect = planner.get_inset_rectangle()

        # Final visualization
        viz.update(
            poses=robot.trajectory,
            edge_points=rect_fit.edge_points,
            rectangle=rectangle,
            coverage_grid=swept_map.get_grid(),
            swept_map_bounds=(
                swept_map.min_x,
                swept_map.max_x,
                swept_map.min_y,
                swept_map.max_y,
            ),
            text_info=f"Complete!\nCoverage: {coverage_ratio:.1%}",
            robot_state="DONE",
            ground_truth_bounds=(table.min_x, table.max_x, table.min_y, table.max_y),
            coverage_area_rect=coverage_area_rect,
        )

        # Wait to see final state
        time.sleep(3.0)

        # Save visualization
        if verbose:
            viz.save("viz_demo_output.png")
            print("\nSaved visualization to viz_demo_output.png")

        return planner.is_complete(), t, planner

    except KeyboardInterrupt:
        print("\nCoverage simulation interrupted")
        return False, t, planner
    finally:
        if verbose:
            viz.close()


def main(speed_multiplier: float = SPEED_MULTIPLIER):
    """Main demo function.

    Args:
        speed_multiplier: Multiplier for all speeds (default: SPEED_MULTIPLIER)
    """
    print("\n" + "=" * 60)
    print("Deskinator Visualization Demo")
    print("=" * 60)
    print("\nThis demo simulates:")
    print("1. Wall-following boundary discovery")
    print("2. Rectangle fitting from edge points")
    print("3. Boustrophedon coverage path planning")
    print("4. Coverage execution")
    if speed_multiplier != 1.0:
        print(f"\nSpeed multiplier: {speed_multiplier}x")
    print("\n" + "=" * 60)

    # Create shared visualizer and swept map for both phases
    viz = Visualizer(figsize=(14, 8))
    swept_map = SweptMap()

    # Phase 1: Wall following
    robot, wall_follower, rect_fit, table, _, _, _ = simulate_wall_following(
        speed_multiplier, viz=viz, swept_map=swept_map
    )

    # Phase 2: Coverage
    if rect_fit.get_rectangle():
        simulate_coverage(
            robot, rect_fit, table, speed_multiplier, viz=viz, swept_map=swept_map
        )
    else:
        print("\nSkipping coverage - no rectangle fitted")

    print("\n" + "=" * 60)
    print("Demo complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
