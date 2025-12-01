"""
Visualization demo for wall-following and coverage algorithm.

Simulates a robot with two front-facing sensors following walls
and generating coverage paths.
"""

import numpy as np
import time
from typing import List, Tuple, Optional

try:
    from planning.coverage import (
        SimpleWallFollower,
        SimpleRectangleFit,
        CoveragePlanner,
    )
    from planning.map2d import SweptMap
    from utils.viz import Visualizer
    from config import GEOM, ALG, LIMS
except ImportError:
    import sys

    sys.path.insert(0, ".")
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


def simulate_wall_following(speed_multiplier: float = SPEED_MULTIPLIER):
    """Simulate wall-following algorithm.

    Args:
        speed_multiplier: Multiplier for all speeds (default: SPEED_MULTIPLIER)
    """
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

    # Create visualizer
    viz = Visualizer(figsize=(14, 8))

    # Simulation parameters
    dt = 0.05  # 20 Hz
    max_time = 120.0  # seconds
    t = 0.0

    print("\nStarting simulation...")
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

                # Create dummy coverage grid
                coverage_grid = None
                bounds = None

                status = f"State: {wall_follower.state.value}\n"
                status += f"Time: {t:.1f}s\n"
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

        print(f"\nWall-following complete after {t:.1f} seconds")
        print(f"Collected {len(rect_fit.edge_points)} edge points")

        # Final rectangle fit
        rect_fit.fit()
        rectangle = rect_fit.get_rectangle()

        if rectangle:
            print(f"Fitted rectangle: {rectangle[3]:.2f} x {rectangle[4]:.2f} m")
            print(f"  Center: ({rectangle[0]:.2f}, {rectangle[1]:.2f})")
            print(f"  Heading: {np.rad2deg(rectangle[2]):.1f}°")
        else:
            print("Failed to fit rectangle")

        # Final visualization update
        viz.update(
            poses=robot.trajectory,
            edge_points=rect_fit.edge_points,
            rectangle=rectangle,
            coverage_grid=None,
            swept_map_bounds=None,
            text_info=f"Complete!\nEdge points: {len(rect_fit.edge_points)}\nRectangle fitted: {rectangle is not None}",
            robot_state="COMPLETE",
            ground_truth_bounds=(table.min_x, table.max_x, table.min_y, table.max_y),
        )

        # Wait a bit to see final state
        time.sleep(2.0)

        return robot, wall_follower, rect_fit, table

    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
        return robot, wall_follower, rect_fit, table


def simulate_coverage(
    robot, rect_fit, table, speed_multiplier: float = SPEED_MULTIPLIER
):
    """Simulate coverage phase with boustrophedon paths.

    Args:
        robot: Simulated robot instance
        rect_fit: Rectangle fitter instance
        table: Table simulator instance
        speed_multiplier: Multiplier for all speeds (default: SPEED_MULTIPLIER)
    """
    print("\n" + "=" * 60)
    print("Coverage Simulation")
    print("=" * 60)

    rectangle = rect_fit.get_rectangle()
    if not rectangle:
        print("No rectangle available for coverage planning")
        return

    print(f"Planning coverage for rectangle: {rectangle[3]:.2f} x {rectangle[4]:.2f} m")

    # Create coverage planner
    planner = CoveragePlanner()
    planner.set_rectangle(rectangle)
    lanes = planner.build_lanes()

    print(f"Generated {len(lanes)} coverage lanes")

    # Create swept map
    swept_map = SweptMap()

    # Create visualizer
    viz = Visualizer(figsize=(14, 8))

    # Reset robot to start of first lane
    if lanes:
        start_waypoint = lanes[0][0]
        robot.pose = np.array([start_waypoint[0], start_waypoint[1], 0.0])
        robot.trajectory = [tuple(robot.pose)]

    # Simple pure pursuit controller
    def pure_pursuit(pose, waypoint, lookahead=0.15):
        """Simple pure pursuit controller."""
        x, y, theta = pose
        wx, wy = waypoint

        dx = wx - x
        dy = wy - y
        dist = np.sqrt(dx * dx + dy * dy)

        # If very close to waypoint, still allow small forward motion and turning
        # This prevents getting stuck when waypoints overlap or are very close
        if dist < 0.02:  # Very close - just turn towards next waypoint
            # Desired heading
            desired_theta = np.arctan2(dy, dx)
            dtheta = desired_theta - theta
            dtheta = ((dtheta + np.pi) % (2 * np.pi)) - np.pi  # Wrap to [-pi, pi]
            # Turn in place if heading error is large, otherwise allow small forward motion
            if abs(dtheta) > np.deg2rad(30):
                return 0.0, 2.0 * dtheta * speed_multiplier
            else:
                return (
                    LIMS.V_BASE * 0.3 * speed_multiplier,
                    2.0 * dtheta * speed_multiplier,
                )
        elif dist < 0.05:  # Close but not very close - slow forward motion
            desired_theta = np.arctan2(dy, dx)
            dtheta = desired_theta - theta
            dtheta = ((dtheta + np.pi) % (2 * np.pi)) - np.pi
            v = LIMS.V_BASE * 0.3 * speed_multiplier
            omega = 2.0 * dtheta * speed_multiplier
            omega = np.clip(
                omega,
                -LIMS.OMEGA_MAX * speed_multiplier,
                LIMS.OMEGA_MAX * speed_multiplier,
            )
            return v, omega

        # Desired heading
        desired_theta = np.arctan2(dy, dx)

        # Heading error
        dtheta = desired_theta - theta
        dtheta = ((dtheta + np.pi) % (2 * np.pi)) - np.pi  # Wrap to [-pi, pi]

        # Control
        v = LIMS.V_BASE if dist > lookahead else LIMS.V_BASE * 0.5
        v *= speed_multiplier
        omega = 2.0 * dtheta  # Proportional control
        omega = np.clip(
            omega, -LIMS.OMEGA_MAX * speed_multiplier, LIMS.OMEGA_MAX * speed_multiplier
        )

        return v, omega

    # Simulation parameters
    dt = 0.05
    max_time = 180.0
    t = 0.0

    print("\nStarting coverage simulation...")

    try:
        last_advance_time = (
            -1.0
        )  # Track when we last advanced to prevent rapid advances

        while t < max_time and not planner.is_complete():
            pose = robot.get_pose()

            # Get current waypoint
            current_lane = planner.get_current_lane()
            if not current_lane:
                break

            waypoint = current_lane[planner.current_waypoint_idx]

            # Control
            v, omega = pure_pursuit(pose, waypoint)

            # Check if waypoint reached
            dx = waypoint[0] - pose[0]
            dy = waypoint[1] - pose[1]
            dist = np.sqrt(dx * dx + dy * dy)

            # Advance waypoint if close enough, but prevent rapid advances (at least 0.1s between advances)
            # This prevents getting stuck when waypoints overlap (e.g., end of lane 0 = start of lane 1)
            if dist < 0.05 and (t - last_advance_time) > 0.1:
                planner.advance_waypoint()
                last_advance_time = t

                # After advancing, if the new waypoint is also very close (overlapping waypoints),
                # skip it immediately but only once per advance
                new_lane = planner.get_current_lane()
                if new_lane and planner.current_waypoint_idx < len(new_lane):
                    new_waypoint = new_lane[planner.current_waypoint_idx]
                    new_dx = new_waypoint[0] - pose[0]
                    new_dy = new_waypoint[1] - pose[1]
                    new_dist = np.sqrt(new_dx * new_dx + new_dy * new_dy)

                    # Skip overlapping waypoints (common in boustrophedon when lanes share endpoints)
                    if (
                        new_dist < 0.05
                        and planner.current_waypoint_idx < len(new_lane) - 1
                    ):
                        planner.advance_waypoint()

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

                status = f"Coverage Mode\n"
                status += f"Time: {t:.1f}s\n"
                status += f"Lane: {planner.current_lane_idx + 1}/{len(lanes)}\n"
                status += f"Coverage: {swept_map.coverage_ratio(rectangle):.1%}"

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
                )

            t += dt

        print(f"\nCoverage complete after {t:.1f} seconds")
        coverage_ratio = swept_map.coverage_ratio(rectangle)
        print(f"Coverage ratio: {coverage_ratio:.1%}")

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
        )

        # Wait to see final state
        time.sleep(3.0)

        # Save visualization
        viz.save("viz_demo_output.png")
        print("\nSaved visualization to viz_demo_output.png")

    except KeyboardInterrupt:
        print("\nCoverage simulation interrupted")
    finally:
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

    # Phase 1: Wall following
    robot, wall_follower, rect_fit, table = simulate_wall_following(speed_multiplier)

    # Phase 2: Coverage
    if rect_fit.get_rectangle():
        simulate_coverage(robot, rect_fit, table, speed_multiplier)
    else:
        print("\nSkipping coverage - no rectangle fitted")

    print("\n" + "=" * 60)
    print("Demo complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
