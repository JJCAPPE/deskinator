"""
Robot performance testing framework.

Runs multiple trials of table cleaning simulation and collects
performance metrics. Exports results to Excel for analysis.

Usage:
    python robot_testing.py --trials 25 --output results.xlsx
"""

import numpy as np
import argparse
import time
from typing import List, Tuple, Optional, Dict
import sys
from pathlib import Path

# Add parent directory to path for imports when running from tests/
parent_dir = Path(__file__).parent.parent
if str(parent_dir) not in sys.path:
    sys.path.insert(0, str(parent_dir))
# Add tests directory to path for local imports
tests_dir = Path(__file__).parent
if str(tests_dir) not in sys.path:
    sys.path.insert(0, str(tests_dir))

# Import our modules
from planning.coverage import (
    SimpleWallFollower,
    SimpleRectangleFit,
    CoveragePlanner,
)
from planning.map2d import SweptMap
from utils.viz import Visualizer
from config import GEOM, ALG, LIMS
from analysis_utils import calculate_rectangle_error, calculate_trajectory_distance

# Import viz_demo components for robot simulation
from viz_demo import SimulatedRobot, TableSimulator


# Sentinel object to indicate visualization should be disabled
_NO_VIZ = object()


def run_single_trial(
    trial_num: int,
    speed_multiplier: float = 2.0,
    verbose: bool = False,
    viz: Optional[Visualizer] = _NO_VIZ,
) -> Dict:
    """
    Run a single trial and collect all metrics.

    Args:
        trial_num: Trial number for logging
        speed_multiplier: Speed multiplier for simulation (higher = faster, default: 2.0)
        verbose: Print progress messages
        viz: Optional Visualizer instance for real-time visualization

    Returns:
        Dictionary with all collected metrics
    """
    if verbose:
        print(f"\nTrial {trial_num}:")
        print("-" * 40)

    # Create table (standard 2m x 2m)
    table = TableSimulator(center=(0.0, 0.0), width=2.0, height=2.0, heading=0.0)
    ground_truth = table.get_ground_truth_rect()

    # Random start position
    start_x = np.random.uniform(table.min_x + 0.2, table.max_x - 0.2)
    start_y = np.random.uniform(table.min_y + 0.2, table.max_y - 0.2)
    start_theta = np.random.uniform(0, 2 * np.pi)
    start_pose = (start_x, start_y, start_theta)

    if verbose:
        print(
            f"  Start: ({start_x:.2f}, {start_y:.2f}, {np.rad2deg(start_theta):.1f}°)"
        )

    # Create robot
    robot = SimulatedRobot(start_pose, table.get_bounds())

    # Create wall follower - match viz_demo.py exactly: multiply BOTH speeds
    wall_follower = SimpleWallFollower(
        forward_speed=ALG.BOUNDARY_SPEED * speed_multiplier,
        turn_speed=LIMS.OMEGA_MAX * speed_multiplier,
    )

    # Create rectangle fitter
    rect_fit = SimpleRectangleFit()

    # Create swept map
    swept_map = SweptMap()

    # Visualizer is passed from main (or None if --no-viz was used)
    # No need to create one here - it's created in main and reused for all trials

    # Simulation parameters
    dt = 0.05
    max_time = 300.0

    # === PHASE 1: Boundary Discovery ===
    t_boundary_start = time.time()
    t = 0.0

    while t < max_time and not wall_follower.is_complete():
        # Read sensors
        left_on, right_on = robot.read_sensors()

        # Update wall follower
        pose = robot.get_pose()
        v, omega = wall_follower.update(pose, left_on, right_on, dt)

        # Update robot
        robot.update(v, omega, dt)

        # Update swept map
        if v > 0:
            swept_map.add_forward_sweep(pose, v * dt)

        # Add edge points
        edge_points = wall_follower.get_edge_points()
        if edge_points:
            current_count = len(rect_fit.edge_points)
            if len(edge_points) > current_count:
                for point in edge_points[current_count:]:
                    rect_fit.add_edge_point(point)

        # Fit rectangle periodically
        if len(rect_fit.edge_points) > 4 and len(rect_fit.edge_points) % 5 == 0:
            rect_fit.fit()

        # Update visualization every 5 steps (4 Hz) - match viz_demo.py exactly
        if viz is not None and int(t / dt) % 5 == 0:
            poses = robot.trajectory
            edge_points_list = rect_fit.edge_points
            rectangle = rect_fit.get_rectangle()
            ground_truth_rect = table.get_ground_truth_rect()

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

            status = f"Trial {trial_num} - Boundary Discovery\n"
            status += f"State: {wall_follower.state.value}\n"
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

    # Final rectangle fit
    rect_fit.fit()
    rectangle = rect_fit.get_rectangle()

    boundary_time = time.time() - t_boundary_start
    boundary_sim_time = t

    if verbose:
        print(
            f"  Boundary discovery: {boundary_sim_time:.1f}s (wall time: {boundary_time:.2f}s)"
        )
        print(f"  Edge points: {len(rect_fit.edge_points)}")

    # Final visualization update for boundary phase
    if viz is not None:
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
            text_info=f"Trial {trial_num} - Boundary Complete!\nEdge points: {len(rect_fit.edge_points)}\nRectangle fitted: {rectangle is not None}",
            robot_state="COMPLETE",
            ground_truth_bounds=(table.min_x, table.max_x, table.min_y, table.max_y),
        )

    # Track trajectory at boundary completion
    boundary_trajectory_length = len(robot.trajectory)

    # === PHASE 2: Coverage ===
    coverage_success = False
    coverage_time_real = 0.0
    coverage_sim_time = 0.0

    if rectangle:
        t_coverage_start = time.time()
        t = 0.0

        # Create coverage planner
        planner = CoveragePlanner()
        planner.set_rectangle(rectangle)
        lanes = planner.build_lanes(start_pose=robot.get_pose())

        if verbose:
            print(f"  Coverage lanes: {len(lanes)}")

        # Pure pursuit controller - match viz_demo.py exactly
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
                scale = min_scale + (1.0 - min_scale) * (
                    1.0 - abs(dtheta) / turn_in_place_threshold
                )
                v = v_base * scale

                # If we are very close to waypoint, slow down further
                if dist < 0.1:
                    v *= 0.5

                # Angular velocity control while moving (gentler)
                kp_omega_moving = 2.0
                omega = kp_omega_moving * dtheta
                omega = np.clip(omega, -omega_max, omega_max)

            return v, omega

        # Coverage execution
        max_coverage_time = (
            600.0  # Large safety timeout (10 minutes) - should not be reached
        )
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
                if verbose:
                    print(f"  [Coverage] Starting at waypoint, advancing if needed...")
                # Will be handled in main loop

        # Continue until path is complete (with safety timeout)
        while not planner.is_complete() and t < max_coverage_time:
            pose = robot.get_pose()
            waypoint_data = planner.get_current_waypoint()

            if not waypoint_data:
                break

            wx, wy, wtheta = waypoint_data
            waypoint = (wx, wy)

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
                    break

                # Skip control update this iteration since we advanced
                continue

            # Control logic: prioritize orientation when close to waypoint
            if position_reached:
                # At waypoint position - turn in place to match orientation
                if not orientation_reached:
                    v = 0.0  # Stop forward motion
                    omega = 1.5 * dtheta  # Turn in place (smooth)
                    omega = np.clip(omega, -1.5, 1.5)
                else:
                    # Both reached but not advanced yet (shouldn't happen, but safe)
                    v = 0.0
                    omega = 0.0
            elif dist < 0.15:  # Approaching waypoint - start considering orientation
                # Blend between pure pursuit and orientation correction
                v_pp, omega_pp = pure_pursuit(pose, waypoint)

                # Reduce forward speed if orientation error is large
                orientation_factor = max(0.3, 1.0 - abs_dtheta / np.deg2rad(45))
                v = v_pp * orientation_factor

                # Blend angular velocities: pure pursuit + orientation correction
                omega_orient = 1.0 * dtheta  # Orientation correction
                omega = 0.5 * omega_pp + 0.5 * omega_orient
                omega = np.clip(omega, -1.5, 1.5)
            else:
                # Far from waypoint - use pure pursuit
                v, omega = pure_pursuit(pose, waypoint)

            # Update robot
            robot.update(v, omega, dt)

            # Update swept map
            if v > 0:
                swept_map.add_forward_sweep(pose, v * dt)

            # Update visualization every 5 steps - match viz_demo.py exactly
            if viz is not None and int(t / dt) % 5 == 0:
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

                status = f"Trial {trial_num} - Coverage Mode\n"
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

        coverage_time_real = time.time() - t_coverage_start
        coverage_sim_time = t
        coverage_success = planner.is_complete()

        if verbose:
            print(
                f"  Coverage: {coverage_sim_time:.1f}s (wall time: {coverage_time_real:.2f}s)"
            )
            print(f"  Complete: {coverage_success}")

        # Final visualization update for coverage phase
        if viz is not None:
            coverage_ratio = swept_map.coverage_ratio(rectangle)
            coverage_area_rect = planner.get_inset_rectangle()

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
                text_info=f"Trial {trial_num} - Complete!\nCoverage: {coverage_ratio:.1%}",
                robot_state="DONE",
                ground_truth_bounds=(
                    table.min_x,
                    table.max_x,
                    table.min_y,
                    table.max_y,
                ),
                coverage_area_rect=coverage_area_rect,
            )

    # === Collect Metrics ===
    metrics = {}

    # 1. Points on edge found
    metrics["edge_points"] = len(rect_fit.edge_points)

    # 2. Rectangle fitting error (%)
    metrics["rect_error_pct"] = calculate_rectangle_error(rectangle, ground_truth)

    # 3. Coverage % (full rectangle)
    if rectangle:
        metrics["coverage_full_pct"] = swept_map.coverage_ratio(rectangle) * 100.0
    else:
        metrics["coverage_full_pct"] = 0.0

    # 4. Coverage % (inset rectangle)
    if rectangle:
        planner_temp = CoveragePlanner()
        planner_temp.set_rectangle(rectangle)
        inset_rect = planner_temp.get_inset_rectangle()
        if inset_rect:
            metrics["coverage_inset_pct"] = swept_map.coverage_ratio(inset_rect) * 100.0
        else:
            metrics["coverage_inset_pct"] = 0.0
    else:
        metrics["coverage_inset_pct"] = 0.0

    # 5. Boundary discovery time
    metrics["boundary_time_s"] = boundary_sim_time

    # 6. Coverage time
    metrics["coverage_time_s"] = coverage_sim_time

    # 7. Total time
    metrics["total_time_s"] = boundary_sim_time + coverage_sim_time

    # 8. Distance covered
    metrics["distance_m"] = calculate_trajectory_distance(robot.trajectory)

    if verbose:
        print(f"  Metrics collected:")
        print(f"    Edge points: {metrics['edge_points']}")
        print(f"    Rect error: {metrics['rect_error_pct']:.2f}%")
        print(f"    Coverage (full): {metrics['coverage_full_pct']:.2f}%")
        print(f"    Coverage (inset): {metrics['coverage_inset_pct']:.2f}%")
        print(f"    Distance: {metrics['distance_m']:.2f}m")

    return metrics


def export_to_excel(results: List[Dict], output_path: str):
    """
    Export trial results to Excel with raw data and summary sheets.

    Args:
        results: List of metric dictionaries from trials
        output_path: Path to output Excel file
    """
    try:
        from openpyxl import Workbook
        from openpyxl.styles import Font, Alignment, PatternFill
    except ImportError:
        print("\nError: openpyxl not installed. Install with:")
        print("  pip install openpyxl")
        return

    wb = Workbook()

    # === Sheet 1: Trial Data ===
    ws_data = wb.active
    ws_data.title = "Trial Data"

    # Headers
    headers = [
        "Trial",
        "Edge Points",
        "Rect Error (%)",
        "Coverage Full (%)",
        "Coverage Inset (%)",
        "Boundary Time (s)",
        "Coverage Time (s)",
        "Total Time (s)",
        "Distance (m)",
    ]

    ws_data.append(headers)

    # Format headers
    for cell in ws_data[1]:
        cell.font = Font(bold=True)
        cell.alignment = Alignment(horizontal="center")
        cell.fill = PatternFill(
            start_color="B4C7E7", end_color="B4C7E7", fill_type="solid"
        )

    # Data rows
    metric_keys = [
        "edge_points",
        "rect_error_pct",
        "coverage_full_pct",
        "coverage_inset_pct",
        "boundary_time_s",
        "coverage_time_s",
        "total_time_s",
        "distance_m",
    ]

    for i, result in enumerate(results, start=1):
        row = [i] + [result.get(key, 0.0) for key in metric_keys]
        ws_data.append(row)

    # Set column widths
    ws_data.column_dimensions["A"].width = 8
    for col in ["B", "C", "D", "E", "F", "G", "H", "I"]:
        ws_data.column_dimensions[col].width = 16

    # === Sheet 2: Summary Statistics ===
    ws_summary = wb.create_sheet("Summary")

    # Headers
    ws_summary.append(["Metric", "Mean", "Std Dev", "Min", "Max"])
    for cell in ws_summary[1]:
        cell.font = Font(bold=True)
        cell.alignment = Alignment(horizontal="center")
        cell.fill = PatternFill(
            start_color="B4C7E7", end_color="B4C7E7", fill_type="solid"
        )

    # Calculate statistics for each metric
    metric_names = [
        "Edge Points",
        "Rect Error (%)",
        "Coverage Full (%)",
        "Coverage Inset (%)",
        "Boundary Time (s)",
        "Coverage Time (s)",
        "Total Time (s)",
        "Distance (m)",
    ]

    for metric_name, metric_key in zip(metric_names, metric_keys):
        values = [r[metric_key] for r in results if metric_key in r]
        if values:
            mean_val = np.mean(values)
            std_val = np.std(values)
            min_val = np.min(values)
            max_val = np.max(values)
            ws_summary.append([metric_name, mean_val, std_val, min_val, max_val])

    # Set column widths
    ws_summary.column_dimensions["A"].width = 20
    for col in ["B", "C", "D", "E"]:
        ws_summary.column_dimensions[col].width = 14

    # Save workbook
    wb.save(output_path)
    print(f"\n✓ Results exported to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Run robot performance testing trials",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "--trials",
        type=int,
        default=25,
        help="Number of trials to run (default: 25)",
    )

    parser.add_argument(
        "--output",
        type=str,
        default="robot_test_results.xlsx",
        help="Output Excel file path (default: robot_test_results.xlsx)",
    )

    parser.add_argument(
        "--speed",
        type=float,
        default=2.0,
        help="Speed multiplier for simulation (default: 2.0) - multiplies ALL speeds",
    )

    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print detailed progress for each trial",
    )

    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Disable real-time visualization (enabled by default)",
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for reproducibility (optional)",
    )

    args = parser.parse_args()

    # Set random seed if provided
    if args.seed is not None:
        np.random.seed(args.seed)
        print(f"Random seed set to: {args.seed}")

    print("=" * 60)
    print("Robot Performance Testing Framework")
    print("=" * 60)
    print(f"Trials: {args.trials}")
    print(f"Output: {args.output}")
    print(f"Speed multiplier: {args.speed}x (multiplies ALL speeds)")
    print(f"Visualization: {'Disabled' if args.no_viz else 'Enabled (real-time)'}")
    print("=" * 60)

    # Create shared visualizer for all trials (enabled by default, matching viz_demo.py)
    # This allows us to see each trial's progress sequentially in the same window
    viz = None
    if not args.no_viz:
        viz = Visualizer(figsize=(14, 8))

    # Run trials
    results = []
    start_time = time.time()

    for trial_num in range(1, args.trials + 1):
        if not args.verbose:
            print(f"Running trial {trial_num}/{args.trials}...", end="", flush=True)

        metrics = run_single_trial(trial_num, args.speed, args.verbose, viz=viz)
        results.append(metrics)

        if not args.verbose:
            print(" ✓")

    # Close shared visualizer if it was created
    if viz is not None:
        viz.close()

    total_time = time.time() - start_time

    print("\n" + "=" * 60)
    print(f"All {args.trials} trials completed in {total_time:.1f}s")
    print("=" * 60)

    # Export to Excel
    export_to_excel(results, args.output)

    # Print summary
    print("\n" + "=" * 60)
    print("Summary Statistics")
    print("=" * 60)

    metric_keys = [
        ("Edge Points", "edge_points"),
        ("Rect Error (%)", "rect_error_pct"),
        ("Coverage Full (%)", "coverage_full_pct"),
        ("Coverage Inset (%)", "coverage_inset_pct"),
        ("Boundary Time (s)", "boundary_time_s"),
        ("Coverage Time (s)", "coverage_time_s"),
        ("Total Time (s)", "total_time_s"),
        ("Distance (m)", "distance_m"),
    ]

    for name, key in metric_keys:
        values = [r[key] for r in results]
        mean_val = np.mean(values)
        std_val = np.std(values)
        print(f"{name:25s}: {mean_val:8.2f} ± {std_val:6.2f}")

    print("=" * 60)


if __name__ == "__main__":
    main()
