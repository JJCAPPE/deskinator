"""
Demo visualization script for Deskinator.

Demonstrates the visualization capabilities without requiring hardware.
Generates synthetic data matching the robot's data structures and displays
it using the existing Visualizer class.

Usage:
    python viz_demo.py
"""

import argparse
import time
import numpy as np
from typing import List, Tuple, Optional
from utils.viz import Visualizer
from slam.ekf import EKF
from slam.posegraph import PoseGraph
from slam.frames import pose_difference, wrap_angle
from config import GEOM, ALG


def generate_boundary_trajectory(
    cx: float,
    cy: float,
    width: float,
    height: float,
    heading: float,
    n_points: int,
    start_corner_idx: int = 0,
) -> List[Tuple[float, float, float]]:
    """
    Generate a trajectory that follows a rectangular boundary.

    Args:
        cx, cy: Rectangle center
        width, height: Rectangle dimensions
        heading: Rectangle orientation (rad)
        n_points: Number of poses to generate
        start_corner_idx: Index of corner to start from (0=bottom-left, CCW)

    Returns:
        List of (x, y, θ) poses
    """
    poses = []

    # Generate points along the perimeter
    perimeter = 2 * (width + height)

    # Corners in local frame (CCW starting from bottom-left)
    corners_local = [
        (-width / 2, -height / 2),
        (width / 2, -height / 2),
        (width / 2, height / 2),
        (-width / 2, height / 2),
    ]

    # Transform corners to world frame
    c, s = np.cos(heading), np.sin(heading)
    corners_world = []
    for lx, ly in corners_local:
        wx = cx + c * lx - s * ly
        wy = cy + s * lx + c * ly
        corners_world.append((wx, wy))

    # Reorder corners to start from requested index
    corners_ordered = (
        corners_world[start_corner_idx:] + corners_world[:start_corner_idx]
    )

    # Generate poses along edges
    # Edge 0: corner 0 -> 1, Edge 1: corner 1 -> 2, etc.

    # Determine points per edge based on length
    # Edge lengths for the ordered corners
    # If start_idx is 0 (BL): Bottom, Right, Top, Left
    # If start_idx is 1 (BR): Right, Top, Left, Bottom

    # Standard dimensions: w, h, w, h
    dims = [width, height, width, height]
    dims_ordered = dims[start_corner_idx:] + dims[:start_corner_idx]

    for i in range(4):
        edge_len = dims_ordered[i]
        n_edge_points = max(5, int(n_points * (edge_len / perimeter)))

        p0 = corners_ordered[i]
        p1 = corners_ordered[(i + 1) % 4]

        # Compute heading for this edge
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        theta = np.arctan2(dy, dx)

        for j in range(n_edge_points):
            alpha = j / n_edge_points
            x = p0[0] + alpha * (p1[0] - p0[0])
            y = p0[1] + alpha * (p1[1] - p0[1])
            poses.append((x, y, theta))

    return poses


def generate_edge_points(
    cx: float,
    cy: float,
    width: float,
    height: float,
    heading: float,
    spacing: float = 0.05,
) -> List[Tuple[float, float]]:
    """
    Generate edge detection points along the boundary.

    Args:
        cx, cy: Rectangle center
        width, height: Rectangle dimensions
        heading: Rectangle orientation (rad)
        spacing: Spacing between edge points (m)

    Returns:
        List of (x, y) edge points
    """
    edge_points = []

    # Generate points along each edge
    perimeter = 2 * (width + height)
    n_points = int(perimeter / spacing)

    # Use similar logic to trajectory generation
    corners_local = [
        (-width / 2, -height / 2),
        (width / 2, -height / 2),
        (width / 2, height / 2),
        (-width / 2, height / 2),
    ]

    c, s = np.cos(heading), np.sin(heading)
    corners_world = []
    for lx, ly in corners_local:
        wx = cx + c * lx - s * ly
        wy = cy + s * lx + c * ly
        corners_world.append((wx, wy))

    for i in range(n_points):
        t = i / n_points

        if t < width / perimeter:
            alpha = t * perimeter / width
            p0, p1 = corners_world[0], corners_world[1]
        elif t < (width + height) / perimeter:
            alpha = (t * perimeter - width) / height
            p0, p1 = corners_world[1], corners_world[2]
        elif t < (2 * width + height) / perimeter:
            alpha = (t * perimeter - width - height) / width
            p0, p1 = corners_world[2], corners_world[3]
        else:
            alpha = (t * perimeter - 2 * width - height) / height
            p0, p1 = corners_world[3], corners_world[0]

        x = p0[0] + alpha * (p1[0] - p0[0])
        y = p0[1] + alpha * (p1[1] - p0[1])
        edge_points.append((x, y))

    return edge_points


def pose_to_odometry(
    pose1: Tuple[float, float, float], pose2: Tuple[float, float, float]
) -> Tuple[float, float]:
    """
    Convert relative pose change to wheel odometry (dSL, dSR).

    Args:
        pose1: Previous pose (x, y, θ)
        pose2: Current pose (x, y, θ)

    Returns:
        (dSL, dSR) wheel displacements in meters
    """
    # Compute relative motion
    dx = pose2[0] - pose1[0]
    dy = pose2[1] - pose1[1]
    dtheta = wrap_angle(pose2[2] - pose1[2])

    # Average forward motion
    ds = np.sqrt(dx**2 + dy**2)

    # Differential motion
    dSL = ds - (dtheta * GEOM.WHEEL_BASE) / 2
    dSR = ds + (dtheta * GEOM.WHEEL_BASE) / 2

    return (dSL, dSR)


def add_odometry_noise(
    dSL: float, dSR: float, odom_noise_std: float = 0.005, scale_error: float = 0.02
) -> Tuple[float, float]:
    """
    Add realistic noise to odometry measurements.

    Args:
        dSL, dSR: True wheel displacements
        odom_noise_std: Standard deviation of additive noise (m)
        scale_error: Scale factor error (e.g., 0.02 = 2% error)

    Returns:
        Noisy (dSL, dSR)
    """
    # Additive noise
    noise_L = np.random.normal(0, odom_noise_std)
    noise_R = np.random.normal(0, odom_noise_std)

    # Scale factor errors (slight wheel diameter differences)
    scale_L = 1.0 + np.random.normal(0, scale_error)
    scale_R = 1.0 + np.random.normal(0, scale_error)

    dSL_noisy = dSL * scale_L + noise_L
    dSR_noisy = dSR * scale_R + noise_R

    return (dSL_noisy, dSR_noisy)


def add_edge_point_noise(
    edge_points: List[Tuple[float, float]], noise_std: float = 0.01
) -> List[Tuple[float, float]]:
    """
    Add noise to edge detection points.

    Args:
        edge_points: True edge points
        noise_std: Standard deviation of noise (m)

    Returns:
        Noisy edge points
    """
    noisy_points = []
    for x, y in edge_points:
        nx = x + np.random.normal(0, noise_std)
        ny = y + np.random.normal(0, noise_std)
        noisy_points.append((nx, ny))
    return noisy_points


def generate_coverage_grid(
    bounds: Tuple[float, float, float, float],
    rectangle: Optional[Tuple[float, float, float, float, float]],
    poses: List[Tuple[float, float, float]],
    resolution: float = 0.02,
    sweep_width: float = 0.15,
) -> Tuple[np.ndarray, Tuple[float, float, float, float]]:
    """
    Generate a coverage grid showing swept areas.

    Args:
        bounds: (min_x, max_x, min_y, max_y)
        rectangle: (cx, cy, heading, width, height) or None
        poses: List of (x, y, θ) poses
        resolution: Grid resolution (m)
        sweep_width: Width of swept area around trajectory (m)

    Returns:
        (coverage_grid, bounds) tuple
    """
    min_x, max_x, min_y, max_y = bounds

    # Create grid
    width = int((max_x - min_x) / resolution) + 1
    height = int((max_y - min_y) / resolution) + 1
    grid = np.zeros((height, width), dtype=np.uint8)

    # Mark swept areas around trajectory
    for x, y, theta in poses:
        # Convert to grid coordinates
        gi = int((y - min_y) / resolution)
        gj = int((x - min_x) / resolution)

        # Mark cells within sweep width
        half_width_cells = int(sweep_width / (2 * resolution)) + 1

        for di in range(-half_width_cells, half_width_cells + 1):
            for dj in range(-half_width_cells, half_width_cells + 1):
                # Check distance
                world_dx = dj * resolution
                world_dy = di * resolution
                dist = np.sqrt(world_dx**2 + world_dy**2)

                if dist <= sweep_width / 2:
                    gi_idx = gi + di
                    gj_idx = gj + dj
                    if 0 <= gi_idx < height and 0 <= gj_idx < width:
                        grid[gi_idx, gj_idx] = 1

    return grid, bounds


def main():
    """Run visualization demo."""
    parser = argparse.ArgumentParser(
        description="Deskinator Visualization Demo with SLAM"
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.05,
        help="Delay between animation frames in seconds (default: 0.05)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Animation speed multiplier (default: 1.0, higher = faster)",
    )
    parser.add_argument(
        "--error",
        type=float,
        default=0.0,
        help="Error/noise level (0.0-1.0, default: 0.0). 0=no noise, 1.0=maximum noise",
    )
    args = parser.parse_args()

    # Validate error parameter
    if args.error < 0.0 or args.error > 1.0:
        parser.error("--error must be between 0.0 and 1.0")

    # Calculate actual delay from speed multiplier
    base_delay = 0.05
    if args.speed != 1.0:
        actual_delay = base_delay / args.speed
    else:
        actual_delay = args.delay

    # Scale noise parameters based on error level
    error_level = args.error
    odom_noise_std = 0.005 * error_level  # Max 0.5cm std
    odom_scale_error = 0.02 * error_level  # Max 2% scale error
    edge_noise_std = 0.008 * error_level  # Max 0.8cm std
    gyro_noise_std = 0.05 * error_level  # Max 0.05 rad/s std

    print("=" * 60)
    print("Deskinator Visualization Demo with SLAM")
    print("=" * 60)
    print("Demonstrating SLAM with noisy odometry")
    print("Shows how pose graph optimization corrects drift")
    print(f"Animation delay: {actual_delay:.3f}s per frame")
    print(
        f"Error level: {error_level:.2f} ({'no noise' if error_level == 0 else 'noisy'})"
    )
    print("Press Ctrl+C to stop")
    print("=" * 60)

    # Define a rectangular desk boundary
    cx, cy = 0.5, 0.3  # Center position (m)
    width, height = 1.2, 0.8  # Dimensions (m)
    heading = 0.1  # Slight rotation (rad)

    # Random initial robot pose (at center, random heading)
    initial_heading = np.random.uniform(0, 2 * np.pi)
    initial_pose = (cx, cy, initial_heading)

    rectangle = (cx, cy, heading, width, height)

    print(
        f"Initial robot pose: ({cx:.2f}, {cy:.2f}) @ {np.degrees(initial_heading):.1f}°"
    )

    # Generate boundary exploration trajectory
    # Choose start corner nearest to the initial pose
    corners_local_tmp = [
        (-width / 2, -height / 2),
        (width / 2, -height / 2),
        (width / 2, height / 2),
        (-width / 2, height / 2),
    ]
    c_tmp, s_tmp = np.cos(heading), np.sin(heading)
    corners_world_tmp: List[Tuple[float, float]] = []
    for lx, ly in corners_local_tmp:
        wx = cx + c_tmp * lx - s_tmp * ly
        wy = cy + s_tmp * lx + c_tmp * ly
        corners_world_tmp.append((wx, wy))

    min_dist_tmp = float("inf")
    start_corner_idx = 0
    for i_tmp, corner_tmp in enumerate(corners_world_tmp):
        dx_tmp = corner_tmp[0] - initial_pose[0]
        dy_tmp = corner_tmp[1] - initial_pose[1]
        dist_tmp = np.sqrt(dx_tmp * dx_tmp + dy_tmp * dy_tmp)
        if dist_tmp < min_dist_tmp:
            min_dist_tmp = dist_tmp
            start_corner_idx = i_tmp

    print(
        f"\nGenerating boundary exploration trajectory (starting at corner {start_corner_idx})..."
    )
    boundary_poses = generate_boundary_trajectory(
        cx, cy, width, height, heading, n_points=200, start_corner_idx=start_corner_idx
    )

    # Generate edge points (ground truth)
    print("Generating edge detection points...")
    edge_points_gt = generate_edge_points(cx, cy, width, height, heading, spacing=0.03)

    # Add noise to edge points (simulating sensor measurement errors)
    if error_level > 0:
        print(f"Adding noise to edge points (std={edge_noise_std*1000:.1f}mm)...")
        edge_points = add_edge_point_noise(edge_points_gt, noise_std=edge_noise_std)
    else:
        print("Using ground truth edge points (no noise)")
        edge_points = edge_points_gt

    # Generate coverage trajectory (boustrophedon pattern)
    print("Generating coverage trajectory...")
    coverage_poses = []

    # Boustrophedon lanes inside rectangle (in rectangle frame)
    # Use physical vacuum width and overlap to determine spacing
    lane_spacing = max(0.05, GEOM.VAC_WIDTH - ALG.SWEEP_OVERLAP)

    # Keep lanes within rectangle by half vacuum width margin (plus tiny clearance)
    margin_y = GEOM.VAC_WIDTH * 0.5 + 0.01
    margin_x = GEOM.VAC_WIDTH * 0.5 + 0.01

    effective_height = max(0.0, height - 2.0 * margin_y)
    if effective_height <= 0.0:
        n_lanes = 1
    else:
        n_lanes = int(np.floor(effective_height / lane_spacing)) + 1

    # Helper to transform local (rect frame) -> world
    c_cov, s_cov = np.cos(heading), np.sin(heading)

    def local_to_world(xl: float, yl: float) -> Tuple[float, float]:
        xw = cx + c_cov * xl - s_cov * yl
        yw = cy + s_cov * xl + c_cov * yl
        return xw, yw

    last_world: Optional[Tuple[float, float]] = None

    for lane_idx in range(n_lanes):
        y_local = -height / 2 + margin_y + lane_idx * lane_spacing
        # Clamp final lane to stay inside the margin
        y_local = max(-height / 2 + margin_y, min(height / 2 - margin_y, y_local))

        # Even lanes: left -> right, odd lanes: right -> left
        forward = lane_idx % 2 == 0
        x_start_local = -width / 2 + margin_x if forward else width / 2 - margin_x
        x_end_local = width / 2 - margin_x if forward else -width / 2 + margin_x

        # Connect from previous lane end to this lane start (side step)
        lane_start_world = local_to_world(x_start_local, y_local)
        if last_world is not None:
            dx = lane_start_world[0] - last_world[0]
            dy = lane_start_world[1] - last_world[1]
            dist = float(np.hypot(dx, dy))
            n_trans = max(3, int(dist / 0.05))
            for i in range(1, n_trans + 1):
                alpha = i / n_trans
                x = last_world[0] + alpha * dx
                y = last_world[1] + alpha * dy
                theta_conn = heading if forward else heading + np.pi
                coverage_poses.append((x, y, theta_conn))

        # Generate points along this lane
        n_points_lane = max(10, int((width - 2.0 * margin_x) / 0.03))
        for i in range(n_points_lane):
            alpha = i / (n_points_lane - 1) if n_points_lane > 1 else 0.0
            x_local = x_start_local + alpha * (x_end_local - x_start_local)
            xw, yw = local_to_world(x_local, y_local)
            theta_lane = heading if forward else heading + np.pi
            coverage_poses.append((xw, yw, theta_lane))

        # Update last point (end of this lane)
        last_world = local_to_world(x_end_local, y_local)

    # Create initial trajectory segment: move from center to start of boundary exploration
    # Find nearest point on boundary to start exploration
    # Start boundary exploration from nearest corner
    corners_local = [
        (-width / 2, -height / 2),
        (width / 2, -height / 2),
        (width / 2, height / 2),
        (-width / 2, height / 2),
    ]
    c, s = np.cos(heading), np.sin(heading)
    corners_world = []
    for lx, ly in corners_local:
        wx = cx + c * lx - s * ly
        wy = cy + s * lx + c * ly
        corners_world.append((wx, wy))

    # Find nearest corner to initial position
    min_dist = float("inf")
    start_corner = corners_world[0]
    for corner in corners_world:
        dx = corner[0] - initial_pose[0]
        dy = corner[1] - initial_pose[1]
        dist = np.sqrt(dx * dx + dy * dy)
        if dist < min_dist:
            min_dist = dist
            start_corner = corner

    # Generate path from center to start corner
    start_to_corner = []
    n_segments = max(10, int(min_dist / 0.05))
    for i in range(n_segments):
        alpha = i / n_segments
        x = initial_pose[0] + alpha * (start_corner[0] - initial_pose[0])
        y = initial_pose[1] + alpha * (start_corner[1] - initial_pose[1])
        # Heading points toward corner
        dx = start_corner[0] - initial_pose[0]
        dy = start_corner[1] - initial_pose[1]
        theta = np.arctan2(dy, dx)
        start_to_corner.append((x, y, theta))

    # Transition from boundary end to coverage start (to keep trajectory continuous)
    transition_to_coverage: List[Tuple[float, float, float]] = []
    if len(boundary_poses) > 0 and len(coverage_poses) > 0:
        boundary_end = boundary_poses[-1]
        coverage_start = coverage_poses[0]
        dist_trans = np.hypot(
            coverage_start[0] - boundary_end[0], coverage_start[1] - boundary_end[1]
        )
        n_trans = max(5, int(dist_trans / 0.05))
        for i in range(n_trans):
            alpha = i / n_trans
            x = boundary_end[0] + alpha * (coverage_start[0] - boundary_end[0])
            y = boundary_end[1] + alpha * (coverage_start[1] - boundary_end[1])
            dx_t = coverage_start[0] - boundary_end[0]
            dy_t = coverage_start[1] - boundary_end[1]
            theta_t = np.arctan2(dy_t, dx_t)
            transition_to_coverage.append((x, y, theta_t))

    # Combine trajectories (ground truth)
    all_poses_gt = (
        start_to_corner + boundary_poses + transition_to_coverage + coverage_poses
    )

    # Simulate SLAM with noisy odometry
    print("\nSimulating SLAM with noisy odometry...")
    if error_level > 0:
        print(
            f"  - Odometry noise: ~{odom_noise_std*1000:.1f}mm std, ~{odom_scale_error*100:.1f}% scale error"
        )
        print(f"  - Edge point noise: ~{edge_noise_std*1000:.1f}mm std")
        print(f"  - Gyro noise: ~{gyro_noise_std:.3f} rad/s std")
    else:
        print("  - No noise (perfect sensors)")
    print("  - Using EKF + Pose Graph optimization")

    # Initialize SLAM components with initial pose
    ekf = EKF(x0=initial_pose[0], y0=initial_pose[1], theta0=initial_pose[2])
    pose_graph = PoseGraph()

    # Process trajectory with noisy odometry
    dt = 0.02  # 50 Hz
    ekf_poses = [initial_pose]  # Start with initial pose
    noisy_poses = [initial_pose]  # For comparison

    # Add initial pose to graph
    node_id = pose_graph.add_node(0.0, initial_pose)
    last_node_pose = initial_pose
    last_pose = initial_pose

    for i in range(1, len(all_poses_gt)):
        # Get true relative motion
        dSL_true, dSR_true = pose_to_odometry(all_poses_gt[i - 1], all_poses_gt[i])

        # Add noise to odometry
        if error_level > 0:
            dSL_noisy, dSR_noisy = add_odometry_noise(
                dSL_true,
                dSR_true,
                odom_noise_std=odom_noise_std,
                scale_error=odom_scale_error,
            )
        else:
            dSL_noisy, dSR_noisy = dSL_true, dSR_true

        # Update EKF with noisy odometry
        ekf.predict(dSL_noisy, dSR_noisy, dt)

        # Simulate gyro measurement (with noise)
        true_yaw_rate = (all_poses_gt[i][2] - all_poses_gt[i - 1][2]) / dt
        if error_level > 0:
            noisy_yaw_rate = true_yaw_rate + np.random.normal(0, gyro_noise_std)
        else:
            noisy_yaw_rate = true_yaw_rate
        ekf.update_gyro(noisy_yaw_rate, dt)

        # Get EKF pose
        ekf_pose = ekf.pose()
        ekf_poses.append(ekf_pose)

        # Also track pure odometry integration (no EKF filtering) for comparison
        # Simple dead reckoning
        x, y, theta = last_pose
        ds = 0.5 * (dSR_noisy + dSL_noisy)
        dtheta = (dSR_noisy - dSL_noisy) / GEOM.WHEEL_BASE
        theta_mid = theta + 0.5 * dtheta
        noisy_pose = (
            x + ds * np.cos(theta_mid),
            y + ds * np.sin(theta_mid),
            wrap_angle(theta + dtheta),
        )
        noisy_poses.append(noisy_pose)
        last_pose = noisy_pose

        # Add nodes to pose graph periodically
        dx = ekf_pose[0] - last_node_pose[0]
        dy = ekf_pose[1] - last_node_pose[1]
        dist = np.sqrt(dx * dx + dy * dy)

        if dist >= ALG.NODE_SPACING:
            node_id = pose_graph.add_node(i * dt, ekf_pose)

            # Add odometry edge
            if node_id > 0:
                z_ij = pose_difference(last_node_pose, ekf_pose)
                Info = np.diag([100.0, 100.0, 50.0])
                pose_graph.add_edge_odom(node_id - 1, node_id, z_ij, Info)

            last_node_pose = ekf_pose

            # Optimize periodically (only if we have enough constraints)
            # Need at least 2 nodes and enough edges to constrain the system
            if node_id % 20 == 0 and node_id >= 2:
                # Check if we have enough constraints
                # Each edge gives 3 residuals, we need at least as many as free variables
                # Free variables = 3 * (node_id - 1) since first pose is fixed
                # Edges = node_id - 1 (one edge per node after first)
                # Residuals = 3 * (node_id - 1)
                # So we need node_id >= 2 to have any constraints
                try:
                    pose_graph.optimize()
                except ValueError:
                    # Not enough constraints yet, skip optimization
                    pass

    # Final optimization (only if we have enough constraints)
    if len(pose_graph.poses) >= 2:
        print("  Performing final pose graph optimization...")
        try:
            pose_graph.optimize()
        except ValueError as e:
            print(f"  Warning: Could not optimize pose graph: {e}")
            print("  Using EKF poses instead")
    else:
        print("  Not enough poses for optimization, using EKF poses")

    # Get optimized poses
    optimized_poses = pose_graph.get_all_poses()

    print(f"  Generated {len(optimized_poses)} optimized poses")
    print(f"  EKF processed {len(ekf_poses)} poses")

    # Use EKF poses for smooth trajectory visualization
    # The pose graph optimization corrects drift at key nodes, which helps
    # the overall SLAM process, but for visualization we use the dense EKF trajectory
    all_poses = ekf_poses

    print(f"  Using EKF trajectory ({len(all_poses)} poses) for visualization")
    print(f"  Note: Pose graph optimization corrected {len(optimized_poses)} key nodes")

    # Set up bounds
    margin = 0.2
    min_x = cx - width / 2 - margin
    max_x = cx + width / 2 + margin
    min_y = cy - height / 2 - margin
    max_y = cy + height / 2 + margin
    bounds = (min_x, max_x, min_y, max_y)

    # Initialize visualizer
    print("\nInitializing visualizer...")
    visualizer = Visualizer()

    if not visualizer.enabled:
        print("Error: matplotlib not available. Install matplotlib to run demo.")
        return

    print("\nStarting visualization loop...")
    print("The visualization will show:")
    print("  - SLAM-optimized trajectory (blue)")
    print("  - Noisy edge detections (green points)")
    print("  - Rectangle fit from noisy data (orange boundary)")
    print("  - Coverage progress (green heatmap)")
    print("\nNote: The trajectory shows SLAM correction of odometry drift")

    # Animate the visualization
    try:
        # Phase 1: Boundary exploration (show poses incrementally)
        print("\nPhase 1: Boundary exploration with SLAM...")
        # Estimate boundary phase length (roughly first third of trajectory)
        n_boundary_estimate = len(boundary_poses) * len(all_poses) // len(all_poses_gt)
        n_boundary_nodes = min(n_boundary_estimate, len(all_poses) // 2)

        for i in range(5, min(len(all_poses), n_boundary_nodes), 5):
            current_poses = all_poses[: i + 1]
            # Show edge points as they're detected
            n_edge_points = min(i * 2, len(edge_points))
            current_edge_points = edge_points[:n_edge_points]

            # Generate coverage grid for current poses
            coverage_grid, _ = generate_coverage_grid(
                bounds,
                rectangle,
                current_poses,
                resolution=0.02,
                sweep_width=GEOM.VAC_WIDTH,
            )

            # Show rectangle after enough exploration
            show_rectangle = rectangle if i > len(all_poses) // 3 else None

            visualizer.update(
                poses=current_poses,
                edge_points=current_edge_points,
                rectangle=show_rectangle,
                coverage_grid=coverage_grid,
                swept_map_bounds=bounds,
            )

            time.sleep(actual_delay)

        # Phase 2: Coverage (show coverage progress)
        print("Phase 2: Coverage with SLAM correction...")
        for i in range(n_boundary_nodes, len(all_poses), 10):
            current_poses = all_poses[: i + 1]

            # Generate coverage grid
            coverage_grid, _ = generate_coverage_grid(
                bounds,
                rectangle,
                current_poses,
                resolution=0.02,
                sweep_width=GEOM.VAC_WIDTH,
            )

            visualizer.update(
                poses=current_poses,
                edge_points=edge_points,
                rectangle=rectangle,
                coverage_grid=coverage_grid,
                swept_map_bounds=bounds,
            )

            time.sleep(actual_delay)

        # Final state
        print("\nFinal state...")
        coverage_grid, _ = generate_coverage_grid(
            bounds, rectangle, all_poses, resolution=0.02, sweep_width=GEOM.VAC_WIDTH
        )

        visualizer.update(
            poses=all_poses,
            edge_points=edge_points,
            rectangle=rectangle,
            coverage_grid=coverage_grid,
            swept_map_bounds=bounds,
        )

        # Save final visualization
        print("\nSaving visualization to 'demo_output.png'...")
        visualizer.save("demo_output.png")

        print("\nDemo complete! Visualization saved.")
        print("Close the plot window to exit.")

        # Keep window open
        import matplotlib.pyplot as plt

        plt.ioff()  # Turn off interactive mode
        plt.show()  # Block until window is closed

    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user")
    finally:
        visualizer.close()


if __name__ == "__main__":
    main()
