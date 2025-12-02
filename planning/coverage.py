"""
Coverage planning and wall-following algorithm.

Implements a simple wall-following algorithm for boundary discovery
and boustrophedon path planning for coverage.
"""

import numpy as np
import time
from enum import Enum
from typing import List, Tuple, Optional, Callable
from dataclasses import dataclass

try:
    from ..config import GEOM, ALG
except ImportError:
    from config import GEOM, ALG


class WallFollowState(Enum):
    """States for wall-following algorithm."""

    FIND_WALL = "find_wall"
    TURN_AWAY = "turn_away"
    FORWARD_UNTIL_OFF = "forward_until_off"
    TURN_BACK = "turn_back"
    FORWARD_ALONG_WALL = "forward_along_wall"
    COMPLETE = "complete"


@dataclass
class EdgePoint:
    """Edge detection point."""

    x: float
    y: float
    timestamp: float


class SimpleWallFollower:
    """
    Simple wall-following algorithm for boundary discovery.

    Algorithm:
    1. Go straight until finding a wall
    2. If right sensor goes off, turn left until sensor is back + 2°
    3. Go forwards until sensor goes off
    4. Turn in the same direction as the first time until sensor is back on
    5. Go forwards
    6. Repeat until table is mapped out
    """

    def __init__(self, forward_speed: float = 0.1, turn_speed: float = 0.5):
        """
        Initialize wall follower.

        Args:
            forward_speed: Forward velocity (m/s)
            turn_speed: Angular velocity for turning (rad/s)
        """
        self.state = WallFollowState.FIND_WALL
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed

        # Track edge points
        self.edge_points: List[EdgePoint] = []

        # State tracking
        self.turn_direction = 0.0  # +1 for left (CCW), -1 for right (CW)
        self.initial_turn_direction = (
            None  # Store the initial direction to maintain consistency
        )
        self.start_pose: Optional[Tuple[float, float, float]] = None
        self.lap_complete = False

        # Rotation tracking for IMU-based completion
        self.total_rotation = 0.0  # Cumulative rotation in radians
        self.last_theta: Optional[float] = None  # Previous theta for rotation tracking

        # Parameters
        self.sensor_back_threshold = 2.0  # degrees
        self.lap_close_distance = 0.1  # m - distance to start to consider lap complete
        self.rotation_completion_threshold = 2.1 * np.pi  # 360 degrees in radians

    def update(
        self,
        pose: Tuple[float, float, float],
        left_sensor_on: bool,
        right_sensor_on: bool,
        dt: float,
    ) -> Tuple[float, float]:
        """
        Update wall-following algorithm.

        Args:
            pose: Current robot pose (x, y, theta)
            left_sensor_on: True if left sensor detects table (not off edge)
            right_sensor_on: True if right sensor detects table (not off edge)
            dt: Time step (s)

        Returns:
            (v, omega) velocity commands
        """
        x, y, theta = pose

        # Initialize start pose on first call
        if self.start_pose is None:
            self.start_pose = pose
            self.last_theta = theta

        # Track cumulative rotation (IMU-based completion)
        if self.last_theta is not None:
            # Compute change in theta, handling wrap-around
            dtheta = theta - self.last_theta
            # Normalize to [-pi, pi]
            dtheta = ((dtheta + np.pi) % (2 * np.pi)) - np.pi
            # Accumulate absolute rotation (always positive)
            self.total_rotation += abs(dtheta)
        self.last_theta = theta

        # Check if we've completed a lap based on rotation (360 degrees)
        if not self.lap_complete:
            if self.total_rotation >= self.rotation_completion_threshold:
                self.lap_complete = True
                self.state = WallFollowState.COMPLETE
            # Also check position-based completion (backup method)
            elif self.start_pose:
                dx = x - self.start_pose[0]
                dy = y - self.start_pose[1]
                dist = np.sqrt(dx * dx + dy * dy)
                if dist < self.lap_close_distance and len(self.edge_points) > 10:
                    # Check if we're heading in roughly the same direction
                    dtheta = abs(theta - self.start_pose[2])
                    dtheta = min(dtheta, 2 * np.pi - dtheta)
                    if dtheta < np.deg2rad(45):
                        self.lap_complete = True
                        self.state = WallFollowState.COMPLETE

        if self.lap_complete:
            return 0.0, 0.0

        # State machine
        if self.state == WallFollowState.FIND_WALL:
            # Go straight until we hit a wall (sensor goes off)
            if not right_sensor_on or not left_sensor_on:
                # Hit a wall - determine which sensor and set initial direction
                if not right_sensor_on:
                    self.turn_direction = 1.0  # Turn left (CCW)
                    self.initial_turn_direction = 1.0  # Store for consistency
                    self._add_edge_point(pose, "right")
                elif not left_sensor_on:
                    self.turn_direction = -1.0  # Turn right (CW)
                    self.initial_turn_direction = -1.0  # Store for consistency
                    self._add_edge_point(pose, "left")
                # Start turning away
                self.sensor_back_theta = None
                self.state = WallFollowState.TURN_AWAY
            return self.forward_speed, 0.0

        elif self.state == WallFollowState.TURN_AWAY:
            # Turn away from wall until sensor is back + 2°
            # Check if sensor is back on
            sensor_back = False
            if self.turn_direction > 0:  # Turning left (away from right wall)
                sensor_back = right_sensor_on
            else:  # Turning right (away from left wall)
                sensor_back = left_sensor_on

            if sensor_back:
                if self.sensor_back_theta is None:
                    # Just got sensor back - record angle and continue turning
                    self.sensor_back_theta = theta
                else:
                    # Continue turning until we've turned 2° past sensor back point
                    dtheta = theta - self.sensor_back_theta
                    dtheta = (
                        (dtheta + np.pi) % (2 * np.pi)
                    ) - np.pi  # Wrap to [-pi, pi]

                    if abs(dtheta) >= np.deg2rad(self.sensor_back_threshold):
                        # Reached target angle, now go forward
                        self.state = WallFollowState.FORWARD_UNTIL_OFF
            else:
                # Sensor not back yet, keep turning
                pass

            return 0.0, self.turn_speed * self.turn_direction

        elif self.state == WallFollowState.FORWARD_UNTIL_OFF:
            # Go forward until sensor goes off (hitting next corner/edge)
            if not right_sensor_on or not left_sensor_on:
                if not right_sensor_on:
                    self._add_edge_point(pose, "right")
                if not left_sensor_on:
                    self._add_edge_point(pose, "left")
                # Turn in same direction as before
                self.state = WallFollowState.TURN_BACK
            return self.forward_speed, 0.0

        elif self.state == WallFollowState.TURN_BACK:
            # Turn in same direction until sensor is back on
            sensor_back = False
            if self.turn_direction > 0:  # Turning left
                sensor_back = right_sensor_on
            else:  # Turning right
                sensor_back = left_sensor_on

            if sensor_back:
                self.state = WallFollowState.FORWARD_ALONG_WALL
            return 0.0, self.turn_speed * self.turn_direction

        elif self.state == WallFollowState.FORWARD_ALONG_WALL:
            # Go forward along wall
            # Check if we hit an edge
            if not right_sensor_on or not left_sensor_on:
                if not right_sensor_on:
                    self._add_edge_point(pose, "right")
                elif not left_sensor_on:
                    self._add_edge_point(pose, "left")

                # Maintain consistent direction based on initial turn direction
                # The turn direction should match the initial direction to keep going
                # in the same direction (clockwise or counterclockwise) around the table
                if self.initial_turn_direction is not None:
                    self.turn_direction = self.initial_turn_direction
                else:
                    # Fallback: determine direction based on which sensor went off
                    if not right_sensor_on:
                        self.turn_direction = 1.0  # Turn left (CCW)
                    elif not left_sensor_on:
                        self.turn_direction = -1.0  # Turn right (CW)

                # Go back to turn away state
                self.sensor_back_theta = None
                self.state = WallFollowState.TURN_AWAY
                return self.forward_speed, 0.0

            # Normal forward motion
            return self.forward_speed, 0.0

        return 0.0, 0.0

    def _add_edge_point(self, pose: Tuple[float, float, float], side: str):
        """Add an edge point based on sensor position."""
        x, y, theta = pose

        # Sensor position in robot frame
        if side == "left":
            sensor_lat = GEOM.SENSOR_LAT[0] if len(GEOM.SENSOR_LAT) > 0 else 0.1
        else:
            sensor_lat = GEOM.SENSOR_LAT[-1] if len(GEOM.SENSOR_LAT) > 1 else -0.1

        # Transform to world frame
        sensor_x = x + GEOM.SENSOR_FWD * np.cos(theta) - sensor_lat * np.sin(theta)
        sensor_y = y + GEOM.SENSOR_FWD * np.sin(theta) + sensor_lat * np.cos(theta)

        import time

        self.edge_points.append(EdgePoint(sensor_x, sensor_y, time.time()))

    def get_edge_points(self) -> List[Tuple[float, float]]:
        """Get list of edge points as (x, y) tuples."""
        return [(p.x, p.y) for p in self.edge_points]

    def is_complete(self) -> bool:
        """Check if wall-following is complete."""
        return self.lap_complete

    def get_total_rotation(self) -> float:
        """Get cumulative rotation in radians."""
        return self.total_rotation

    def get_total_rotation_degrees(self) -> float:
        """Get cumulative rotation in degrees."""
        return np.rad2deg(self.total_rotation)


class SimpleRectangleFit:
    """
    Rectangle fitting from edge points using line detection.

    Fits a rectangle whose edges go through the most points by:
    1. Detecting straight line segments using RANSAC
    2. Finding 4 lines that form orthogonal pairs (rectangle edges)
    3. Scoring rectangles based on number of points near edges
    4. Selecting the rectangle with maximum edge point score
    """

    def __init__(self):
        self.edge_points: List[Tuple[float, float]] = []
        self.rectangle: Optional[Tuple[float, float, float, float, float]] = None
        self.is_confident = False

    def add_edge_point(self, point: Tuple[float, float]):
        """Add an edge point."""
        self.edge_points.append(point)

    def _point_to_line_distance(
        self, point: np.ndarray, line_point: np.ndarray, line_dir: np.ndarray
    ) -> float:
        """Compute perpendicular distance from point to line."""
        # Line: line_point + t * line_dir
        # Vector from line_point to point
        to_point = point - line_point
        # Project onto line direction
        proj = np.dot(to_point, line_dir)
        # Perpendicular component
        perp = to_point - proj * line_dir
        return np.linalg.norm(perp)

    def _fit_line_ransac(
        self,
        points: np.ndarray,
        inlier_threshold: float,
        min_inliers: int,
        max_iterations: int = 100,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """
        Fit a line using RANSAC.

        Returns:
            (line_point, line_dir, inlier_mask) or None if no good line found
        """
        if len(points) < 2:
            return None

        best_inliers = None
        best_line_point = None
        best_line_dir = None
        best_inlier_count = 0

        for _ in range(max_iterations):
            # Randomly select 2 points
            idx = np.random.choice(len(points), size=2, replace=False)
            p1, p2 = points[idx[0]], points[idx[1]]

            # Skip if points are too close
            if np.linalg.norm(p2 - p1) < 1e-6:
                continue

            # Line direction (normalized)
            line_dir = p2 - p1
            line_dir = line_dir / np.linalg.norm(line_dir)
            line_point = p1

            # Find inliers (points close to line)
            distances = np.array(
                [self._point_to_line_distance(p, line_point, line_dir) for p in points]
            )
            inlier_mask = distances < inlier_threshold
            inlier_count = np.sum(inlier_mask)

            if inlier_count > best_inlier_count and inlier_count >= min_inliers:
                best_inlier_count = inlier_count
                best_inliers = inlier_mask
                best_line_point = line_point
                best_line_dir = line_dir

        if best_inliers is None:
            return None

        # Refit line using all inliers (least squares)
        inlier_points = points[best_inliers]
        if len(inlier_points) < 2:
            return None

        # Fit line using PCA (principal component)
        centroid = inlier_points.mean(axis=0)
        centered = inlier_points - centroid
        if len(centered) < 2:
            return None

        # Use SVD to find principal direction
        try:
            U, S, Vt = np.linalg.svd(centered, full_matrices=False)
            line_dir = Vt[0]  # First principal component
            line_dir = line_dir / np.linalg.norm(line_dir)
            line_point = centroid
        except:
            # Fallback: use first two points
            line_dir = best_line_dir
            line_point = best_line_point

        return (line_point, line_dir, best_inliers)

    def _lines_are_orthogonal(
        self, dir1: np.ndarray, dir2: np.ndarray, tolerance_deg: float = 10.0
    ) -> bool:
        """Check if two lines are orthogonal (within tolerance)."""
        dot_product = abs(np.dot(dir1, dir2))
        angle_rad = np.arccos(np.clip(dot_product, -1.0, 1.0))
        angle_deg = np.rad2deg(angle_rad)
        # Check if angle is close to 90 degrees
        return abs(angle_deg - 90.0) < tolerance_deg

    def _score_rectangle(
        self,
        points: np.ndarray,
        lines: List[Tuple[np.ndarray, np.ndarray]],
        edge_threshold: float,
    ) -> int:
        """
        Score a rectangle by counting points near any of its 4 edges.

        Args:
            points: All edge points
            lines: List of 4 (line_point, line_dir) tuples representing rectangle edges
            edge_threshold: Distance threshold for considering a point "on" an edge

        Returns:
            Number of points near any edge
        """
        if len(lines) != 4:
            return 0

        on_edge_mask = np.zeros(len(points), dtype=bool)

        for line_point, line_dir in lines:
            distances = np.array(
                [self._point_to_line_distance(p, line_point, line_dir) for p in points]
            )
            on_edge_mask |= distances < edge_threshold

        return np.sum(on_edge_mask)

    def _lines_to_rectangle(
        self, lines: List[Tuple[np.ndarray, np.ndarray]]
    ) -> Optional[Tuple[float, float, float, float, float]]:
        """
        Convert 4 orthogonal lines to rectangle representation.

        Args:
            lines: List of 4 (line_point, line_dir) tuples

        Returns:
            (cx, cy, heading, width, height) or None if invalid
        """
        if len(lines) != 4:
            return None

        # Group lines into two pairs of parallel lines
        # Find two pairs: one pair parallel, another pair parallel and orthogonal to first pair

        # Try to find two parallel line pairs
        parallel_pairs = []
        used = set()

        for i in range(4):
            if i in used:
                continue
            dir1 = lines[i][1]
            pair = [i]
            for j in range(i + 1, 4):
                if j in used:
                    continue
                dir2 = lines[j][1]
                # Check if parallel (dot product close to ±1)
                dot = abs(np.dot(dir1, dir2))
                if dot > 0.95:  # Nearly parallel
                    pair.append(j)
                    used.add(i)
                    used.add(j)
                    parallel_pairs.append(pair)
                    break

        if len(parallel_pairs) != 2:
            return None

        # Get the two line directions (should be orthogonal)
        dir1 = lines[parallel_pairs[0][0]][1]
        dir2 = lines[parallel_pairs[1][0]][1]

        if not self._lines_are_orthogonal(dir1, dir2):
            return None

        # Compute rectangle corners by finding intersections
        # For each pair, find the two lines and compute their intersections with the other pair

        # Get line equations: ax + by + c = 0
        # For line through point p with direction d: d[1]*(x-p[0]) - d[0]*(y-p[1]) = 0
        # So: d[1]*x - d[0]*y - d[1]*p[0] + d[0]*p[1] = 0
        # a = d[1], b = -d[0], c = -d[1]*p[0] + d[0]*p[1]

        def line_to_abc(
            line_point: np.ndarray, line_dir: np.ndarray
        ) -> Tuple[float, float, float]:
            a = line_dir[1]
            b = -line_dir[0]
            c = -line_dir[1] * line_point[0] + line_dir[0] * line_point[1]
            return (a, b, c)

        def line_intersection(
            a1: float, b1: float, c1: float, a2: float, b2: float, c2: float
        ) -> Optional[np.ndarray]:
            """Find intersection of two lines."""
            det = a1 * b2 - a2 * b1
            if abs(det) < 1e-6:
                return None
            x = (b1 * c2 - b2 * c1) / det
            y = (a2 * c1 - a1 * c2) / det
            return np.array([x, y])

        # Get all 4 line equations
        line_eqs = []
        for pair_idx in parallel_pairs:
            for line_idx in pair_idx:
                line_point, line_dir = lines[line_idx]
                line_eqs.append(line_to_abc(line_point, line_dir))

        # Find intersections (corners)
        corners = []
        for i in range(2):  # First pair
            for j in range(2, 4):  # Second pair
                corner = line_intersection(
                    line_eqs[i][0],
                    line_eqs[i][1],
                    line_eqs[i][2],
                    line_eqs[j][0],
                    line_eqs[j][1],
                    line_eqs[j][2],
                )
                if corner is not None:
                    corners.append(corner)

        if len(corners) < 4:
            return None

        corners = np.array(corners)

        # Compute rectangle center
        center = corners.mean(axis=0)

        # Compute heading (angle of first direction)
        heading = np.arctan2(dir1[1], dir1[0])

        # Transform corners to rectangle-local frame
        c, s = np.cos(heading), np.sin(heading)
        R = np.array([[c, s], [-s, c]])  # Rotation to local frame

        local_corners = (corners - center) @ R.T

        # Compute width and height
        width = local_corners[:, 0].max() - local_corners[:, 0].min()
        height = local_corners[:, 1].max() - local_corners[:, 1].min()

        if width <= 0 or height <= 0:
            return None

        return (center[0], center[1], heading, width, height)

    def fit(self):
        """
        Fit rectangle to edge points by finding edges that go through the most points.

        Uses RANSAC to detect straight line segments, then finds 4 orthogonal lines
        that form a rectangle, scoring each by the number of points near the edges.
        """
        if len(self.edge_points) < 8:  # Need at least 8 points for 4 lines
            self.is_confident = False
            return

        points = np.array(self.edge_points)

        # Parameters for line detection
        inlier_threshold = 0.02  # 2cm threshold for points on a line
        min_inliers_per_line = max(
            3, len(points) // 10
        )  # At least 10% of points per line
        edge_threshold = 0.03  # 3cm threshold for scoring rectangle edges

        # Detect multiple lines using iterative RANSAC
        remaining_points = points.copy()
        detected_lines = []
        max_lines = 8  # Try to find up to 8 lines (we'll select best 4)

        for _ in range(max_lines):
            if len(remaining_points) < min_inliers_per_line:
                break

            result = self._fit_line_ransac(
                remaining_points, inlier_threshold, min_inliers_per_line
            )
            if result is None:
                break

            line_point, line_dir, inlier_mask = result
            detected_lines.append((line_point, line_dir))

            # Remove inliers for next iteration
            remaining_points = remaining_points[~inlier_mask]

        if len(detected_lines) < 4:
            # Fallback: try with lower threshold
            remaining_points = points.copy()
            detected_lines = []
            inlier_threshold *= 1.5
            min_inliers_per_line = max(2, len(points) // 15)

            for _ in range(max_lines):
                if len(remaining_points) < min_inliers_per_line:
                    break
                result = self._fit_line_ransac(
                    remaining_points, inlier_threshold, min_inliers_per_line
                )
                if result is None:
                    break
                line_point, line_dir, inlier_mask = result
                detected_lines.append((line_point, line_dir))
                remaining_points = remaining_points[~inlier_mask]

        if len(detected_lines) < 4:
            self.is_confident = False
            return

        # Try all combinations of 4 lines to find best rectangle
        best_rect = None
        best_score = -1

        from itertools import combinations

        for line_indices in combinations(range(len(detected_lines)), 4):
            candidate_lines = [detected_lines[i] for i in line_indices]

            # Check if lines form orthogonal pairs
            dirs = [line[1] for line in candidate_lines]
            has_orthogonal_pairs = False

            # Check if we can form two orthogonal pairs
            for i in range(4):
                for j in range(i + 1, 4):
                    if self._lines_are_orthogonal(dirs[i], dirs[j]):
                        # Check if remaining two are also orthogonal
                        other_indices = [k for k in range(4) if k != i and k != j]
                        if len(other_indices) == 2:
                            if self._lines_are_orthogonal(
                                dirs[other_indices[0]], dirs[other_indices[1]]
                            ):
                                has_orthogonal_pairs = True
                                break
                if has_orthogonal_pairs:
                    break

            if not has_orthogonal_pairs:
                continue

            # Try to convert to rectangle
            rect = self._lines_to_rectangle(candidate_lines)
            if rect is None:
                continue

            # Score rectangle by counting points near edges
            score = self._score_rectangle(points, candidate_lines, edge_threshold)

            if score > best_score:
                best_score = score
                best_rect = rect

        # Fallback: if no good rectangle found, try simpler approach
        # Test different rotation angles and find best fit
        if best_rect is None or best_score < len(points) * 0.3:
            # Try grid search over angles
            best_rect_fallback = None
            best_score_fallback = -1

            for angle_deg in range(0, 91, 5):  # Coarser grid for speed
                angle = np.deg2rad(angle_deg)
                c, s = np.cos(angle), np.sin(angle)
                R = np.array([[c, -s], [s, c]])

                rotated = points @ R.T
                min_x, min_y = rotated.min(axis=0)
                max_x, max_y = rotated.max(axis=0)

                # Create 4 edge lines in rotated frame
                edge_lines = [
                    (np.array([min_x, min_y]), np.array([1.0, 0.0])),  # Bottom
                    (np.array([max_x, min_y]), np.array([0.0, 1.0])),  # Right
                    (np.array([max_x, max_y]), np.array([-1.0, 0.0])),  # Top
                    (np.array([min_x, max_y]), np.array([0.0, -1.0])),  # Left
                ]

                # Transform lines back to world frame
                world_lines = []
                for line_point, line_dir in edge_lines:
                    world_point = line_point @ R
                    world_dir = line_dir @ R
                    world_dir = world_dir / np.linalg.norm(world_dir)
                    world_lines.append((world_point, world_dir))

                # Score this rectangle
                score = self._score_rectangle(points, world_lines, edge_threshold)

                if score > best_score_fallback:
                    best_score_fallback = score
                    width = max_x - min_x
                    height = max_y - min_y
                    center_rot = np.array([(min_x + max_x) / 2, (min_y + max_y) / 2])
                    center_world = center_rot @ R
                    best_rect_fallback = (
                        center_world[0],
                        center_world[1],
                        angle,
                        width,
                        height,
                    )

            if best_score_fallback > best_score:
                best_rect = best_rect_fallback
                best_score = best_score_fallback

        if best_rect:
            self.rectangle = best_rect
            # Consider confident if we have enough points and good score
            min_score_threshold = max(8, len(self.edge_points) * 0.4)
            self.is_confident = (
                len(self.edge_points) > 20 and best_score >= min_score_threshold
            )

    def get_rectangle(self) -> Optional[Tuple[float, float, float, float, float]]:
        """
        Get fitted rectangle.

        Returns:
            (cx, cy, heading, width, height) or None
        """
        return self.rectangle


class CoveragePlanner:
    """
    Boustrophedon path planner for coverage.

    Generates alternating lanes inside a rectangle with in-place turns between lanes.
    Waypoints are (x, y, theta) tuples where theta is the desired orientation.
    """

    def __init__(self):
        self.rectangle: Optional[Tuple[float, float, float, float, float]] = None
        self.lanes: List[List[Tuple[float, float]]] = []  # Original lane definitions
        self.path: List[Tuple[float, float, float]] = (
            []
        )  # Complete path with orientations
        self.current_waypoint_idx = 0

    def set_rectangle(self, rect: Tuple[float, float, float, float, float]):
        """
        Set the rectangle to plan coverage for.

        Args:
            rect: (cx, cy, heading, width, height)
        """
        self.rectangle = rect

    def get_inset_rectangle(self) -> Optional[Tuple[float, float, float, float, float]]:
        """
        Compute inset rectangle for coverage planning.

        The detected rectangle represents where sensors detected edges.
        We need to inset by the distance from wheelbase to where the vacuum would be
        when sensors are at the detected edge. This ensures the vacuum stays on the table.

        When sensors are at edge, vacuum is at: SENSOR_FWD + SENSOR_TO_VAC from wheelbase
        So we inset by this distance to ensure vacuum doesn't go off the table.

        Returns:
            (cx, cy, heading, width, height) inset rectangle, or None if no rectangle set
        """
        if not self.rectangle:
            return None

        cx, cy, heading, width, height = self.rectangle

        # Inset distance: distance from wheelbase to vacuum position when sensors are at edge
        # vacuum_offset = SENSOR_FWD + SENSOR_TO_VAC
        # This is the distance from wheelbase to vacuum when sensors are at edge
        vacuum_offset = GEOM.SENSOR_FWD + GEOM.SENSOR_TO_VAC

        # Inset from each side (reduce width and height by 2 * inset)
        # Use abs() since SENSOR_TO_VAC is negative (vacuum behind sensors)
        inset_width = max(0.01, width - 2 * abs(vacuum_offset))  # Ensure positive
        inset_height = max(0.01, height - 2 * abs(vacuum_offset))  # Ensure positive

        # Center remains the same, only dimensions change
        return (cx, cy, heading, inset_width, inset_height)

    def build_lanes(
        self, start_pose: Optional[Tuple[float, float, float]] = None
    ) -> List[List[Tuple[float, float]]]:
        """
        Build boustrophedon lanes.

        Args:
            start_pose: Optional robot pose (x, y, theta) to optimize start point.
                       If provided, will choose the corner closest to this pose.

        Returns:
            List of lanes, each lane is a list of waypoints
        """
        if not self.rectangle:
            return []

        # Use inset rectangle for coverage planning
        # This accounts for vacuum being behind sensors
        inset_rect = self.get_inset_rectangle()
        if not inset_rect:
            return []

        cx, cy, heading, width, height = inset_rect

        # Additional inset to account for robot size and overlap
        additional_inset = ALG.RECT_INSET if hasattr(ALG, "RECT_INSET") else 0.05
        lane_width = (
            GEOM.VAC_WIDTH - ALG.SWEEP_OVERLAP
            if hasattr(ALG, "SWEEP_OVERLAP")
            else GEOM.VAC_WIDTH - 0.02
        )

        # Compute number of lanes (using already-inset rectangle)
        inner_width = width - 2 * additional_inset
        inner_height = height - 2 * additional_inset

        # Determine lane direction (along longer dimension)
        if inner_width > inner_height:
            # Lanes along height (vertical)
            num_lanes = int(np.ceil(inner_width / lane_width))
            lane_spacing = inner_width / max(1, num_lanes)
            lane_length = inner_height
            lanes_vertical = True
        else:
            # Lanes along width (horizontal)
            num_lanes = int(np.ceil(inner_height / lane_width))
            lane_spacing = inner_height / max(1, num_lanes)
            lane_length = inner_width
            lanes_vertical = False

        # Rotation matrix
        c, s = np.cos(heading), np.sin(heading)
        R = np.array([[c, -s], [s, c]])

        # Generate base lanes
        base_lanes = []
        for i in range(num_lanes):
            # Compute lane endpoints in rectangle-local frame
            if lanes_vertical:
                # Vertical lanes
                x_local = -inner_width / 2 + (i + 0.5) * lane_spacing
                start_local = np.array([x_local, -inner_height / 2])
                end_local = np.array([x_local, inner_height / 2])
            else:
                # Horizontal lanes
                y_local = -inner_height / 2 + (i + 0.5) * lane_spacing
                start_local = np.array([-inner_width / 2, y_local])
                end_local = np.array([inner_width / 2, y_local])

            # Transform to world frame
            start_world = np.array([cx, cy]) + R @ start_local
            end_world = np.array([cx, cy]) + R @ end_local

            base_lanes.append((start_world, end_world))

        # Helper to create lanes list from base lanes with specific ordering
        def create_lanes_variant(
            reverse_order: bool, reverse_direction: bool
        ) -> List[List[Tuple[float, float]]]:
            variant_lanes = []
            indices = range(num_lanes)
            if reverse_order:
                indices = reversed(indices)

            for i, idx in enumerate(indices):
                start, end = base_lanes[idx]
                
                # Determine direction for this lane (alternating)
                # If reverse_direction is True, we flip the logic
                # Standard: even indices (0, 2, ...) go start->end
                #           odd indices (1, 3, ...) go end->start
                
                # However, "i" here is the index in the sequence of visitation
                # So we just alternate based on i
                
                # Actually, to maintain the boustrophedon pattern correctly when reversing order,
                # we need to be careful.
                # Let's just define the direction based on the visitation index i.
                
                use_start_to_end = (i % 2 == 0)
                if reverse_direction:
                    use_start_to_end = not use_start_to_end
                
                if use_start_to_end:
                    lane = [(start[0], start[1]), (end[0], end[1])]
                else:
                    lane = [(end[0], end[1]), (start[0], start[1])]
                
                variant_lanes.append(lane)
            return variant_lanes

        # If start_pose is provided, find the best variant
        if start_pose is not None:
            robot_pos = np.array([start_pose[0], start_pose[1]])
            best_lanes = []
            min_dist = float("inf")

            # Try 4 variants
            # 1. Normal order, normal direction
            # 2. Normal order, reversed direction (start at other end of first lane)
            # 3. Reversed order (start at last lane), normal direction
            # 4. Reversed order, reversed direction
            
            variants = [
                (False, False),
                (False, True),
                (True, False),
                (True, True),
            ]

            for rev_order, rev_dir in variants:
                candidate_lanes = create_lanes_variant(rev_order, rev_dir)
                if not candidate_lanes:
                    continue
                    
                # Check distance to start of first lane
                first_pt = np.array(candidate_lanes[0][0])
                dist = np.linalg.norm(robot_pos - first_pt)
                
                if dist < min_dist:
                    min_dist = dist
                    best_lanes = candidate_lanes
            
            self.lanes = best_lanes
        else:
            # Default behavior
            self.lanes = create_lanes_variant(False, False)

        # Build complete path with orientations and transitions
        self.path = self._build_path_with_transitions(
            self.lanes, heading, lanes_vertical
        )
        self.current_waypoint_idx = 0

        return self.lanes

    def _build_path_with_transitions(
        self,
        lanes: List[List[Tuple[float, float]]],
        rectangle_heading: float,
        lanes_vertical: bool,
    ) -> List[Tuple[float, float, float]]:
        """
        Build complete path with orientations and in-place turns between lanes.

        Args:
            lanes: List of lanes, each lane is [(x1, y1), (x2, y2)]
            rectangle_heading: Heading of the rectangle
            lanes_vertical: True if lanes are vertical (along height), False if horizontal

        Returns:
            List of waypoints as (x, y, theta) tuples
        """
        path = []

        if not lanes:
            return path

        for i, lane in enumerate(lanes):
            start_pt = lane[0]
            end_pt = lane[1]

            # Compute lane direction vector
            dx = end_pt[0] - start_pt[0]
            dy = end_pt[1] - start_pt[1]
            lane_heading = np.arctan2(dy, dx)

            # Start of lane: add waypoint with correct orientation
            path.append((start_pt[0], start_pt[1], lane_heading))

            # End of lane: add waypoint with same orientation (still pointing along lane)
            path.append((end_pt[0], end_pt[1], lane_heading))

            # If not the last lane, add transition waypoints
            if i < len(lanes) - 1:
                next_lane = lanes[i + 1]
                next_start = next_lane[0]

                # Compute heading to face next lane start
                dx_to_next = next_start[0] - end_pt[0]
                dy_to_next = next_start[1] - end_pt[1]
                heading_to_next = np.arctan2(dy_to_next, dx_to_next)

                # Turn in place at end of current lane to face next lane start
                path.append((end_pt[0], end_pt[1], heading_to_next))

                # Move to start of next lane (with same heading)
                path.append((next_start[0], next_start[1], heading_to_next))

                # Compute next lane direction
                next_end = next_lane[1]
                next_dx = next_end[0] - next_start[0]
                next_dy = next_end[1] - next_start[1]
                next_lane_heading = np.arctan2(next_dy, next_dx)

                # Turn in place at start of next lane to face lane direction
                path.append((next_start[0], next_start[1], next_lane_heading))

        # Add final waypoint at the end of the last lane to ensure completion detection
        if lanes:
            last_lane = lanes[-1]
            last_end = last_lane[1]  # End point of last lane
            # Use the same heading as the last lane
            last_start = last_lane[0]
            dx = last_end[0] - last_start[0]
            dy = last_end[1] - last_start[1]
            final_heading = np.arctan2(dy, dx)
            # Add final waypoint (duplicate of last lane end with same heading)
            # This ensures we have a clear completion point
            path.append((last_end[0], last_end[1], final_heading))

        return path

    def get_current_lane(self) -> Optional[List[Tuple[float, float]]]:
        """
        Get current waypoint for path following.
        Returns the next waypoint in the path as a single-point list for compatibility.
        """
        if not self.path or self.current_waypoint_idx >= len(self.path):
            return None

        # Return current waypoint as a single-point path for compatibility
        waypoint = self.path[self.current_waypoint_idx]
        return [(waypoint[0], waypoint[1])]

    def get_current_waypoint(self) -> Optional[Tuple[float, float, float]]:
        """Get current waypoint with orientation."""
        if not self.path or self.current_waypoint_idx >= len(self.path):
            return None
        return self.path[self.current_waypoint_idx]

    def advance_waypoint(self):
        """Advance to next waypoint in the path."""
        if not self.path:
            return

        # Advance to next waypoint, allowing advancement past the last waypoint
        # This ensures is_complete() can properly detect completion
        if self.current_waypoint_idx < len(self.path):
            self.current_waypoint_idx += 1

    def get_current_lane_index(self) -> Optional[int]:
        """
        Get the current lane index based on waypoint index.
        Returns the lane we're currently working on.
        """
        if not self.path or not self.lanes:
            return None

        # Each lane has 2 waypoints (start, end)
        # Transitions between lanes add 3 waypoints (turn at end, move to next, turn at start)
        # Pattern: [lane0_start(0), lane0_end(1), turn_end(2), move_to_lane1(3), turn_start_lane1(4), lane1_start(5), lane1_end(6), ...]

        idx = self.current_waypoint_idx
        waypoints_per_lane = 2
        transition_waypoints = 3
        cycle_length = (
            waypoints_per_lane + transition_waypoints
        )  # 5 waypoints per cycle after first lane

        # First lane: indices 0-1
        if idx < waypoints_per_lane:
            return 0

        # After first lane, pattern repeats every 5 waypoints
        # Lane 1 cycle: indices 2-6 (transition 2-4, lane 1: 5-6)
        # Lane 2 cycle: indices 7-11 (transition 7-9, lane 2: 10-11)

        # Subtract first lane waypoints
        remaining = idx - waypoints_per_lane

        # Which cycle are we in? (0-indexed, so cycle 0 = lane 1, cycle 1 = lane 2, etc.)
        cycle = remaining // cycle_length

        # Position within cycle (0-4)
        pos_in_cycle = remaining % cycle_length

        # Determine lane: if we're past the transition midpoint, we're working on the next lane
        if (
            pos_in_cycle >= transition_waypoints - 1
        ):  # At or past "move to next lane" waypoint
            lane_idx = cycle + 1  # Next lane
        else:
            lane_idx = cycle  # Still on previous lane (transitioning from it)

        # Clamp to valid range
        if lane_idx >= len(self.lanes):
            return len(self.lanes) - 1

        return lane_idx

    def is_complete(self) -> bool:
        """Check if all waypoints are complete."""
        if not self.path:
            return False
        return self.current_waypoint_idx >= len(self.path)
