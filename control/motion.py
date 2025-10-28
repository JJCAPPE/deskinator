"""
Motion controllers for boundary following and path following.
"""

import numpy as np
from typing import List, Tuple, Optional
from ..config import LIMS, ALG, GEOM
from ..slam.frames import wrap_angle


class MotionController:
    """High-level motion control for boundary and path following."""

    def __init__(self):
        """Initialize motion controller."""
        self.mode = "idle"  # idle, boundary, path
        self.edge_event_active = False
        self.edge_event_side = None
        self.edge_event_step = 0  # 0=brake, 1=backoff, 2=rotate, 3=sidestep

        # Path following state
        self.current_path = []
        self.current_waypoint_idx = 0

        # Pure pursuit parameters
        self.lookahead_dist = 0.15  # m

    def cmd_boundary(
        self, ekf_pose: Tuple[float, float, float], edge_ctx: dict
    ) -> Tuple[float, float]:
        """
        Generate command for boundary discovery/following.

        Args:
            ekf_pose: Current pose (x, y, θ)
            edge_ctx: Edge sensor context with 'sensors' list

        Returns:
            (v, ω) command
        """
        # Simple strategy: move forward slowly, react to edges
        v_target = 0.08  # Slow forward
        omega_target = 0.0

        # Check for warnings (single sensor)
        sensors = edge_ctx.get("sensors", [0.0] * 4)

        if min(sensors) < ALG.EDGE_THRESH * 1.5:
            # Slow down near edge
            v_target = 0.04

        return (v_target, omega_target)

    def cmd_follow_path(
        self,
        ekf_pose: Tuple[float, float, float],
        path: List[Tuple[float, float]],
        swept_map,
    ) -> Tuple[float, float]:
        """
        Generate command for path following using pure pursuit.

        Args:
            ekf_pose: Current pose (x, y, θ)
            path: List of (x, y) waypoints
            swept_map: SweptMap instance

        Returns:
            (v, ω) command
        """
        if not path or self.current_waypoint_idx >= len(path):
            return (0.0, 0.0)

        x, y, theta = ekf_pose

        # Find lookahead point
        lookahead_point = self._find_lookahead_point(x, y, path)

        if lookahead_point is None:
            # Reached end of path
            return (0.0, 0.0)

        # Compute heading error
        dx = lookahead_point[0] - x
        dy = lookahead_point[1] - y
        target_heading = np.arctan2(dy, dx)
        heading_error = wrap_angle(target_heading - theta)

        # Pure pursuit control
        v_target = LIMS.V_MAX * 0.6  # 60% of max speed

        # Curvature (simplified pure pursuit)
        L = np.sqrt(dx * dx + dy * dy)
        if L > 1e-3:
            omega_target = 2.0 * v_target * np.sin(heading_error) / L
        else:
            omega_target = 0.0

        # Limit angular velocity
        omega_target = np.clip(omega_target, -LIMS.OMEGA_MAX, LIMS.OMEGA_MAX)

        # Slow down for sharp turns
        if abs(omega_target) > LIMS.OMEGA_MAX * 0.5:
            v_target *= 0.5

        return (v_target, omega_target)

    def _find_lookahead_point(
        self, x: float, y: float, path: List[Tuple[float, float]]
    ) -> Optional[Tuple[float, float]]:
        """Find lookahead point on path."""
        # Find closest point on path
        min_dist = float("inf")
        closest_idx = self.current_waypoint_idx

        for i in range(self.current_waypoint_idx, len(path)):
            px, py = path[i]
            dist = np.sqrt((px - x) ** 2 + (py - y) ** 2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Find point at lookahead distance
        for i in range(closest_idx, len(path)):
            px, py = path[i]
            dist = np.sqrt((px - x) ** 2 + (py - y) ** 2)
            if dist >= self.lookahead_dist:
                self.current_waypoint_idx = i
                return (px, py)

        # Return last point if close to end
        if len(path) > 0:
            self.current_waypoint_idx = len(path) - 1
            return path[-1]

        return None

    def handle_edge_event(self, side: str):
        """
        Trigger edge event handling sequence.

        Args:
            side: 'left' or 'right'
        """
        self.edge_event_active = True
        self.edge_event_side = side
        self.edge_event_step = 0

    def update_edge_event(
        self, ekf_pose: Tuple[float, float, float], dt: float
    ) -> Tuple[float, float]:
        """
        Update edge event handling state machine.

        Args:
            ekf_pose: Current pose (x, y, θ)
            dt: Time step

        Returns:
            (v, ω) command
        """
        if not self.edge_event_active:
            return (0.0, 0.0)

        x, y, theta = ekf_pose

        if self.edge_event_step == 0:
            # Brake
            self.edge_event_step = 1
            self.backoff_start_pose = ekf_pose
            return (0.0, 0.0)

        elif self.edge_event_step == 1:
            # Backoff
            dist_backed = np.sqrt(
                (x - self.backoff_start_pose[0]) ** 2
                + (y - self.backoff_start_pose[1]) ** 2
            )

            if dist_backed >= ALG.POST_EDGE_BACKOFF:
                self.edge_event_step = 2
                self.rotate_start_heading = theta
                return (0.0, 0.0)
            else:
                return (-LIMS.V_REV_MAX, 0.0)

        elif self.edge_event_step == 2:
            # Rotate away from edge
            rotation_angle = np.pi / 4  # 45 degrees
            if self.edge_event_side == "left":
                rotation_angle = -rotation_angle

            heading_diff = wrap_angle(theta - self.rotate_start_heading)

            if abs(heading_diff - rotation_angle) < 0.1:
                self.edge_event_step = 3
                self.sidestep_start_pose = ekf_pose
                return (0.0, 0.0)
            else:
                # Turn
                omega_dir = 1.0 if rotation_angle > 0 else -1.0
                return (0.0, omega_dir * LIMS.OMEGA_MAX * 0.3)

        elif self.edge_event_step == 3:
            # Side-step along boundary
            dist_stepped = np.sqrt(
                (x - self.sidestep_start_pose[0]) ** 2
                + (y - self.sidestep_start_pose[1]) ** 2
            )

            if dist_stepped >= ALG.POST_EDGE_SIDE_STEP:
                self.edge_event_active = False
                self.edge_event_step = 0
                return (0.0, 0.0)
            else:
                return (LIMS.V_MAX * 0.3, 0.0)

        return (0.0, 0.0)

    def set_path(self, path: List[Tuple[float, float]]):
        """Set current path for following."""
        self.current_path = path
        self.current_waypoint_idx = 0

    def is_path_complete(self) -> bool:
        """Check if current path is complete."""
        return self.current_waypoint_idx >= len(self.current_path) - 1
