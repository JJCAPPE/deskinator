"""
Motion controllers for boundary following and path following.
"""

import numpy as np
from typing import List, Tuple, Optional
from ..config import LIMS, ALG, GEOM, I2C
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
        self.current_path_id = None
        self.current_waypoint_idx = 0

        # Boundary discovery helper state
        self.boundary_phase = 0.0
        self._last_boundary_ts = None

        # Watchdog recovery state
        self.recovery_active = False
        self.recovery_step = 0
        self._recovery_start_pose: Optional[Tuple[float, float, float]] = None
        self._recovery_start_heading = 0.0
        self._recovery_turn_dir = 1

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
        sensors = edge_ctx.get("sensors", []) or [1.0] * len(I2C.MUX_CHANS)
        timestamp = edge_ctx.get("timestamp")

        if timestamp is None:
            dt = 1.0 / ALG.FUSE_HZ
        else:
            if self._last_boundary_ts is None:
                dt = 1.0 / ALG.FUSE_HZ
            else:
                dt = max(0.0, timestamp - self._last_boundary_ts)
            self._last_boundary_ts = timestamp

        # Compute average readings by side
        def _avg(indices: tuple[int, int]) -> float:
            vals = [sensors[i] for i in indices if i < len(sensors)]
            if not vals:
                return 1.0
            return float(np.mean(vals))

        left_mean = _avg(I2C.LEFT_PAIR)
        right_mean = _avg(I2C.RIGHT_PAIR)

        # Normalized edge balance: positive when right edge is closer
        balance = float(np.clip(left_mean - right_mean, -1.0, 1.0))

        # Slow down as any sensor nears the edge threshold
        proximity = min(left_mean, right_mean)
        edge_intensity = np.clip(
            (ALG.EDGE_THRESH - proximity) / ALG.EDGE_THRESH, 0.0, 1.0
        )

        v_target = ALG.BOUNDARY_SPEED * (1.0 - 0.5 * edge_intensity)
        v_target = float(np.clip(v_target, ALG.BOUNDARY_MIN_SPEED, ALG.BOUNDARY_SPEED))

        # Update slow oscillation phase for gentle wall search
        self.boundary_phase = (self.boundary_phase + ALG.BOUNDARY_SWAY_RATE * dt) % (
            2 * np.pi
        )

        sway = ALG.BOUNDARY_SWAY_GAIN * np.sin(self.boundary_phase)
        omega_target = ALG.BOUNDARY_EDGE_GAIN * balance + sway
        omega_target = float(
            np.clip(omega_target, -LIMS.OMEGA_MAX * 0.5, LIMS.OMEGA_MAX * 0.5)
        )

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
        self.set_path(path)

        if not self.current_path:
            return (0.0, 0.0)

        if self.current_waypoint_idx >= len(self.current_path):
            return (0.0, 0.0)

        x, y, theta = ekf_pose

        # Estimate how much of the upcoming path is still unswept
        fresh_ratio = 1.0
        if swept_map is not None and hasattr(swept_map, "is_swept"):
            samples = 0
            fresh = 0
            sample_end = min(len(self.current_path), self.current_waypoint_idx + 12)
            for idx in range(self.current_waypoint_idx, sample_end):
                px, py = self.current_path[idx]
                samples += 1
                if not swept_map.is_swept(px, py):
                    fresh += 1
            if samples > 0:
                fresh_ratio = fresh / samples

        lookahead_scale = np.clip(0.8 + 0.5 * (1.0 - fresh_ratio), 0.6, 1.4)
        dynamic_lookahead = self.lookahead_dist * lookahead_scale

        # Find lookahead point
        lookahead_point = self._find_lookahead_point(
            x, y, self.current_path, dynamic_lookahead
        )

        if lookahead_point is None:
            # Reached end of path
            return (0.0, 0.0)

        # Compute heading error
        dx = lookahead_point[0] - x
        dy = lookahead_point[1] - y
        target_heading = np.arctan2(dy, dx)
        heading_error = wrap_angle(target_heading - theta)

        # Pure pursuit control with coverage-aware speed scaling
        v_scale = np.clip(0.6 + 0.4 * (1.0 - fresh_ratio), 0.4, 1.0)
        v_target = LIMS.V_MAX * v_scale

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
        self,
        x: float,
        y: float,
        path: List[Tuple[float, float]],
        lookahead_dist: Optional[float] = None,
    ) -> Optional[Tuple[float, float]]:
        """Find lookahead point on path."""
        # Find closest point on path
        min_dist = float("inf")
        closest_idx = self.current_waypoint_idx

        target_dist = (
            lookahead_dist if lookahead_dist is not None else self.lookahead_dist
        )

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
            if dist >= target_dist:
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
        if not path:
            self.current_path = []
            self.current_path_id = None
            self.current_waypoint_idx = 0
            return

        path_id = id(path)
        if self.current_path_id != path_id:
            self.current_path = path
            self.current_path_id = path_id
            self.current_waypoint_idx = 0

    def is_path_complete(self) -> bool:
        """Check if current path is complete."""
        if not self.current_path:
            return False
        return self.current_waypoint_idx >= len(self.current_path) - 1

    def reset_path_progress(self):
        """Reset tracking so a new lane can be assigned."""
        self.current_path = []
        self.current_path_id = None
        self.current_waypoint_idx = 0

    def start_watchdog_recovery(self):
        """Initiate a watchdog recovery maneuver."""
        if self.edge_event_active:
            # Edge handling takes precedence
            return

        self.recovery_active = True
        self.recovery_step = 0
        self._recovery_start_pose = None
        self._recovery_turn_dir *= -1
        if self._recovery_turn_dir == 0:
            self._recovery_turn_dir = 1

    def update_recovery(
        self, ekf_pose: Tuple[float, float, float], dt: float
    ) -> Tuple[float, float]:
        """State machine for watchdog recovery."""
        if not self.recovery_active:
            return (0.0, 0.0)

        x, y, theta = ekf_pose

        if self.recovery_step == 0:
            self.recovery_step = 1
            self._recovery_start_pose = ekf_pose
            return (-LIMS.V_REV_MAX * 0.8, 0.0)

        if self.recovery_step == 1:
            if self._recovery_start_pose is None:
                self._recovery_start_pose = ekf_pose
                return (-LIMS.V_REV_MAX * 0.8, 0.0)
            dist = np.sqrt(
                (x - self._recovery_start_pose[0]) ** 2
                + (y - self._recovery_start_pose[1]) ** 2
            )
            if dist >= ALG.RECOVERY_BACKOFF:
                self.recovery_step = 2
                self._recovery_start_heading = theta
                return (0.0, 0.0)
            return (-LIMS.V_REV_MAX * 0.8, 0.0)

        if self.recovery_step == 2:
            target_angle = self._recovery_turn_dir * ALG.RECOVERY_TURN_ANGLE
            heading_diff = wrap_angle(theta - self._recovery_start_heading)
            remaining = target_angle - heading_diff
            if abs(remaining) < 0.05:
                self.recovery_step = 3
                self._recovery_start_pose = ekf_pose
                return (0.0, 0.0)
            omega_dir = np.sign(remaining) if remaining != 0 else np.sign(target_angle)
            return (0.0, omega_dir * LIMS.OMEGA_MAX * 0.3)

        if self.recovery_step == 3:
            if self._recovery_start_pose is None:
                self._recovery_start_pose = ekf_pose
                return (LIMS.V_MAX * 0.25, 0.0)
            dist = np.sqrt(
                (x - self._recovery_start_pose[0]) ** 2
                + (y - self._recovery_start_pose[1]) ** 2
            )
            if dist >= ALG.RECOVERY_BACKOFF:
                self.recovery_active = False
                self.recovery_step = 0
                self._recovery_start_pose = None
                return (0.0, 0.0)
            return (LIMS.V_MAX * 0.25, 0.0)

        self.recovery_active = False
        self.recovery_step = 0
        self._recovery_start_pose = None
        return (0.0, 0.0)
