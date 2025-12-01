"""
Robust Wall Following Behavior (Simplified).

Strategy: "Turn-Until-Clear" with margin.
1. Drive forward with slight bias towards wall.
2. If edge detected (sensor off table):
   - Stop immediately.
   - Turn AWAY from edge until ALL sensors are back on the table.
   - Continue turning for a small EXTRA angle (safety margin).
   - Resume driving.

This produces a tight "stitching" motion along the edge without backing up.
"""

import numpy as np
import math
from enum import Enum, auto
from typing import Tuple, Optional, List

try:
    from ..config import ALG, LIMS, GEOM, I2C
except ImportError:
    from config import ALG, LIMS, GEOM, I2C


class WallState(Enum):
    FIND_WALL = auto()  # Driving forward to find first edge
    AVOID = auto()  # Turning in place until clear
    FOLLOW = auto()  # Driving along wall with bias
    DONE = auto()  # Completed lap


class WallFollower:
    """
    Simplified Wall Follower.
    """

    # Configuration
    EXTRA_TURN_ANGLE = np.deg2rad(5)  # Extra turn after sensors clear
    FOLLOW_SPEED = 0.10  # m/s
    TURN_SPEED = 0.3  # rad/s
    WALL_BIAS = 0.15  # rad/s - bias towards wall

    MIN_LAP_DISTANCE = 6.0
    LAP_CLOSURE_RADIUS = 0.4

    def __init__(self):
        self.state = WallState.FIND_WALL

        # Pose tracking
        self.start_pose: Optional[Tuple[float, float, float]] = None
        self.lap_start_pose: Optional[Tuple[float, float, float]] = None

        # State variables
        self.turn_direction: int = -1  # -1 = right (CW), +1 = left (CCW)
        self.consistent_turn_direction: int = 0  # 0 = Undecided
        self.triggering_sensor: str = "none"

        # Avoidance state tracking
        self.sensors_cleared: bool = False
        self.heading_at_clear: float = 0.0

        # Distance tracking
        self.total_distance: float = 0.0
        self.last_pose: Optional[Tuple[float, float, float]] = None
        self.edge_count: int = 0

        # Debug
        self.corner_poses: List[Tuple[float, float, float]] = []

    def reset(self):
        self.__init__()

    def update(
        self, ekf_pose: Tuple[float, float, float], sensors: List[int], dt: float
    ) -> Tuple[float, float]:
        """
        Update wall follower state machine.
        Returns: (v, omega)
        """
        x, y, theta = ekf_pose

        # Track distance
        if self.last_pose is not None:
            dx = x - self.last_pose[0]
            dy = y - self.last_pose[1]
            self.total_distance += math.sqrt(dx * dx + dy * dy)
        self.last_pose = ekf_pose

        # Sensors: Low value = OFF TABLE (Edge)
        left_raw = (
            sensors[I2C.LEFT_SENSOR_IDX] if len(sensors) > I2C.LEFT_SENSOR_IDX else 255
        )
        right_raw = (
            sensors[I2C.RIGHT_SENSOR_IDX]
            if len(sensors) > I2C.RIGHT_SENSOR_IDX
            else 255
        )

        left_off = left_raw < ALG.EDGE_RAW_THRESH
        right_off = right_raw < ALG.EDGE_RAW_THRESH
        any_edge = left_off or right_off

        # State Machine
        if self.state == WallState.FIND_WALL:
            if self.start_pose is None:
                self.start_pose = ekf_pose

            if any_edge:
                print(f"[WallFollow] Wall found at ({x:.2f}, {y:.2f})")
                self.lap_start_pose = ekf_pose
                self.edge_count += 1
                self._decide_turn_dir(left_off, right_off)
                self._start_avoid(ekf_pose)
                return (0.0, 0.0)

            return (self.FOLLOW_SPEED, 0.0)

        elif self.state == WallState.AVOID:
            return self._state_avoid(ekf_pose, left_off, right_off)

        elif self.state == WallState.FOLLOW:
            if any_edge:
                self.edge_count += 1
                # Check if we hit the "other" wall (e.g. inside corner or drift)
                # Update turn direction if needed
                self._decide_turn_dir(left_off, right_off)
                self._start_avoid(ekf_pose)
                return (0.0, 0.0)

            # Check lap closure
            if self._check_lap_closure(ekf_pose):
                self.state = WallState.DONE
                return (0.0, 0.0)

            # Drive with bias towards the wall
            # If we turn RIGHT (CW, -1) to avoid wall, wall is on LEFT.
            # To bias TOWARDS the wall (Left), we need positive omega.
            # omega = -(-1) * bias = +bias. Correct.

            # If we turn LEFT (CCW, +1) to avoid wall, wall is on RIGHT.
            # To bias TOWARDS the wall (Right), we need negative omega.
            # omega = -(+1) * bias = -bias. Correct.

            omega = -self.consistent_turn_direction * self.WALL_BIAS
            return (self.FOLLOW_SPEED, omega)

        elif self.state == WallState.DONE:
            return (0.0, 0.0)

        return (0.0, 0.0)

    def _decide_turn_dir(self, left_off, right_off):
        """Decide which way to turn based on which sensor fell off."""
        if self.consistent_turn_direction != 0:
            return  # Already locked in

        if left_off:
            self.triggering_sensor = "left"
            self.consistent_turn_direction = -1  # Turn Right (CW)
        elif right_off:
            self.triggering_sensor = "right"
            self.consistent_turn_direction = 1  # Turn Left (CCW)
        else:
            # Both off or neither (shouldn't happen if triggered) - default to Right
            self.consistent_turn_direction = -1

    def _start_avoid(self, pose):
        """Start the avoidance maneuver."""
        self.state = WallState.AVOID
        self.sensors_cleared = False
        self.heading_at_clear = 0.0
        print(f"[WallFollow] Edge! Turning {self.consistent_turn_direction}...")

    def _state_avoid(self, pose, left_off, right_off) -> Tuple[float, float]:
        """Execute turn until clear sequence."""
        x, y, theta = pose

        # Check if ALL sensors are on the table
        all_on_table = (not left_off) and (not right_off)

        if not self.sensors_cleared:
            if all_on_table:
                self.sensors_cleared = True
                self.heading_at_clear = theta
                print(
                    f"[WallFollow] Sensors clear. Adding {np.rad2deg(self.EXTRA_TURN_ANGLE):.1f} deg margin..."
                )

            # Turn in the safe direction
            return (0.0, self.consistent_turn_direction * self.TURN_SPEED)

        else:
            # Sensors have cleared, perform extra turn
            turned_extra = abs(self._wrap_angle(theta - self.heading_at_clear))

            if turned_extra >= self.EXTRA_TURN_ANGLE:
                print(f"[WallFollow] Turn complete. Resuming follow.")
                self.state = WallState.FOLLOW
                return (0.0, 0.0)

            # Continue turning
            return (0.0, self.consistent_turn_direction * self.TURN_SPEED)

    def _check_lap_closure(self, pose) -> bool:
        if self.lap_start_pose is None:
            return False
        if self.total_distance < self.MIN_LAP_DISTANCE:
            return False
        if self.edge_count < 4:
            return False

        dx = pose[0] - self.lap_start_pose[0]
        dy = pose[1] - self.lap_start_pose[1]
        dist = math.sqrt(dx * dx + dy * dy)

        return dist < self.LAP_CLOSURE_RADIUS

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
