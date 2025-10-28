"""
Coverage path planner.

Generates boustrophedon (lawn-mower) paths for cleaning coverage.
"""

import numpy as np
from typing import List, Tuple, Optional
from ..config import ALG, GEOM, LIMS
from ..slam.frames import rotation_matrix


class CoveragePlanner:
    """Generates coverage paths for desk cleaning."""

    def __init__(self):
        """Initialize coverage planner."""
        self.rectangle = None  # (cx, cy, heading, width, height)
        self.lanes = []  # List of lanes, each lane is a list of waypoints
        self.current_lane_idx = 0
        self.current_waypoint_idx = 0

    def set_rectangle(self, rect: Tuple[float, float, float, float, float]):
        """
        Set the target rectangle for coverage.

        Args:
            rect: Rectangle (cx, cy, heading, width, height)
        """
        self.rectangle = rect
        self.lanes = []
        self.current_lane_idx = 0
        self.current_waypoint_idx = 0

    def build_lanes(self) -> List[List[Tuple[float, float]]]:
        """
        Build boustrophedon lanes for coverage.

        Returns:
            List of lanes, each lane is a list of (x, y) waypoints
        """
        if not self.rectangle:
            return []

        cx, cy, heading, width, height = self.rectangle

        # Inset rectangle by safety margin
        safety_margin = GEOM.SENSOR_FWD + LIMS.V_MAX**2 / (2 * LIMS.A_MAX)
        width_inset = width - 2 * safety_margin
        height_inset = height - 2 * safety_margin

        if width_inset <= 0 or height_inset <= 0:
            print("Warning: Rectangle too small for safe coverage")
            return []

        # Determine lane orientation (along long axis)
        if width > height:
            # Lanes run along width
            lane_length = width_inset
            lane_spacing = GEOM.VAC_WIDTH - ALG.SWEEP_OVERLAP
            n_lanes = int(height_inset / lane_spacing) + 1
            perpendicular = True
        else:
            # Lanes run along height
            lane_length = height_inset
            lane_spacing = GEOM.VAC_WIDTH - ALG.SWEEP_OVERLAP
            n_lanes = int(width_inset / lane_spacing) + 1
            perpendicular = False

        # Generate lanes in rectangle frame
        lanes_local = []

        for i in range(n_lanes):
            # Offset perpendicular to lane direction
            offset = (
                -0.5 * (width_inset if not perpendicular else height_inset)
                + i * lane_spacing
            )

            if perpendicular:
                # Lane runs along width (x direction)
                start = (-0.5 * lane_length, offset)
                end = (0.5 * lane_length, offset)
            else:
                # Lane runs along height (y direction)
                start = (offset, -0.5 * lane_length)
                end = (offset, 0.5 * lane_length)

            # Alternate direction for boustrophedon
            if i % 2 == 1:
                start, end = end, start

            # Create waypoints along lane
            n_waypoints = max(2, int(lane_length / 0.1))  # ~10cm spacing
            waypoints_local = []

            for j in range(n_waypoints + 1):
                frac = j / n_waypoints
                wx = start[0] + frac * (end[0] - start[0])
                wy = start[1] + frac * (end[1] - start[1])
                waypoints_local.append((wx, wy))

            lanes_local.append(waypoints_local)

        # Transform to world frame
        R = rotation_matrix(heading)
        lanes_world = []

        for lane_local in lanes_local:
            lane_world = []
            for wx_local, wy_local in lane_local:
                local = np.array([wx_local, wy_local])
                world = np.array([cx, cy]) + R @ local
                lane_world.append((world[0], world[1]))
            lanes_world.append(lane_world)

        self.lanes = lanes_world
        return self.lanes

    def get_current_waypoint(self) -> Optional[Tuple[float, float]]:
        """
        Get current target waypoint.

        Returns:
            (x, y) waypoint or None if done
        """
        if not self.lanes or self.current_lane_idx >= len(self.lanes):
            return None

        lane = self.lanes[self.current_lane_idx]

        if self.current_waypoint_idx >= len(lane):
            # Move to next lane
            self.current_lane_idx += 1
            self.current_waypoint_idx = 0
            return self.get_current_waypoint()

        return lane[self.current_waypoint_idx]

    def advance_waypoint(self):
        """Advance to next waypoint."""
        self.current_waypoint_idx += 1

    def get_current_lane(self) -> Optional[List[Tuple[float, float]]]:
        """
        Get current lane.

        Returns:
            List of (x, y) waypoints or None
        """
        if not self.lanes or self.current_lane_idx >= len(self.lanes):
            return None
        return self.lanes[self.current_lane_idx]

    def is_complete(self) -> bool:
        """Check if all lanes are complete."""
        return self.current_lane_idx >= len(self.lanes)

    def reset(self):
        """Reset to start of coverage."""
        self.current_lane_idx = 0
        self.current_waypoint_idx = 0
