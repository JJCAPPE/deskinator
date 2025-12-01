"""
2D swept coverage map.

Tracks which areas have been cleaned using a raster grid.
"""

import numpy as np
from typing import Tuple, Optional
try:
    from ..config import ALG, GEOM
    from ..slam.frames import rotation_matrix
except ImportError:
    from config import ALG, GEOM
    from slam.frames import rotation_matrix


class SweptMap:
    """2D occupancy grid tracking swept coverage."""

    def __init__(
        self,
        bounds: Optional[Tuple[float, float, float, float]] = None,
        resolution: float = ALG.GRID_RES,
    ):
        """
        Initialize swept map.

        Args:
            bounds: (min_x, max_x, min_y, max_y) or None for auto-expand
            resolution: Grid cell size in meters
        """
        self.resolution = resolution

        if bounds:
            self.min_x, self.max_x, self.min_y, self.max_y = bounds
            self.auto_expand = False
        else:
            # Start with a reasonable default and expand as needed
            self.min_x, self.max_x = -1.0, 1.0
            self.min_y, self.max_y = -1.0, 1.0
            self.auto_expand = True

        self._initialize_grid()

    def _initialize_grid(self):
        """Initialize or reinitialize the grid."""
        self.width = int((self.max_x - self.min_x) / self.resolution) + 1
        self.height = int((self.max_y - self.min_y) / self.resolution) + 1
        self.grid = np.zeros((self.height, self.width), dtype=np.uint8)

    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices."""
        i = int((y - self.min_y) / self.resolution)
        j = int((x - self.min_x) / self.resolution)
        return (i, j)

    def _expand_if_needed(self, x: float, y: float):
        """Expand grid if point is outside current bounds."""
        if not self.auto_expand:
            return

        old_min_x, old_max_x = self.min_x, self.max_x
        old_min_y, old_max_y = self.min_y, self.max_y

        new_min_x, new_max_x = old_min_x, old_max_x
        new_min_y, new_max_y = old_min_y, old_max_y

        if x < new_min_x:
            new_min_x = x - 1.0
        if x > new_max_x:
            new_max_x = x + 1.0
        if y < new_min_y:
            new_min_y = y - 1.0
        if y > new_max_y:
            new_max_y = y + 1.0

        expanded = (
            new_min_x != old_min_x
            or new_max_x != old_max_x
            or new_min_y != old_min_y
            or new_max_y != old_max_y
        )

        if expanded:
            old_grid = self.grid.copy()

            # Update bounds and rebuild grid
            self.min_x, self.max_x = new_min_x, new_max_x
            self.min_y, self.max_y = new_min_y, new_max_y
            self._initialize_grid()

            # Reproject previous coverage into new grid
            for i in range(old_grid.shape[0]):
                for j in range(old_grid.shape[1]):
                    if old_grid[i, j] == 0:
                        continue

                    wx = old_min_x + j * self.resolution
                    wy = old_min_y + i * self.resolution
                    gi, gj = self._world_to_grid(wx, wy)

                    if 0 <= gi < self.height and 0 <= gj < self.width:
                        self.grid[gi, gj] = 1

    def add_trajectory_sweep(
        self,
        pose_start: Tuple[float, float, float],
        pose_end: Tuple[float, float, float],
    ):
        """
        Mark area as swept by interpolating between two poses.

        Args:
            pose_start: Starting pose (x, y, θ)
            pose_end: Ending pose (x, y, θ)
        """
        x1, y1, theta1 = pose_start
        x2, y2, theta2 = pose_end

        # Calculate distance
        dx = x2 - x1
        dy = y2 - y1
        dist = (dx * dx + dy * dy) ** 0.5

        # Calculate angle difference (shortest path)
        dtheta = theta2 - theta1
        # Normalize to -pi..pi
        while dtheta > np.pi:
            dtheta -= 2 * np.pi
        while dtheta < -np.pi:
            dtheta += 2 * np.pi

        # If movement is negligible, skip
        if dist < 0.001 and abs(dtheta) < 0.001:
            return

        # Expand grid if needed
        self._expand_if_needed(x1, y1)
        self._expand_if_needed(x2, y2)

        # Vacuum params
        vac_offset = GEOM.SENSOR_FWD + GEOM.SENSOR_TO_VAC
        vac_width = GEOM.VAC_WIDTH

        # Determine number of steps
        # We want steps small enough so vacuum width covers gaps
        # Step size ~ resolution / 2
        step_size = self.resolution / 2.0
        
        # Steps for translation
        n_trans = int(dist / step_size)
        
        # Steps for rotation (arc length at vacuum offset)
        # Arc length ~ |dtheta| * |vac_offset|
        arc_len = abs(dtheta * vac_offset)
        n_rot = int(arc_len / step_size)
        
        n_steps = max(1, n_trans, n_rot)

        for k in range(n_steps + 1):
            t = k / n_steps
            
            # Interpolate pose
            x = x1 + t * dx
            y = y1 + t * dy
            theta = theta1 + t * dtheta
            
            # Vacuum center position
            vac_x = x + vac_offset * np.cos(theta)
            vac_y = y + vac_offset * np.sin(theta)

            # Mark rectangular footprint
            # Use a small length for the "brush" at this instant
            # effectively drawing a thick line
            self._mark_rectangle(vac_x, vac_y, theta, vac_width, step_size)

    def add_forward_sweep(self, pose: Tuple[float, float, float], ds: float):
        """
        Mark area as swept during forward motion.
        
        DEPRECATED: Use add_trajectory_sweep instead for better accuracy.

        Args:
            pose: Robot pose (x, y, θ)
            ds: Distance traveled (positive for forward)
        """
        if ds <= 0:
            return  # Only count forward motion

        x, y, theta = pose
        
        # Calculate end pose assuming straight line motion
        x_end = x + ds * np.cos(theta)
        y_end = y + ds * np.sin(theta)
        
        self.add_trajectory_sweep(pose, (x_end, y_end, theta))

    def _mark_rectangle(
        self, cx: float, cy: float, theta: float, width: float, length: float
    ):
        """Mark a rectangular region as swept."""
        # Sample points within the rectangle
        R = rotation_matrix(theta)

        n_width = max(1, int(width / self.resolution))
        n_length = max(1, int(length / self.resolution))

        for i in range(-n_width // 2, n_width // 2 + 1):
            for j in range(-n_length // 2, n_length // 2 + 1):
                # Local coordinates
                local = np.array([j * self.resolution, i * self.resolution])

                # World coordinates
                world = np.array([cx, cy]) + R @ local

                # Grid coordinates
                gi, gj = self._world_to_grid(world[0], world[1])

                # Mark as swept
                if 0 <= gi < self.height and 0 <= gj < self.width:
                    self.grid[gi, gj] = 1

    def coverage_ratio(
        self, rect: Optional[Tuple[float, float, float, float, float]] = None
    ) -> float:
        """
        Compute coverage ratio.

        Args:
            rect: Rectangle (cx, cy, heading, width, height) to compute coverage within,
                  or None for entire grid

        Returns:
            Coverage ratio (0.0 to 1.0)
        """
        if rect is None:
            # Coverage of entire grid
            total = self.grid.size
            swept = np.sum(self.grid)
            return swept / total if total > 0 else 0.0
        else:
            # Coverage within rectangle
            cx, cy, heading, width, height = rect

            # Create mask for rectangle region
            mask = self._rectangle_mask(cx, cy, heading, width, height)

            total = np.sum(mask)
            swept = np.sum(self.grid[mask > 0])

            return swept / total if total > 0 else 0.0

    def _rectangle_mask(
        self, cx: float, cy: float, heading: float, width: float, height: float
    ) -> np.ndarray:
        """Create a mask for a rectangular region."""
        mask = np.zeros_like(self.grid)

        R_inv = rotation_matrix(-heading)

        for i in range(self.height):
            for j in range(self.width):
                # World coordinates of grid cell
                wx = self.min_x + j * self.resolution
                wy = self.min_y + i * self.resolution

                # Relative to rectangle center
                rel = np.array([wx - cx, wy - cy])

                # In rectangle frame
                local = R_inv @ rel

                # Check if inside rectangle
                if abs(local[0]) <= width / 2 and abs(local[1]) <= height / 2:
                    mask[i, j] = 1

        return mask

    def get_grid(self) -> np.ndarray:
        """Get the coverage grid."""
        return self.grid.copy()

    def is_swept(self, x: float, y: float) -> bool:
        """Check if a world point has been marked as swept."""
        gi, gj = self._world_to_grid(x, y)
        if 0 <= gi < self.height and 0 <= gj < self.width:
            return bool(self.grid[gi, gj])
        return False
