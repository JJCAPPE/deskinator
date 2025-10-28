"""
2D swept coverage map.

Tracks which areas have been cleaned using a raster grid.
"""

import numpy as np
from typing import Tuple, Optional
from ..config import ALG, GEOM
from ..slam.frames import rotation_matrix


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

        expanded = False

        if x < self.min_x:
            self.min_x = x - 1.0
            expanded = True
        if x > self.max_x:
            self.max_x = x + 1.0
            expanded = True
        if y < self.min_y:
            self.min_y = y - 1.0
            expanded = True
        if y > self.max_y:
            self.max_y = y + 1.0
            expanded = True

        if expanded:
            # Save old grid
            old_grid = self.grid.copy()
            old_min_x, old_min_y = self.min_x, self.min_y

            # Reinitialize with new bounds
            self._initialize_grid()

            # Copy old data
            # (Simplified: just clear and continue)

    def add_forward_sweep(self, pose: Tuple[float, float, float], ds: float):
        """
        Mark area as swept during forward motion.

        Args:
            pose: Robot pose (x, y, θ)
            ds: Distance traveled (positive for forward)
        """
        if ds <= 0:
            return  # Only count forward motion

        x, y, theta = pose

        # Expand grid if needed
        self._expand_if_needed(x, y)

        # Compute vacuum footprint
        # Vacuum is at SENSOR_FWD + SENSOR_TO_VAC ahead of axle
        vac_offset = GEOM.SENSOR_FWD + GEOM.SENSOR_TO_VAC
        vac_width = GEOM.VAC_WIDTH

        # Sample along the path
        n_samples = max(1, int(ds / (self.resolution / 2)))

        for k in range(n_samples + 1):
            frac = k / max(1, n_samples)
            s = frac * ds

            # Robot position at this point
            x_s = x + s * np.cos(theta)
            y_s = y + s * np.sin(theta)

            # Vacuum center position
            vac_x = x_s + vac_offset * np.cos(theta)
            vac_y = y_s + vac_offset * np.sin(theta)

            # Mark rectangular footprint
            self._mark_rectangle(vac_x, vac_y, theta, vac_width, ds / n_samples)

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
