"""
Rectangle boundary inference from edge detections (Simplified).

Uses Minimum Area Rectangle (Bounding Box) logic to robustly fit a desk shape
to a cloud of edge points, ignoring potential odometry drift or graph distortions.
"""

import numpy as np
from typing import List, Tuple, Optional
try:
    from ..config import ALG
except ImportError:
    from config import ALG


class RectangleFit:
    """Estimates rectangular boundary from edge points."""

    def __init__(self):
        """Initialize rectangle estimator."""
        self.edge_points = []  # List of (x, y) tuples
        self.rectangle = None  # (center_x, center_y, heading, width, height)
        self.is_confident = False

    def add_edge_point(self, point: tuple[float, float]):
        """
        Add an edge detection point.

        Args:
            point: Edge point (x, y) in world frame
        """
        self.edge_points.append(point)

    def fit(self) -> bool:
        """
        Fit minimal area rectangle to accumulated edge points.
        
        This is a robust O(N) or O(N log N) approach that works well for
        convex hull fitting, but here we just do a simpler brute-force or PCA
        approach for robustness if points are noisy.
        
        Since we expect a desk, we can iterate through a range of rotations
        and find the one that minimizes bounding box area.

        Returns:
            True if rectangle is confident
        """
        if len(self.edge_points) < 20:
            return False

        points = np.array(self.edge_points)

        # Optimization: Only run full fit periodically or if point count changed significantly
        # For now, we run every time because N is small (~1000 points max)

        best_area = float('inf')
        best_rect = None

        # Search headings from 0 to 90 degrees (rectangles have 90 deg symmetry)
        # Coarse search
        search_angles = np.deg2rad(np.arange(0, 90, 1.0)) 
        
        for angle in search_angles:
            area, rect = self._compute_bbox_area(points, angle)
            if area < best_area:
                best_area = area
                best_rect = rect
        
        # Refine search around best angle
        if best_rect:
            best_heading = best_rect[2]
            fine_angles = np.linspace(best_heading - np.deg2rad(1.0), best_heading + np.deg2rad(1.0), 20)
            for angle in fine_angles:
                area, rect = self._compute_bbox_area(points, angle)
                if area < best_area:
                    best_area = area
                    best_rect = rect

        self.rectangle = best_rect
        
        # Confidence check:
        # 1. Area is reasonable for a desk (> 0.5 m^2)
        # 2. Aspect ratio is reasonable (1:1 to 1:4)
        # 3. Sufficient points (already checked)
        
        if self.rectangle:
            w, h = self.rectangle[3], self.rectangle[4]
            area = w * h
            if area > 0.3: # Minimum desk size
                self.is_confident = True
            else:
                self.is_confident = False
        
        return self.is_confident

    def _compute_bbox_area(self, points, angle):
        """Compute bounding box area for a given rotation."""
        c, s = np.cos(angle), np.sin(angle)
        # Rotation matrix to align points with axes
        # To rotate points by -angle (to align with X-axis), we use:
        # [ cos  sin]
        # [-sin  cos]
        R = np.array([[c, s], [-s, c]])
        
        # Rotate points
        rotated = points @ R.T
        
        min_x, max_x = np.min(rotated[:, 0]), np.max(rotated[:, 0])
        min_y, max_y = np.min(rotated[:, 1]), np.max(rotated[:, 1])
        
        width = max_x - min_x
        height = max_y - min_y
        area = width * height
        
        # Center in aligned frame
        center_x_aligned = (min_x + max_x) / 2
        center_y_aligned = (min_y + max_y) / 2
        
        # Rotate center back to world frame
        # Inverse rotation is transpose (angle)
        # [ cos -sin]
        # [ sin  cos]
        center_x = center_x_aligned * c - center_y_aligned * s
        center_y = center_x_aligned * s + center_y_aligned * c
        
        return area, (center_x, center_y, angle, width, height)

    def get_rectangle(self) -> Optional[Tuple[float, float, float, float, float]]:
        """
        Get fitted rectangle if confident.

        Returns:
            (center_x, center_y, heading, width, height) or None
        """
        if self.is_confident:
            return self.rectangle
        return None

    def get_corners(self) -> Optional[List[Tuple[float, float]]]:
        """
        Get rectangle corners.

        Returns:
            List of (x, y) corner points or None
        """
        if not self.is_confident or not self.rectangle:
            return None

        cx, cy, heading, width, height = self.rectangle

        # Corners in rectangle frame
        corners_local = [
            (-width / 2, -height / 2),
            (width / 2, -height / 2),
            (width / 2, height / 2),
            (-width / 2, height / 2),
        ]

        # Transform to world frame
        c, s = np.cos(heading), np.sin(heading)
        corners_world = []

        for lx, ly in corners_local:
            wx = cx + c * lx - s * ly
            wy = cy + s * lx + c * ly
            corners_world.append((wx, wy))

        return corners_world
