"""
Rectangle boundary inference from edge detections.

Uses RANSAC line fitting and orthogonality constraints to estimate
the rectangular desk boundary.
"""

import numpy as np
from typing import List, Tuple, Optional
from ..config import ALG
from .frames import wrap_angle


class RectangleFit:
    """Estimates rectangular boundary from edge points."""

    def __init__(self):
        """Initialize rectangle estimator."""
        self.edge_points = []  # List of (x, y) tuples
        self.lines = []  # List of fitted lines (a, b, c) for ax + by + c = 0
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
        Fit rectangle to accumulated edge points.

        Returns:
            True if rectangle is confident
        """
        if len(self.edge_points) < 10:
            return False

        # Convert to numpy array
        points = np.array(self.edge_points)

        # Fit lines using RANSAC
        self.lines = self._fit_lines_ransac(points, n_lines=4)

        if len(self.lines) < 4:
            return False

        # Check orthogonality
        if not self._check_orthogonality(self.lines):
            return False

        # Compute rectangle parameters
        self.rectangle = self._compute_rectangle(self.lines)

        # Check confidence based on perimeter coverage
        perimeter_observed = self._estimate_perimeter_coverage()
        self.is_confident = perimeter_observed >= ALG.RECT_CONF_MIN_LEN

        return self.is_confident

    def _fit_lines_ransac(
        self, points: np.ndarray, n_lines: int = 4, n_iterations: int = 100
    ) -> List[Tuple[float, float, float]]:
        """
        Fit multiple lines using RANSAC.

        Args:
            points: Nx2 array of points
            n_lines: Number of lines to fit
            n_iterations: RANSAC iterations per line

        Returns:
            List of line parameters (a, b, c)
        """
        lines = []
        remaining_points = points.copy()

        for _ in range(n_lines):
            if len(remaining_points) < 10:
                break

            best_line = None
            best_inliers = []

            for _ in range(n_iterations):
                # Random sample
                idx = np.random.choice(len(remaining_points), 2, replace=False)
                p1, p2 = remaining_points[idx]

                # Fit line
                line = self._fit_line_two_points(p1, p2)

                # Count inliers
                dist = self._point_to_line_distance(remaining_points, line)
                inliers = remaining_points[dist < 0.03]  # 3 cm threshold

                if len(inliers) > len(best_inliers):
                    best_inliers = inliers
                    best_line = line

            if best_line is not None and len(best_inliers) > 5:
                # Refit with all inliers
                best_line = self._fit_line_lstsq(best_inliers)
                lines.append(best_line)

                # Remove inliers
                dist = self._point_to_line_distance(remaining_points, best_line)
                remaining_points = remaining_points[dist >= 0.03]

        return lines

    def _fit_line_two_points(
        self, p1: np.ndarray, p2: np.ndarray
    ) -> Tuple[float, float, float]:
        """Fit line through two points."""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        # Line: ax + by + c = 0
        # Normal vector: (a, b) = perpendicular to (dx, dy)
        a = -dy
        b = dx
        c = -(a * p1[0] + b * p1[1])

        # Normalize
        norm = np.sqrt(a * a + b * b)
        if norm > 1e-6:
            a /= norm
            b /= norm
            c /= norm

        return (a, b, c)

    def _fit_line_lstsq(self, points: np.ndarray) -> Tuple[float, float, float]:
        """Fit line to points using least squares."""
        # Use PCA to find principal direction
        mean = np.mean(points, axis=0)
        centered = points - mean

        U, S, Vt = np.linalg.svd(centered, full_matrices=False)
        direction = Vt[0]  # Principal component

        # Line normal is perpendicular to direction
        a = -direction[1]
        b = direction[0]
        c = -(a * mean[0] + b * mean[1])

        # Normalize
        norm = np.sqrt(a * a + b * b)
        if norm > 1e-6:
            a /= norm
            b /= norm
            c /= norm

        return (a, b, c)

    def _point_to_line_distance(
        self, points: np.ndarray, line: Tuple[float, float, float]
    ) -> np.ndarray:
        """Compute distance from points to line."""
        a, b, c = line
        return np.abs(a * points[:, 0] + b * points[:, 1] + c)

    def _check_orthogonality(self, lines: List[Tuple[float, float, float]]) -> bool:
        """Check if lines form near-orthogonal pairs."""
        if len(lines) < 4:
            return False

        # Compute angles of line normals
        angles = []
        for a, b, c in lines:
            angle = np.arctan2(b, a)
            angles.append(angle)

        # Check for two dominant orthogonal directions
        angles = np.array(angles)
        angles = wrap_angle(angles)

        # Simple check: are there pairs ~90° apart?
        tol = np.deg2rad(ALG.RECT_ORTHTOL_DEG)

        for i in range(len(angles)):
            for j in range(i + 1, len(angles)):
                diff = abs(wrap_angle(angles[i] - angles[j]))
                if abs(diff - np.pi / 2) < tol or abs(diff + np.pi / 2) < tol:
                    return True

        return False

    def _compute_rectangle(
        self, lines: List[Tuple[float, float, float]]
    ) -> Tuple[float, float, float, float, float]:
        """
        Compute rectangle parameters from lines.

        Returns:
            (center_x, center_y, heading, width, height)
        """
        # Simplified: use bounding box of edge points
        points = np.array(self.edge_points)

        # Compute center
        center_x = np.mean(points[:, 0])
        center_y = np.mean(points[:, 1])

        # Compute heading (aligned with longest edge)
        # Use PCA
        centered = points - np.array([center_x, center_y])
        U, S, Vt = np.linalg.svd(centered, full_matrices=False)
        heading = np.arctan2(Vt[0, 1], Vt[0, 0])

        # Rotate points to aligned frame
        c, s = np.cos(-heading), np.sin(-heading)
        R = np.array([[c, -s], [s, c]])
        rotated = centered @ R.T

        # Compute bounding box
        min_x, max_x = np.min(rotated[:, 0]), np.max(rotated[:, 0])
        min_y, max_y = np.min(rotated[:, 1]), np.max(rotated[:, 1])

        width = max_x - min_x
        height = max_y - min_y

        return (center_x, center_y, heading, width, height)

    def _estimate_perimeter_coverage(self) -> float:
        """Estimate total perimeter length observed."""
        if not self.rectangle:
            return 0.0

        # Simplified: just use number of points × spacing
        return len(self.edge_points) * 0.02  # Assume ~2cm spacing

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
