"""
Analysis utilities for robot performance metrics.

Provides functions to calculate various performance metrics
for the robot cleaning simulation.
"""

import numpy as np
from typing import List, Tuple, Optional
try:
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    HAS_SHAPELY = True
except ImportError:
    HAS_SHAPELY = False


def calculate_rectangle_error(
    fitted_rect: Optional[Tuple[float, float, float, float, float]],
    ground_truth: Tuple[float, float, float, float, float],
) -> float:
    """
    Calculate the error between fitted rectangle and ground truth.
    
    Uses a fine-grained error metric that combines:
    1. Area overlap error (symmetric difference)
    2. Position error (center offset)
    3. Rotation error (heading difference)
    4. Size error (width/height differences)
    
    Returns error in parts per million (ppm) for finer precision.
    For very small errors, this provides much better resolution than percentage.
    
    Args:
        fitted_rect: (cx, cy, heading, width, height) or None if fitting failed
        ground_truth: (cx, cy, heading, width, height) ground truth rectangle
    
    Returns:
        Error in parts per million (ppm). Returns 1e6 (1,000,000) if fitting failed.
        Perfect match = 0 ppm, 0.01% error = 100 ppm, 0.1% error = 1000 ppm, etc.
    """
    if fitted_rect is None:
        return 1e6  # 1,000,000 ppm = 100% error
    
    if not HAS_SHAPELY:
        print("Warning: shapely not installed, cannot calculate rectangle error")
        return 1e6  # 1,000,000 ppm = 100% error
    
    try:
        gt_cx, gt_cy, gt_heading, gt_width, gt_height = ground_truth
        fit_cx, fit_cy, fit_heading, fit_width, fit_height = fitted_rect
        
        # Validate inputs
        if gt_width <= 0 or gt_height <= 0:
            return 1e6
        if fit_width <= 0 or fit_height <= 0:
            return 1e6
        
        # Create polygons for both rectangles
        def rect_to_polygon(cx, cy, heading, width, height):
            """Convert rectangle to Shapely polygon."""
            # Rotation matrix
            c, s = np.cos(heading), np.sin(heading)
            R = np.array([[c, -s], [s, c]])
            
            # Corners in local frame
            corners_local = np.array([
                [-width / 2, -height / 2],
                [width / 2, -height / 2],
                [width / 2, height / 2],
                [-width / 2, height / 2],
            ])
            
            # Transform to world frame
            corners_world = np.array([cx, cy]) + corners_local @ R.T
            
            return Polygon(corners_world)
        
        gt_poly = rect_to_polygon(*ground_truth)
        fitted_poly = rect_to_polygon(*fitted_rect)
        gt_area = gt_poly.area
        
        if gt_area <= 0:
            return 1e6
        
        # 1. Area overlap error (symmetric difference) - primary metric
        symmetric_diff = gt_poly.symmetric_difference(fitted_poly)
        area_error = symmetric_diff.area
        area_error_ppm = (area_error / gt_area) * 1e6  # Convert to ppm
        
        # 2. Position error (center offset normalized by diagonal)
        # Diagonal length is a good normalization factor
        gt_diagonal = np.sqrt(gt_width**2 + gt_height**2)
        center_offset = np.sqrt((fit_cx - gt_cx)**2 + (fit_cy - gt_cy)**2)
        position_error_ppm = (center_offset / gt_diagonal) * 1e6 if gt_diagonal > 0 else 0
        
        # 3. Rotation error (heading difference normalized)
        # Normalize angle difference to [0, π/2] range
        angle_diff = abs(fit_heading - gt_heading)
        angle_diff = min(angle_diff, 2 * np.pi - angle_diff)  # Wrap to [0, π]
        angle_diff = min(angle_diff, np.pi - angle_diff)  # Wrap to [0, π/2]
        # Convert to ppm: π/2 radians = 90° = maximum error
        rotation_error_ppm = (angle_diff / (np.pi / 2)) * 1e6
        
        # 4. Size error (width/height differences normalized)
        width_error = abs(fit_width - gt_width) / gt_width
        height_error = abs(fit_height - gt_height) / gt_height
        size_error_ppm = ((width_error + height_error) / 2) * 1e6
        
        # Combine errors with weighted average
        # Area error is most important (weight 0.5), others contribute equally (0.167 each)
        combined_error_ppm = (
            0.5 * area_error_ppm +
            0.167 * position_error_ppm +
            0.167 * rotation_error_ppm +
            0.167 * size_error_ppm
        )
        
        return combined_error_ppm
            
    except Exception as e:
        # Catch any exceptions during calculation and return error indicator
        print(f"Warning: Error calculating rectangle error: {e}")
        return 1e6  # 1,000,000 ppm = 100% error


def calculate_trajectory_distance(trajectory: List[Tuple[float, float, float]]) -> float:
    """
    Calculate total distance traveled along trajectory.
    
    Args:
        trajectory: List of (x, y, theta) poses
    
    Returns:
        Total distance in meters
    """
    if len(trajectory) < 2:
        return 0.0
    
    total_distance = 0.0
    for i in range(1, len(trajectory)):
        x1, y1, _ = trajectory[i - 1]
        x2, y2, _ = trajectory[i]
        
        dx = x2 - x1
        dy = y2 - y1
        dist = np.sqrt(dx * dx + dy * dy)
        total_distance += dist
    
    return total_distance


def calculate_average_metrics(
    results: List[dict],
    key: str
) -> Tuple[float, float]:
    """
    Calculate mean and standard deviation for a metric across trials.
    
    Args:
        results: List of trial result dictionaries
        key: Key of the metric to analyze
    
    Returns:
        (mean, std_dev) tuple
    """
    values = [r[key] for r in results if key in r]
    if not values:
        return 0.0, 0.0
    
    mean = np.mean(values)
    std = np.std(values)
    return mean, std
