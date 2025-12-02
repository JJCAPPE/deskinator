"""
Analysis utilities for robot performance metrics.

Provides functions to calculate various performance metrics
for the robot cleaning simulation.
"""

import numpy as np
from typing import List, Tuple, Optional
from shapely.geometry import Polygon
from shapely.ops import unary_union


def calculate_rectangle_error(
    fitted_rect: Optional[Tuple[float, float, float, float, float]],
    ground_truth: Tuple[float, float, float, float, float],
) -> float:
    """
    Calculate the error between fitted rectangle and ground truth.
    
    Computes the symmetric difference (XOR) area between the two rectangles
    and returns it as a percentage of the ground truth area.
    
    Args:
        fitted_rect: (cx, cy, heading, width, height) or None if fitting failed
        ground_truth: (cx, cy, heading, width, height) ground truth rectangle
    
    Returns:
        Error as percentage (0-100+). Returns 100.0 if fitting failed.
    """
    if fitted_rect is None:
        return 100.0
    
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
    
    # Calculate symmetric difference (XOR)
    symmetric_diff = gt_poly.symmetric_difference(fitted_poly)
    error_area = symmetric_diff.area
    gt_area = gt_poly.area
    
    # Return as percentage of ground truth area
    if gt_area > 0:
        return (error_area / gt_area) * 100.0
    else:
        return 0.0


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
