"""
SE(2) coordinate transforms and utilities.

Minimal implementation for visualization demo.
"""

import numpy as np
from typing import Tuple


def rotation_matrix(theta: float) -> np.ndarray:
    """
    Create 2D rotation matrix.
    
    Args:
        theta: Rotation angle (radians)
        
    Returns:
        2x2 rotation matrix
    """
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s], [s, c]])


def wrap_angle(theta: float) -> float:
    """
    Wrap angle to [-pi, pi].
    
    Args:
        theta: Angle (radians)
        
    Returns:
        Wrapped angle
    """
    return ((theta + np.pi) % (2 * np.pi)) - np.pi


def transform_point(pose: Tuple[float, float, float], point: Tuple[float, float]) -> Tuple[float, float]:
    """
    Transform a point from robot frame to world frame.
    
    Args:
        pose: Robot pose (x, y, theta)
        point: Point in robot frame (x, y)
        
    Returns:
        Point in world frame (x, y)
    """
    x, y, theta = pose
    px, py = point
    
    R = rotation_matrix(theta)
    world = np.array([x, y]) + R @ np.array([px, py])
    
    return (world[0], world[1])


def pose_difference(p1: Tuple[float, float, float], p2: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """
    Compute relative pose from p1 to p2.
    
    Args:
        p1: First pose (x, y, theta)
        p2: Second pose (x, y, theta)
        
    Returns:
        Relative pose (dx, dy, dtheta)
    """
    x1, y1, t1 = p1
    x2, y2, t2 = p2
    
    R1_inv = rotation_matrix(-t1)
    rel_pos = R1_inv @ np.array([x2 - x1, y2 - y1])
    
    dtheta = wrap_angle(t2 - t1)
    
    return (rel_pos[0], rel_pos[1], dtheta)

