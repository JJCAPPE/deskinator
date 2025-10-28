"""
Coordinate frame utilities for SE(2).

Provides transformations and angle wrapping for 2D poses.
"""

import numpy as np


def wrap_angle(theta: float) -> float:
    """
    Wrap angle to [-π, π].

    Args:
        theta: Angle in radians

    Returns:
        Wrapped angle in [-π, π]
    """
    return (theta + np.pi) % (2 * np.pi) - np.pi


def pose_compose(
    p1: tuple[float, float, float], p2: tuple[float, float, float]
) -> tuple[float, float, float]:
    """
    Compose two SE(2) poses: result = p1 ⊕ p2.

    Args:
        p1: First pose (x, y, θ)
        p2: Second pose (x, y, θ)

    Returns:
        Composed pose (x, y, θ)
    """
    x1, y1, th1 = p1
    x2, y2, th2 = p2

    c1, s1 = np.cos(th1), np.sin(th1)

    x = x1 + c1 * x2 - s1 * y2
    y = y1 + s1 * x2 + c1 * y2
    th = wrap_angle(th1 + th2)

    return (x, y, th)


def pose_inverse(p: tuple[float, float, float]) -> tuple[float, float, float]:
    """
    Compute inverse of SE(2) pose.

    Args:
        p: Pose (x, y, θ)

    Returns:
        Inverse pose (x, y, θ)
    """
    x, y, th = p
    c, s = np.cos(th), np.sin(th)

    x_inv = -(c * x + s * y)
    y_inv = -(-s * x + c * y)
    th_inv = wrap_angle(-th)

    return (x_inv, y_inv, th_inv)


def pose_difference(
    p1: tuple[float, float, float], p2: tuple[float, float, float]
) -> tuple[float, float, float]:
    """
    Compute relative pose: result = p1⁻¹ ⊕ p2.

    Args:
        p1: First pose (x, y, θ)
        p2: Second pose (x, y, θ)

    Returns:
        Relative pose (x, y, θ)
    """
    return pose_compose(pose_inverse(p1), p2)


def transform_point(
    pose: tuple[float, float, float], point: tuple[float, float]
) -> tuple[float, float]:
    """
    Transform a point from robot frame to world frame.

    Args:
        pose: Robot pose in world frame (x, y, θ)
        point: Point in robot frame (x, y)

    Returns:
        Point in world frame (x, y)
    """
    x, y, th = pose
    px, py = point

    c, s = np.cos(th), np.sin(th)

    x_world = x + c * px - s * py
    y_world = y + s * px + c * py

    return (x_world, y_world)


def transform_point_inverse(
    pose: tuple[float, float, float], point: tuple[float, float]
) -> tuple[float, float]:
    """
    Transform a point from world frame to robot frame.

    Args:
        pose: Robot pose in world frame (x, y, θ)
        point: Point in world frame (x, y)

    Returns:
        Point in robot frame (x, y)
    """
    x, y, th = pose
    px, py = point

    c, s = np.cos(th), np.sin(th)

    dx = px - x
    dy = py - y

    x_robot = c * dx + s * dy
    y_robot = -s * dx + c * dy

    return (x_robot, y_robot)


def rotation_matrix(theta: float) -> np.ndarray:
    """
    Create 2D rotation matrix.

    Args:
        theta: Rotation angle in radians

    Returns:
        2x2 rotation matrix
    """
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def pose_to_matrix(pose: tuple[float, float, float]) -> np.ndarray:
    """
    Convert SE(2) pose to 3x3 homogeneous transformation matrix.

    Args:
        pose: Pose (x, y, θ)

    Returns:
        3x3 transformation matrix
    """
    x, y, th = pose
    c, s = np.cos(th), np.sin(th)

    T = np.array([[c, -s, x], [s, c, y], [0, 0, 1]])
    return T


def matrix_to_pose(T: np.ndarray) -> tuple[float, float, float]:
    """
    Convert 3x3 homogeneous transformation matrix to SE(2) pose.

    Args:
        T: 3x3 transformation matrix

    Returns:
        Pose (x, y, θ)
    """
    x = T[0, 2]
    y = T[1, 2]
    th = np.arctan2(T[1, 0], T[0, 0])

    return (x, y, th)
