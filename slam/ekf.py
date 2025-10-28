"""
Extended Kalman Filter for SE(2) state estimation.

State: [x, y, θ, b_g] where b_g is optional gyro bias.
Inputs: wheel odometry (dSL, dSR) and gyro yaw rate.
"""

import numpy as np
from .frames import wrap_angle
from ..config import GEOM


class EKF:
    """Extended Kalman Filter for robot pose estimation."""

    def __init__(
        self,
        x0: float = 0.0,
        y0: float = 0.0,
        theta0: float = 0.0,
        estimate_bias: bool = False,
    ):
        """
        Initialize EKF.

        Args:
            x0: Initial x position (m)
            y0: Initial y position (m)
            theta0: Initial heading (rad)
            estimate_bias: Whether to estimate gyro bias
        """
        self.estimate_bias = estimate_bias

        # State vector
        if estimate_bias:
            self.x = np.array([x0, y0, theta0, 0.0])  # [x, y, θ, b_g]
            self.P = np.diag([0.01, 0.01, 0.01, 0.1])
        else:
            self.x = np.array([x0, y0, theta0])  # [x, y, θ]
            self.P = np.diag([0.01, 0.01, 0.01])

        # Process noise
        if estimate_bias:
            self.Q = np.diag([0.001, 0.001, 0.001, 0.0001])
        else:
            self.Q = np.diag([0.001, 0.001, 0.001])

        # Measurement noise
        self.R_gyro = 0.01  # Gyro yaw rate variance
        self.R_yaw = 0.1  # Absolute yaw variance (low weight)

    def predict(self, dSL: float, dSR: float, dt: float):
        """
        Prediction step using wheel odometry.

        Args:
            dSL: Left wheel displacement (m)
            dSR: Right wheel displacement (m)
            dt: Time step (s)
        """
        # Extract current state
        x, y, theta = self.x[0], self.x[1], self.x[2]

        # Compute motion
        ds = 0.5 * (dSR + dSL)
        dtheta = (dSR - dSL) / GEOM.WHEEL_BASE

        # Update state (with midpoint integration)
        theta_mid = theta + 0.5 * dtheta
        self.x[0] = x + ds * np.cos(theta_mid)
        self.x[1] = y + ds * np.sin(theta_mid)
        self.x[2] = wrap_angle(theta + dtheta)

        # Jacobian of motion model
        G = np.eye(len(self.x))
        G[0, 2] = -ds * np.sin(theta_mid)
        G[1, 2] = ds * np.cos(theta_mid)

        # Update covariance
        self.P = G @ self.P @ G.T + self.Q

    def update_gyro(self, omega_z: float, dt: float):
        """
        Update with gyro measurement.

        Args:
            omega_z: Measured yaw rate (rad/s)
            dt: Time step (s)
        """
        if self.estimate_bias:
            # Predicted yaw rate = (dθ/dt) + bias
            theta_dot_pred = 0.0  # We don't predict angular velocity
            bias = self.x[3]

            # Innovation
            z = omega_z
            h = theta_dot_pred + bias
            y = z - h

            # Jacobian
            H = np.zeros(len(self.x))
            H[3] = 1.0

            # Kalman gain
            S = H @ self.P @ H.T + self.R_gyro
            K = self.P @ H.T / S

            # Update
            self.x += K * y
            self.P = (np.eye(len(self.x)) - np.outer(K, H)) @ self.P

        # Integrate gyro for heading (complementary)
        # This is done in the prediction step, so no update needed here

    def update_yaw_abs(self, yaw: float):
        """
        Update with absolute yaw measurement (e.g., from magnetometer).

        Args:
            yaw: Measured absolute yaw (rad)
        """
        # Innovation
        z = yaw
        h = self.x[2]
        y = wrap_angle(z - h)

        # Jacobian
        H = np.zeros(len(self.x))
        H[2] = 1.0

        # Kalman gain
        S = H @ self.P @ H.T + self.R_yaw
        K = self.P @ H.T / S

        # Update
        self.x += K * y
        self.x[2] = wrap_angle(self.x[2])
        self.P = (np.eye(len(self.x)) - np.outer(K, H)) @ self.P

    def pose(self) -> tuple[float, float, float]:
        """
        Get current pose estimate.

        Returns:
            (x, y, θ) in meters and radians
        """
        return (self.x[0], self.x[1], self.x[2])

    def cov(self) -> np.ndarray:
        """
        Get current covariance matrix.

        Returns:
            Covariance matrix
        """
        return self.P.copy()

    def set_pose(self, x: float, y: float, theta: float):
        """
        Set pose estimate (e.g., after loop closure).

        Args:
            x: X position (m)
            y: Y position (m)
            theta: Heading (rad)
        """
        self.x[0] = x
        self.x[1] = y
        self.x[2] = wrap_angle(theta)
