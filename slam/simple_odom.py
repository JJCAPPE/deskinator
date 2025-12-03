"""
Simple odometry-based pose tracker (dead reckoning).

Integrates wheel odometry using differential drive kinematics without filtering.
Matches the approach used in viz_demo.py's SimulatedRobot.
"""

import numpy as np

try:
    from ..config import GEOM
except ImportError:
    from config import GEOM


class SimpleOdometry:
    """Dead reckoning pose tracker using only wheel odometry.
    
    This class provides the same interface as EKF but without sensor fusion.
    It's a direct port of the kinematics used in viz_demo.py's SimulatedRobot.
    """

    def __init__(self, initial_pose=(0.0, 0.0, 0.0)):
        """Initialize simple odometry tracker.
        
        Args:
            initial_pose: Starting pose (x, y, theta) in meters and radians
        """
        self.x = float(initial_pose[0])
        self.y = float(initial_pose[1])
        self.theta = float(initial_pose[2])

    def predict(self, dSL: float, dSR: float, dt: float = 0.02):
        """Update pose from wheel distances using differential drive kinematics.
        
        This implements the same unicycle model as SimulatedRobot.update() in viz_demo.py.
        
        Args:
            dSL: Distance traveled by left wheel (meters)
            dSR: Distance traveled by right wheel (meters)
            dt: Time step (seconds) - not used in odometry but kept for API compatibility
        """
        # Differential drive kinematics
        # Forward distance (average of both wheels)
        ds = (dSL + dSR) / 2.0
        
        # Change in heading (arc length difference / wheelbase)
        dtheta = (dSR - dSL) / GEOM.WHEEL_BASE
        
        # Update heading first
        theta_mid = self.theta + dtheta / 2.0  # Midpoint integration for better accuracy
        
        # Update position using midpoint heading
        self.x += ds * np.cos(theta_mid)
        self.y += ds * np.sin(theta_mid)
        self.theta += dtheta
        
        # Normalize theta to [-pi, pi]
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

    def pose(self) -> tuple:
        """Return current pose estimate.
        
        Returns:
            (x, y, theta) tuple in meters and radians
        """
        return (self.x, self.y, self.theta)

    def set_pose(self, x: float, y: float, theta: float):
        """Override current pose estimate.
        
        Args:
            x: X position (meters)
            y: Y position (meters)
            theta: Heading angle (radians)
        """
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)

    def cov(self):
        """Return covariance matrix (dummy for API compatibility).
        
        Returns:
            3x3 zero matrix (no uncertainty tracking in simple odometry)
        """
        return np.zeros((3, 3))
