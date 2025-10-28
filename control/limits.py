"""
Motion limits and trajectory generation.

Implements acceleration and jerk limits for smooth motion.
"""

import numpy as np
from ..config import LIMS


class VelocityLimiter:
    """Applies acceleration and jerk limits to velocity commands."""

    def __init__(self):
        """Initialize velocity limiter."""
        self.v_current = 0.0
        self.omega_current = 0.0
        self.a_current = 0.0
        self.alpha_current = 0.0

    def limit(
        self, v_target: float, omega_target: float, dt: float
    ) -> tuple[float, float]:
        """
        Apply limits and return achievable velocities.

        Args:
            v_target: Target linear velocity (m/s)
            omega_target: Target angular velocity (rad/s)
            dt: Time step (s)

        Returns:
            (v_limited, omega_limited)
        """
        # Linear velocity
        dv = v_target - self.v_current

        # Compute desired acceleration
        a_desired = dv / dt if dt > 0 else 0.0

        # Apply jerk limit to acceleration
        da = a_desired - self.a_current
        da_max = LIMS.J_MAX * dt
        if abs(da) > da_max:
            da = np.sign(da) * da_max

        self.a_current += da

        # Apply acceleration limit
        if abs(self.a_current) > LIMS.A_MAX:
            self.a_current = np.sign(self.a_current) * LIMS.A_MAX

        # Update velocity
        self.v_current += self.a_current * dt

        # Apply velocity limit
        if abs(self.v_current) > LIMS.V_MAX:
            self.v_current = np.sign(self.v_current) * LIMS.V_MAX

        # Angular velocity (simplified, similar process)
        domega = omega_target - self.omega_current

        # Compute desired angular acceleration
        alpha_desired = domega / dt if dt > 0 else 0.0

        # Apply jerk limit (simplified)
        dalpha = alpha_desired - self.alpha_current
        dalpha_max = LIMS.J_MAX * dt  # Simplified
        if abs(dalpha) > dalpha_max:
            dalpha = np.sign(dalpha) * dalpha_max

        self.alpha_current += dalpha

        # Apply angular acceleration limit
        if abs(self.alpha_current) > LIMS.ALPHA_MAX:
            self.alpha_current = np.sign(self.alpha_current) * LIMS.ALPHA_MAX

        # Update angular velocity
        self.omega_current += self.alpha_current * dt

        # Apply angular velocity limit
        if abs(self.omega_current) > LIMS.OMEGA_MAX:
            self.omega_current = np.sign(self.omega_current) * LIMS.OMEGA_MAX

        return (self.v_current, self.omega_current)

    def stop(self, dt: float) -> tuple[float, float]:
        """
        Emergency stop with maximum deceleration.

        Args:
            dt: Time step (s)

        Returns:
            (v, omega) after braking
        """
        # Apply maximum braking
        if self.v_current > 0:
            self.v_current = max(0.0, self.v_current - LIMS.A_MAX * dt)
        elif self.v_current < 0:
            self.v_current = min(0.0, self.v_current + LIMS.A_MAX * dt)

        if self.omega_current > 0:
            self.omega_current = max(0.0, self.omega_current - LIMS.ALPHA_MAX * dt)
        elif self.omega_current < 0:
            self.omega_current = min(0.0, self.omega_current + LIMS.ALPHA_MAX * dt)

        self.a_current = 0.0
        self.alpha_current = 0.0

        return (self.v_current, self.omega_current)

    def reset(self):
        """Reset to zero velocities."""
        self.v_current = 0.0
        self.omega_current = 0.0
        self.a_current = 0.0
        self.alpha_current = 0.0


def compute_stopping_distance(v: float) -> float:
    """
    Compute stopping distance for given velocity.

    Args:
        v: Current velocity (m/s)

    Returns:
        Stopping distance (m)
    """
    if abs(v) < 1e-6:
        return 0.0
    return v * v / (2 * LIMS.A_MAX)
