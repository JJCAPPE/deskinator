"""
Robust Wall Following Behavior.
Uses "bump & turn" logic to define the table boundary.
"""

import numpy as np
import math
from enum import Enum, auto
from typing import Tuple, Optional, List
from ..config import ALG, LIMS, GEOM, I2C

class WallState(Enum):
    FIND_WALL = auto()      # Driving forward to find first edge
    BACKOFF = auto()        # Backing up after edge detect
    TURN_ALIGN = auto()     # Turning to align with wall
    FOLLOW = auto()         # Driving along wall
    TURN_CORNER = auto()    # Turning 90 degrees at corner (or missed edge)
    DONE = auto()           # Completed lap

class WallFollower:
    def __init__(self):
        self.state = WallState.FIND_WALL
        self.start_pose = None
        self.corner_poses = []
        
        # Action state tracking
        self.action_start_pose = None
        self.action_start_heading = 0.0
        self.target_val = 0.0
        
        # Follow logic
        self.follow_direction = -1 # -1 for Left (CCW), 1 for Right (CW)
        self.last_edge_time = 0.0
        self.dist_since_edge = 0.0
        
        # Lap closure
        self.lap_start_pose = None
        self.lap_dist = 0.0
        
    def update(self, ekf_pose: Tuple[float, float, float], sensors: List[int], dt: float) -> Tuple[float, float]:
        """
        Update wall follower state machine.
        
        Args:
            ekf_pose: (x, y, theta)
            sensors: [left_raw, right_raw]
            dt: time step
            
        Returns:
            (v, omega) command
        """
        x, y, theta = ekf_pose
        
        # Sensor Check (True if OFF TABLE)
        left_trig = sensors[I2C.LEFT_SENSOR_IDX] < ALG.EDGE_RAW_THRESH
        right_trig = sensors[I2C.RIGHT_SENSOR_IDX] < ALG.EDGE_RAW_THRESH
        any_trig = left_trig or right_trig
        
        if self.state == WallState.FIND_WALL:
            # Drive forward until edge
            if any_trig:
                self.state = WallState.BACKOFF
                self.action_start_pose = ekf_pose
                self.target_val = ALG.POST_EDGE_BACKOFF
                self.lap_start_pose = ekf_pose # Approximately start of lap
                self.corner_poses.append(ekf_pose)
                return (0.0, 0.0)
            return (LIMS.V_BASE * 0.5, 0.0)
            
        elif self.state == WallState.BACKOFF:
            # Back up fixed distance
            dist = np.hypot(x - self.action_start_pose[0], y - self.action_start_pose[1])
            if dist >= self.target_val:
                # Transition to TURN
                self.state = WallState.TURN_ALIGN
                self.action_start_heading = theta
                
                # Heuristic: Short run = Correction, Long run = Corner
                if self.last_dist < 0.4:
                    # Correction: Turn 15 degrees away from edge
                    # If we are following LEFT wall (CCW), edge is on Right? 
                    # No, "Find Wall" -> Turn Right 90 -> Wall is on Left.
                    # If we hit edge, we drifted Left. Need to Turn Right (Negative).
                    self.target_val = -np.deg2rad(15)
                else:
                    # Corner: Turn 90 degrees Right
                    self.target_val = -np.pi / 2.0
                    self.corner_poses.append(ekf_pose)
                
                return (0.0, 0.0)
                
            return (-LIMS.V_REV_MAX, 0.0)
            
        elif self.state == WallState.TURN_ALIGN:
            # Turn to target angle
            diff = theta - self.action_start_heading
            # Unwrap
            diff = (diff + np.pi) % (2 * np.pi) - np.pi
            
            remaining = self.target_val - diff
            
            if abs(remaining) < 0.1:
                self.state = WallState.FOLLOW
                self.action_start_pose = ekf_pose
                return (0.0, 0.0)
                
            omega = np.sign(remaining) * LIMS.OMEGA_MAX * 0.5
            return (0.0, omega)
            
        elif self.state == WallState.FOLLOW:
            # Drive forward
            if any_trig:
                # Hit edge
                self.last_dist = np.hypot(x - self.action_start_pose[0], y - self.action_start_pose[1])
                
                self.state = WallState.BACKOFF
                self.action_start_pose = ekf_pose
                self.target_val = ALG.POST_EDGE_BACKOFF
                return (0.0, 0.0)
            
            # Check for Lap Closure
            if self.check_lap_closure(ekf_pose):
                self.state = WallState.DONE
                return (0.0, 0.0)
            
            return (LIMS.V_BASE * 0.8, 0.0)

        return (0.0, 0.0)

    def check_lap_closure(self, pose):
        if self.lap_start_pose is None:
            return False
        
        # Don't trigger immediately at start
        dx = pose[0] - self.lap_start_pose[0]
        dy = pose[1] - self.lap_start_pose[1]
        dist_from_start = np.hypot(dx, dy)
        
        # Estimate total travel
        # Simple metric: elapsed time * speed? Or accumulation?
        # We don't track accumulation here easily without integrating v.
        # Use corner count or just "Time passed"
        
        # If we have found 4 corners, it's a rectangle.
        # If we have found 0 corners (circle), we rely on distance?
        
        is_near_start = dist_from_start < 0.3
        has_corners = len(self.corner_poses) >= 4
        
        # Fallback: If we have many small corrections (circle), corner_poses might be empty.
        # But we assume "Rectangular Desk" task per user prompt ("shape of the table" usually implies finding the bounds).
        
        if is_near_start and has_corners:
            return True
            
        return False

