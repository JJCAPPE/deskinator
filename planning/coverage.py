"""
Coverage planning and wall-following algorithm.

Implements a simple wall-following algorithm for boundary discovery
and boustrophedon path planning for coverage.
"""

import numpy as np
from enum import Enum
from typing import List, Tuple, Optional, Callable
from dataclasses import dataclass

try:
    from ..config import GEOM, ALG
except ImportError:
    from config import GEOM, ALG


class WallFollowState(Enum):
    """States for wall-following algorithm."""
    FIND_WALL = "find_wall"
    TURN_AWAY = "turn_away"
    FORWARD_UNTIL_OFF = "forward_until_off"
    TURN_BACK = "turn_back"
    FORWARD_ALONG_WALL = "forward_along_wall"
    COMPLETE = "complete"


@dataclass
class EdgePoint:
    """Edge detection point."""
    x: float
    y: float
    timestamp: float


class SimpleWallFollower:
    """
    Simple wall-following algorithm for boundary discovery.
    
    Algorithm:
    1. Go straight until finding a wall
    2. If right sensor goes off, turn left until sensor is back + 2°
    3. Go forwards until sensor goes off
    4. Turn in the same direction as the first time until sensor is back on
    5. Go forwards
    6. Repeat until table is mapped out
    """
    
    def __init__(self, forward_speed: float = 0.1, turn_speed: float = 0.5):
        """
        Initialize wall follower.
        
        Args:
            forward_speed: Forward velocity (m/s)
            turn_speed: Angular velocity for turning (rad/s)
        """
        self.state = WallFollowState.FIND_WALL
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        
        # Track edge points
        self.edge_points: List[EdgePoint] = []
        
        # State tracking
        self.turn_direction = 0.0  # +1 for left, -1 for right
        self.start_pose: Optional[Tuple[float, float, float]] = None
        self.lap_complete = False
        
        # Parameters
        self.sensor_back_threshold = 2.0  # degrees
        self.lap_close_distance = 0.1  # m - distance to start to consider lap complete
        
    def update(
        self,
        pose: Tuple[float, float, float],
        left_sensor_on: bool,
        right_sensor_on: bool,
        dt: float,
    ) -> Tuple[float, float]:
        """
        Update wall-following algorithm.
        
        Args:
            pose: Current robot pose (x, y, theta)
            left_sensor_on: True if left sensor detects table (not off edge)
            right_sensor_on: True if right sensor detects table (not off edge)
            dt: Time step (s)
            
        Returns:
            (v, omega) velocity commands
        """
        x, y, theta = pose
        
        # Initialize start pose on first call
        if self.start_pose is None:
            self.start_pose = pose
        
        # Check if we've completed a lap
        if not self.lap_complete and self.start_pose:
            dx = x - self.start_pose[0]
            dy = y - self.start_pose[1]
            dist = np.sqrt(dx*dx + dy*dy)
            if dist < self.lap_close_distance and len(self.edge_points) > 10:
                # Check if we're heading in roughly the same direction
                dtheta = abs(theta - self.start_pose[2])
                dtheta = min(dtheta, 2*np.pi - dtheta)
                if dtheta < np.deg2rad(45):
                    self.lap_complete = True
                    self.state = WallFollowState.COMPLETE
        
        if self.lap_complete:
            return 0.0, 0.0
        
        # State machine
        if self.state == WallFollowState.FIND_WALL:
            # Go straight until we hit a wall (sensor goes off)
            if not right_sensor_on or not left_sensor_on:
                # Hit a wall - determine which sensor
                if not right_sensor_on:
                    self.turn_direction = 1.0  # Turn left
                    self._add_edge_point(pose, "right")
                elif not left_sensor_on:
                    self.turn_direction = -1.0  # Turn right
                    self._add_edge_point(pose, "left")
                # Start turning away
                self.sensor_back_theta = None
                self.state = WallFollowState.TURN_AWAY
            return self.forward_speed, 0.0
            
        elif self.state == WallFollowState.TURN_AWAY:
            # Turn away from wall until sensor is back + 2°
            # Check if sensor is back on
            sensor_back = False
            if self.turn_direction > 0:  # Turning left (away from right wall)
                sensor_back = right_sensor_on
            else:  # Turning right (away from left wall)
                sensor_back = left_sensor_on
            
            if sensor_back:
                if self.sensor_back_theta is None:
                    # Just got sensor back - record angle and continue turning
                    self.sensor_back_theta = theta
                else:
                    # Continue turning until we've turned 2° past sensor back point
                    dtheta = theta - self.sensor_back_theta
                    dtheta = ((dtheta + np.pi) % (2*np.pi)) - np.pi  # Wrap to [-pi, pi]
                    
                    if abs(dtheta) >= np.deg2rad(self.sensor_back_threshold):
                        # Reached target angle, now go forward
                        self.state = WallFollowState.FORWARD_UNTIL_OFF
            else:
                # Sensor not back yet, keep turning
                pass
            
            return 0.0, self.turn_speed * self.turn_direction
            
        elif self.state == WallFollowState.FORWARD_UNTIL_OFF:
            # Go forward until sensor goes off (hitting next corner/edge)
            if not right_sensor_on or not left_sensor_on:
                if not right_sensor_on:
                    self._add_edge_point(pose, "right")
                if not left_sensor_on:
                    self._add_edge_point(pose, "left")
                # Turn in same direction as before
                self.state = WallFollowState.TURN_BACK
            return self.forward_speed, 0.0
            
        elif self.state == WallFollowState.TURN_BACK:
            # Turn in same direction until sensor is back on
            sensor_back = False
            if self.turn_direction > 0:  # Turning left
                sensor_back = right_sensor_on
            else:  # Turning right
                sensor_back = left_sensor_on
            
            if sensor_back:
                self.state = WallFollowState.FORWARD_ALONG_WALL
            return 0.0, self.turn_speed * self.turn_direction
            
        elif self.state == WallFollowState.FORWARD_ALONG_WALL:
            # Go forward along wall
            # Check if we hit an edge
            if not right_sensor_on or not left_sensor_on:
                if not right_sensor_on:
                    self._add_edge_point(pose, "right")
                    self.turn_direction = 1.0  # Turn left
                elif not left_sensor_on:
                    self._add_edge_point(pose, "left")
                    self.turn_direction = -1.0  # Turn right
                # Go back to turn away state
                self.sensor_back_theta = None
                self.state = WallFollowState.TURN_AWAY
            return self.forward_speed, 0.0
        
        return 0.0, 0.0
    
    def _add_edge_point(self, pose: Tuple[float, float, float], side: str):
        """Add an edge point based on sensor position."""
        x, y, theta = pose
        
        # Sensor position in robot frame
        if side == "left":
            sensor_lat = GEOM.SENSOR_LAT[0] if len(GEOM.SENSOR_LAT) > 0 else 0.1
        else:
            sensor_lat = GEOM.SENSOR_LAT[-1] if len(GEOM.SENSOR_LAT) > 1 else -0.1
        
        # Transform to world frame
        sensor_x = x + GEOM.SENSOR_FWD * np.cos(theta) - sensor_lat * np.sin(theta)
        sensor_y = y + GEOM.SENSOR_FWD * np.sin(theta) + sensor_lat * np.cos(theta)
        
        import time
        self.edge_points.append(EdgePoint(sensor_x, sensor_y, time.time()))
    
    def get_edge_points(self) -> List[Tuple[float, float]]:
        """Get list of edge points as (x, y) tuples."""
        return [(p.x, p.y) for p in self.edge_points]
    
    def is_complete(self) -> bool:
        """Check if wall-following is complete."""
        return self.lap_complete


class SimpleRectangleFit:
    """
    Simple rectangle fitting from edge points.
    
    Uses a bounding box approach with rotation optimization.
    """
    
    def __init__(self):
        self.edge_points: List[Tuple[float, float]] = []
        self.rectangle: Optional[Tuple[float, float, float, float, float]] = None
        self.is_confident = False
        
    def add_edge_point(self, point: Tuple[float, float]):
        """Add an edge point."""
        self.edge_points.append(point)
        
    def fit(self):
        """Fit rectangle to edge points."""
        if len(self.edge_points) < 4:
            self.is_confident = False
            return
        
        # Convert to numpy array
        points = np.array(self.edge_points)
        
        # Try different rotation angles
        best_rect = None
        best_area = float('inf')
        
        # Test angles from 0 to 90 degrees in 5-degree increments
        for angle_deg in range(0, 91, 5):
            angle = np.deg2rad(angle_deg)
            c = np.cos(angle)
            s = np.sin(angle)
            R = np.array([[c, -s], [s, c]])
            
            # Rotate points
            rotated = points @ R.T
            
            # Compute bounding box
            min_x, min_y = rotated.min(axis=0)
            max_x, max_y = rotated.max(axis=0)
            
            width = max_x - min_x
            height = max_y - min_y
            area = width * height
            
            # Center in rotated frame
            center_rot = np.array([(min_x + max_x) / 2, (min_y + max_y) / 2])
            
            # Rotate center back to world frame
            center_world = center_rot @ R
            
            if area < best_area:
                best_area = area
                best_rect = (center_world[0], center_world[1], angle, width, height)
        
        if best_rect:
            self.rectangle = best_rect
            # Consider confident if we have enough points
            self.is_confident = len(self.edge_points) > 20
        
    def get_rectangle(self) -> Optional[Tuple[float, float, float, float, float]]:
        """
        Get fitted rectangle.
        
        Returns:
            (cx, cy, heading, width, height) or None
        """
        return self.rectangle


class CoveragePlanner:
    """
    Boustrophedon path planner for coverage.
    
    Generates alternating lanes inside a rectangle.
    """
    
    def __init__(self):
        self.rectangle: Optional[Tuple[float, float, float, float, float]] = None
        self.lanes: List[List[Tuple[float, float]]] = []
        self.current_lane_idx = 0
        self.current_waypoint_idx = 0
        
    def set_rectangle(self, rect: Tuple[float, float, float, float, float]):
        """
        Set the rectangle to plan coverage for.
        
        Args:
            rect: (cx, cy, heading, width, height)
        """
        self.rectangle = rect
        
    def build_lanes(self) -> List[List[Tuple[float, float]]]:
        """
        Build boustrophedon lanes.
        
        Returns:
            List of lanes, each lane is a list of waypoints
        """
        if not self.rectangle:
            return []
        
        cx, cy, heading, width, height = self.rectangle
        
        # Inset rectangle to account for robot size
        inset = ALG.RECT_INSET if hasattr(ALG, 'RECT_INSET') else 0.05
        lane_width = GEOM.VAC_WIDTH - ALG.SWEEP_OVERLAP if hasattr(ALG, 'SWEEP_OVERLAP') else GEOM.VAC_WIDTH - 0.02
        
        # Compute number of lanes
        inner_width = width - 2 * inset
        inner_height = height - 2 * inset
        
        # Determine lane direction (along longer dimension)
        if inner_width > inner_height:
            # Lanes along height (vertical)
            num_lanes = int(np.ceil(inner_width / lane_width))
            lane_spacing = inner_width / max(1, num_lanes)
            lane_length = inner_height
        else:
            # Lanes along width (horizontal)
            num_lanes = int(np.ceil(inner_height / lane_width))
            lane_spacing = inner_height / max(1, num_lanes)
            lane_length = inner_width
        
        # Rotation matrix
        c, s = np.cos(heading), np.sin(heading)
        R = np.array([[c, -s], [s, c]])
        
        lanes = []
        for i in range(num_lanes):
            # Compute lane endpoints in rectangle-local frame
            if inner_width > inner_height:
                # Vertical lanes
                x_local = -inner_width/2 + (i + 0.5) * lane_spacing
                start_local = np.array([x_local, -inner_height/2])
                end_local = np.array([x_local, inner_height/2])
            else:
                # Horizontal lanes
                y_local = -inner_height/2 + (i + 0.5) * lane_spacing
                start_local = np.array([-inner_width/2, y_local])
                end_local = np.array([inner_width/2, y_local])
            
            # Transform to world frame
            start_world = np.array([cx, cy]) + R @ start_local
            end_world = np.array([cx, cy]) + R @ end_local
            
            # Alternate direction for boustrophedon pattern
            if i % 2 == 0:
                lane = [(start_world[0], start_world[1]), (end_world[0], end_world[1])]
            else:
                lane = [(end_world[0], end_world[1]), (start_world[0], start_world[1])]
            
            lanes.append(lane)
        
        self.lanes = lanes
        self.current_lane_idx = 0
        self.current_waypoint_idx = 0
        
        return lanes
    
    def get_current_lane(self) -> Optional[List[Tuple[float, float]]]:
        """Get current lane waypoints."""
        if not self.lanes or self.current_lane_idx >= len(self.lanes):
            return None
        return self.lanes[self.current_lane_idx]
    
    def advance_waypoint(self):
        """Advance to next waypoint or lane."""
        if not self.lanes:
            return
        
        current_lane = self.lanes[self.current_lane_idx]
        if self.current_waypoint_idx < len(current_lane) - 1:
            self.current_waypoint_idx += 1
        else:
            # Move to next lane
            self.current_lane_idx += 1
            self.current_waypoint_idx = 0
    
    def is_complete(self) -> bool:
        """Check if all lanes are complete."""
        if not self.lanes:
            return False
        return self.current_lane_idx >= len(self.lanes)

