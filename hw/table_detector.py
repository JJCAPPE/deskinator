"""
Helper utilities for converting hardware sensor readings to world coordinates.

Replicates the sensor position calculations from viz_demo.py's SimulatedRobot.
"""

import numpy as np

try:
    from ..config import GEOM
    from ..slam.frames import transform_point
except ImportError:
    from config import GEOM
    from slam.frames import transform_point


def compute_sensor_world_position(
    pose: tuple,
    sensor_side: str
) -> tuple:
    """Transform sensor position from robot frame to world frame.
    
    This replicates the logic from viz_demo.py lines 94-115 where SimulatedRobot
    computes sensor positions to check if they're within table bounds.
    
    Args:
        pose: Robot pose (x, y, theta) in world frame
        sensor_side: "left" or "right"
        
    Returns:
        (world_x, world_y) position of the specified sensor
    """
    x, y, theta = pose
    
    # Sensor forward offset (same for both sensors)
    sensor_fwd = GEOM.SENSOR_FWD
    
    # Sensor lateral offset
    if sensor_side == "left":
        if len(GEOM.SENSOR_LAT) > 0:
            sensor_lat = GEOM.SENSOR_LAT[0]  # Left sensor (positive = left)
        else:
            sensor_lat = 0.1  # Fallback
    elif sensor_side == "right":
        if len(GEOM.SENSOR_LAT) > 1:
            sensor_lat = GEOM.SENSOR_LAT[1]  # Right sensor (negative = right)
        else:
            sensor_lat = -0.1  # Fallback
    else:
        raise ValueError(f"Invalid sensor_side: {sensor_side}. Must be 'left' or 'right'")
    
    # Sensor position in robot frame
    sensor_robot_frame = (sensor_fwd, sensor_lat)
    
    # Transform to world frame using the standard transform_point utility
    world_x, world_y = transform_point(pose, sensor_robot_frame)
    
    return (world_x, world_y)
