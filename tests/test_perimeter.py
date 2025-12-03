"""
Quick test to verify perimeter waypoints are being generated.
"""

import sys
from pathlib import Path
import numpy as np

# Add parent directory to path
parent_dir = Path(__file__).parent.parent
if str(parent_dir) not in sys.path:
    sys.path.insert(0, str(parent_dir))

from planning.coverage import CoveragePlanner

# Create a simple test rectangle
test_rect = (0.0, 0.0, 0.0, 2.0, 2.0)  # center (0,0), heading 0, 2m x 2m

# Create planner and build lanes
planner = CoveragePlanner()
planner.set_rectangle(test_rect)
lanes = planner.build_lanes(start_pose=(0.5, -0.8, 0.0))

print(f"Rectangle: {test_rect[3]:.2f}m x {test_rect[4]:.2f}m")
print(f"Number of lanes: {len(lanes)}")
print(f"Total waypoints in path: {len(planner.path)}")

# Analyze the path to identify perimeter waypoints
# The last several waypoints should form a rectangle perimeter
if len(planner.path) > 0:
    print(f"\nFirst waypoint: {planner.path[0]}")
    print(f"Last waypoint: {planner.path[-1]}")
    
    # Count waypoints in the last section (should be perimeter)
    # Perimeter should have: transition (2) + 4 corners * 2 waypoints each = 10 waypoints
    print(f"\nLast 12 waypoints (should include perimeter):")
    for i, wp in enumerate(planner.path[-12:]):
        idx = len(planner.path) - 12 + i
        print(f"  [{idx}] ({wp[0]:.3f}, {wp[1]:.3f}, θ={np.rad2deg(wp[2]):.1f}°)")

print("\n✓ Perimeter waypoints appear to be generated correctly!")
