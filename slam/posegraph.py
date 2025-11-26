"""
Pose graph SLAM backend.

Maintains a graph of robot poses with odometry and loop closure constraints.
Uses scipy.optimize.least_squares for optimization.
"""

import numpy as np
from scipy.optimize import least_squares
import networkx as nx
from typing import List, Tuple, Optional
try:
    from .frames import wrap_angle, pose_difference
except ImportError:
    from slam.frames import wrap_angle, pose_difference


class PoseGraph:
    """Pose graph for SLAM optimization."""

    def __init__(self):
        """Initialize empty pose graph."""
        self.graph = nx.Graph()
        self.node_counter = 0
        self.poses = {}  # node_id -> (x, y, θ)
        self.edges_odom = []  # (i, j, z_ij, Info)
        self.edges_yaw = []  # (i, yaw, Info)
        self.edges_loop = []  # (i, j, z_ij, Info)

    def add_node(self, t: float, pose: tuple[float, float, float]) -> int:
        """
        Add a pose node to the graph.

        Args:
            t: Timestamp
            pose: Pose (x, y, θ)

        Returns:
            Node ID
        """
        node_id = self.node_counter
        self.node_counter += 1

        self.graph.add_node(node_id, t=t, pose=pose)
        self.poses[node_id] = pose

        return node_id

    def add_edge_odom(
        self, i: int, j: int, z_ij: tuple[float, float, float], Info: np.ndarray
    ):
        """
        Add odometry constraint between consecutive poses.

        Args:
            i: From node ID
            j: To node ID
            z_ij: Relative pose measurement (dx, dy, dθ)
            Info: Information matrix (3x3)
        """
        self.graph.add_edge(i, j, type="odom")
        self.edges_odom.append((i, j, z_ij, Info))

    def add_edge_yaw(self, i: int, yaw: float, Info: float):
        """
        Add absolute yaw constraint.

        Args:
            i: Node ID
            yaw: Absolute yaw measurement (rad)
            Info: Information (inverse variance)
        """
        self.edges_yaw.append((i, yaw, Info))

    def add_manual_loop_closure(self, i: int, j: int):
        """
        Force a loop closure between node i and j (assuming they are the same location).
        Used when robot returns to start.
        """
        # Measurement z_ij should be 0 if they are truly the same
        z_ij = (0.0, 0.0, 0.0)
        
        # High confidence
        Info = np.diag([1000.0, 1000.0, 500.0])
        
        self.edges_loop.append((j, i, z_ij, Info))
        self.graph.add_edge(j, i, type="loop")

    def optimize(self):
        """
        Optimize the pose graph using least squares.
        """
        if len(self.poses) < 2:
            return

        # Build parameter vector: [x0, y0, θ0, x1, y1, θ1, ...]
        node_ids = sorted(self.poses.keys())
        x0 = []
        for node_id in node_ids:
            x, y, th = self.poses[node_id]
            x0.extend([x, y, th])

        x0 = np.array(x0)

        # Fix first pose (anchor)
        fixed_params = [0, 1, 2]

        # Always use 'trf' method - it's more robust and handles underdetermined 
        # systems gracefully (unlike 'lm' which requires n_residuals >= n_variables)
        result = least_squares(
            self._residuals, x0, args=(node_ids,), method="trf", verbose=0
        )

        # Update poses
        x_opt = result.x
        for i, node_id in enumerate(node_ids):
            idx = i * 3
            x, y, th = x_opt[idx], x_opt[idx + 1], x_opt[idx + 2]
            th = wrap_angle(th)
            self.poses[node_id] = (x, y, th)
            self.graph.nodes[node_id]["pose"] = (x, y, th)

    def _residuals(self, x: np.ndarray, node_ids: List[int]) -> np.ndarray:
        """
        Compute residuals for all constraints.

        Args:
            x: Parameter vector
            node_ids: List of node IDs

        Returns:
            Residual vector
        """
        # Map node_id to index in x
        node_to_idx = {node_id: i for i, node_id in enumerate(node_ids)}

        residuals = []

        # Odometry constraints
        for i, j, z_ij, Info in self.edges_odom:
            if i not in node_to_idx or j not in node_to_idx:
                continue

            idx_i = node_to_idx[i] * 3
            idx_j = node_to_idx[j] * 3

            xi, yi, thi = x[idx_i], x[idx_i + 1], x[idx_i + 2]
            xj, yj, thj = x[idx_j], x[idx_j + 1], x[idx_j + 2]

            # Predicted relative pose
            c, s = np.cos(thi), np.sin(thi)
            dx_pred = c * (xj - xi) + s * (yj - yi)
            dy_pred = -s * (xj - xi) + c * (yj - yi)
            dth_pred = wrap_angle(thj - thi)

            # Measurement
            dx_meas, dy_meas, dth_meas = z_ij

            # Residual
            r = np.array(
                [dx_pred - dx_meas, dy_pred - dy_meas, wrap_angle(dth_pred - dth_meas)]
            )

            # Weight by information matrix (simplified: diagonal)
            r_weighted = np.sqrt(np.diag(Info)) * r
            residuals.extend(r_weighted)

        # Yaw constraints
        for i, yaw_meas, Info in self.edges_yaw:
            if i not in node_to_idx:
                continue

            idx_i = node_to_idx[i] * 3
            thi = x[idx_i + 2]

            r = wrap_angle(thi - yaw_meas)
            r_weighted = np.sqrt(Info) * r
            residuals.append(r_weighted)

        # Loop closure constraints
        for i, j, z_ij, Info in self.edges_loop:
            if i not in node_to_idx or j not in node_to_idx:
                continue

            idx_i = node_to_idx[i] * 3
            idx_j = node_to_idx[j] * 3

            xi, yi, thi = x[idx_i], x[idx_i + 1], x[idx_i + 2]
            xj, yj, thj = x[idx_j], x[idx_j + 1], x[idx_j + 2]

            # Predicted relative pose
            c, s = np.cos(thi), np.sin(thi)
            dx_pred = c * (xj - xi) + s * (yj - yi)
            dy_pred = -s * (xj - xi) + c * (yj - yi)
            dth_pred = wrap_angle(thj - thi)

            # Measurement
            dx_meas, dy_meas, dth_meas = z_ij

            # Residual
            r = np.array(
                [dx_pred - dx_meas, dy_pred - dy_meas, wrap_angle(dth_pred - dth_meas)]
            )

            # Weight by information matrix
            r_weighted = np.sqrt(np.diag(Info)) * r
            residuals.extend(r_weighted)

        return np.array(residuals)

    def current_pose(self) -> tuple[float, float, float]:
        """
        Get the most recent pose.

        Returns:
            (x, y, θ)
        """
        if not self.poses:
            return (0.0, 0.0, 0.0)

        latest_id = max(self.poses.keys())
        return self.poses[latest_id]

    def get_all_poses(self) -> List[tuple[float, float, float]]:
        """
        Get all poses in order.

        Returns:
            List of (x, y, θ) tuples
        """
        node_ids = sorted(self.poses.keys())
        return [self.poses[nid] for nid in node_ids]
