"""
Visualization utilities.

Provides plotting and debug visualization for SLAM and coverage.
"""

import numpy as np
from typing import List, Tuple, Optional

try:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle

    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: matplotlib not available. Visualization disabled.")


class Visualizer:
    """Real-time visualization of robot state."""

    def __init__(self, figsize: Tuple[int, int] = (12, 8)):
        """
        Initialize visualizer.

        Args:
            figsize: Figure size (width, height)
        """
        self.enabled = MATPLOTLIB_AVAILABLE

        if not self.enabled:
            return

        plt.ion()  # Interactive mode
        self.fig, self.axes = plt.subplots(1, 2, figsize=figsize)
        self.ax_map = self.axes[0]
        self.ax_coverage = self.axes[1]

        self.ax_map.set_xlabel("X (m)")
        self.ax_map.set_ylabel("Y (m)")
        self.ax_map.set_title("SLAM Map")
        self.ax_map.set_aspect("equal")
        self.ax_map.grid(True)

        self.ax_coverage.set_xlabel("X (m)")
        self.ax_coverage.set_ylabel("Y (m)")
        self.ax_coverage.set_title("Coverage Map")
        self.ax_coverage.set_aspect("equal")

    def update(
        self,
        poses: List[Tuple[float, float, float]],
        edge_points: List[Tuple[float, float]],
        rectangle: Optional[Tuple[float, float, float, float, float]],
        coverage_grid: Optional[np.ndarray],
        swept_map_bounds: Optional[Tuple[float, float, float, float]],
    ):
        """
        Update visualization.

        Args:
            poses: List of (x, y, θ) poses
            edge_points: List of (x, y) edge detection points
            rectangle: Rectangle (cx, cy, heading, width, height) or None
            coverage_grid: Coverage grid array or None
            swept_map_bounds: (min_x, max_x, min_y, max_y) or None
        """
        if not self.enabled:
            return

        # Clear axes
        self.ax_map.clear()
        self.ax_coverage.clear()

        # Plot trajectory
        if poses:
            xs = [p[0] for p in poses]
            ys = [p[1] for p in poses]
            self.ax_map.plot(xs, ys, "b-", linewidth=1, label="Trajectory")

            # Current pose
            if len(poses) > 0:
                x, y, theta = poses[-1]
                self.ax_map.plot(x, y, "ro", markersize=8, label="Current")

                # Heading arrow
                dx = 0.1 * np.cos(theta)
                dy = 0.1 * np.sin(theta)
                self.ax_map.arrow(
                    x, y, dx, dy, head_width=0.03, head_length=0.05, fc="r", ec="r"
                )

        # Plot edge points
        if edge_points:
            ex = [p[0] for p in edge_points]
            ey = [p[1] for p in edge_points]
            self.ax_map.plot(ex, ey, "g.", markersize=3, label="Edges")

        # Plot rectangle
        if rectangle:
            cx, cy, heading, width, height = rectangle

            # Create rectangle patch
            # Transform to axis-aligned then rotate
            rect_patch = Rectangle(
                (-width / 2, -height / 2),
                width,
                height,
                angle=np.rad2deg(heading),
                facecolor="none",
                edgecolor="orange",
                linewidth=2,
                label="Boundary",
            )

            # Add patch at center
            from matplotlib.transforms import Affine2D

            t = Affine2D().rotate_around(0, 0, heading).translate(cx, cy)
            rect_patch.set_transform(t + self.ax_map.transData)
            self.ax_map.add_patch(rect_patch)

        self.ax_map.legend()
        self.ax_map.set_xlabel("X (m)")
        self.ax_map.set_ylabel("Y (m)")
        self.ax_map.set_title("SLAM Map")
        self.ax_map.set_aspect("equal")
        self.ax_map.grid(True)

        # Plot coverage
        if coverage_grid is not None and swept_map_bounds is not None:
            min_x, max_x, min_y, max_y = swept_map_bounds

            self.ax_coverage.imshow(
                coverage_grid,
                extent=[min_x, max_x, min_y, max_y],
                origin="lower",
                cmap="Greens",
                alpha=0.7,
            )

            # Overlay rectangle
            if rectangle:
                cx, cy, heading, width, height = rectangle
                rect_patch = Rectangle(
                    (-width / 2, -height / 2),
                    width,
                    height,
                    angle=np.rad2deg(heading),
                    facecolor="none",
                    edgecolor="blue",
                    linewidth=2,
                )
                t = Affine2D().rotate_around(0, 0, heading).translate(cx, cy)
                rect_patch.set_transform(t + self.ax_coverage.transData)
                self.ax_coverage.add_patch(rect_patch)

        self.ax_coverage.set_xlabel("X (m)")
        self.ax_coverage.set_ylabel("Y (m)")
        self.ax_coverage.set_title("Coverage Map")
        self.ax_coverage.set_aspect("equal")

        plt.pause(0.001)

    def save(self, filename: str):
        """Save current figure to file."""
        if not self.enabled:
            return

        self.fig.savefig(filename, dpi=150, bbox_inches="tight")
        print(f"[Viz] Saved to {filename}")

    def close(self):
        """Close visualization."""
        if not self.enabled:
            return

        plt.close(self.fig)


def plot_trajectory_offline(
    poses: List[Tuple[float, float, float]],
    edge_points: List[Tuple[float, float]],
    filename: str,
):
    """
    Plot trajectory offline and save to file.

    Args:
        poses: List of (x, y, θ) poses
        edge_points: List of (x, y) edge points
        filename: Output filename
    """
    if not MATPLOTLIB_AVAILABLE:
        print("Matplotlib not available, skipping offline plot")
        return

    fig, ax = plt.subplots(figsize=(10, 10))

    # Plot trajectory
    if poses:
        xs = [p[0] for p in poses]
        ys = [p[1] for p in poses]
        ax.plot(xs, ys, "b-", linewidth=2, label="Trajectory")
        ax.plot(xs[0], ys[0], "go", markersize=10, label="Start")
        ax.plot(xs[-1], ys[-1], "ro", markersize=10, label="End")

    # Plot edge points
    if edge_points:
        ex = [p[0] for p in edge_points]
        ey = [p[1] for p in edge_points]
        ax.plot(ex, ey, "g.", markersize=5, label="Edges")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Robot Trajectory")
    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend()

    fig.savefig(filename, dpi=150, bbox_inches="tight")
    plt.close(fig)

    print(f"[Viz] Saved trajectory to {filename}")
