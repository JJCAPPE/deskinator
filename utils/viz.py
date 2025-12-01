"""
Visualization utilities.

Provides plotting and debug visualization for SLAM and coverage.
Optimized for real-time performance using persistent artists.
"""

import numpy as np
from typing import List, Tuple, Optional

try:
    import matplotlib.pyplot as plt
    import matplotlib.transforms as transforms
    from matplotlib.patches import Rectangle, Circle
    from matplotlib.collections import LineCollection

    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: matplotlib not available. Visualization disabled.")

try:
    from .config import GEOM
except ImportError:
    try:
        from config import GEOM
    except ImportError:
        # Fallback defaults
        class GEOM:
            SENSOR_FWD = 0.2185
            SENSOR_TO_VAC = -0.79308
            VAC_WIDTH = 0.2
            SENSOR_LAT = (0.1, -0.1)


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

        try:
            plt.style.use("fast")  # Use fast style if available
            plt.ion()  # Interactive mode
            self.fig, self.axes = plt.subplots(1, 2, figsize=figsize)
        except Exception as e:
            print(f"[Viz] Error initializing matplotlib: {e}")
            self.enabled = False
            return

        # --- SLAM MAP SETUP ---
        self.ax_map = self.axes[0]
        self.ax_map.set_xlabel("X (m)")
        self.ax_map.set_ylabel("Y (m)")
        self.ax_map.set_title("SLAM Map")
        self.ax_map.set_aspect("equal")
        self.ax_map.grid(True)

        # Initialize Artists (Empty)
        # Trajectory
        (self.ln_traj,) = self.ax_map.plot(
            [], [], "b-", linewidth=1, label="Trajectory"
        )

        # Current Pose
        (self.ln_pose,) = self.ax_map.plot(
            [], [], "o", color="gray", markersize=8, label="Current", zorder=5
        )

        # Heading Arrow (using a simple line for speed, or a reusable arrow patch)
        # Arrows are hard to reuse efficiently in matplotlib, we'll redraw it or use a line
        # Using a line for heading is much faster than creating a FancyArrow every frame
        (self.ln_heading,) = self.ax_map.plot([], [], "k-", linewidth=2, zorder=6)

        # Edges (using plot is faster than scatter for simple dots)
        (self.ln_edges,) = self.ax_map.plot(
            [], [], "g.", markersize=3, label="Edges", alpha=0.6
        )

        # Tactile hits
        (self.ln_tactile,) = self.ax_map.plot(
            [], [], "rx", markersize=8, markeredgewidth=2, label="Tactile"
        )

        # Loop constraints (LineCollection for speed)
        self.lc_loops = LineCollection(
            [],
            colors="green",
            linestyles="--",
            linewidths=1.5,
            alpha=0.6,
            label="Loop Closure",
        )
        self.ax_map.add_collection(self.lc_loops)

        # Robot Body Patches
        self.patch_robot = Rectangle(
            (0, 0),
            0,
            0,
            facecolor="purple",
            edgecolor="darkviolet",
            linewidth=1.5,
            alpha=0.7,
            zorder=5,
        )
        self.ax_map.add_patch(self.patch_robot)

        self.sensor_circles = []
        if hasattr(GEOM, "SENSOR_LAT"):
            for _ in GEOM.SENSOR_LAT:
                c = Circle(
                    (0, 0),
                    0.02,
                    facecolor="red",
                    edgecolor="black",
                    alpha=0.8,
                    zorder=10,
                )
                self.ax_map.add_patch(c)
                self.sensor_circles.append(c)

        # Boundary Rectangle
        self.patch_boundary = Rectangle(
            (0, 0),
            0,
            0,
            facecolor="none",
            edgecolor="orange",
            linewidth=2,
            label="Boundary",
        )
        self.ax_map.add_patch(self.patch_boundary)

        # Ground Truth Rectangle
        self.patch_ground_truth = Rectangle(
            (0, 0),
            0,
            0,
            facecolor="none",
            edgecolor="red",
            linewidth=2,
            linestyle="--",
            label="Ground Truth",
        )
        self.ax_map.add_patch(self.patch_ground_truth)

        # Text Status
        self.txt_status = self.ax_map.text(
            0.02,
            0.98,
            "",
            transform=self.ax_map.transAxes,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
            zorder=20,
        )

        # Legend (create once)
        self.ax_map.legend(loc="lower left", fontsize="small")

        # --- COVERAGE MAP SETUP ---
        self.ax_coverage = self.axes[1]
        self.ax_coverage.set_xlabel("X (m)")
        self.ax_coverage.set_ylabel("Y (m)")
        self.ax_coverage.set_title("Coverage Map")
        self.ax_coverage.set_aspect("equal")

        # Coverage Image
        # Initialize with dummy data
        self.img_coverage = self.ax_coverage.imshow(
            np.zeros((10, 10)), cmap="Greens", origin="lower", alpha=0.7, vmin=0, vmax=1
        )

        # Coverage Boundary Overlay
        self.patch_cov_boundary = Rectangle(
            (0, 0), 0, 0, facecolor="none", edgecolor="blue", linewidth=2
        )
        self.ax_coverage.add_patch(self.patch_cov_boundary)

        # Show window
        plt.show(block=False)
        plt.pause(0.1)

    def update(
        self,
        poses: List[Tuple[float, float, float]],
        edge_points: List[Tuple[float, float]],
        rectangle: Optional[Tuple[float, float, float, float, float]],
        coverage_grid: Optional[np.ndarray],
        swept_map_bounds: Optional[Tuple[float, float, float, float]],
        loop_constraints: Optional[
            List[Tuple[Tuple[float, float], Tuple[float, float]]]
        ] = None,
        text_info: str = "",
        robot_state: str = "IDLE",
        tactile_hits: Optional[List[Tuple[float, float]]] = None,
        ground_truth_bounds: Optional[Tuple[float, float, float, float]] = None,
    ):
        """Update visualization efficiently."""
        if not self.enabled:
            return

        # 1. Update Status Text
        self.txt_status.set_text(text_info)

        # 2. Update Trajectory
        if poses:
            # Extract x, y arrays
            # Using numpy is faster than list comprehension if available, but lists are okay for <10k points
            xs = [p[0] for p in poses]
            ys = [p[1] for p in poses]
            self.ln_traj.set_data(xs, ys)

            # Update Current Pose Marker
            current_pose = poses[-1]
            cx, cy, ctheta = current_pose
            self.ln_pose.set_data([cx], [cy])

            # Update Color based on state
            color = "gray"
            if "BOUNDARY" in robot_state:
                color = "orange"
            elif "COVERAGE" in robot_state:
                color = "green"
            elif "DONE" in robot_state:
                color = "blue"
            elif "ERROR" in robot_state:
                color = "red"
            self.ln_pose.set_color(color)

            # Update Heading Line (length 0.15m)
            hx = cx + 0.15 * np.cos(ctheta)
            hy = cy + 0.15 * np.sin(ctheta)
            self.ln_heading.set_data([cx, hx], [cy, hy])
            self.ln_heading.set_color(color)

            # Update Robot Body Patch (Purple Rectangle)
            # Robot dimensions: 220mm length x 200mm width
            ROBOT_LENGTH = 0.220  # 220mm in meters (forward/backward)
            ROBOT_WIDTH = 0.200  # 200mm in meters (left/right)

            # Offset: 42mm BEHIND the sensor line
            # Sensor line is at GEOM.SENSOR_FWD (typically ~0.218m)
            # Box starts at SENSOR_FWD - 0.042
            # But the box is defined by center or corner.

            # Let's assume SENSOR_FWD is the front of the robot for "sensor line" context
            # If "start 42mm behind sensor line" means the FRONT of the box is 42mm behind sensors:
            box_front_x_robot = GEOM.SENSOR_FWD - 0.042
            box_center_x_robot = box_front_x_robot - (ROBOT_LENGTH / 2)

            # Center robot at pose position + offset
            # Rotate offset vector
            offset_x = box_center_x_robot * np.cos(ctheta)
            offset_y = box_center_x_robot * np.sin(ctheta)

            box_center_x_world = cx + offset_x
            box_center_y_world = cy + offset_y

            robot_t = (
                transforms.Affine2D()
                .rotate(ctheta)
                .translate(box_center_x_world, box_center_y_world)
            )
            self.patch_robot.set_width(
                ROBOT_LENGTH
            )  # Actually usually width is y-axis in local frame, length x-axis
            # Wait, Matplotlib Rectangle (0,0) width height is (x, y)
            # So width=ROBOT_LENGTH (x-axis size), height=ROBOT_WIDTH (y-axis size)
            self.patch_robot.set_width(ROBOT_LENGTH)
            self.patch_robot.set_height(ROBOT_WIDTH)
            self.patch_robot.set_xy(
                (-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2)
            )  # Center relative to transform
            self.patch_robot.set_transform(robot_t + self.ax_map.transData)
            # Ensure visibility
            self.patch_robot.set_visible(True)

            # Update Sensors (Red Circles)
            if hasattr(GEOM, "SENSOR_LAT"):
                for i, lat_offset in enumerate(GEOM.SENSOR_LAT):
                    if i < len(self.sensor_circles):
                        # Calculate world position
                        sx_world = (
                            cx
                            + GEOM.SENSOR_FWD * np.cos(ctheta)
                            - lat_offset * np.sin(ctheta)
                        )
                        sy_world = (
                            cy
                            + GEOM.SENSOR_FWD * np.sin(ctheta)
                            + lat_offset * np.cos(ctheta)
                        )
                        self.sensor_circles[i].center = (sx_world, sy_world)
                        self.sensor_circles[i].set_visible(True)
        else:
            # Hide robot when no poses
            self.patch_robot.set_visible(False)
            for c in self.sensor_circles:
                c.set_visible(False)

        # 3. Update Edges
        if edge_points:
            ex = [p[0] for p in edge_points]
            ey = [p[1] for p in edge_points]
            self.ln_edges.set_data(ex, ey)

        # 4. Update Tactile Hits
        if tactile_hits:
            tx = [p[0] for p in tactile_hits]
            ty = [p[1] for p in tactile_hits]
            self.ln_tactile.set_data(tx, ty)

        # 5. Update Loop Constraints
        if loop_constraints:
            # LineCollection expects a list of segments: [(x0, y0), (x1, y1)]
            segments = [[p1, p2] for p1, p2 in loop_constraints]
            self.lc_loops.set_segments(segments)

        # 6. Update Boundary Rectangle
        if rectangle:
            cx, cy, heading, width, height = rectangle
            rect_t = (
                transforms.Affine2D().rotate_around(0, 0, heading).translate(cx, cy)
            )

            self.patch_boundary.set_width(width)
            self.patch_boundary.set_height(height)
            self.patch_boundary.set_xy((-width / 2, -height / 2))
            self.patch_boundary.set_transform(rect_t + self.ax_map.transData)

            # Also update the one on coverage map
            self.patch_cov_boundary.set_width(width)
            self.patch_cov_boundary.set_height(height)
            self.patch_cov_boundary.set_xy((-width / 2, -height / 2))
            self.patch_cov_boundary.set_transform(rect_t + self.ax_coverage.transData)
        else:
            # Hide if not confident
            self.patch_boundary.set_width(0)
            self.patch_cov_boundary.set_width(0)

        # 7. Update Ground Truth
        if ground_truth_bounds:
            min_x, max_x, min_y, max_y = ground_truth_bounds
            self.patch_ground_truth.set_width(max_x - min_x)
            self.patch_ground_truth.set_height(max_y - min_y)
            self.patch_ground_truth.set_xy((min_x, min_y))

        # 8. Update Coverage Map
        if coverage_grid is not None and swept_map_bounds is not None:
            min_x, max_x, min_y, max_y = swept_map_bounds
            self.img_coverage.set_data(coverage_grid)
            self.img_coverage.set_extent((min_x, max_x, min_y, max_y))
            self.img_coverage.set_clim(0, 1)  # Ensure scaling stays consistent

        # 9. Dynamic Autoscaling (Optional - can be expensive so maybe do it less often)
        # Matplotlib's relim is fast enough for lines
        self.ax_map.relim()
        self.ax_map.autoscale_view()

        self.ax_coverage.relim()
        self.ax_coverage.autoscale_view()

        # Render
        # Pause is necessary for the GUI event loop to process
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
