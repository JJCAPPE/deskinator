"""
Temporary helper to plot motor error distribution using the
same formatting as tests/visualize_results.py.
"""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats


def plot_gaussian_distribution(values, metric_name, output_path, bins=20):
    """
    Plot histogram with fitted Gaussian distribution.
    Matches styling from tests/visualize_results.py.
    """
    values = np.array(values)

    mean = np.mean(values)
    std = np.std(values)

    fig, ax = plt.subplots(figsize=(10, 6))

    ax.hist(
        values,
        bins=bins,
        density=True,
        alpha=0.7,
        color="steelblue",
        edgecolor="black",
        label="Data",
    )

    x = np.linspace(values.min(), values.max(), 200)
    gaussian = stats.norm.pdf(x, mean, std)
    ax.plot(
        x,
        gaussian,
        "r-",
        linewidth=2.5,
        label=f"Gaussian fit\nμ={mean:.2f}, σ={std:.2f}",
    )

    ax.axvline(
        mean,
        color="darkred",
        linestyle="--",
        linewidth=2,
        label=f"Mean: {mean:.2f}",
    )
    ax.axvline(
        mean - std,
        color="orange",
        linestyle=":",
        linewidth=1.5,
        label=f"μ ± σ: [{mean - std:.2f}, {mean + std:.2f}]",
    )
    ax.axvline(mean + std, color="orange", linestyle=":", linewidth=1.5)

    ax.set_xlabel(metric_name, fontsize=12, fontweight="bold")
    ax.set_ylabel("Probability Density", fontsize=12, fontweight="bold")
    ax.set_title(
        f"Distribution of {metric_name}\n({len(values)} trials)",
        fontsize=14,
        fontweight="bold",
        pad=20,
    )

    ax.legend(loc="upper right", fontsize=10, framealpha=0.9)
    ax.grid(True, alpha=0.3, linestyle="--")

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close()

    print(f"✓ Saved: {output_path}")


def load_errors(csv_path: Path) -> np.ndarray:
    """Load the motor errors from CSV."""
    return np.loadtxt(csv_path, delimiter=",", skiprows=1, usecols=1)


def main():
    parser = argparse.ArgumentParser(
        description="Generate motor error distribution plot."
    )
    parser.add_argument("csv_file", type=Path, help="Path to motor-errors.csv")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("deliverables/images"),
        help="Directory to place the generated plot",
    )
    args = parser.parse_args()

    errors = load_errors(args.csv_file)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    output_path = args.output_dir / "distribution-motor-errors.png"
    plot_gaussian_distribution(errors, "Motor Error (m)", output_path, bins=20)


if __name__ == "__main__":
    main()
