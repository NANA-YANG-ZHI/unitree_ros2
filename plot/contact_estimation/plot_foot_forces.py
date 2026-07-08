"""
Plot foot_force and foot_force_est previously saved by save_foot_forces.py,
as two separate time-series plots (one line per foot).

Only needs numpy/matplotlib -- no pinocchio/rosbag2_py/ROS2 required, so
this can run anywhere (e.g. plain Windows) once the .npz has been produced
by save_foot_forces.py in the ROS2 environment.

Usage:
    python plot_foot_forces.py [npz_path]

Defaults to example/data/2026_07_07/usable_data/excitation_bag_v4_foot_forces.npz
(the default output of save_foot_forces.py) if no npz_path is given.
"""

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

# foot_force / foot_force_est column order, per LowState.msg convention
FOOT_NAMES = ["FR", "FL", "RR", "RL"]

DEFAULT_NPZ_PATH = Path(__file__).resolve().parent / "excitation_bag_v4_foot_forces.npz"


def _plot_one(t, values, title_prefix, out_path):
    n_feet = len(FOOT_NAMES)
    fig, axes = plt.subplots(n_feet, 1, sharex=True, figsize=(10, 2.2 * n_feet))

    for i, foot_name in enumerate(FOOT_NAMES):
        axes[i].plot(t, values[:, i], color=f"C{i}")
        axes[i].set_title(f"{title_prefix}: {foot_name}")
        axes[i].set_ylabel("force")
        axes[i].grid(True)
    axes[-1].set_xlabel("time (s)")

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"Saved {out_path}")


def plot_foot_forces(npz_path):
    data = np.load(npz_path)
    t, foot_force, foot_force_est = data["t"], data["foot_force"], data["foot_force_est"]

    stem = Path(npz_path).with_suffix("").name
    _plot_one(t, foot_force, "foot_force (raw sensor)", f"{stem}_foot_force.png")
    _plot_one(t, foot_force_est, "foot_force_est (onboard estimate)", f"{stem}_foot_force_est.png")


if __name__ == "__main__":
    npz_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_NPZ_PATH
    plot_foot_forces(npz_path)
