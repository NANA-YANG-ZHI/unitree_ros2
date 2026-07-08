"""
Plot the estimated per-foot z-forces and contact states previously saved by
run_contact_estimation.py (example/src/src/contact_estimation/run_contact_estimation.py).

Only needs numpy/matplotlib -- no pinocchio/rosbag2_py/ROS2 required, so
this can run anywhere (e.g. plain Windows) once the .npz has been produced
by run_contact_estimation.py in the ROS2 environment.

Usage:
    python plot_contact_estimate.py [npz_path]

Defaults to excitation_bag_v5_contact_estimate.npz (this directory) if no
npz_path is given.
"""

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

DEFAULT_NPZ_PATH = Path(__file__).resolve().parent / "excitation_bag_v5_contact_estimate.npz"


def plot_contact_estimate(npz_path):
    data = np.load(npz_path, allow_pickle=True)
    t = data["t"]
    est_fz_filtered = data["est_fz_filtered"]
    contact_states = data["contact_states"]
    foot_names = [str(name) for name in data["foot_names"]]

    n_feet = len(foot_names)
    fig, axes = plt.subplots(n_feet + 1, 1, sharex=True, figsize=(10, 2.2 * (n_feet + 1)))

    for i, foot_name in enumerate(foot_names):
        axes[i].plot(t, est_fz_filtered[:, i], color=f"C{i}")
        axes[i].set_title(f"est_fz_filtered: {foot_name}")
        axes[i].set_ylabel("force")
        axes[i].grid(True)

    for i, foot_name in enumerate(foot_names):
        # offset each foot's 0/1 trace so overlapping contact periods stay readable
        axes[-1].step(t, contact_states[:, i].astype(float) + i * 1.2, where="post", label=foot_name)
    axes[-1].set_title("contact_states (stacked, offset per foot)")
    axes[-1].set_xlabel("time (s)")
    axes[-1].set_yticks([])
    axes[-1].legend()
    axes[-1].grid(True)

    fig.suptitle(f"Contact estimate: {npz_path}")
    fig.tight_layout()

    out_path = Path(npz_path).with_suffix("").name + "_plot.png"
    fig.savefig(out_path, dpi=150)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    npz_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_NPZ_PATH
    plot_contact_estimate(npz_path)
