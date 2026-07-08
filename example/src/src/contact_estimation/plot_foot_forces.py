"""
Read foot_force and foot_force_est from a recorded rosbag2 folder and plot
them as two separate time-series plots (one line per foot).

Usage:
    python plot_foot_forces.py [bag_path]

Defaults to example/data/2026_07_07/usable_data/excitation_bag_v4 if no
bag_path is given.
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt

from bag_reader import read_lowstate_bag
from go2_model import load_go2_model

# foot_force / foot_force_est column order, per LowState.msg convention
FOOT_NAMES = ["FR", "FL", "RR", "RL"]

DEFAULT_BAG_PATH = Path(__file__).resolve().parents[3] / "data" / "2026_07_07" / "usable_data" / "excitation_bag_v4"


def plot_foot_forces(bag_path):
    model, _data = load_go2_model()
    samples = read_lowstate_bag(str(bag_path), model)

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 8))

    for i, foot_name in enumerate(FOOT_NAMES):
        axes[0].plot(samples.t, samples.foot_force[:, i], label=foot_name)
    axes[0].set_title("foot_force (raw sensor)")
    axes[0].set_ylabel("force")
    axes[0].legend()
    axes[0].grid(True)

    for i, foot_name in enumerate(FOOT_NAMES):
        axes[1].plot(samples.t, samples.foot_force_est[:, i], label=foot_name)
    axes[1].set_title("foot_force_est (onboard estimate)")
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("force")
    axes[1].legend()
    axes[1].grid(True)

    fig.suptitle(f"Foot forces: {bag_path}")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    bag_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BAG_PATH
    plot_foot_forces(bag_path)
