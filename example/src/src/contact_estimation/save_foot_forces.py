"""
Read foot_force and foot_force_est from a recorded rosbag2 folder and save
them to a small .npz file for later plotting (see plot_foot_forces.py).

Needs pinocchio + rosbag2_py/rclpy (see bag_reader.py) -- run this half
inside the Docker/WSL2 ROS2 environment, then copy the resulting .npz
wherever you want to plot it (plot_foot_forces.py only needs numpy/matplotlib).

Usage:
    python save_foot_forces.py [bag_path] [out_path]

Defaults to example/data/2026_07_07/usable_data/excitation_bag_v4 if no
bag_path is given, and <bag_path>_foot_forces.npz next to the bag if no
out_path is given.
"""

import sys
from pathlib import Path

import numpy as np

from bag_reader import read_lowstate_bag
from go2_model import load_go2_model

DEFAULT_BAG_PATH = Path(__file__).resolve().parents[3] / "data" / "2026_07_07" / "usable_data" / "excitation_bag_v4"


def save_foot_forces(bag_path, out_path=None):
    bag_path = Path(bag_path)
    model, _data = load_go2_model()
    samples = read_lowstate_bag(str(bag_path), model)

    if out_path is None:
        out_path = bag_path.parent / f"{bag_path.name}_foot_forces.npz"

    np.savez(
        out_path,
        t=samples.t,
        foot_force=samples.foot_force,          # (N,4), raw sensor, [FR,FL,RR,RL] order
        foot_force_est=samples.foot_force_est,  # (N,4), onboard estimate, [FR,FL,RR,RL] order
    )
    print(f"Saved {len(samples.t)} samples ({samples.t[-1]:.1f}s) to {out_path}")
    return out_path


if __name__ == "__main__":
    bag_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BAG_PATH
    out_path = sys.argv[2] if len(sys.argv) > 2 else None
    save_foot_forces(bag_path, out_path)
