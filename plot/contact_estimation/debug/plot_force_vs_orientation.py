"""
Debug plot: overlays each foot's estimated z-force against its raw sensor
reading, alongside base roll/pitch, on a shared time axis.

Written to investigate a tilt-correlated bias in the contact estimator: the
estimated force fails to drop to ~0 during swing phase whenever the base has
significant roll/pitch, even though the raw sensor's swing-phase floor stays
flat throughout. Lining the two force traces up against roll/pitch makes that
correlation visible directly instead of having to cross-reference printed
numbers.

Needs the .npz produced by run_contact_estimation.py (must contain q, so the
"foot_forces"-only npz variant will not work -- use the "contact_estimate"
one).

Usage:
    python plot_force_vs_orientation.py [npz_path]

Defaults to plot/all_bags/excitation_bag_v95_contact_estimate.npz if no
npz_path is given.
"""

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

DEFAULT_NPZ_PATH = Path(__file__).resolve().parents[2] / "all_bags" / "excitation_bag_v95_contact_estimate.npz"

# est_fz_filtered columns are [fl, fr, rl, rr]; raw foot_force columns are
# [FR, FL, RR, RL] (firmware order, see npz_reader.py). This maps each
# estimator column to its correctly-paired raw sensor column.
EST_NAMES = ["fl_foot", "fr_foot", "rl_foot", "rr_foot"]
RAW_COL_FOR_EST = {"fl_foot": 1, "fr_foot": 0, "rl_foot": 3, "rr_foot": 2}


def quat_xyzw_to_rpy(quat_xyzw):
    x, y, z, w = quat_xyzw[:, 0], quat_xyzw[:, 1], quat_xyzw[:, 2], quat_xyzw[:, 3]
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1, 1))
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def plot_force_vs_orientation(npz_path):
    data = np.load(npz_path, allow_pickle=True)
    t = data["t"]
    q = data["q"]
    est_fz_filtered = data["est_fz_filtered"]  # (N,4) fl,fr,rl,rr
    raw = data["foot_force"]  # (N,4) FR,FL,RR,RL

    roll, pitch, yaw = quat_xyzw_to_rpy(q[:, 3:7])

    n_feet = len(EST_NAMES)
    fig, axes = plt.subplots(n_feet + 2, 1, sharex=True, figsize=(11, 2.3 * (n_feet + 2)))

    axes[0].plot(t, np.degrees(roll), label="roll", color="C0")
    axes[0].plot(t, np.degrees(pitch), label="pitch", color="C1")
    axes[0].set_title("base orientation (roll/pitch)")
    axes[0].set_ylabel("deg")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(t, np.degrees(yaw), label="yaw", color="C4")
    axes[1].set_title("base orientation (yaw)")
    axes[1].set_ylabel("deg")
    axes[1].legend()
    axes[1].grid(True)

    for i, name in enumerate(EST_NAMES):
        ax = axes[i + 2]
        ax.plot(t, raw[:, RAW_COL_FOR_EST[name]], label="raw sensor", color="C2", alpha=0.7)
        ax.plot(t, est_fz_filtered[:, i], label="estimate (filtered)", color="C3", alpha=0.9)
        ax.set_title(name)
        ax.set_ylabel("force")
        ax.legend()
        ax.grid(True)

    axes[-1].set_xlabel("time (s)")
    fig.suptitle(f"Force estimate vs. raw sensor vs. base orientation: {Path(npz_path).name}")
    fig.tight_layout()

    out_path = Path(npz_path).with_suffix("").name + "_force_vs_orientation.png"
    fig.savefig(out_path, dpi=150)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    npz_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_NPZ_PATH
    plot_force_vs_orientation(npz_path)
