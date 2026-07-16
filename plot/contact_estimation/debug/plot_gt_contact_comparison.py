"""
Plot the ground-truth contact comparison saved by
example/src/src/contact_estimation/compute_gt_contact_signals.py: overlays the
kinematic (apply_contact_detection_gt), raw-sensor-threshold, and
foot_position_body-threshold boolean contact signals per foot.

rr_foot is not included -- excluded upstream (known-broken raw sensor).

Only needs numpy/matplotlib -- run in the plot-tools-run container (no
pinocchio needed, this only reads the already-computed .npz).

Usage:
    python plot_gt_contact_comparison.py [npz_path]
"""

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

DEFAULT_NPZ_PATH = Path(__file__).resolve().parents[2] / "all_bags" / "excitation_bag_v95_gt_contact_comparison.npz"


def plot_gt_contact_comparison(npz_path):
    data = np.load(npz_path, allow_pickle=True)
    t = data["t"]
    foot_names = [str(name) for name in data["foot_names"]]
    kin_contact = data["kin_contact"]
    sensor_bool = data["sensor_bool"]
    fpb_bool = data["fpb_bool"]

    n_feet = len(foot_names)
    fig, axes = plt.subplots(n_feet, 1, sharex=True, figsize=(11, 2.6 * n_feet))
    if n_feet == 1:
        axes = [axes]

    for i, name in enumerate(foot_names):
        K, S, F = kin_contact[:, i], sensor_bool[:, i], fpb_bool[:, i]
        agreement_ks = 100 * np.mean(K == S)
        agreement_kf = 100 * np.mean(K == F)
        agreement_sf = 100 * np.mean(S == F)

        ax = axes[i]
        ax.step(t, K.astype(float), where="post", label="kinematic (apply_contact_detection_gt)", color="C0")
        ax.step(t, S.astype(float) + 1.15, where="post", label="raw sensor threshold", color="C1")
        ax.step(t, F.astype(float) + 2.3, where="post", label="foot_position_body threshold", color="C2")
        ax.set_yticks([])
        ax.set_title(
            f"{name}  (kin-vs-sensor={agreement_ks:.1f}%  kin-vs-fpb={agreement_kf:.1f}%  sensor-vs-fpb={agreement_sf:.1f}%)"
        )
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, axis="x")

    axes[-1].set_xlabel("time (s)")
    fig.suptitle(f"Ground-truth contact comparison: {Path(npz_path).name}\n"
                 f"(rr_foot excluded -- known-broken raw sensor)")
    fig.tight_layout()

    out_path = Path(npz_path).with_suffix("").name + "_plot.png"
    fig.savefig(out_path, dpi=150)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    npz_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_NPZ_PATH
    plot_gt_contact_comparison(npz_path)
