"""
Zoomed-in comparison of the three ContactDetector injection-law variants
("hg", "sliding", "mixing" -- see err_mapping_func in contact_detection.py)
on the same bag: overlays est_fz_filtered from all three per foot over a
short time window, so cycle-by-cycle differences between the algorithms are
actually visible (a full-trial plot averages them out).

Expects three .npz files already produced by run_contact_estimation.py
(one per --alg), named "<bag_name>_contact_estimate_<alg>.npz" in --out-dir --
this is exactly what run_alg_comparison.sh produces.

Only needs numpy/matplotlib -- run in the plot-tools-run container.

Usage:
    python plot_alg_comparison.py --bag-name excitation_bag_v95 --t0 25 --t1 28 --out-dir .
"""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

ALGS = ["hg", "sliding", "mixing"]
ALG_COLORS = {"hg": "C0", "sliding": "C1", "mixing": "C2"}
FOOT_NAMES = ["fl_foot", "fr_foot", "rl_foot", "rr_foot"]


def plot_comparison(out_dir, bag_name, t0, t1):
    out_dir = Path(out_dir)

    runs = {}
    for alg in ALGS:
        npz_path = out_dir / f"{bag_name}_contact_estimate_{alg}.npz"
        runs[alg] = np.load(npz_path, allow_pickle=True)

    # time grid is identical across the three runs (same bag, same resampling)
    t = runs["mixing"]["t"]
    mask = (t >= t0) & (t <= t1)
    t_zoom = t[mask]

    fig, axes = plt.subplots(len(FOOT_NAMES), 1, sharex=True, figsize=(11, 2.6 * len(FOOT_NAMES)))

    for i, name in enumerate(FOOT_NAMES):
        ax = axes[i]
        for alg in ALGS:
            est = runs[alg]["est_fz_filtered"][mask, i]
            ax.plot(t_zoom, est, label=alg, color=ALG_COLORS[alg], linewidth=1.8)
        ax.set_title(name)
        ax.set_ylabel("est_fz_filtered (N)")
        ax.legend(loc="upper right", fontsize=8)
        ax.set_xlim(t_zoom[0], t_zoom[-1])
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("time (s)")
    fig.suptitle(f"hg vs. sliding vs. mixing, zoomed t={t0}-{t1}s: {bag_name}")
    fig.tight_layout()

    out_path = out_dir / f"{bag_name}_alg_comparison_zoom.png"
    fig.savefig(out_path, dpi=150)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bag-name", required=True, help="Bag basename, e.g. excitation_bag_v95 (matches the .npz file prefix)")
    parser.add_argument("--t0", type=float, default=25.0, help="Zoom window start (s)")
    parser.add_argument("--t1", type=float, default=28.0, help="Zoom window end (s)")
    parser.add_argument("--out-dir", default=".", help="Directory holding the three input .npz files and where the plot is saved")
    args = parser.parse_args()

    plot_comparison(args.out_dir, args.bag_name, args.t0, args.t1)
