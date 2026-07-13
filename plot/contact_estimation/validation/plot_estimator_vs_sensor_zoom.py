"""
Zoomed-in comparison of the fixed force estimator (est_fz_filtered) against
the raw onboard sensor (foot_force), for fl_foot/fr_foot/rl_foot stacked.
Shows a short time window so cycle-by-cycle agreement is visible, with the
contact thresholds drawn in and a strip marking where the two threshold
decisions agree/disagree.

rr_foot is excluded: known-broken raw sensor.

Only needs numpy/matplotlib -- run in the plot-tools-run container.

Usage:
    python plot_estimator_vs_sensor_zoom.py [npz_path] [--t0 25] [--t1 33]
"""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

DEFAULT_NPZ_PATH = Path(__file__).resolve().parents[1] / "excitation_bag_v95_contact_estimate.npz"

FOOT_NAMES = ["fl_foot", "fr_foot", "rl_foot"]  # rr_foot excluded: known-broken sensor
EST_COL_FOR_FOOT = {"fl_foot": 0, "fr_foot": 1, "rl_foot": 2}       # est_fz_filtered order: fl,fr,rl,rr
RAW_COL_FOR_FOOT = {"fl_foot": 1, "fr_foot": 0, "rl_foot": 3}       # foot_force order: FR,FL,RR,RL

EST_THRESHOLD = 25.0
SENSOR_THRESHOLD = 20.0


def plot_zoom(npz_path, t0, t1, out_path):
    d = np.load(npz_path, allow_pickle=True)
    t = d["t"]
    est_fz_filtered = d["est_fz_filtered"]
    foot_force = d["foot_force"]

    mask = (t >= t0) & (t <= t1)
    t_zoom = t[mask]

    fig, axes = plt.subplots(len(FOOT_NAMES), 1, sharex=True, figsize=(11, 3.0 * len(FOOT_NAMES)))

    for i, name in enumerate(FOOT_NAMES):
        est = est_fz_filtered[mask, EST_COL_FOR_FOOT[name]]
        raw = foot_force[mask, RAW_COL_FOR_FOOT[name]]
        est_on = est > EST_THRESHOLD
        raw_on = raw > SENSOR_THRESHOLD
        agree = est_on == raw_on

        ax = axes[i]

        # shade agreement/disagreement behind the traces
        change_points = np.where(np.diff(agree.astype(int)) != 0)[0] + 1
        starts = np.concatenate(([0], change_points))
        ends = np.concatenate((change_points, [len(t_zoom)]))
        for s, e in zip(starts, ends):
            color = "#d6f5d6" if agree[s] else "#f7d6d6"
            ax.axvspan(t_zoom[s], t_zoom[min(e, len(t_zoom) - 1)], color=color, zorder=0)

        ax.plot(t_zoom, est, label="estimator (est_fz_filtered)", color="C0", linewidth=1.8)
        ax.plot(t_zoom, raw, label="raw sensor (foot_force)", color="C1", linewidth=1.8)
        ax.axhline(EST_THRESHOLD, color="C0", linestyle="--", linewidth=1, alpha=0.7)
        ax.axhline(SENSOR_THRESHOLD, color="C1", linestyle="--", linewidth=1, alpha=0.7)

        agreement_pct = 100 * np.mean(agree)
        ax.set_title(f"{name}  (threshold agreement over this window: {agreement_pct:.1f}%)")
        ax.set_ylabel("force (N)")
        ax.legend(loc="upper right", fontsize=8)
        ax.set_xlim(t_zoom[0], t_zoom[-1])

    axes[-1].set_xlabel("time (s)")
    fig.suptitle(f"Estimator vs. raw sensor, zoomed t={t0}-{t1}s: {Path(npz_path).name}\n"
                 f"(green = threshold decisions agree, red = disagree; rr_foot excluded)")
    fig.tight_layout()

    fig.savefig(out_path, dpi=150)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("npz_path", nargs="?", default=str(DEFAULT_NPZ_PATH))
    parser.add_argument("--t0", type=float, default=25.0, help="Zoom window start (s)")
    parser.add_argument("--t1", type=float, default=33.0, help="Zoom window end (s)")
    parser.add_argument("--out", default=None, help="Output PNG path (default: <npz_name>_estimator_vs_sensor_zoom.png)")
    args = parser.parse_args()

    out_path = args.out or (Path(args.npz_path).with_suffix("").name + "_estimator_vs_sensor_zoom.png")
    plot_zoom(args.npz_path, args.t0, args.t1, out_path)
