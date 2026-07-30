"""
Zoomed-in comparison of the fixed force estimator (est_fz_filtered) against
the raw onboard sensor (foot_force), for fl_foot/fr_foot/rl_foot stacked.
Shows a short time window so cycle-by-cycle agreement is visible, with the
contact thresholds drawn in and a strip marking where the two threshold
decisions agree/disagree. Each subplot's title reports that same
green/red agreement as a single matching-score percentage.

Optionally (--felan-npz), adds a second column of subplots showing the
FeLaN training data's contact_state (q/qd/qdd/tau_act/contact_state npz,
e.g. example/src/src/input_data_gen4FeLaN/felan_input_data/run_*.npz)
against the raw sensor threshold decision, for the same per-foot rows.

rr_foot is excluded: known-broken raw sensor.

Only needs numpy/matplotlib -- run in the plot-tools-run container.

Usage:
    python plot_estimator_vs_sensor_zoom.py [npz_path] [--t0 25] [--t1 33]
    python plot_estimator_vs_sensor_zoom.py [npz_path] --show   # interactive window instead of saving a PNG
    python plot_estimator_vs_sensor_zoom.py [npz_path] --felan-npz FELAN_NPZ

--show needs an interactive matplotlib backend (TkAgg, via python3-tk --
see plot/Dockerfile) and X11 forwarded into the container (see plot/run.sh).
"""

import argparse
import sys
from pathlib import Path

import matplotlib
if "--show" not in sys.argv:
    matplotlib.use("Agg")  # non-interactive backend for headless PNG saving
import numpy as np
import matplotlib.pyplot as plt

DEFAULT_NPZ_PATH = Path(__file__).resolve().parents[2] / "all_bags" / "excitation_bag_v95_contact_estimate.npz"

FOOT_NAMES = ["fl_foot", "fr_foot", "rl_foot"]  # rr_foot excluded: known-broken sensor
EST_COL_FOR_FOOT = {"fl_foot": 0, "fr_foot": 1, "rl_foot": 2}       # est_fz_filtered order: fl,fr,rl,rr
RAW_COL_FOR_FOOT = {"fl_foot": 1, "fr_foot": 0, "rl_foot": 3}       # foot_force order: FR,FL,RR,RL
FELAN_COL_FOR_FOOT = {"fl_foot": 0, "fr_foot": 1, "rl_foot": 2}     # contact_state order: fl,fr,rl,rr (data_input_spec.md)

EST_THRESHOLD = 25.0
SENSOR_THRESHOLD = 20.0


def _shade_agreement(ax, t_zoom, agree):
    """Shade agree/disagree spans behind a subplot's traces (green/red)."""
    change_points = np.where(np.diff(agree.astype(int)) != 0)[0] + 1
    starts = np.concatenate(([0], change_points))
    ends = np.concatenate((change_points, [len(t_zoom)]))
    for s, e in zip(starts, ends):
        color = "#d6f5d6" if agree[s] else "#f7d6d6"
        ax.axvspan(t_zoom[s], t_zoom[min(e, len(t_zoom) - 1)], color=color, zorder=0)


def plot_zoom(npz_path, t0, t1, out_path, show=False, felan_npz_path=None):
    d = np.load(npz_path, allow_pickle=True)
    t = d["t"]
    est_fz_filtered = d["est_fz_filtered"]
    foot_force = d["foot_force"]

    mask = (t >= t0) & (t <= t1)
    t_zoom = t[mask]

    felan_contact_state = None
    if felan_npz_path is not None:
        fd = np.load(felan_npz_path, allow_pickle=True)
        felan_contact_state = fd["contact_state"][mask]

    ncols = 2 if felan_contact_state is not None else 1
    fig, axes = plt.subplots(len(FOOT_NAMES), ncols, sharex=True,
                              figsize=(11 * ncols, 3.0 * len(FOOT_NAMES)), squeeze=False)

    for i, name in enumerate(FOOT_NAMES):
        est = est_fz_filtered[mask, EST_COL_FOR_FOOT[name]]
        raw = foot_force[mask, RAW_COL_FOR_FOOT[name]]
        est_on = est > EST_THRESHOLD
        raw_on = raw > SENSOR_THRESHOLD
        agree = est_on == raw_on

        ax = axes[i, 0]
        _shade_agreement(ax, t_zoom, agree)

        ax.plot(t_zoom, est, label="estimator (est_fz_filtered)", color="C0", linewidth=1.8)
        ax.plot(t_zoom, raw, label="raw sensor (foot_force)", color="C1", linewidth=1.8)
        ax.axhline(EST_THRESHOLD, color="C0", linestyle="--", linewidth=1, alpha=0.7)
        ax.axhline(SENSOR_THRESHOLD, color="C1", linestyle="--", linewidth=1, alpha=0.7)

        matching_score = 100 * np.mean(agree)  # % of samples shaded green (same agree array as the shading)
        ax.set_title(f"{name}  (matching score: {matching_score:.1f}%)")
        ax.set_ylabel("force (N)")
        ax.legend(loc="upper right", fontsize=8)
        ax.set_xlim(t_zoom[0], t_zoom[-1])

        if felan_contact_state is not None:
            fstate = felan_contact_state[:, FELAN_COL_FOR_FOOT[name]]
            fagree = fstate == raw_on

            ax2 = axes[i, 1]
            _shade_agreement(ax2, t_zoom, fagree)

            ax2.step(t_zoom, fstate.astype(int), where="post", label="contact_state (FeLaN input)",
                     color="C2", linewidth=1.8)
            ax2.step(t_zoom, raw_on.astype(int), where="post", label="raw sensor threshold",
                     color="C1", linewidth=1.2, linestyle="--", alpha=0.8)

            matching_score2 = 100 * np.mean(fagree)
            ax2.set_title(f"{name}  contact_state vs raw sensor (matching score: {matching_score2:.1f}%)")
            ax2.set_ylabel("contact")
            ax2.set_ylim(-0.2, 1.2)
            ax2.legend(loc="upper right", fontsize=8)
            ax2.set_xlim(t_zoom[0], t_zoom[-1])

    for ax in axes[-1]:
        ax.set_xlabel("time (s)")

    title = (f"Estimator vs. raw sensor, zoomed t={t0}-{t1}s: {Path(npz_path).name}\n"
             f"(green = threshold decisions agree, red = disagree; rr_foot excluded)")
    if felan_npz_path is not None:
        title += f"\nRight column: FeLaN contact_state from {Path(felan_npz_path).name}"
    fig.suptitle(title)
    fig.tight_layout()

    if show:
        plt.show()
    else:
        fig.savefig(out_path, dpi=150)
        print(f"Saved {out_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("npz_path", nargs="?", default=str(DEFAULT_NPZ_PATH))
    parser.add_argument("--t0", type=float, default=25.0, help="Zoom window start (s)")
    parser.add_argument("--t1", type=float, default=33.0, help="Zoom window end (s)")
    parser.add_argument("--out", default=None, help="Output PNG path (default: <npz_name>_estimator_vs_sensor_zoom.png)")
    parser.add_argument("--show", action="store_true", help="Open an interactive window instead of saving a PNG")
    parser.add_argument("--felan-npz", default=None,
                         help="FeLaN input-data npz (q/qd/qdd/tau_act/contact_state) to add as a second "
                              "subplot column, e.g. example/src/src/input_data_gen4FeLaN/felan_input_data/run_*.npz")
    args = parser.parse_args()

    out_path = args.out or (Path(args.npz_path).with_suffix("").name + "_estimator_vs_sensor_zoom.png")
    plot_zoom(args.npz_path, args.t0, args.t1, out_path, show=args.show, felan_npz_path=args.felan_npz)
