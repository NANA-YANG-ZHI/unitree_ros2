"""
Compare recorded actual robot state (sportmodestate) against the desired
excitation commands (excitation_velocity / excitation_height /
excitation_pitch_roll `/desired` topics), from the .npz files produced by
example/data/bag_topic_to_npz.py + example/data/convert_bags_to_npz.sh
(e.g. example/data/npz_data/2026_07_21/).

Plots six signals in a 2x3 grid, actual (solid) vs desired (dashed, only
drawn when the matching desired .npz exists for this bag+segment stem):
  row 0: Vx, Vy, Vyaw              (sportmodestate.velocity/yaw_speed vs.
                                     excitation_velocity/desired)
  row 1: body height, roll, pitch  (sportmodestate.body_height/imu_state.rpy
                                     vs. excitation_height/desired,
                                     excitation_pitch_roll/desired)

Only needs numpy/matplotlib -- run in the plot-tools-run container.

Usage:
    python plot_excitation_vis.py <bag_stem> [--npz-dir DIR] [--out OUT.png] [--show]

Example:
    python plot_excitation_vis.py excitation_bag_00_0
        -> loads example/data/npz_data/2026_07_21/excitation_bag_00_0_sportmodestate.npz
           + the matching *_excitation_{velocity,height,pitch_roll}_desired.npz
        -> saves excitation_bag_00_0_excitation_vis.png
"""

import argparse
import sys
from pathlib import Path

import matplotlib
if "--show" not in sys.argv:
    matplotlib.use("Agg")  # non-interactive backend for headless PNG saving
import numpy as np
import matplotlib.pyplot as plt

# plot/excitation_vis/plot_excitation_vis.py -> example/data/npz_data/2026_07_21
DEFAULT_NPZ_DIR = Path(__file__).resolve().parents[2] / "example" / "data" / "npz_data" / "2026_07_27"


def _load_desired(npz_dir, stem, suffix):
    path = Path(npz_dir) / f"{stem}_{suffix}.npz"
    if not path.exists():
        return None
    return np.load(path, allow_pickle=True)


def plot_excitation_vis(stem, npz_dir, out_path, show=False):
    npz_dir = Path(npz_dir)
    act_path = npz_dir / f"{stem}_sportmodestate.npz"
    if not act_path.exists():
        raise FileNotFoundError(f"No sportmodestate npz for stem '{stem}' in {npz_dir}")
    act = np.load(act_path, allow_pickle=True)

    vel_des = _load_desired(npz_dir, stem, "excitation_velocity_desired")
    height_des = _load_desired(npz_dir, stem, "excitation_height_desired")
    pr_des = _load_desired(npz_dir, stem, "excitation_pitch_roll_desired")
    for name, des in [("velocity", vel_des), ("height", height_des), ("pitch_roll", pr_des)]:
        if des is None:
            print(f"  (no excitation_{name}_desired npz for '{stem}' -- plotting actual only)")

    # `t` in each npz is absolute epoch time (see bag_topic_to_npz.py); since
    # these all come from the same recorded segment, they share one time
    # base -- zero to the earliest sample across whichever files exist.
    t_act = act["t"]
    t0 = min([t_act[0]] + [d["t"][0] for d in (vel_des, height_des, pr_des) if d is not None])
    t_act_rel = t_act - t0

    vx_act = act["velocity"][:, 0]
    vy_act = act["velocity"][:, 1]
    vyaw_act = act["yaw_speed"]
    height_act = act["body_height"]
    roll_act_deg = np.rad2deg(act["imu_state.rpy"][:, 0])
    pitch_act_deg = np.rad2deg(act["imu_state.rpy"][:, 1])

    fig, axes = plt.subplots(2, 3, figsize=(15, 7))
    fig.suptitle(f"Actual vs Desired — {stem}", fontsize=13)

    def plot_signal(ax, act_vals, title, ylabel, des, des_key, deg=False):
        ax.plot(t_act_rel, act_vals, color="steelblue", linewidth=1.0, alpha=0.9, label="actual")
        if des is not None:
            t_des_rel = des["t"] - t0
            des_vals = des[des_key]
            if deg:
                des_vals = np.rad2deg(des_vals)
            ax.plot(t_des_rel, des_vals, "k--", linewidth=1.2, label="desired")
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.set_xlabel("Time (s)")
        ax.grid(True, alpha=0.4)
        ax.legend(fontsize=8)

    plot_signal(axes[0, 0], vx_act, "Forward Velocity", "Vx (m/s)", vel_des, "point.x")
    plot_signal(axes[0, 1], vy_act, "Lateral Velocity", "Vy (m/s)", vel_des, "point.y")
    plot_signal(axes[0, 2], vyaw_act, "Yaw Rate", "Vyaw (rad/s)", vel_des, "point.z")
    plot_signal(axes[1, 0], height_act, "Body Height (absolute)", "Height (m)", height_des, "point.z")
    plot_signal(axes[1, 1], roll_act_deg, "Roll", "Roll (deg)", pr_des, "point.x", deg=True)
    plot_signal(axes[1, 2], pitch_act_deg, "Pitch", "Pitch (deg)", pr_des, "point.y", deg=True)

    fig.tight_layout()

    if show:
        plt.show()
    else:
        fig.savefig(out_path, dpi=150)
        print(f"Saved {out_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("stem", help="Bag+segment stem, e.g. excitation_bag_00_0 (matches <stem>_sportmodestate.npz)")
    parser.add_argument("--npz-dir", default=str(DEFAULT_NPZ_DIR), help="Directory holding the *_sportmodestate.npz / *_desired.npz files")
    parser.add_argument("--out", default=None, help="Output PNG path (default: <stem>_excitation_vis.png)")
    parser.add_argument("--show", action="store_true", help="Open an interactive window instead of saving a PNG")
    args = parser.parse_args()

    out_path = args.out or f"{args.stem}_excitation_vis.png"
    plot_excitation_vis(args.stem, args.npz_dir, out_path, show=args.show)
