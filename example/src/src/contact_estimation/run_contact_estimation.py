"""
Run the sensorless generalized-momentum contact estimator (contact_detection.py)
over a recorded Go2 rosbag2 folder and save the result as a single .npz file.

Usage (inside an environment with pinocchio + rosbag2_py/rclpy, e.g. the
osrf/ros:foxy-desktop Docker container from docker/ with `pip3 install pin`):

    python3 run_contact_estimation.py \
        /path/to/example/data/2026_07_07/usable_data/excitation_bag_v4

Writes <bag_path>_contact_estimate.npz to plot/contact_estimation/ by default
(override with --out). Load it back with:

    data = np.load("excitation_bag_v4_contact_estimate.npz")
    data["t"], data["contact_states"], data["est_fz_filtered"], ...
"""

import argparse
import os
from pathlib import Path

import numpy as np

from go2_model import load_go2_model, make_go2_contact_detector
from bag_reader import read_lowstate_bag

# Matches ContactDetector.foot_names order (contact_detection.py) and the
# z-axis columns of its 12-dim est_f/est_f_filtered vectors (indices 2,5,8,11).
FOOT_NAMES = ["fl_foot", "fr_foot", "rl_foot", "rr_foot"]
FOOT_Z_INDEX = [2, 5, 8, 11]

# example/src/src/contact_estimation/run_contact_estimation.py -> repo root -> plot/contact_estimation
DEFAULT_OUT_DIR = Path(__file__).resolve().parents[4] / "plot" / "contact_estimation"


def run(bag_path, out_path=None, bandwidth=30, alg="mixing", resample_freq=None, urdf_path=None):
    model, data = load_go2_model(urdf_path=urdf_path)
    samples = read_lowstate_bag(bag_path, model, resample_freq=resample_freq)

    # ContactDetector integrates its observer with a fixed dt = 1/freq --
    # this must match the bag's actual (resampled) sample spacing, not the
    # library's freq=200 default, or the observer's dynamics will be wrong.
    detector_freq = 1.0 / samples.dt
    detector = make_go2_contact_detector(model, data, bandwidth=bandwidth, freq=detector_freq, alg=alg)

    N = len(samples.t)
    est_f_history = np.zeros((N, 12))
    est_f_filtered_history = np.zeros((N, 12))
    contact_history = np.zeros((N, len(FOOT_NAMES)), dtype=bool)

    for i in range(N):
        est_f, est_f_filtered, contact_states = detector.apply_contact_detection(
            samples.q[i], samples.v[i], samples.tau[i]
        )
        est_f_history[i] = est_f
        est_f_filtered_history[i] = est_f_filtered
        contact_history[i] = [contact_states[name] for name in FOOT_NAMES]

    if out_path is None:
        bag_name = os.path.basename(os.path.normpath(bag_path))
        DEFAULT_OUT_DIR.mkdir(parents=True, exist_ok=True)
        out_path = str(DEFAULT_OUT_DIR / f"{bag_name}_contact_estimate.npz")

    np.savez(
        out_path,
        t=samples.t,
        q=samples.q,
        v=samples.v,
        tau=samples.tau,
        foot_force=samples.foot_force,          # raw onboard sensor, [FR,FL,RR,RL] order
        foot_force_est=samples.foot_force_est,  # onboard estimate, [FR,FL,RR,RL] order
        joint_order=np.array(samples.joint_order),
        dt=samples.dt,
        est_f=est_f_history,                                  # (N,12) full vector as returned by apply_contact_detection
        est_f_filtered=est_f_filtered_history,                # (N,12) filtered version
        est_fz=est_f_history[:, FOOT_Z_INDEX],                # (N,4) per-foot z-force, fl/fr/rl/rr order
        est_fz_filtered=est_f_filtered_history[:, FOOT_Z_INDEX],  # (N,4) same, filtered
        contact_states=contact_history,         # (N,4) bool, fl/fr/rl/rr order
        foot_names=np.array(FOOT_NAMES),
    )

    print(f"Saved {N} samples ({samples.t[-1]:.1f}s @ {detector_freq:.0f} Hz) to {out_path}")
    for i, name in enumerate(FOOT_NAMES):
        frac = contact_history[:, i].mean()
        print(f"  {name}: in contact {frac * 100:5.1f}% of the time")

    return out_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_path", help="Path to a rosbag2 folder, e.g. .../usable_data/excitation_bag_v4")
    parser.add_argument("--out", default=None, help="Output .npz path (default: <bag_path>_contact_estimate.npz next to the bag)")
    parser.add_argument("--bandwidth", type=float, default=30, help="Observer bandwidth in rad/s (default: 30)")
    parser.add_argument("--alg", default="mixing", choices=["hg", "sliding", "mixing"], help="Observer injection law (default: mixing)")
    parser.add_argument("--resample-freq", type=float, default=None, help="Resample frequency in Hz (default: auto-derived from the bag's /lowstate rate)")
    parser.add_argument("--urdf", default=None, help="Path to a Go2 URDF (e.g. quad-stack/robots/go2_description/urdf/go2.urdf) to use instead of the default vendored MJCF")
    args = parser.parse_args()

    run(
        args.bag_path,
        out_path=args.out,
        bandwidth=args.bandwidth,
        alg=args.alg,
        resample_freq=args.resample_freq,
        urdf_path=args.urdf,
    )
