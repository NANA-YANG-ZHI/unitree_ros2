"""
Run the sensorless generalized-momentum contact estimator (contact_detection.py)
over a Go2 lowstate/sportmode npz pair (produced by
example/data/bag_topic_to_npz.py from a recorded rosbag2 folder) and save the
result as a single .npz file.

Usage (inside an environment with pinocchio, e.g. the osrf/ros:foxy-desktop
Docker container from docker/ with `pip3 install pin` -- no rosbag2_py/rclpy
needed, this script only reads .npz):

    python3 run_contact_estimation.py \
        /path/to/example/data/npz_data/excitation_bag_v4_lowstate.npz \
        /path/to/example/data/npz_data/excitation_bag_v4_sportmodestate.npz

Writes <bag_name>_contact_estimate.npz to plot/all_bags/ by default
(override with --out). Load it back with:

    data = np.load("excitation_bag_v4_contact_estimate.npz")
    data["t"], data["contact_states"], data["est_fz_filtered"], ...
"""

import argparse
from pathlib import Path

import numpy as np

from go2_model import DEFAULT_MJCF_PATH, DEFAULT_URDF_PATH, load_go2_model, make_go2_contact_detector
from npz_reader import read_lowstate_npz

# Matches ContactDetector.foot_names order (contact_detection.py) and the
# z-axis columns of its 12-dim est_f/est_f_filtered vectors (indices 2,5,8,11).
FOOT_NAMES = ["fl_foot", "fr_foot", "rl_foot", "rr_foot"]
FOOT_Z_INDEX = [2, 5, 8, 11]

# example/src/src/contact_estimation/run_contact_estimation.py -> repo root -> plot/all_bags
DEFAULT_OUT_DIR = Path(__file__).resolve().parents[4] / "plot" / "all_bags"


def run(lowstate_npz, sportmode_npz, out_path=None, bandwidth=30, alg="mixing", resample_freq=None,
        urdf_path=DEFAULT_URDF_PATH, mjcf_path=None):
    model, data = load_go2_model(mjcf_path=mjcf_path, urdf_path=None if mjcf_path is not None else urdf_path)
    samples = read_lowstate_npz(lowstate_npz, model, sportmode_npz_path=sportmode_npz, resample_freq=resample_freq)

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
        bag_name = Path(lowstate_npz).name.replace("_lowstate.npz", "")
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
    parser.add_argument("lowstate_npz", help="Path to a <bag>_lowstate.npz produced by bag_topic_to_npz.py")
    parser.add_argument("sportmode_npz", help="Path to a <bag>_sportmodestate.npz produced by bag_topic_to_npz.py")
    parser.add_argument("--out", default=None, help="Output .npz path (default: plot/all_bags/<bag_name>_contact_estimate.npz)")
    parser.add_argument("--bandwidth", type=float, default=30, help="Observer bandwidth in rad/s (default: 30)")
    parser.add_argument("--alg", default="mixing", choices=["hg", "sliding", "mixing"], help="Observer injection law (default: mixing)")
    parser.add_argument("--resample-freq", type=float, default=None, help="Resample frequency in Hz (default: auto-derived from the bag's /lowstate rate)")
    parser.add_argument(
        "--urdf", default=str(DEFAULT_URDF_PATH),
        help=f"Path to the Go2 URDF model to use (default, vendored copy: {DEFAULT_URDF_PATH})",
    )
    parser.add_argument(
        "--mjcf", nargs="?", const=str(DEFAULT_MJCF_PATH), default=None,
        help="Use an MJCF model instead of the default URDF. Bare "
             f"--mjcf uses the vendored copy ({DEFAULT_MJCF_PATH}); "
             "--mjcf PATH uses a different MJCF.",
    )
    args = parser.parse_args()

    run(
        args.lowstate_npz,
        args.sportmode_npz,
        out_path=args.out,
        bandwidth=args.bandwidth,
        alg=args.alg,
        resample_freq=args.resample_freq,
        urdf_path=args.urdf,
        mjcf_path=args.mjcf,
    )
