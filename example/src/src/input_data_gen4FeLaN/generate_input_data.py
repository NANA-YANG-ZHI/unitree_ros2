"""
Main pipeline: reads a Go2 bag's <bag>_lowstate.npz / <bag>_sportmodestate.npz
pair (produced by example/data/bag_topic_to_npz.py), assembles the q/qd/qdd/
tau_act layout documented in example/data_input_spec.md, runs the sensorless
generalized-momentum contact estimator (contact_estimation/contact_detection.py)
over it to get contact_state, and prints a matching-score comparison against
the raw onboard foot-force sensor -- same methodology as
plot/contact_estimation/validation/plot_estimator_vs_sensor_zoom.py's
per-subplot title (threshold both signals, report % samples where the two
threshold decisions agree).

Frame conventions (see example/data_input_spec.md):
  - q  = [base_linear_pos(3, world/odom frame, SportModeState.position),
          base_ang_pos(4, quaternion xyzw), joint_pos(12)]
  - qd = [base_linear_vel(3, BODY frame -- SportModeState.velocity is used
          as-is, NOT rotated; empirically confirmed already body-frame by
          check_velocity_frame.py),
          base_ang_vel(3, body frame, imu_state.gyroscope), joint_vel(12)]
  - qdd = finite_diff + low-pass of qd (see finite_diff_filter.py) --
          Go2 does not report usable acceleration directly.

Not populated (see data_input_spec.md): Torque Cmd, Total Ext Torque (tau fb)
-- neither /lowstate nor /sportmodestate carries these.

Writes two separate .npz files:
  1. The spec-defined training data (q, qd, qdd, tau_act) -- to
     example/src/src/input_data_gen4FeLaN/felan_input_data/ by default
     (override with --out).
  2. The contact-estimation result (per-foot force estimate + contact_states,
     same schema as contact_estimation/run_contact_estimation.py so the
     existing plot/contact_estimation/*.py plotting scripts work unmodified
     against it) -- to plot/contact_estimation/contact_estimation_result_npz/
     by default (override with --contact-out).

Usage (inside an environment with pinocchio, e.g. the osrf/ros:foxy-desktop
Docker container from docker/ with `pip3 install pin`):

    python3 generate_input_data.py \
        /path/to/example/data/npz_data/excitation_bag_v9_0_lowstate.npz \
        /path/to/example/data/npz_data/excitation_bag_v9_0_sportmodestate.npz
"""

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# Reuse the Go2 Pinocchio model / joint-index map / contact detector /
# motor-index map already built for contact_estimation, rather than
# duplicating them here.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "contact_estimation"))

from go2_model import DEFAULT_MJCF_PATH, DEFAULT_URDF_PATH, build_joint_index_maps, load_go2_model, make_go2_contact_detector  # noqa: E402
from npz_reader import UNITREE_MOTOR_INDEX  # noqa: E402

from finite_diff_filter import DEFAULT_LOWPASS_FREQ, compute_qdd

FOOT_NAMES = ["fl_foot", "fr_foot", "rl_foot", "rr_foot"]
FOOT_Z_INDEX = {"fl_foot": 2, "fr_foot": 5, "rl_foot": 8, "rr_foot": 11}
RAW_COL_FOR_FOOT = {"fl_foot": 1, "fr_foot": 0, "rl_foot": 3, "rr_foot": 2}  # firmware order FR,FL,RR,RL

# rr_foot's raw sensor is known broken (flat ~88-92N regardless of gait, see
# contact_estimation/compute_gt_contact_signals.py) -- excluded from the
# matching-score report, same as the validation scripts.
MATCHING_SCORE_FEET = ["fl_foot", "fr_foot", "rl_foot"]
EST_THRESHOLD = 25.0
SENSOR_THRESHOLD = 20.0

# example/src/src/input_data_gen4FeLaN/generate_input_data.py -> felan_input_data (sibling folder)
DEFAULT_FELAN_OUT_DIR = Path(__file__).resolve().parent / "felan_input_data"

# bag names are "excitation_bag_v<run>_<split>" (see example/data/npz_data/) --
# run_<id>.npz only wants the "<run>_<split>" identifier, e.g. "4_0", not the
# full recording name.
_BAG_PREFIX_RE = re.compile(r"^excitation_bag_v")


def _run_id(bag_name):
    return _BAG_PREFIX_RE.sub("", bag_name)
# example/src/src/input_data_gen4FeLaN/generate_input_data.py -> repo root -> plot/contact_estimation/contact_estimation_result_npz
DEFAULT_CONTACT_OUT_DIR = Path(__file__).resolve().parents[4] / "plot" / "contact_estimation" / "contact_estimation_result_npz"


@dataclass
class FelanSamples:
    t: np.ndarray               # (N,) seconds, relative to first /lowstate sample
    q: np.ndarray                # (N, 19) [base_linear_pos(3), base_ang_pos_xyzw(4), joint_pos(12)]
    v: np.ndarray                # (N, 18) [base_linear_vel(3, body), base_ang_vel(3, body), joint_vel(12)]
    qdd: np.ndarray              # (N, 18) finite-diff + low-pass of v
    tau: np.ndarray              # (N, 12) tau_act, order == joint_order == v[:, 6:18]
    foot_force: np.ndarray       # (N, 4), [FR,FL,RR,RL] -- raw onboard sensor
    foot_force_est: np.ndarray  # (N, 4), [FR,FL,RR,RL] -- onboard estimate
    joint_order: list           # length-12 joint names, matches tau's column order
    dt: float


def _load_lowstate_fields(lowstate_npz_path):
    low = np.load(lowstate_npz_path, allow_pickle=True)
    t = low["t"]
    quat_wxyz = low["imu_state.quaternion"]
    gyro = low["imu_state.gyroscope"]
    motor_q = np.stack([low[f"motor_state.{i}.q"] for i in range(12)], axis=1)
    motor_dq = np.stack([low[f"motor_state.{i}.dq"] for i in range(12)], axis=1)
    motor_tau = np.stack([low[f"motor_state.{i}.tau_est"] for i in range(12)], axis=1)
    foot_force = low["foot_force"]
    foot_force_est = low["foot_force_est"]
    return t, quat_wxyz, gyro, motor_q, motor_dq, motor_tau, foot_force, foot_force_est


def _load_sportmode_fields(sportmode_npz_path):
    sport = np.load(sportmode_npz_path, allow_pickle=True)
    return sport["t"], sport["position"], sport["velocity"]


def load_felan_samples(lowstate_npz_path, sportmode_npz_path, model,
                        resample_freq=None, qdd_cutoff_hz=DEFAULT_LOWPASS_FREQ) -> FelanSamples:
    (lowstate_t, quat_wxyz, gyro, motor_q, motor_dq, motor_tau,
     foot_force, foot_force_est) = _load_lowstate_fields(lowstate_npz_path)
    sportmode_t, base_pos_world, base_vel_body = _load_sportmode_fields(sportmode_npz_path)

    order_idx = np.argsort(lowstate_t)
    lowstate_t = lowstate_t[order_idx]
    quat_wxyz = quat_wxyz[order_idx]
    gyro = gyro[order_idx]
    motor_q = motor_q[order_idx]
    motor_dq = motor_dq[order_idx]
    motor_tau = motor_tau[order_idx]
    foot_force = foot_force[order_idx]
    foot_force_est = foot_force_est[order_idx]

    order_idx_s = np.argsort(sportmode_t)
    sportmode_t = sportmode_t[order_idx_s]
    base_pos_world = base_pos_world[order_idx_s]
    base_vel_body = base_vel_body[order_idx_s]

    # bag_topic_to_npz.py zeroes each dump's `t` independently to its own
    # first sample, so this treats the first /lowstate and first
    # /sportmodestate sample as simultaneous (same approximation
    # contact_estimation/npz_reader.py relies on; adjacent samples are only
    # ~2ms apart so the error is negligible).
    if resample_freq is None:
        median_dt = np.median(np.diff(lowstate_t))
        resample_freq = np.clip(round((1.0 / median_dt) / 50.0) * 50.0, 100, 1000)
    dt = 1.0 / resample_freq
    t_grid = np.arange(0.0, lowstate_t[-1], dt)
    N = len(t_grid)

    def interp(t_src, values):
        values = np.asarray(values)
        return np.stack([np.interp(t_grid, t_src, values[:, c]) for c in range(values.shape[1])], axis=1)

    quat_wxyz_r = interp(lowstate_t, quat_wxyz)
    # Linear-interpolating quaternion components independently and
    # renormalizing is a simplification (proper SLERP would be more
    # correct), but adjacent ~500 Hz samples are only ~2ms apart so the
    # orientation change between them is negligible.
    quat_wxyz_r /= np.linalg.norm(quat_wxyz_r, axis=1, keepdims=True)
    quat_xyzw_r = quat_wxyz_r[:, [1, 2, 3, 0]]  # Pinocchio wants scalar-last

    gyro_r = interp(lowstate_t, gyro)
    motor_q_r = interp(lowstate_t, motor_q)
    motor_dq_r = interp(lowstate_t, motor_dq)
    motor_tau_r = interp(lowstate_t, motor_tau)
    foot_force_r = interp(lowstate_t, foot_force)
    foot_force_est_r = interp(lowstate_t, foot_force_est)

    base_pos_world_r = interp(sportmode_t, base_pos_world)
    # NOT rotated: SportModeState.velocity is used as-is. See module
    # docstring / data_input_spec.md -- empirically confirmed to already be
    # body-frame, unlike contact_estimation/npz_reader.py's read_lowstate_npz
    # which rotates it under the (contradicted) world-frame assumption.
    base_vel_body_r = interp(sportmode_t, base_vel_body)

    idx_maps = build_joint_index_maps(model)
    joint_order = idx_maps["order"]

    q = np.zeros((N, model.nq))
    v = np.zeros((N, model.nv))
    tau = np.zeros((N, 12))

    q[:, 0:3] = base_pos_world_r  # base_linear_pos (world/odom frame)
    q[:, 3:7] = quat_xyzw_r       # base_ang_pos (quaternion, xyzw)
    v[:, 0:3] = base_vel_body_r   # base_linear_vel (body frame)
    v[:, 3:6] = gyro_r            # base_ang_vel (body frame)

    for col, joint_name in enumerate(joint_order):
        midx = UNITREE_MOTOR_INDEX[joint_name]
        q[:, idx_maps["idx_q"][joint_name]] = motor_q_r[:, midx]
        v[:, idx_maps["idx_v"][joint_name]] = motor_dq_r[:, midx]
        tau[:, col] = motor_tau_r[:, midx]

    qdd = compute_qdd(v, resample_freq, lowpass_freq=qdd_cutoff_hz)

    return FelanSamples(
        t=t_grid, q=q, v=v, qdd=qdd, tau=tau,
        foot_force=foot_force_r, foot_force_est=foot_force_est_r,
        joint_order=joint_order, dt=dt,
    )


def run_contact_estimation(samples: FelanSamples, model, data, bandwidth=30, alg="mixing"):
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

    return est_f_history, est_f_filtered_history, contact_history


def print_matching_scores(samples: FelanSamples, est_f_filtered_history):
    """Threshold the estimator's filtered z-force and the raw sensor
    independently, then report % of samples where the two threshold
    decisions agree -- same methodology as the per-subplot titles in
    plot/contact_estimation/validation/plot_estimator_vs_sensor_zoom.py."""
    print(f"\nContact-estimator vs. raw foot-force-sensor matching score "
          f"(EST_THRESHOLD={EST_THRESHOLD:.0f}N, SENSOR_THRESHOLD={SENSOR_THRESHOLD:.0f}N):")
    for name in MATCHING_SCORE_FEET:
        est = est_f_filtered_history[:, FOOT_Z_INDEX[name]]
        raw = samples.foot_force[:, RAW_COL_FOR_FOOT[name]]
        est_on = est > EST_THRESHOLD
        raw_on = raw > SENSOR_THRESHOLD
        matching_score = 100 * np.mean(est_on == raw_on)
        print(f"  {name}: {matching_score:5.1f}%")
    print("  (rr_foot excluded: raw sensor known broken, see contact_estimation/compute_gt_contact_signals.py)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("lowstate_npz", help="Path to a <bag>_lowstate.npz produced by bag_topic_to_npz.py")
    parser.add_argument("sportmode_npz", help="Path to a <bag>_sportmodestate.npz produced by bag_topic_to_npz.py")
    parser.add_argument("--out", default=None, help="Path for the spec-defined data .npz (default: felan_input_data/run_<bag_name>.npz)")
    parser.add_argument("--contact-out", default=None, help="Path for the contact-estimation result .npz (default: plot/contact_estimation/contact_estimation_result_npz/<bag_name>_contact_estimate.npz)")
    parser.add_argument("--resample-freq", type=float, default=None, help="Resample frequency in Hz (default: auto-derived from the bag's /lowstate rate)")
    parser.add_argument("--qdd-cutoff", type=float, default=DEFAULT_LOWPASS_FREQ, help=f"Low-pass cutoff (Hz) for finite-diff acceleration (default: {DEFAULT_LOWPASS_FREQ})")
    parser.add_argument("--bandwidth", type=float, default=30, help="Contact-observer bandwidth in rad/s (default: 30)")
    parser.add_argument("--alg", default="mixing", choices=["hg", "sliding", "mixing"], help="Contact-observer injection law (default: mixing)")
    parser.add_argument("--urdf", default=str(DEFAULT_URDF_PATH), help=f"Path to the Go2 URDF model (default: {DEFAULT_URDF_PATH})")
    parser.add_argument("--mjcf", nargs="?", const=str(DEFAULT_MJCF_PATH), default=None,
                         help="Use an MJCF model instead of the default URDF.")
    args = parser.parse_args()

    model, data = load_go2_model(mjcf_path=args.mjcf, urdf_path=None if args.mjcf is not None else args.urdf)
    samples = load_felan_samples(
        args.lowstate_npz, args.sportmode_npz, model,
        resample_freq=args.resample_freq, qdd_cutoff_hz=args.qdd_cutoff,
    )

    est_f, est_f_filtered, contact_states = run_contact_estimation(
        samples, model, data, bandwidth=args.bandwidth, alg=args.alg,
    )

    print_matching_scores(samples, est_f_filtered)

    bag_name = Path(args.lowstate_npz).name.replace("_lowstate.npz", "")

    if args.out is None:
        DEFAULT_FELAN_OUT_DIR.mkdir(parents=True, exist_ok=True)
        felan_out_path = str(DEFAULT_FELAN_OUT_DIR / f"run_{_run_id(bag_name)}.npz")
    else:
        felan_out_path = args.out

    if args.contact_out is None:
        DEFAULT_CONTACT_OUT_DIR.mkdir(parents=True, exist_ok=True)
        contact_out_path = str(DEFAULT_CONTACT_OUT_DIR / f"{bag_name}_contact_estimate.npz")
    else:
        contact_out_path = args.contact_out

    # File 1: exactly the spec-defined data_input_spec.md layout (q/qd/qdd/tau_act/contact_state).
    np.savez(
        felan_out_path,
        t=samples.t,
        q=samples.q,
        qd=samples.v,
        qdd=samples.qdd,
        tau_act=samples.tau,
        contact_state=contact_states,                   # (N,4) bool, fl/fr/rl/rr order (see FOOT_NAMES)
        joint_order=np.array(samples.joint_order),
        dt=samples.dt,
    )
    print(f"\nSaved {len(samples.t)} samples ({samples.t[-1]:.1f}s @ {1.0 / samples.dt:.0f} Hz) "
          f"of spec-defined data to {felan_out_path}")

    # File 2: contact-estimation result -- same key names as
    # contact_estimation/run_contact_estimation.py so the existing
    # plot/contact_estimation/*.py plotting scripts work unmodified against it.
    zaxis_index = [FOOT_Z_INDEX[name] for name in FOOT_NAMES]
    np.savez(
        contact_out_path,
        t=samples.t,
        q=samples.q,
        v=samples.v,
        tau=samples.tau,
        foot_force=samples.foot_force,                # raw onboard sensor, [FR,FL,RR,RL] order
        foot_force_est=samples.foot_force_est,        # onboard estimate, [FR,FL,RR,RL] order
        joint_order=np.array(samples.joint_order),
        dt=samples.dt,
        est_f=est_f,                                   # (N,12) full vector as returned by apply_contact_detection
        est_f_filtered=est_f_filtered,                # (N,12) filtered version
        est_fz=est_f[:, zaxis_index],                  # (N,4) per-foot z-force, fl/fr/rl/rr order
        est_fz_filtered=est_f_filtered[:, zaxis_index],  # (N,4) same, filtered
        contact_states=contact_states,                # (N,4) bool, fl/fr/rl/rr order
        foot_names=np.array(FOOT_NAMES),
    )
    print(f"Saved contact-estimation result to {contact_out_path}")
    for i, name in enumerate(FOOT_NAMES):
        frac = contact_states[:, i].mean()
        print(f"  {name}: in contact {frac * 100:5.1f}% of the time")
    print("\nNOTE: Torque Cmd and Total Ext Torque (tau fb) are not populated -- "
          "neither is present on /lowstate or /sportmodestate (see data_input_spec.md).")


if __name__ == "__main__":
    main()
