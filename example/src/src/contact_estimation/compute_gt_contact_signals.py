"""
Compare two ways of deciding "is this foot touching the ground" against real
Go2 hardware data:

1. ContactDetector.apply_contact_detection_gt(q) -- purely kinematic, from
   our own Pinocchio model (thresholds each foot's filtered world-Z height).
2. The raw onboard foot-force sensor, thresholded in Newtons.

Plus a bonus third signal, SportModeState.foot_position_body -- the robot's
own onboard estimate of each foot's position relative to its body, computed
by Unitree's firmware independent of our model.

Reads the *raw*, unprocessed per-topic dumps produced by bag_topic_to_npz.py
(not the resampled/interpolated contact_estimate.npz from run_contact_estimation.py),
so this script redoes the same parsing bag_reader.py normally does (quaternion
reorder, motor-index mapping), reusing bag_reader.py's UNITREE_MOTOR_INDEX
constant rather than re-deriving it.

Critical fix baked into building `q` here: bag_reader.py deliberately leaves
the base position q[0:3]=0 for real hardware bags (correct for the force
estimator, which is translation-invariant, but that makes
apply_contact_detection_gt's absolute-world-Z-height threshold meaningless --
verified, it gives ~-0.2m always, never near the tiny yaml thresholds, so
every foot reads "in contact" 100% of the time regardless of real gait). This
script instead sets q[2] = SportModeState.body_height (interpolated onto the
lowstate time grid), which is a real estimate of the base's height above the
ground, making the threshold physically meaningful.

RR foot is excluded entirely: its raw sensor is known broken (flat ~88-92N
throughout any trial, established via prior investigation), so any ground
truth comparison involving it would be pure noise.

Runs in the pinocchio-capable (unitree_ros2_dev) container -- no matplotlib
import at all. Saves a results .npz for a separate plotting script
(plot/contact_estimation/debug/plot_gt_contact_comparison.py, run in the
plot-tools-run container) to consume.

Usage:
    python compute_gt_contact_signals.py [lowstate_npz] [sportmode_npz] [--sensor-threshold N] [--fpb-threshold Z] [--out out.npz]
"""

import argparse
from pathlib import Path

import numpy as np

import go2_model
from bag_reader import UNITREE_MOTOR_INDEX

DEFAULT_LOWSTATE_NPZ = Path(__file__).resolve().parents[3] / "data" / "2026_07_07" / "usable_data" / "excitation_bag_v95_lowstate.npz"
DEFAULT_SPORTMODE_NPZ = Path(__file__).resolve().parents[3] / "data" / "2026_07_07" / "usable_data" / "excitation_bag_v95_sportmodestate.npz"
DEFAULT_OUT_DIR = Path(__file__).resolve().parents[4] / "plot" / "contact_estimation"

FOOT_NAMES = ["fl_foot", "fr_foot", "rl_foot"]  # rr_foot excluded: known-broken sensor
RAW_COL_FOR_FOOT = {"fl_foot": 1, "fr_foot": 0, "rl_foot": 3}  # firmware order FR,FL,RR,RL; rr(2) excluded


def build_q(low, sport, model):
    t = low["t"]
    quat_xyzw = low["imu_state.quaternion"][:, [1, 2, 3, 0]]  # wxyz -> Pinocchio xyzw
    motor_q = np.stack([low[f"motor_state.{i}.q"] for i in range(12)], axis=1)
    body_height_interp = np.interp(t, sport["t"], sport["body_height"])

    idx_maps = go2_model.build_joint_index_maps(model)
    q = np.zeros((len(t), model.nq))
    q[:, 6] = 1.0  # placeholder identity quat, overwritten below
    q[:, 3:7] = quat_xyzw
    q[:, 2] = body_height_interp
    for joint_name in idx_maps["order"]:
        q[:, idx_maps["idx_q"][joint_name]] = motor_q[:, UNITREE_MOTOR_INDEX[joint_name]]
    return q, t


def kinematic_ground_truth(model, data, q, t):
    detector = go2_model.make_go2_contact_detector(model, data, freq=1.0 / np.median(np.diff(t)))
    kin_contact = np.zeros((len(t), len(FOOT_NAMES)), dtype=bool)
    for i in range(len(t)):
        _, _, contact_states = detector.apply_contact_detection_gt(q[i])
        kin_contact[i] = [contact_states[name] for name in FOOT_NAMES]
    return kin_contact


def sensor_ground_truth(low, sport, t, threshold):
    foot_force_low = low["foot_force"]
    sensor_bool = np.stack(
        [foot_force_low[:, RAW_COL_FOR_FOOT[name]] > threshold for name in FOOT_NAMES], axis=1
    )
    print("\nOrdering sanity check (sportmode vs lowstate foot_force correlation, expect high e.g. >0.9):")
    for name, col in RAW_COL_FOR_FOOT.items():
        sport_col_interp = np.interp(t, sport["t"], sport["foot_force"][:, col])
        corr = np.corrcoef(sport_col_interp, foot_force_low[:, col])[0, 1]
        print(f"  {name}: {corr:.3f}")
    return sensor_bool


def foot_position_body_signal(sport, t, threshold):
    foot_pos_body = sport["foot_position_body"].reshape(-1, 4, 3)
    fpb_z = {
        name: np.interp(t, sport["t"], foot_pos_body[:, col, 2])
        for name, col in RAW_COL_FOR_FOOT.items()
    }
    print("\nfoot_position_body z (body frame, meters) per foot -- pick --fpb-threshold from these if not given:")
    for name in FOOT_NAMES:
        z = fpb_z[name]
        print(f"  {name}: min={z.min():.3f} max={z.max():.3f} mean={z.mean():.3f}")
    if threshold is None:
        threshold = float(np.mean([fpb_z[name].mean() for name in FOOT_NAMES]))
        print(f"  (no --fpb-threshold given, defaulting to mean of means: {threshold:.3f})")
    fpb_bool = np.stack([fpb_z[name] < threshold for name in FOOT_NAMES], axis=1)
    fpb_z_stacked = np.stack([fpb_z[name] for name in FOOT_NAMES], axis=1)
    return fpb_bool, fpb_z_stacked, threshold


def best_lag_agreement(a, b, max_lag):
    scores = []
    lags = range(-max_lag, max_lag + 1)
    for lag in lags:
        if lag > 0:
            aa, bb = a[lag:], b[:-lag]
        elif lag < 0:
            aa, bb = a[:lag], b[-lag:]
        else:
            aa, bb = a, b
        scores.append(100 * np.mean(aa == bb))
    best_idx = int(np.argmax(scores))
    return list(lags)[best_idx], scores[best_idx], scores[max_lag]


def report_metrics(kin_contact, sensor_bool, fpb_bool, lag_window):
    print("\nComparison metrics (fl_foot, fr_foot, rl_foot only -- rr_foot excluded, known-broken raw sensor):\n")
    for i, name in enumerate(FOOT_NAMES):
        K, S, F = kin_contact[:, i], sensor_bool[:, i], fpb_bool[:, i]
        agreement_ks = 100 * np.mean(K == S)
        agreement_kf = 100 * np.mean(K == F)
        agreement_sf = 100 * np.mean(S == F)

        both_contact = 100 * np.mean(K & S)
        both_swing = 100 * np.mean(~K & ~S)
        k_only = 100 * np.mean(K & ~S)
        s_only = 100 * np.mean(~K & S)

        kin_duty = 100 * np.mean(K)
        sensor_duty = 100 * np.mean(S)
        fpb_duty = 100 * np.mean(F)

        best_lag, best_agree, zero_lag_agree = best_lag_agreement(K, S, lag_window)

        print(f"{name}:")
        print(f"  duty cycle: kinematic={kin_duty:5.1f}%  sensor={sensor_duty:5.1f}%  foot_position_body={fpb_duty:5.1f}%")
        print(f"  agreement:  kin-vs-sensor={agreement_ks:5.1f}%  kin-vs-fpb={agreement_kf:5.1f}%  sensor-vs-fpb={agreement_sf:5.1f}%")
        print(f"  kin-vs-sensor confusion: both_contact={both_contact:5.1f}% both_swing={both_swing:5.1f}% "
              f"kin_only={k_only:5.1f}% sensor_only={s_only:5.1f}%")
        print(f"  kin-vs-sensor lag check: zero_lag={zero_lag_agree:5.1f}%  best_lag={best_lag:+d} samples ({best_agree:5.1f}%)")
        print()

    print("NOTE: rr_foot is excluded from this report entirely -- its raw sensor reads a flat ~88-92N "
          "throughout the whole trial regardless of gait (known hardware fault, established via prior "
          "investigation), so any comparison involving it would be pure noise, not signal.")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("lowstate_npz", nargs="?", default=str(DEFAULT_LOWSTATE_NPZ))
    parser.add_argument("sportmode_npz", nargs="?", default=str(DEFAULT_SPORTMODE_NPZ))
    parser.add_argument("--sensor-threshold", type=float, default=20.0,
                         help="Raw sensor force (N) above which a foot is considered in contact (default: 20.0)")
    parser.add_argument("--fpb-threshold", type=float, default=None,
                         help="foot_position_body z (m) below which a foot is considered in contact "
                              "(default: computed from data, see printed min/max/mean)")
    parser.add_argument("--lag-window", type=int, default=10,
                         help="Max +/- sample shift to search for best-lag agreement (default: 10)")
    parser.add_argument("--out", default=None, help="Output .npz path (default: plot/contact_estimation/<name>_gt_contact_comparison.npz)")
    args = parser.parse_args()

    low = np.load(args.lowstate_npz, allow_pickle=True)
    sport = np.load(args.sportmode_npz, allow_pickle=True)

    model, data = go2_model.load_go2_model(urdf_path=go2_model.DEFAULT_URDF_PATH)

    q, t = build_q(low, sport, model)
    print(f"Loaded {len(t)} lowstate samples ({t[-1]:.1f}s)")

    kin_contact = kinematic_ground_truth(model, data, q, t)
    sensor_bool = sensor_ground_truth(low, sport, t, args.sensor_threshold)
    fpb_bool, fpb_z, fpb_threshold = foot_position_body_signal(sport, t, args.fpb_threshold)

    report_metrics(kin_contact, sensor_bool, fpb_bool, args.lag_window)

    if args.out is None:
        DEFAULT_OUT_DIR.mkdir(parents=True, exist_ok=True)
        bag_name = Path(args.lowstate_npz).name.replace("_lowstate.npz", "")
        out_path = str(DEFAULT_OUT_DIR / f"{bag_name}_gt_contact_comparison.npz")
    else:
        out_path = args.out

    np.savez(
        out_path,
        t=t,
        foot_names=np.array(FOOT_NAMES),
        kin_contact=kin_contact,
        sensor_bool=sensor_bool,
        fpb_bool=fpb_bool,
        fpb_z=fpb_z,
        sensor_threshold=args.sensor_threshold,
        fpb_threshold=fpb_threshold,
    )
    print(f"\nSaved comparison data to {out_path}")


if __name__ == "__main__":
    main()
