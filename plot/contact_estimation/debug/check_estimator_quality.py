"""
Check the quality of the (fixed) force-based contact estimator
(ContactDetector.apply_contact_detection, via compute_jacobian_feet_combined_T)
by comparing its stored output in excitation_bag_v95_contact_estimate.npz
against the three independent ground-truth signals computed by
example/src/src/contact_estimation/compute_gt_contact_signals.py (kinematic
apply_contact_detection_gt, raw sensor threshold, and the robot's own
onboard foot_position_body threshold), saved in
excitation_bag_v95_gt_contact_comparison.npz.

Pure numpy -- no pinocchio, no matplotlib. Both inputs are already-computed
.npz files, so this can run anywhere.

rr_foot is excluded throughout (known-broken raw sensor).

Usage:
    python check_estimator_quality.py [contact_estimate_npz] [gt_comparison_npz]
"""

import sys
from pathlib import Path

import numpy as np

DEFAULT_ESTIMATE_NPZ = Path(__file__).resolve().parents[1] / "excitation_bag_v95_contact_estimate.npz"
DEFAULT_GT_NPZ = Path(__file__).resolve().parents[1] / "excitation_bag_v95_gt_contact_comparison.npz"

FOOT_NAMES = ["fl_foot", "fr_foot", "rl_foot"]  # rr_foot excluded: known-broken sensor
EST_COL_FOR_FOOT = {"fl_foot": 0, "fr_foot": 1, "rl_foot": 2}  # est npz's contact_states/est_fz order is fl,fr,rl,rr
RAW_COL_FOR_FOOT = {"fl_foot": 1, "fr_foot": 0, "rl_foot": 3}  # est npz's raw foot_force order is FR,FL,RR,RL


def nearest_index_map(t_from, t_to):
    """For each timestamp in t_to, find the index of the nearest sample in t_from."""
    idx = np.searchsorted(t_from, t_to)
    idx = np.clip(idx, 1, len(t_from) - 1)
    left, right = t_from[idx - 1], t_from[idx]
    idx -= (t_to - left) < (right - t_to)
    return idx


def best_lag_agreement(a, b, max_lag):
    lags = range(-max_lag, max_lag + 1)
    scores = []
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


def main():
    est_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_ESTIMATE_NPZ
    gt_path = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_GT_NPZ

    est = np.load(est_path, allow_pickle=True)
    gt = np.load(gt_path, allow_pickle=True)

    t_est = est["t"]
    t_gt = gt["t"]
    contact_states_est = est["contact_states"]  # (N_est, 4) fl,fr,rl,rr
    est_fz_filtered = est["est_fz_filtered"]     # (N_est, 4) fl,fr,rl,rr
    foot_force_raw = est["foot_force"]           # (N_est, 4) FR,FL,RR,RL

    kin_contact = gt["kin_contact"]  # (N_gt, 3) fl,fr,rl
    sensor_bool = gt["sensor_bool"]  # (N_gt, 3)
    fpb_bool = gt["fpb_bool"]        # (N_gt, 3)

    # Align the observer's (est-grid) signals onto the ground-truth grid via nearest-neighbor sampling
    idx = nearest_index_map(t_est, t_gt)
    print(f"Aligning {len(t_est)}-sample estimator grid onto {len(t_gt)}-sample ground-truth grid "
          f"(nearest-neighbor, max time gap {np.max(np.abs(t_est[idx]-t_gt)):.4f}s)\n")

    print("Continuous force-estimate quality (est_fz_filtered vs raw sensor force, Pearson correlation "
          "over 3-46s to exclude startup transient and the final standing segment):\n")
    mask = (t_est > 3) & (t_est < 46)
    for name in FOOT_NAMES:
        est_col = EST_COL_FOR_FOOT[name]
        raw_col = RAW_COL_FOR_FOOT[name]
        corr = np.corrcoef(est_fz_filtered[mask, est_col], foot_force_raw[mask, raw_col])[0, 1]
        print(f"  {name}: corr={corr:.3f}")

    print("\nDiscrete contact-decision quality (observer's contact_states vs each ground truth), "
          "aligned onto the ground-truth grid:\n")

    for i, name in enumerate(FOOT_NAMES):
        O = contact_states_est[idx, EST_COL_FOR_FOOT[name]].astype(bool)
        K = kin_contact[:, i]
        S = sensor_bool[:, i]
        F = fpb_bool[:, i]

        agree_ok = 100 * np.mean(O == K)
        agree_os = 100 * np.mean(O == S)
        agree_of = 100 * np.mean(O == F)

        both_contact = 100 * np.mean(O & K)
        both_swing = 100 * np.mean(~O & ~K)
        o_only = 100 * np.mean(O & ~K)
        k_only = 100 * np.mean(~O & K)

        o_duty = 100 * np.mean(O)
        k_duty = 100 * np.mean(K)

        best_lag, best_agree, zero_lag_agree = best_lag_agreement(O, K, max_lag=10)

        print(f"{name}:")
        print(f"  duty cycle: observer={o_duty:5.1f}%  kinematic_gt={k_duty:5.1f}%")
        print(f"  agreement:  observer-vs-kinematic={agree_ok:5.1f}%  observer-vs-sensor={agree_os:5.1f}%  observer-vs-fpb={agree_of:5.1f}%")
        print(f"  observer-vs-kinematic confusion: both_contact={both_contact:5.1f}% both_swing={both_swing:5.1f}% "
              f"observer_only={o_only:5.1f}% kinematic_only={k_only:5.1f}%")
        print(f"  observer-vs-kinematic lag check: zero_lag={zero_lag_agree:5.1f}%  best_lag={best_lag:+d} samples ({best_agree:5.1f}%)")
        print()

    print("For context, recall the ground truths agree with EACH OTHER (kinematic vs sensor) at roughly "
          "96.5% (fl), 81.5% (fr), 87.5% (rl) -- comparing the observer's numbers above against these gives "
          "a sense of whether the estimator is close to that same hardware-noise ceiling, or meaningfully worse.")
    print("\nrr_foot excluded throughout: known-broken raw sensor.")


if __name__ == "__main__":
    main()
