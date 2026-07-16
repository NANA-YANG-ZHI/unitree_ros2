"""
Load /lowstate (unitree_go/msg/LowState) and /lf/sportmodestate or
/sportmodestate (unitree_go/msg/SportModeState) data from the raw per-topic
.npz dumps produced by example/data/bag_topic_to_npz.py, and produce
time-aligned q/v/tau arrays in the exact layout
contact_detection.ContactDetector.apply_contact_detection expects for a Go2
Pinocchio model (see go2_model.load_go2_model).

npz-reading is intentionally decoupled from Pinocchio model-building: this
module only needs the joint-name -> index map from go2_model.build_joint_index_maps,
no kinematics/dynamics are computed here.
"""

from dataclasses import dataclass

import numpy as np

from go2_model import build_joint_index_maps

# Motor index -> leg/joint mapping in unitree_go/msg/LowState's motor_state[]
# array (from example/src/include/common/motor_crc.h). NOTE this is a
# DIFFERENT order than the MJCF's FL,FR,RL,RR body declaration order.
UNITREE_MOTOR_INDEX = {
    "FR_hip_joint": 0, "FR_thigh_joint": 1, "FR_calf_joint": 2,
    "FL_hip_joint": 3, "FL_thigh_joint": 4, "FL_calf_joint": 5,
    "RR_hip_joint": 6, "RR_thigh_joint": 7, "RR_calf_joint": 8,
    "RL_hip_joint": 9, "RL_thigh_joint": 10, "RL_calf_joint": 11,
}


def _quat_xyzw_to_rotmat(quat_xyzw):
    """Batched xyzw quaternion -> rotation matrix R such that v_world = R @ v_body."""
    x, y, z, w = quat_xyzw[:, 0], quat_xyzw[:, 1], quat_xyzw[:, 2], quat_xyzw[:, 3]
    R = np.empty((quat_xyzw.shape[0], 3, 3))
    R[:, 0, 0] = 1 - 2 * (y**2 + z**2)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x**2 + z**2)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x**2 + y**2)
    return R


@dataclass
class Go2BagSamples:
    t: np.ndarray              # (N,) seconds, relative to first /lowstate sample
    q: np.ndarray              # (N, 19) Pinocchio configuration
    v: np.ndarray              # (N, 18) Pinocchio generalized velocity
    tau: np.ndarray            # (N, 12) actuated-joint torques, order == joint_order == v[:, 6:18]
    foot_force: np.ndarray      # (N, 4), [FR,FL,RR,RL] order -- raw foot force sensor, not required by the pipeline
    foot_force_est: np.ndarray  # (N, 4), [FR,FL,RR,RL] order -- onboard reference signal, not required by the pipeline
    joint_order: list          # length-12 MJCF joint names, matches tau's column order
    dt: float


def _load_lowstate_npz(lowstate_npz_path):
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


def _load_sportmode_npz(sportmode_npz_path):
    sport = np.load(sportmode_npz_path, allow_pickle=True)
    return sport["t"], sport["velocity"]


def read_lowstate_npz(lowstate_npz_path, model, sportmode_npz_path=None,
                       resample_freq=None, use_sportmode_velocity=True) -> Go2BagSamples:
    (lowstate_t, quat_wxyz, gyro, motor_q, motor_dq, motor_tau,
     foot_force, foot_force_est) = _load_lowstate_npz(lowstate_npz_path)

    if use_sportmode_velocity:
        if sportmode_npz_path is None:
            raise ValueError("sportmode_npz_path is required when use_sportmode_velocity=True")
        sportmode_t, base_lin_vel = _load_sportmode_npz(sportmode_npz_path)

    order_idx = np.argsort(lowstate_t)
    lowstate_t = lowstate_t[order_idx]
    quat_wxyz = quat_wxyz[order_idx]
    gyro = gyro[order_idx]
    motor_q = motor_q[order_idx]
    motor_dq = motor_dq[order_idx]
    motor_tau = motor_tau[order_idx]
    foot_force = foot_force[order_idx]
    foot_force_est = foot_force_est[order_idx]

    # bag_topic_to_npz.py zeroes each dump's `t` independently, to its own
    # first sample -- so this treats the first /lowstate and first
    # /lf/sportmodestate sample as simultaneous. Same approximation
    # compute_gt_contact_signals.py already relies on; adjacent samples are
    # only ~2ms apart so the error is negligible for a low-pass-filtered
    # contact estimator.
    if resample_freq is None:
        median_dt = np.median(np.diff(lowstate_t))
        resample_freq = np.clip(round((1.0 / median_dt) / 50.0) * 50.0, 100, 1000)
    dt = 1.0 / resample_freq
    t_grid = np.arange(0.0, lowstate_t[-1], dt)
    N = len(t_grid)

    def interp_cols(t_src, values):
        values = np.asarray(values)
        return np.stack([np.interp(t_grid, t_src, values[:, c]) for c in range(values.shape[1])], axis=1)

    quat_wxyz_r = interp_cols(lowstate_t, quat_wxyz)
    # Linear-interpolating quaternion components independently and
    # renormalizing is a simplification (proper SLERP would be more correct),
    # but adjacent ~500 Hz samples are only ~2ms apart so orientation change
    # between them is negligible.
    quat_wxyz_r /= np.linalg.norm(quat_wxyz_r, axis=1, keepdims=True)
    quat_xyzw_r = quat_wxyz_r[:, [1, 2, 3, 0]]

    gyro_r = interp_cols(lowstate_t, gyro)
    motor_q_r = interp_cols(lowstate_t, motor_q)
    motor_dq_r = interp_cols(lowstate_t, motor_dq)
    motor_tau_r = interp_cols(lowstate_t, motor_tau)
    foot_force_r = interp_cols(lowstate_t, foot_force)
    foot_force_est_r = interp_cols(lowstate_t, foot_force_est)

    if use_sportmode_velocity:
        order_idx_s = np.argsort(sportmode_t)
        sportmode_t = sportmode_t[order_idx_s]
        base_lin_vel = base_lin_vel[order_idx_s]
        base_lin_vel_world_r = interp_cols(sportmode_t, base_lin_vel)
        # SportModeState.velocity is documented (see read_motion_state.cpp) as
        # being expressed in the Odometry (world) frame, but Pinocchio's
        # free-flyer convention -- and thus ContactDetector's M/C/g and
        # momentum residual -- needs the base's linear velocity in its own
        # local (body) frame. Rotate world -> body with the IMU orientation,
        # already resampled onto the same t_grid.
        R_world_from_body = _quat_xyzw_to_rotmat(quat_xyzw_r)
        base_lin_vel_r = np.einsum("nji,nj->ni", R_world_from_body, base_lin_vel_world_r)
    else:
        base_lin_vel_r = np.zeros((N, 3))

    idx_maps = build_joint_index_maps(model)
    joint_order = idx_maps["order"]

    q = np.zeros((N, model.nq))
    v = np.zeros((N, model.nv))
    tau = np.zeros((N, 12))

    # Base position does not affect any quantity ContactDetector computes
    # (M, C, g, and body-frame Jacobians are invariant to the floating
    # base's absolute world position -- only relative joint configuration
    # and base orientation matter), so it's safely left at the origin.
    q[:, 3:7] = quat_xyzw_r  # Pinocchio quaternion is scalar-last [qx,qy,qz,qw]
    v[:, 0:3] = base_lin_vel_world_r  # body-frame linear velocity
    v[:, 3:6] = gyro_r          # body-frame angular velocity

    for col, joint_name in enumerate(joint_order):
        midx = UNITREE_MOTOR_INDEX[joint_name]
        q[:, idx_maps["idx_q"][joint_name]] = motor_q_r[:, midx]
        v[:, idx_maps["idx_v"][joint_name]] = motor_dq_r[:, midx]
        tau[:, col] = motor_tau_r[:, midx]

    return Go2BagSamples(
        t=t_grid, q=q, v=v, tau=tau,
        foot_force=foot_force_r, foot_force_est=foot_force_est_r,
        joint_order=joint_order, dt=dt,
    )
