"""
Empirically checks whether SportModeState.velocity is expressed in the
world (Odometry) frame or the base (body) frame.

The comments in read_motion_state.cpp and data_input_spec.md say ".velocity"
is world-frame and needs to be rotated into body frame downstream (see
bag_reader.py). This script verifies that claim from recorded data instead
of trusting the comments: SportModeState.position is unambiguously
world-frame (it's the accumulated odometry pose), so its time-derivative is
a ground-truth world-frame velocity. Comparing raw ".velocity" against that
ground truth directly (H_world) vs. after rotating body->world with the IMU
orientation (H_body) tells us which frame the raw field is actually in --
whichever hypothesis has lower error against the position-derivative wins.

Needs a "*_sportmodestate.npz" produced by bag_topic_to_npz.py (t, position,
velocity, imu_state.quaternion). No ROS2 environment required to run this
script -- only numpy/matplotlib.

Usage:
    python check_velocity_frame.py [npz_path]

Defaults to example/data/npz_data/excitation_bag_v95_sportmodestate.npz.
"""

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

DEFAULT_NPZ_PATH = (
    Path(__file__).resolve().parents[3] / "example" / "data" / "npz_data" / "excitation_bag_v95_sportmodestate.npz"
)


def quat_wxyz_to_rotmat(quat_wxyz):
    """Batched wxyz quaternion -> rotation matrix R such that v_world = R @ v_body."""
    w, x, y, z = quat_wxyz[:, 0], quat_wxyz[:, 1], quat_wxyz[:, 2], quat_wxyz[:, 3]
    R = np.empty((quat_wxyz.shape[0], 3, 3))
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


def quat_wxyz_to_yaw(quat_wxyz):
    w, x, y, z = quat_wxyz[:, 0], quat_wxyz[:, 1], quat_wxyz[:, 2], quat_wxyz[:, 3]
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)


def rmse(a, b):
    return float(np.sqrt(np.mean((a - b) ** 2)))


def check_velocity_frame(npz_path):
    data = np.load(npz_path)
    t = data["t"] - data["t"][0]  # bag_topic_to_npz.py saves absolute epoch time; zero locally for readability
    position = data["position"]
    velocity = data["velocity"]
    quat_wxyz = data["imu_state.quaternion"]

    R = quat_wxyz_to_rotmat(quat_wxyz)  # v_world = R @ v_body
    yaw = quat_wxyz_to_yaw(quat_wxyz)

    v_fd_world = np.gradient(position, t, axis=0)  # ground truth, world-frame

    v_as_world = velocity  # H_world: raw .velocity is already world-frame
    v_rot_to_world = np.einsum("nij,nj->ni", R, velocity)  # H_body: raw is body-frame, rotate body->world

    axes_xy = slice(0, 2)  # vx, vy only -- vz is mostly noise for a walking legged robot
    err_world = rmse(v_as_world[:, axes_xy], v_fd_world[:, axes_xy])
    err_body = rmse(v_rot_to_world[:, axes_xy], v_fd_world[:, axes_xy])

    corr_world = np.corrcoef(v_as_world[:, axes_xy].ravel(), v_fd_world[:, axes_xy].ravel())[0, 1]
    corr_body = np.corrcoef(v_rot_to_world[:, axes_xy].ravel(), v_fd_world[:, axes_xy].ravel())[0, 1]

    print(f"Loaded {len(t)} samples over {t[-1] - t[0]:.1f}s from {npz_path}")
    print(f"Yaw range: {np.degrees(yaw.max() - yaw.min()):.1f} deg (need this non-trivial for the two "
          f"hypotheses to be distinguishable)")
    print()
    print("Hypothesis                                            RMSE (m/s)   corr")
    print(f"  H_world: raw .velocity IS world-frame               {err_world:10.4f}   {corr_world:.4f}")
    print(f"  H_body:  raw .velocity is body-frame (rotated R@v)  {err_body:10.4f}   {corr_body:.4f}")
    print()

    if err_world < err_body:
        verdict = "WORLD"
        print(f"VERDICT: SportModeState.velocity is in the WORLD (Odometry) frame "
              f"(H_world RMSE={err_world:.4f} < H_body RMSE={err_body:.4f}).")
    else:
        verdict = "BODY"
        print(f"VERDICT: SportModeState.velocity is in the BODY (base) frame "
              f"(H_body RMSE={err_body:.4f} < H_world RMSE={err_world:.4f}).")

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 8))

    axes[0].plot(t, v_fd_world[:, 0], "k-", linewidth=1.2, label="d(position)/dt (ground truth, world)")
    axes[0].plot(t, v_as_world[:, 0], color="C0", alpha=0.8, label="raw .velocity (H_world)")
    axes[0].plot(t, v_rot_to_world[:, 0], color="C3", alpha=0.8, label="R @ .velocity (H_body)")
    axes[0].set_ylabel("vx (m/s)")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.4)

    axes[1].plot(t, v_fd_world[:, 1], "k-", linewidth=1.2, label="d(position)/dt (ground truth, world)")
    axes[1].plot(t, v_as_world[:, 1], color="C0", alpha=0.8, label="raw .velocity (H_world)")
    axes[1].plot(t, v_rot_to_world[:, 1], color="C3", alpha=0.8, label="R @ .velocity (H_body)")
    axes[1].set_ylabel("vy (m/s)")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.4)

    axes[2].plot(t, np.degrees(yaw), color="C4")
    axes[2].set_ylabel("yaw (deg)")
    axes[2].set_xlabel("time (s)")
    axes[2].grid(True, alpha=0.4)

    fig.suptitle(f"Velocity frame check ({verdict} wins): {Path(npz_path).name}")
    fig.tight_layout()

    out_path = Path(npz_path).with_suffix("").name + "_velocity_frame_check.png"
    out_path = Path(__file__).resolve().parent / out_path
    fig.savefig(out_path, dpi=150)
    print(f"\nSaved {out_path}")


if __name__ == "__main__":
    npz_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_NPZ_PATH
    check_velocity_frame(npz_path)
