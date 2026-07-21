"""
Regression test for compute_jacobian_feet_combined_T (contact_detection.py).

Originally written to diagnose a tilt-triggered bias in the Go2 contact-force
estimator: black-box testing showed a spurious force appearing whenever the
base was tilted, which ruled out an IMU-quaternion sign bug and pointed at
the hand-rolled spatial-transform code that used to live in
compute_jacobian_feet_combined_T (a CoM-lever-arm-plus-double-spatial-transform
detour). That code has been replaced with the direct, provably-correct
formula: the generalized force from a pure contact force F applied at the
foot is J_lin(foot)^T @ F (fundamental force/velocity Jacobian duality -- no
lever arm needed, since re-expressing F in trunk-frame axes only requires a
rotation, not moving the point of application).

This script now just checks that against two independent references at an
identity, a flat, and a heavily tilted real sample, plus a rotation-angle
sweep to confirm the error no longer grows with tilt.

Needs pinocchio (run in the unitree_ros2_dev container, not plot-tools-run).

Usage:
    python verify_contact_jacobian.py [npz_path]
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "example" / "src" / "src" / "contact_estimation"))
import pinocchio as pino
import contact_detection as cd
from go2_model import load_go2_model

DEFAULT_NPZ_PATH = Path(__file__).resolve().parents[1] / "excitation_bag_v95_contact_estimate.npz"
FOOT_NAMES = ["fl_foot", "fr_foot", "rl_foot", "rr_foot"]


def ref_jacobian_world(model, data, q):
    """Independent reference: per foot, world-aligned linear Jacobian
    transposed directly -- no dependency on contact_detection.py at all."""
    blocks = []
    for name in FOOT_NAMES:
        fid = model.getFrameId(name)
        J6 = pino.getFrameJacobian(model, data, fid, pino.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        blocks.append(J6[0:3, :].T)
    return np.hstack(blocks)


def prep(model, data, q, v=None):
    if v is None:
        v = np.zeros(model.nv)
    pino.computeJointJacobians(model, data, q)
    pino.forwardKinematics(model, data, q, v)
    pino.updateFramePlacements(model, data)


def rpy_deg(R):
    roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
    pitch = np.degrees(np.arcsin(np.clip(-R[2, 0], -1, 1)))
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    return roll, pitch, yaw


def quat_from_axis_angle(axis, angle_rad):
    axis = np.array(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    s = np.sin(angle_rad / 2)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, np.cos(angle_rad / 2)])


def check(model, data, q, v, label):
    prep(model, data, q, v)
    R_body = data.oMf[model.getFrameId("body")].rotation
    roll, pitch, yaw = rpy_deg(R_body)

    Js = cd.compute_jacobian_feet_combined_T(model, data, q)
    ref_world = ref_jacobian_world(model, data, q)
    # Js is defined in trunk-frame axes (force f_trunk), one 3-column block
    # per foot; rotating each block back to world axes must match the
    # independent world-frame reference exactly.
    Js_world = np.hstack([Js[:, 3 * i:3 * i + 3] @ R_body.T for i in range(4)])
    err = np.abs(Js_world - ref_world).max()
    print(f"  {label:16s} roll={roll:6.1f} pitch={pitch:6.1f} yaw={yaw:6.1f}  max|err|={err:.2e}")
    return err


def main():
    npz_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_NPZ_PATH
    model, data = load_go2_model()

    d = np.load(npz_path, allow_pickle=True)
    t, q_all, v_all = d["t"], d["q"], d["v"]
    idx_flat = int(np.searchsorted(t, 0.0))
    idx_tilt = int(np.searchsorted(t, 12.0))

    q_identity = q_all[idx_flat].copy()
    q_identity[3:7] = [0.0, 0.0, 0.0, 1.0]

    print("[Real samples] Js @ R_body^T must equal the independent world-frame reference:\n")
    errs = []
    errs.append(check(model, data, q_identity, v_all[idx_flat], "q_identity"))
    errs.append(check(model, data, q_all[idx_flat].copy(), v_all[idx_flat], "q_flat (t=0)"))
    errs.append(check(model, data, q_all[idx_tilt].copy(), v_all[idx_tilt], "q_tilt (t=12)"))
    print()

    print("[Rotation sweep] error must stay ~0 as tilt angle grows (no more angle-dependent bias):\n")
    q_base = q_all[idx_flat].copy()
    for axis_name, axis in [("roll(x)", [1, 0, 0]), ("pitch(y)", [0, 1, 0]), ("yaw(z)", [0, 0, 1])]:
        for angle_deg in [0, 10, 45, 90]:
            q = q_base.copy()
            q[3:7] = quat_from_axis_angle(axis, np.radians(angle_deg))
            errs.append(check(model, data, q, v_all[idx_flat], f"{axis_name} {angle_deg}deg"))

    print()
    max_err = max(errs)
    print(f"Overall max|err| across all checks: {max_err:.2e}")
    print("PASS" if max_err < 1e-8 else "FAIL")


if __name__ == "__main__":
    main()
