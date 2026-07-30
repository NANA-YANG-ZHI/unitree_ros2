"""
Desired trajectory for the excitation_height node.

Reads the Fourier-series coefficients excitation_height.cpp generated at
startup (excitation_height_params.json, written to the node's working
directory) and reconstructs the exact same commanded trajectory offline with
NumPy -- the coefficients are randomized per run (see SEED in the .cpp), so
unlike walk_with_sin_height.py this cannot be replayed from hardcoded
constants.

Usage:
    python excitation_height.py [path/to/excitation_height_params.json]

Z is the BodyHeight relative offset (m) sent to the robot, clamped to
[h_min, h_max] on the robot side.
"""

import json
import sys

import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

PARAMS_FILE = sys.argv[1] if len(sys.argv) > 1 else "excitation_height_params.json"

with open(PARAMS_FILE, "r") as f:
    P = json.load(f)

ORDER   = P["order"]
DT      = P["dt"]
SETTLE_TIME  = P["settle_time"]
EXCITE_TIME  = P["excite_time"]
RESTORE_TIME = P["restore_time"]
VX      = P["vx"]
H_MIN   = P["h_min"]
H_MAX   = P["h_max"]
Q0      = np.array(P["q0"])          # (njoints,)
A       = np.array(P["A"])           # (order, njoints)
B       = np.array(P["B"])           # (order, njoints)
PHI0    = np.array(P["phi0"])        # (njoints,)


def eval_fourier(t):
    """q(t), shape (len(t), njoints). Mirrors FourierExcitation::eval()."""
    omega_f = 2.0 * np.pi / EXCITE_TIME
    t = np.atleast_1d(t)
    q = np.tile(Q0[None, :], (t.shape[0], 1)).astype(float)
    for k in range(1, ORDER + 1):
        phase = omega_f * k * t[:, None] + PHI0[None, :]
        denom = omega_f * k
        q = q + (np.sin(phase) * A[k - 1, :][None, :] / denom -
                 np.cos(phase) * B[k - 1, :][None, :] / denom)
    return q


# --- Build 3-phase trajectory ---
total_time = SETTLE_TIME + EXCITE_TIME + RESTORE_TIME
t = np.arange(0.0, total_time, DT)

x = np.zeros_like(t)
y = np.zeros_like(t)
z = np.zeros_like(t)  # BodyHeight relative offset (m)

h_end = float(eval_fourier(EXCITE_TIME)[0, 0])

settle_mask  = t < SETTLE_TIME
excite_mask  = (t >= SETTLE_TIME) & (t < SETTLE_TIME + EXCITE_TIME)
restore_mask = t >= SETTLE_TIME + EXCITE_TIME

z[settle_mask] = Q0[0]

excite_t = t[excite_mask] - SETTLE_TIME
z[excite_mask] = eval_fourier(excite_t)[:, 0]
x[excite_mask] = VX * excite_t

restore_alpha = np.clip((t[restore_mask] - (SETTLE_TIME + EXCITE_TIME)) / RESTORE_TIME, 0.0, 1.0)
z[restore_mask] = h_end + restore_alpha * (0.0 - h_end)
x[restore_mask] = VX * EXCITE_TIME

z_clamped = np.clip(z, H_MIN, H_MAX)

# --- Time-series plots ---
fig, axes = plt.subplots(2, 3, figsize=(15, 8))
fig.suptitle("Desired Trajectory — excitation_height", fontsize=13)

ax = axes[0, 0]
ax.plot(t, x, color="steelblue")
ax.set_xlabel("Time (s)")
ax.set_ylabel("X (m)")
ax.set_title("Forward Position")
ax.grid(True, alpha=0.4)

ax = axes[0, 1]
ax.plot(t, y, color="darkorange")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Y (m)")
ax.set_title("Lateral Position")
ax.set_ylim(-0.3, 0.3)
ax.grid(True, alpha=0.4)

ax = axes[0, 2]
ax.plot(t, z, color="seagreen", label="height offset (unclamped)")
ax.plot(t, z_clamped, color="tomato", linestyle="--", linewidth=0.9, label="height offset (sent to robot)")
ax.axhline(H_MAX, color="black", linestyle=":", linewidth=0.8, label=f"spec max {H_MAX} m")
ax.axhline(H_MIN, color="black", linestyle=":", linewidth=0.8, label=f"spec min {H_MIN} m")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Z offset (m)")
ax.set_title("Body Height Offset (relative)")
ax.legend(fontsize=7, loc="upper right")
ax.grid(True, alpha=0.4)

# --- Time derivatives ---
vx = np.gradient(x, DT)
vy = np.gradient(y, DT)
vz = np.gradient(z, DT)
vz_clamped = np.gradient(z_clamped, DT)

ax = axes[1, 0]
ax.plot(t, vx, color="steelblue")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Vx (m/s)")
ax.set_title("Forward Velocity")
ax.grid(True, alpha=0.4)

ax = axes[1, 1]
ax.plot(t, vy, color="darkorange")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Vy (m/s)")
ax.set_title("Lateral Velocity")
ax.grid(True, alpha=0.4)

ax = axes[1, 2]
ax.plot(t, vz, color="seagreen", label="height rate (unclamped)")
ax.plot(t, vz_clamped, color="tomato", linestyle="--", linewidth=0.9, label="height rate (sent to robot)")
ax.set_xlabel("Time (s)")
ax.set_xlim([10, 90])
ax.set_ylim([-0.1, 0.1])
ax.set_ylabel("Z rate (m/s)")
ax.set_title("Body Height Rate")
ax.legend(fontsize=7, loc="upper right")
ax.grid(True, alpha=0.4)

plt.tight_layout()
fig.savefig("excitation_height_timeseries.png", dpi=150)
print("Saved: excitation_height_timeseries.png")

# --- 3D trajectory ---
fig2 = plt.figure(figsize=(9, 6))
ax3d = fig2.add_subplot(111, projection="3d")

ax3d.plot(x[settle_mask],  y[settle_mask],  z_clamped[settle_mask],  color="gray",       linewidth=1.5, label="settle")
ax3d.plot(x[excite_mask],  y[excite_mask],  z_clamped[excite_mask],  color="steelblue",  linewidth=2.0, label="excite (fourier height)")
ax3d.plot(x[restore_mask], y[restore_mask], z_clamped[restore_mask], color="darkorange", linewidth=1.5, label="restore")

ax3d.set_xlabel("X (m)")
ax3d.set_ylabel("Y (m)")
ax3d.set_zlabel("Z (m)")
ax3d.set_title("3D Body Trajectory")
ax3d.legend()

plt.tight_layout()
fig2.savefig("excitation_height_3d.png", dpi=150)
print("Saved: excitation_height_3d.png")

plt.show()
