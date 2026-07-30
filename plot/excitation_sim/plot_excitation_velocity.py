"""
Desired trajectory for the excitation_velocity node.

Reads the Fourier-series coefficients excitation_velocity.cpp generated at
startup (excitation_velocity_params.json, written to the node's working
directory) and reconstructs the exact same commanded (vx, vy, vyaw) trajectory
offline with NumPy -- the coefficients are randomized per run (see SEED in the
.cpp).

The commanded body-frame velocities are also integrated (unicycle kinematics,
Euler steps of DT) into an idealized world-frame pose (x, y, yaw) so the
trajectory can be viewed as an actual spatial path rather than a curve in
velocity space. This is NOT measured robot odometry -- it's what the
trajectory *would* look like if the robot tracked vx/vy/vyaw perfectly with no
slip or dynamics lag.

Usage:
    python plot_excitation_velocity.py [path/to/excitation_velocity_params.json]
"""

import json
import sys

import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

PARAMS_FILE = sys.argv[1] if len(sys.argv) > 1 else "excitation_velocity_params.json"

with open(PARAMS_FILE, "r") as f:
    P = json.load(f)

ORDER   = P["order"]
DT      = P["dt"]
SETTLE_TIME  = P["settle_time"]
EXCITE_TIME  = P["excite_time"]
RESTORE_TIME = P["restore_time"]
VX_LIMIT    = P["vx_limit"]
VY_LIMIT    = P["vy_limit"]
VYAW_LIMIT  = P["vyaw_limit"]
Q0      = np.array(P["q0"])          # (3,) == [0, 0, 0]
A       = np.array(P["A"])           # (order, 3)
B       = np.array(P["B"])           # (order, 3)
PHI0    = np.array(P["phi0"])        # (3,)


def eval_fourier(t):
    """q(t) = [vx, vy, vyaw](t), shape (len(t), 3). Mirrors FourierExcitation::eval()."""
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

vel = np.zeros((t.shape[0], 3))  # [vx, vy, vyaw]

settle_mask  = t < SETTLE_TIME
excite_mask  = (t >= SETTLE_TIME) & (t < SETTLE_TIME + EXCITE_TIME)
restore_mask = t >= SETTLE_TIME + EXCITE_TIME

excite_t = t[excite_mask] - SETTLE_TIME
vel[excite_mask] = eval_fourier(excite_t)

vel_end = eval_fourier(EXCITE_TIME)[0]
restore_alpha = np.clip((t[restore_mask] - (SETTLE_TIME + EXCITE_TIME)) / RESTORE_TIME, 0.0, 1.0)
vel[restore_mask] = vel_end[None, :] + restore_alpha[:, None] * (0.0 - vel_end[None, :])

limits = np.array([VX_LIMIT, VY_LIMIT, VYAW_LIMIT])
vel_clamped = np.clip(vel, -limits, limits)

# --- Time-series plots ---
labels = ["vx (m/s)", "vy (m/s)", "vyaw (rad/s)"]
accel_labels = ["ax (m/s^2)", "ay (m/s^2)", "ayaw (rad/s^2)"]
colors = ["steelblue", "darkorange", "seagreen"]

fig, axes = plt.subplots(2, 3, figsize=(15, 8))
fig.suptitle("Desired Trajectory — excitation_velocity", fontsize=13)

for i in range(3):
    ax = axes[0, i]
    ax.plot(t, vel[:, i], color=colors[i], label="unclamped")
    ax.plot(t, vel_clamped[:, i], color="tomato", linestyle="--", linewidth=0.9, label="sent to robot")
    ax.axhline(limits[i], color="black", linestyle=":", linewidth=0.8, label=f"limit ±{limits[i]:.2f}")
    ax.axhline(-limits[i], color="black", linestyle=":", linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(labels[i])
    ax.set_title(labels[i])
    ax.legend(fontsize=7, loc="upper right")
    ax.grid(True, alpha=0.4)

    acc = np.gradient(vel[:, i], DT)
    acc_clamped = np.gradient(vel_clamped[:, i], DT)

    ax = axes[1, i]
    ax.plot(t, acc, color=colors[i], label="unclamped")
    ax.plot(t, acc_clamped, color="tomato", linestyle="--", linewidth=0.9, label="sent to robot")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(accel_labels[i])
    ax.set_title(accel_labels[i])
    ax.legend(fontsize=7, loc="upper right")
    ax.grid(True, alpha=0.4)

plt.tight_layout()
fig.savefig("excitation_velocity_timeseries.png", dpi=150)
print("Saved: excitation_velocity_timeseries.png")

# --- Integrate commanded body-frame velocities into a world-frame pose ---
# Unicycle kinematics, Euler steps of DT, using what's actually sent to the
# robot (vel_clamped). yaw[i] is the heading accumulated from vyaw.
x = np.zeros_like(t)
y = np.zeros_like(t)
yaw = np.zeros_like(t)
for i in range(1, t.shape[0]):
    vx_i, vy_i, vyaw_i = vel_clamped[i - 1]
    yaw[i] = yaw[i - 1] + vyaw_i * DT
    x[i] = x[i - 1] + (vx_i * np.cos(yaw[i - 1]) - vy_i * np.sin(yaw[i - 1])) * DT
    y[i] = y[i - 1] + (vx_i * np.sin(yaw[i - 1]) + vy_i * np.cos(yaw[i - 1])) * DT
yaw_deg = np.rad2deg(yaw)

# --- 3D trajectory: X, Y position + yaw (rotation) as Z ---
fig2 = plt.figure(figsize=(9, 6))
ax3d = fig2.add_subplot(111, projection="3d")

ax3d.plot(x[settle_mask],  y[settle_mask],  yaw_deg[settle_mask],
          color="gray",       linewidth=1.5, label="settle")
ax3d.plot(x[excite_mask],  y[excite_mask],  yaw_deg[excite_mask],
          color="steelblue",  linewidth=2.0, label="excite (fourier velocity)")
ax3d.plot(x[restore_mask], y[restore_mask], yaw_deg[restore_mask],
          color="darkorange", linewidth=1.5, label="restore")
          

ax3d.set_xlabel("X (m)")
ax3d.set_ylabel("Y (m)")
ax3d.set_zlabel("Yaw (deg)")
ax3d.set_title("3D Trajectory (position + heading)")
ax3d.legend()

plt.tight_layout()
fig2.savefig("excitation_velocity_3d.png", dpi=150)
print("Saved: excitation_velocity_3d.png")

# --- Top-down path with heading arrows (rotation visualization) ---
fig3, ax4 = plt.subplots(figsize=(8, 8))

ax4.plot(x[settle_mask],  y[settle_mask],  color="gray",       linewidth=1.5, label="settle")
ax4.plot(x[excite_mask],  y[excite_mask],  color="steelblue",  linewidth=2.0, label="excite (fourier velocity)")
ax4.plot(x[restore_mask], y[restore_mask], color="darkorange", linewidth=1.5, label="restore")

stride = max(1, int(round(0.5 / DT)))  # one heading arrow every ~0.5 s
ax4.quiver(x[::stride], y[::stride], np.cos(yaw[::stride]), np.sin(yaw[::stride]),
           color="black", width=0.004, label="heading")

ax4.set_xlabel("X (m)")
ax4.set_ylabel("Y (m)")
ax4.set_title("Top-Down Path with Heading (Rotation) Arrows")
ax4.legend(fontsize=8, loc="best")
ax4.grid(True, alpha=0.4)
ax4.set_aspect("equal", adjustable="box")

plt.tight_layout()
fig3.savefig("excitation_velocity_topdown.png", dpi=150)
print("Saved: excitation_velocity_topdown.png")

plt.show()
