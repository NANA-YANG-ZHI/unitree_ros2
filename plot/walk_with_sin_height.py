"""
Desired trajectory for walk_with_sin_height node.
Parameters must match walk_with_sin_height.cpp exactly.

Z is the BodyHeight relative offset (m) sent to the robot, range [-0.18, 0.03].
"""

import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# --- Parameters matching C++ node ---
DT           = 0.002   # s
SETTLE_TIME  = 2.0     # s
WALK_TIME    = 5.0     # s (1 full cycle)
RESTORE_TIME = 2.0     # s
VX           = 0.1     # m/s
H_CENTER     = -0.075  # m  (BodyHeight offset at wave center)
H_AMP        = 0.06    # m  (sin amplitude)
H_PERIOD     = 5.0     # s

# Height bounds from Unitree spec
H_MIN = -0.18
H_MAX =  0.03

# --- Build trajectory ---
total_time = SETTLE_TIME + WALK_TIME + RESTORE_TIME
t = np.arange(0.0, total_time, DT)

x = np.zeros_like(t)
y = np.zeros_like(t)
z = np.zeros_like(t)   # BodyHeight relative offset (m)

for i, ti in enumerate(t):
    if ti < SETTLE_TIME:
        x[i] = 0.0
        z[i] = H_CENTER
    elif ti < SETTLE_TIME + WALK_TIME:
        wt = ti - SETTLE_TIME
        x[i] = VX * wt
        z[i] = H_CENTER + H_AMP * np.sin(2.0 * np.pi * wt / H_PERIOD)
    else:
        h_start = H_CENTER + H_AMP * np.sin(2.0 * np.pi * WALK_TIME / H_PERIOD)
        alpha = min((ti - (SETTLE_TIME + WALK_TIME)) / RESTORE_TIME, 1.0)
        x[i] = VX * WALK_TIME
        z[i] = h_start + alpha * (0.0 - h_start)

y[:] = 0.0

# --- Time-series plots ---
fig, axes = plt.subplots(1, 3, figsize=(15, 4))
fig.suptitle("Desired Trajectory — walk_with_sin_height", fontsize=13)

ax = axes[0]
ax.plot(t, x, color="steelblue")
ax.set_xlabel("Time (s)")
ax.set_ylabel("X (m)")
ax.set_title("Forward Position")
ax.grid(True, alpha=0.4)

ax = axes[1]
ax.plot(t, y, color="darkorange")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Y (m)")
ax.set_title("Lateral Position")
ax.set_ylim(-0.3, 0.3)
ax.grid(True, alpha=0.4)

ax = axes[2]
ax.plot(t, z, color="seagreen", label="height offset")
ax.axhline(H_CENTER + H_AMP, color="red",  linestyle="--", linewidth=0.9, label=f"upper: {H_CENTER+H_AMP:.3f} m")
ax.axhline(H_CENTER - H_AMP, color="blue", linestyle="--", linewidth=0.9, label=f"lower: {H_CENTER-H_AMP:.3f} m")
ax.axhline(H_MAX, color="black", linestyle=":", linewidth=0.8, label=f"spec max {H_MAX} m")
ax.axhline(H_MIN, color="black", linestyle=":", linewidth=0.8, label=f"spec min {H_MIN} m")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Z offset (m)")
ax.set_title("Body Height Offset (relative)")
ax.legend(fontsize=7, loc="upper right")
ax.grid(True, alpha=0.4)

plt.tight_layout()
fig.savefig("walk_with_sin_height_timeseries.png", dpi=150)
print("Saved: walk_with_sin_height_timeseries.png")

# --- 3D trajectory ---
fig2 = plt.figure(figsize=(9, 6))
ax3d = fig2.add_subplot(111, projection="3d")

# Color-code by phase
settle_mask = t < SETTLE_TIME
walk_mask   = (t >= SETTLE_TIME) & (t < SETTLE_TIME + WALK_TIME)
restore_mask = t >= SETTLE_TIME + WALK_TIME

ax3d.plot(x[settle_mask],  y[settle_mask],  z[settle_mask],  color="gray",      linewidth=1.5, label="settle")
ax3d.plot(x[walk_mask],    y[walk_mask],    z[walk_mask],    color="steelblue", linewidth=2.0, label="walk (sin height)")
ax3d.plot(x[restore_mask], y[restore_mask], z[restore_mask], color="darkorange",linewidth=1.5, label="restore")

ax3d.set_xlabel("X (m)")
ax3d.set_ylabel("Y (m)")
ax3d.set_zlabel("Z (m)")
ax3d.set_title("3D Body Trajectory")
ax3d.legend()

plt.tight_layout()
fig2.savefig("walk_with_sin_height_3d.png", dpi=150)
print("Saved: walk_with_sin_height_3d.png")

plt.show()
