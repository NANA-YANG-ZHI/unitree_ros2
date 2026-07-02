"""
Desired trajectory for walk_with_sin_roll node.
Parameters must match walk_with_sin_roll.cpp exactly.

Roll is the Euler() roll angle (deg) sent to the robot, range [-10, 10].
"""

import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# --- Parameters matching C++ node ---
DT             = 0.002   # s
SETTLE_TIME    = 1.0     # s
WALK_TIME      = 5.0     # s (1 full cycle)
RESTORE_TIME   = 1.0     # s
VX             = 0.1     # m/s
ROLL_CENTER_DEG = 0.0    # deg  (roll at wave center)
ROLL_AMP_DEG    = 10.0   # deg  (sin amplitude)
ROLL_PERIOD     = 5.0    # s

# --- Build trajectory ---
total_time = SETTLE_TIME + WALK_TIME + RESTORE_TIME
t = np.arange(0.0, total_time, DT)

x = np.zeros_like(t)
y = np.zeros_like(t)
roll = np.zeros_like(t)   # Euler roll (deg)

for i, ti in enumerate(t):
    if ti < SETTLE_TIME:
        x[i] = 0.0
        roll[i] = ROLL_CENTER_DEG
    elif ti < SETTLE_TIME + WALK_TIME:
        wt = ti - SETTLE_TIME
        x[i] = VX * wt
        roll[i] = ROLL_CENTER_DEG + ROLL_AMP_DEG * np.sin(2.0 * np.pi * wt / ROLL_PERIOD)
    else:
        roll_start = ROLL_CENTER_DEG + ROLL_AMP_DEG * np.sin(2.0 * np.pi * WALK_TIME / ROLL_PERIOD)
        alpha = min((ti - (SETTLE_TIME + WALK_TIME)) / RESTORE_TIME, 1.0)
        x[i] = VX * WALK_TIME
        roll[i] = roll_start + alpha * (0.0 - roll_start)

y[:] = 0.0

# --- Time-series plots ---
fig, axes = plt.subplots(1, 3, figsize=(15, 4))
fig.suptitle("Desired Trajectory — walk_with_sin_roll", fontsize=13)

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
ax.plot(t, roll, color="seagreen", label="roll")
ax.axhline(ROLL_CENTER_DEG + ROLL_AMP_DEG, color="red",  linestyle="--", linewidth=0.9,
           label=f"upper: {ROLL_CENTER_DEG+ROLL_AMP_DEG:.1f} deg")
ax.axhline(ROLL_CENTER_DEG - ROLL_AMP_DEG, color="blue", linestyle="--", linewidth=0.9,
           label=f"lower: {ROLL_CENTER_DEG-ROLL_AMP_DEG:.1f} deg")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Roll (deg)")
ax.set_title("Body Roll")
ax.legend(fontsize=7, loc="upper right")
ax.grid(True, alpha=0.4)

plt.tight_layout()
fig.savefig("walk_with_sin_roll_timeseries.png", dpi=150)
print("Saved: walk_with_sin_roll_timeseries.png")

# --- 3D trajectory (X, Y, roll angle) ---
fig2 = plt.figure(figsize=(9, 6))
ax3d = fig2.add_subplot(111, projection="3d")

# Color-code by phase
settle_mask  = t < SETTLE_TIME
walk_mask    = (t >= SETTLE_TIME) & (t < SETTLE_TIME + WALK_TIME)
restore_mask = t >= SETTLE_TIME + WALK_TIME

ax3d.plot(x[settle_mask],  y[settle_mask],  roll[settle_mask],  color="gray",       linewidth=1.5, label="settle")
ax3d.plot(x[walk_mask],    y[walk_mask],    roll[walk_mask],    color="steelblue",  linewidth=2.0, label="walk (sin roll)")
ax3d.plot(x[restore_mask], y[restore_mask], roll[restore_mask], color="darkorange", linewidth=1.5, label="restore")

ax3d.set_xlabel("X (m)")
ax3d.set_ylabel("Y (m)")
ax3d.set_zlabel("Roll (deg)")
ax3d.set_title("3D Trajectory (position + roll)")
ax3d.legend()

plt.tight_layout()
fig2.savefig("walk_with_sin_roll_3d.png", dpi=150)
print("Saved: walk_with_sin_roll_3d.png")

plt.show()
