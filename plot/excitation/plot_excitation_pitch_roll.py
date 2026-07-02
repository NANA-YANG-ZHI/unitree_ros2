"""
Desired trajectory for the excitation_pitch_roll node.

Reads the Fourier-series coefficients excitation_pitch_roll.cpp generated at
startup (excitation_pitch_roll_params.json, written to the node's working
directory) and reconstructs the exact same commanded (roll, pitch) trajectory
offline with NumPy -- the coefficients are randomized per run (see SEED in the
.cpp).

Usage:
    python excitation_pitch_roll.py [path/to/excitation_pitch_roll_params.json]

Roll/pitch are the Euler() angles (deg) sent to the robot, clamped to
±amp_limit_deg.
"""

import json
import sys

import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

PARAMS_FILE = sys.argv[1] if len(sys.argv) > 1 else "excitation_pitch_roll_params.json"

with open(PARAMS_FILE, "r") as f:
    P = json.load(f)

ORDER   = P["order"]
DT      = P["dt"]
SETTLE_TIME    = P["settle_time"]
EXCITE_TIME    = P["excite_time"]
RESTORE_TIME   = P["restore_time"]
AMP_LIMIT_DEG  = P["amp_limit_deg"]
Q0      = np.array(P["q0"])          # (2,) == [0, 0], rad
A       = np.array(P["A"])           # (order, 2)
B       = np.array(P["B"])           # (order, 2)
PHI0    = np.array(P["phi0"])        # (2,)


def eval_fourier(t):
    """q(t) = [roll, pitch](t) in rad, shape (len(t), 2). Mirrors FourierExcitation::eval()."""
    omega_f = 2.0 * np.pi / EXCITE_TIME
    t = np.atleast_1d(t)
    q = np.tile(Q0[None, :], (t.shape[0], 1)).astype(float)
    for k in range(1, ORDER + 1):
        phase = 2.0 * omega_f * k * t[:, None] + PHI0[None, :]
        denom = 2.0 * omega_f * k
        q = q + (np.sin(phase) * A[k - 1, :][None, :] / denom -
                 np.cos(phase) * B[k - 1, :][None, :] / denom)
    return q


# --- Build 3-phase trajectory ---
total_time = SETTLE_TIME + EXCITE_TIME + RESTORE_TIME
t = np.arange(0.0, total_time, DT)

rp = np.zeros((t.shape[0], 2))  # [roll, pitch], rad

settle_mask  = t < SETTLE_TIME
excite_mask  = (t >= SETTLE_TIME) & (t < SETTLE_TIME + EXCITE_TIME)
restore_mask = t >= SETTLE_TIME + EXCITE_TIME

excite_t = t[excite_mask] - SETTLE_TIME
rp[excite_mask] = eval_fourier(excite_t)

rp_end = eval_fourier(EXCITE_TIME)[0]
restore_alpha = np.clip((t[restore_mask] - (SETTLE_TIME + EXCITE_TIME)) / RESTORE_TIME, 0.0, 1.0)
rp[restore_mask] = rp_end[None, :] + restore_alpha[:, None] * (0.0 - rp_end[None, :])

limit_rad = np.deg2rad(AMP_LIMIT_DEG)
rp_clamped = np.clip(rp, -limit_rad, limit_rad)

rp_deg = np.rad2deg(rp)
rp_clamped_deg = np.rad2deg(rp_clamped)

# --- Time-series plots ---
labels = ["Roll (deg)", "Pitch (deg)"]
colors = ["seagreen", "steelblue"]

fig, axes = plt.subplots(1, 2, figsize=(11, 4))
fig.suptitle("Desired Trajectory — excitation_pitch_roll", fontsize=13)

for i, ax in enumerate(axes):
    ax.plot(t, rp_deg[:, i], color=colors[i], label="unclamped")
    ax.plot(t, rp_clamped_deg[:, i], color="tomato", linestyle="--", linewidth=0.9, label="sent to robot")
    ax.axhline(AMP_LIMIT_DEG, color="black", linestyle=":", linewidth=0.8, label=f"spec ±{AMP_LIMIT_DEG:.1f} deg")
    ax.axhline(-AMP_LIMIT_DEG, color="black", linestyle=":", linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(labels[i])
    ax.set_title(labels[i])
    ax.legend(fontsize=7, loc="upper right")
    ax.grid(True, alpha=0.4)

plt.tight_layout()
fig.savefig("excitation_pitch_roll_timeseries.png", dpi=150)
print("Saved: excitation_pitch_roll_timeseries.png")

# --- Roll-vs-pitch phase-plane plot (no forward walking in this experiment) ---
fig2, ax2 = plt.subplots(figsize=(6, 6))

ax2.plot(rp_clamped_deg[settle_mask, 0],  rp_clamped_deg[settle_mask, 1],
         color="gray",       linewidth=1.5, label="settle")
ax2.plot(rp_clamped_deg[excite_mask, 0],  rp_clamped_deg[excite_mask, 1],
         color="steelblue",  linewidth=2.0, label="excite (fourier roll/pitch)")
ax2.plot(rp_clamped_deg[restore_mask, 0], rp_clamped_deg[restore_mask, 1],
         color="darkorange", linewidth=1.5, label="restore")

ax2.set_xlabel("Roll (deg)")
ax2.set_ylabel("Pitch (deg)")
ax2.set_title("Roll vs. Pitch Phase Plane")
ax2.legend(fontsize=8)
ax2.grid(True, alpha=0.4)
ax2.set_aspect("equal", adjustable="box")

plt.tight_layout()
fig2.savefig("excitation_pitch_roll_phase.png", dpi=150)
print("Saved: excitation_pitch_roll_phase.png")

plt.show()
