"""
Standalone re-implementation of FourierExcitation::generate_random_param
(NumPy, matching the C++ port's math in example/src/src/common/excitation.cpp)
to produce sample excitation_*_params.json files for dry-running the
plot/excitation_*.py scripts without needing to build/run the ROS2 nodes.

The values below (order, param_range, duration, q0, seed, timing, limits)
mirror the constexpr defaults at the top of excitation_height.cpp /
excitation_velocity.cpp / excitation_pitch_roll.cpp. If you change those
constants on the C++ side, update them here too to keep the dry-run plots
representative -- or just run the real node once and plot its actual
excitation_*_params.json instead.

Usage:
    python gen_sample_params.py
"""
import json
import numpy as np


def generate_constraints(order):
    cA = np.ones(order)
    row0 = np.array([1.0 / (i + 1) for i in range(order)])
    row1 = np.array([float(i + 1) for i in range(order)])
    diff = row0 - row1
    pivot = diff[1]
    cB = np.array([diff / pivot, row1])
    return cA, cB


def generate_random_param(order, njoints, param_range, seed):
    rng = np.random.default_rng(seed)
    cA, cB = generate_constraints(order)

    A = np.zeros((order, njoints))
    B = np.zeros((order, njoints))
    for j in range(njoints):
        param_norm = param_range[j] * 0.5 / order
        A[:, j] = rng.uniform(-1, 1, order) * param_norm
        B[:, j] = rng.uniform(-1, 1, order) * param_norm

    for j in range(njoints):
        A[-1, j] = -np.dot(cA[:-1], A[:-1, j])
        B[1, j] = -np.dot(cB[0][2:], B[2:, j])
        B[0, j] = -np.dot(cB[1][1:], B[1:, j])

    phi0 = rng.uniform(-np.pi/3, np.pi/3, njoints)
    return A, B, phi0


def write_params(path, order, njoints, param_range, duration, q0, seed, extra):
    A, B, phi0 = generate_random_param(order, njoints, param_range, seed)
    j = {
        "order": order,
        "njoints": njoints,
        "param_range": list(param_range),
        "duration": duration,
        "q0": list(q0),
        "A": A.tolist(),
        "B": B.tolist(),
        "phi0": phi0.tolist(),
    }
    j.update(extra)
    with open(path, "w") as f:
        json.dump(j, f, indent=2)
    print("wrote", path)


if __name__ == "__main__":
    # Matches excitation_height.cpp
    write_params(
        "excitation_height_params.json", order=3, njoints=1, param_range=[0.05],
        duration=35.0, q0=[-0.075], seed=42,
        extra={"dt": 0.002, "settle_time": 1.0, "excite_time": 35.0, "restore_time": 2.0,
               "vx": 0.1, "h_min": -0.18, "h_max": 0.03},
    )

    # Matches excitation_velocity.cpp
    write_params(
        "excitation_velocity_params.json", order=3, njoints=3, param_range=[0.12, 0.07, 0.2],
        duration=70.0, q0=[0.0, 0.0, 0.0], seed=42,
        extra={"dt": 0.002, "settle_time": 1.0, "excite_time": 70.0, "restore_time": 2.0,
               "vx_limit": 0.4, "vy_limit": 0.25, "vyaw_limit": 0.8},
    )

    # Matches excitation_pitch_roll.cpp
    param_range_rad = np.deg2rad(25.0)
    write_params(
        "excitation_pitch_roll_params.json", order=3, njoints=2,
        param_range=[param_range_rad, param_range_rad],
        duration=70.0, q0=[0.0, 0.0], seed=42,
        extra={"dt": 0.002, "settle_time": 1.0, "excite_time": 70.0, "restore_time": 1.0,
               "param_range_deg": 25.0, "clamp_limit_deg": 30.0},
    )
