# Excitation Parameters

Current tuning for the three `FourierExcitation`-based nodes in
`example/src/src/excitation/`. Each node uses `order=3` Fourier harmonics and
`seed=42` (change `SEED` in the `.cpp` for a different random "tryout" of the
same statistical envelope).

Recap of what each knob controls:
- **`param_range`** — raw amplitude scale for the randomly drawn Fourier
  coefficients, *before* the zero-position/velocity/acceleration constraint
  and the `1/(2*omega_f*k)` synthesis attenuation. Not a hard bound — the
  actual swing of `q(t)` comes out well below `param_range` (roughly ~25% of
  it with `order=3`, `duration=10s`).
- **`duration`** (== `EXCITE_TIME` here) — period of the fundamental
  frequency. Longer `duration` also makes the actual swing *bigger* for the
  same `param_range` (shrinks `omega_f`, which shrinks the attenuation
  denominator).
- **`q0`** — constant offset the (zero-mean) excitation oscillates around.
- **hard clamp** — safety backstop applied to the final commanded value,
  independent of `param_range`/`duration`, so a bad random draw can't exceed
  hardware limits.

## `excitation_height.cpp` (1D: body height offset)

| Param | Value |
|---|---|
| `njoints` | 1 |
| `param_range` | 0.05 m |
| `q0` (center, `H_CENTER`) | -0.075 m |
| `duration` / `EXCITE_TIME` | 10.0 s |
| hard clamp | [-0.18, 0.03] m (Unitree spec) |
| `SETTLE_TIME` / `RESTORE_TIME` | 1.0 s / 2.0 s |
| forward walk `VX` | 0.1 m/s (during excite phase) |

## `excitation_velocity.cpp` (3D: vx, vy, vyaw)

| Param | Value |
|---|---|
| `njoints` | 3 |
| `param_range` | vx: 0.25 m/s, vy: 0.15 m/s, vyaw: 0.4 rad/s |
| `q0` | [0, 0, 0] |
| `duration` / `EXCITE_TIME` | 10.0 s |
| hard clamp | vx: ±0.4 m/s, vy: ±0.25 m/s, vyaw: ±0.8 rad/s |
| Unitree spec (for reference) | vx: -2.5~3.8 m/s, vy: ±1.0 m/s, vyaw: ±4 rad/s — current clamp is still conservative vs. spec |
| `SETTLE_TIME` / `RESTORE_TIME` | 1.0 s / 2.0 s |

## `excitation_pitch_roll.cpp` (2D: roll, pitch)

| Param | Value |
|---|---|
| `njoints` | 2 |
| `param_range` (`PARAM_RANGE_DEG`) | 30° (roll and pitch) |
| `q0` | [0, 0] |
| `duration` / `EXCITE_TIME` | 10.0 s |
| hard clamp (`CLAMP_LIMIT_DEG`) | ±20° |
| Unitree spec (for reference) | roll/pitch: ±0.75 rad (≈±43°) — current clamp is still conservative vs. spec |
| `SETTLE_TIME` / `RESTORE_TIME` | 1.0 s / 1.0 s |
| actual observed swing (seed=42) | ~±7.5° |

## Files in this folder

- `plot_excitation_height.py`, `plot_excitation_velocity.py`,
  `plot_excitation_pitch_roll.py` — reconstruct and plot the *desired*
  trajectory from a node's `excitation_*_params.json` (no ROS2/robot needed).
- `compare_excitation_height.py`, `compare_excitation_velocity.py`,
  `compare_excitation_pitch_roll.py` — plot desired vs. actual from a
  recorded `ros2 bag` (needs `rosbag2_py`/`rclpy`, see each script's
  docstring for the exact `ros2 bag record` command).
- `gen_sample_params.py` — dry-run helper that regenerates sample
  `excitation_*_params.json` files (matching the `.cpp` constants above)
  without needing to build/run the ROS2 nodes.
