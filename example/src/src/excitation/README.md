# Excitation Nodes

Four ROS 2 nodes (package `unitree_ros2_example`) that command a random,
constrained Fourier-series trajectory (`FourierExcitation`, see
`example/src/include/common/excitation.hpp`) for system identification. Each
writes the coefficients it used to an `excitation_*_params.json` file in the
current working directory on startup, so `plot/excitation/*.py` can
reconstruct and plot the exact commanded trajectory offline, or compare it
against a recorded `ros2 bag` of the actual robot response. See
`plot/excitation/README.md` for the tuning-parameter reference tables and the
plotting/comparison scripts.

All three axis-specific nodes (`excitation_height`, `excitation_pitch_roll`,
`excitation_velocity`) run one axis at a time, standalone. `excitation_combined`
runs all three simultaneously in one node/control loop.

## Nodes

| Executable | Source | Axis | njoints | Commands |
|---|---|---|---|---|
| `excitation_height` | `excitation_height.cpp` | body height offset | 1 | `BodyHeight()` + constant forward `Move()` |
| `excitation_pitch_roll` | `excitation_pitch_roll.cpp` | roll, pitch | 2 | `Euler()` |
| `excitation_velocity` | `excitation_velocity.cpp` | vx, vy, vyaw | 3 | `Move()` |
| `excitation_combined` | `excitation_combined.cpp` | height + roll/pitch + vx/vy/vyaw | 1+2+3 | `BodyHeight()` + `Euler()` + `Move()`, all per tick |

## Running / passing parameters

Every tunable (Fourier `order`, `param_range`, `seed`, `excite_time`, and
`h_center` on the height node) is a ROS 2 parameter with a compile-time
default, overridable at launch instead of rebuilding:
```
ros2 run unitree_ros2_example excitation_height --ros-args -p param_range:=[0.08] -p seed:=7
ros2 run unitree_ros2_example excitation_pitch_roll --ros-args -p param_range_deg:=[20,20]
ros2 run unitree_ros2_example excitation_velocity --ros-args -p param_range:=[0.3,0.2,0.5]
```
or with a params YAML file: `--ros-args --params-file params.yaml`.

Hard safety clamps (`H_MIN`/`H_MAX`, `CLAMP_LIMIT_DEG`,
`VX_LIMIT`/`VY_LIMIT`/`VYAW_LIMIT`) are compile-time constants on all four
nodes, on purpose — they're the backstop param overrides can't bypass.

### `excitation_combined` parameters

Each axis keeps its own `order`/`param_range`/`seed`, prefixed so they don't
collide; `excite_time` is shared since all three axes start and stop
together on one timeline (`SETTLE_TIME`/`RESTORE_TIME` are fixed constants,
not params).

| Param | Type | Default | Meaning |
|---|---|---|---|
| `height_order` | int | 3 | Fourier order, height |
| `height_param_range` | double[1] | [0.05] | height amplitude scale (m) |
| `height_seed` | uint | 42 | RNG seed, height |
| `height_h_center` | double | -0.075 | height offset to oscillate around (m) |
| `pr_order` | int | 3 | Fourier order, roll/pitch |
| `pr_param_range_deg` | double[2] | [30, 30] | roll/pitch amplitude scale (deg) |
| `pr_seed` | uint | 42 | RNG seed, roll/pitch |
| `vel_order` | int | 3 | Fourier order, velocity |
| `vel_param_range` | double[3] | [0.25, 0.15, 0.4] | vx/vy/vyaw amplitude scale (m/s, m/s, rad/s) |
| `vel_seed` | uint | 42 | RNG seed, velocity |
| `excite_time` | double | 10.0 | shared excitation duration (s), all 3 axes |

Example:
```
ros2 run unitree_ros2_example excitation_combined --ros-args \
  -p height_param_range:=[0.08] -p pr_param_range_deg:=[20.0,20.0] \
  -p vel_param_range:=[0.3,0.2,0.5] -p excite_time:=15.0 -p height_seed:=7
```

`excitation_combined` has no separate forward-walk speed on the height
axis (unlike the standalone `excitation_height`) — `Move()` is driven
entirely by the velocity excitation, since both can't independently command
the same `Move()` call at once.

## Topics

| Topic | Type | Published by |
|---|---|---|
| `/api/sport/request` | `unitree_api/msg/Request` | all 4 |
| `/excitation_height/desired` | `geometry_msgs/msg/PointStamped` (x=vx, y=0, z=absolute height) | `excitation_height`, `excitation_combined` |
| `/excitation_pitch_roll/desired` | `geometry_msgs/msg/PointStamped` (x=roll, y=pitch, z=0) | `excitation_pitch_roll`, `excitation_combined` |
| `/excitation_velocity/desired` | `geometry_msgs/msg/PointStamped` (x=vx, y=vy, z=vyaw) | `excitation_velocity`, `excitation_combined` |
| `sportmodestate` | `unitree_go/msg/SportModeState` (subscribed) | all 4 |

Record everything needed for the `plot/excitation/compare_excitation_*.py`
scripts in one bag:
```
ros2 bag record -o excitation_bag /lf/sportmodestate /excitation_height/desired /excitation_pitch_roll/desired /excitation_velocity/desired
```
