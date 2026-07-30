# Tutorial: generating FeLaN input data and validating it against the sensor

This covers the full pipeline:

0. `example/data/bag_topic_to_npz.py` — dumps raw rosbag2 recordings down to
   the per-topic `_lowstate.npz` / `_sportmodestate.npz` pairs the rest of
   the pipeline consumes. Skip this step if `example/data/npz_data/` already
   has the pair you need.
1. `example/src/src/input_data_gen4FeLaN/generate_input_data.py` (+ `run_all_bags.sh`) —
   turns raw `_lowstate.npz` / `_sportmodestate.npz` bag dumps into the
   spec-defined FeLaN training data (`q`/`qd`/`qdd`/`tau_act`/`contact_state`)
   plus a contact-estimation result file used for validation plots.
2. `plot/contact_estimation/validation/plot_all_estimator_vs_sensor_zoom.py` —
   plots the estimator's per-foot force against the raw onboard sensor for
   every contact-estimation result, so you can eyeball how good the contact
   detection is.

These scripts run in **different containers** — steps 0/1 need `pinocchio`
and/or ROS2 (`rosbag2_py`/`rclpy`), step 2 needs a working `matplotlib`. See
"Containers" below.

## 0. Generate `example/data/npz_data/` from raw bags

Raw rosbag2 recordings live under `example/data/2026_07_07/usable_data/`
(one folder per bag, e.g. `excitation_bag_v4/`, each split into segments
`excitation_bag_v4_0.db3`, `excitation_bag_v4_1.db3`, ... per the bag's
`metadata.yaml`). `bag_topic_to_npz.py` dumps a single topic from every
segment of a bag folder to one `.npz` per segment.

This script needs `rosbag2_py`/`rclpy`/`rosidl_runtime_py`/`PyYAML`, so it
must run inside `unitree_ros2_dev` **with the ROS2/cyclonedds setup sourced**
(unlike step 1's script, this one does touch the ROS2 bag-reading stack):

```bash
docker exec -it unitree_ros2_dev bash
source /opt/ros/humble/setup.bash
source /workspace/cyclonedds_ws/install/setup.bash
cd /workspace/example/data

python3 bag_topic_to_npz.py 2026_07_07/usable_data/excitation_bag_v4 /lowstate
python3 bag_topic_to_npz.py 2026_07_07/usable_data/excitation_bag_v4 /lf/sportmodestate
```

Each call produces one `.npz` per segment:

```
npz_data/excitation_bag_v4_0_lowstate.npz
npz_data/excitation_bag_v4_1_lowstate.npz
npz_data/excitation_bag_v4_0_lf_sportmodestate.npz
npz_data/excitation_bag_v4_1_lf_sportmodestate.npz
```

Run it once per bag folder per topic (`/lowstate` and `/lf/sportmodestate`,
or `/sportmodestate` for the older bags that use that topic name instead —
check `type_map`/the error message if the topic name is wrong, it lists the
bag's available topics). There's no batch script for this step yet — each
`bag_topic_to_npz.py` call handles one bag folder + one topic (all its
segments) at a time; loop over bag folders yourself if you need to redo all
of them. `--out-dir` overrides the default `npz_data/` output location.

## 1. Generate the FeLaN input data

### Inputs

Raw per-bag npz pairs already live in `example/data/npz_data/`:

```
excitation_bag_v<run>_<split>_lowstate.npz
excitation_bag_v<run>_<split>_sportmodestate.npz   (or _lf_sportmodestate.npz)
```

These are produced upstream by `example/data/bag_topic_to_npz.py` — you don't
need to regenerate them unless you have new bags.

### Run it for a single bag

```bash
docker exec -it unitree_ros2_dev bash
cd /workspace/example/src/src/input_data_gen4FeLaN
python3 generate_input_data.py \
    ../../../data/npz_data/excitation_bag_v4_0_lowstate.npz \
    ../../../data/npz_data/excitation_bag_v4_0_lf_sportmodestate.npz
```

(Container mounts the repo root at `/workspace`, so absolute paths like
`/workspace/example/data/npz_data/...` work too.)

This writes two files:

- `felan_input_data/run_<run>_<split>.npz` — the spec-defined training data
  (`t`, `q`, `qd`, `qdd`, `tau_act`, `contact_state`, `joint_order`, `dt`).
- `plot/contact_estimation/contact_estimation_result_npz/<bag>_contact_estimate.npz`
  — same schema as `run_contact_estimation.py`'s output, used by step 2.

It also prints, per foot, the % of samples where the estimator's threshold
decision and the raw sensor's threshold decision agree (`EST_THRESHOLD=25N`
vs `SENSOR_THRESHOLD=20N` — only used for this printed report, not for the
`contact_state` written to the npz; that one uses the per-foot thresholds in
`contact_estimation/contact_param.yaml`).

Useful flags (all optional): `--out`, `--contact-out`, `--resample-freq`,
`--qdd-cutoff`, `--bandwidth`, `--alg {hg,sliding,mixing}`, `--urdf`, `--mjcf`.
Run `python3 generate_input_data.py --help` for details.

### Run it for every bag at once

```bash
docker exec -it unitree_ros2_dev bash
cd /workspace/example/src/src/input_data_gen4FeLaN
./run_all_bags.sh
```

This loops over every `*_lowstate.npz` in `example/data/npz_data/`, finds its
matching sportmode npz, and calls `generate_input_data.py` on each pair.
Note: the script's header comment assumes an Anaconda-Prompt/conda
(`hybrid_ctrl` env) workflow — inside the `unitree_ros2_dev` container the
`conda activate` line is a no-op if conda isn't installed there; if it fails
for that reason, just run the loop manually or strip the conda lines and call
`python3` directly (pinocchio already resolves in that container's default
Python). Prints a final `Done: N succeeded, M failed/skipped.` summary.

## 2. Plot estimator vs. sensor (zoomed validation plots)

### Run it

```bash
./plot/run.sh                                    # starts/attaches plot-tools-run
docker exec -it plot-tools-run bash
cd /workspace/plot/contact_estimation/validation
python plot_all_estimator_vs_sensor_zoom.py
```

Defaults: reads every `*_contact_estimate.npz` from
`plot/contact_estimation/contact_estimation_result_npz/` (produced in step 1)
and writes one `<bag>_contact_estimate_estimator_vs_sensor_zoom.png` per bag
into `plot/contact_estimation/validation/` (next to the script), zoomed to
`t ∈ [0, 30]` seconds by default.

Useful flags:

```bash
python plot_all_estimator_vs_sensor_zoom.py --t0 25 --t1 33
python plot_all_estimator_vs_sensor_zoom.py --npz-dir DIR --out-dir DIR
```

## Containers

| Script | Container | Why |
|---|---|---|
| `bag_topic_to_npz.py` | `unitree_ros2_dev` (`docker/run.sh`), **with ROS2/cyclonedds sourced** | needs `rosbag2_py`/`rclpy`/`rosidl_runtime_py` to read the raw `.db3` bag segments. |
| `generate_input_data.py`, `run_all_bags.sh` | `unitree_ros2_dev` (`docker/run.sh`) | imports `pinocchio`; the default host shell's `matplotlib` is broken but pinocchio isn't needed there. This container mounts the whole repo at `/workspace`. |
| `plot_all_estimator_vs_sensor_zoom.py` | `plot-tools-run` (`plot/run.sh`) | only needs `numpy`/`matplotlib`; the default shell's system `matplotlib` fails with `_ARRAY_API not found` (built against NumPy 1.x, NumPy 2.2.6 installed). This container mounts the whole repo at `/workspace`, same as `unitree_ros2_dev`. |

Only step 0 (`bag_topic_to_npz.py`) touches live bags and needs
ROS2/cyclonedds sourced. Steps 1 and 2 consume already-converted `.npz`
files, so they don't.

## End-to-end example (one bag)

```bash
# Step 1 -- generate + estimate (unitree_ros2_dev)
docker exec -it unitree_ros2_dev bash -c "
  cd /workspace/example/src/src/input_data_gen4FeLaN &&
  python3 generate_input_data.py \
    /workspace/example/data/npz_data/excitation_bag_v4_0_lowstate.npz \
    /workspace/example/data/npz_data/excitation_bag_v4_0_lf_sportmodestate.npz
"

# Step 2 -- plot (plot-tools-run)
docker exec -it plot-tools-run bash -c "
  cd /workspace/contact_estimation/validation &&
  python plot_all_estimator_vs_sensor_zoom.py
"
```

Output PNG: `plot/contact_estimation/validation/excitation_bag_v4_0_contact_estimate_estimator_vs_sensor_zoom.png`.
