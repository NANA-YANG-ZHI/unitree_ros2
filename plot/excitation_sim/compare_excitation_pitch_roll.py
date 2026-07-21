"""
Compare recorded rosbag data against the desired commands from
excitation_pitch_roll.

How to record (two terminals, both after sourcing your ROS2 workspace):
    # terminal 1
    ros2 bag record -o excitation_pitch_roll_bag /lf/sportmodestate /excitation_pitch_roll/desired
    # terminal 2 (start right after, or right before, terminal 1)
    ros2 run <your_package> excitation_pitch_roll
    # Ctrl+C both once the node prints "Done."

The bag must contain:
  /lf/sportmodestate               (unitree_go/msg/SportModeState)    — actual robot state
  /excitation_pitch_roll/desired   (geometry_msgs/msg/PointStamped)   — desired commands
    point.x = commanded roll (rad)
    point.y = commanded pitch (rad)
    point.z = 0 (unused)

Actual roll/pitch come from imu_state.rpy[0]/[1] (rad).

Usage (source your ROS2 workspace first):
    python3 compare_excitation_pitch_roll.py <path_to_bag_folder>
"""

import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print("Usage: python3 compare_excitation_pitch_roll.py <bag_folder>")
    sys.exit(1)

bag_path = sys.argv[1]

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("ERROR: rosbag2_py / rclpy not found. Source your ROS2 workspace:\n"
          "  source /opt/ros/<distro>/setup.bash\n"
          "  source ~/unitree_ros2/install/setup.bash")
    sys.exit(1)

# ----------------------------------------------------------------
# Read bag
# ----------------------------------------------------------------
reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
converter_options = rosbag2_py.ConverterOptions("", "")
reader.open(storage_options, converter_options)

type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

ACTUAL_TOPIC  = "/lf/sportmodestate" if "/lf/sportmodestate" in type_map else "/sportmodestate"
DESIRED_TOPIC = "/excitation_pitch_roll/desired"

for topic in [ACTUAL_TOPIC, DESIRED_TOPIC]:
    if topic not in type_map:
        print(f"ERROR: topic '{topic}' not found in bag.\nAvailable: {list(type_map.keys())}")
        sys.exit(1)

print(f"Actual  topic : {ACTUAL_TOPIC}")
print(f"Desired topic : {DESIRED_TOPIC}")

act_ts, roll_act, pitch_act = [], [], []
des_ts, roll_des, pitch_des = [], [], []

while reader.has_next():
    topic, data, ts_ns = reader.read_next()
    t_s = ts_ns * 1e-9

    if topic == ACTUAL_TOPIC:
        msg = deserialize_message(data, get_message(type_map[topic]))
        act_ts.append(t_s)
        roll_act.append(msg.imu_state.rpy[0])
        pitch_act.append(msg.imu_state.rpy[1])

    elif topic == DESIRED_TOPIC:
        msg = deserialize_message(data, get_message(type_map[topic]))
        des_ts.append(t_s)
        roll_des.append(msg.point.x)
        pitch_des.append(msg.point.y)

if not act_ts or not des_ts:
    print("ERROR: missing messages — did you record both topics?")
    sys.exit(1)

t0 = min(act_ts[0], des_ts[0])
t_act = np.array(act_ts) - t0
t_des = np.array(des_ts) - t0

roll_act_deg  = np.rad2deg(np.array(roll_act))
pitch_act_deg = np.rad2deg(np.array(pitch_act))
roll_des_deg  = np.rad2deg(np.array(roll_des))
pitch_des_deg = np.rad2deg(np.array(pitch_des))

print(f"Actual : {len(t_act)} messages, duration {t_act[-1]:.1f} s")
print(f"Desired: {len(t_des)} messages, duration {t_des[-1]:.1f} s")

# ----------------------------------------------------------------
# Plot — 2×2 grid
#   row 0: roll | pitch             (desired + actual, deg)
#   row 1: roll error | pitch error
# ----------------------------------------------------------------
fig, axes = plt.subplots(2, 2, figsize=(11, 8))
fig.suptitle("Actual vs Desired — excitation_pitch_roll", fontsize=13)

des_vals = [roll_des_deg, pitch_des_deg]
act_vals = [roll_act_deg, pitch_act_deg]
titles = ["Roll", "Pitch"]

for i in range(2):
    axes[0, i].plot(t_des, des_vals[i], "k--", linewidth=1.2, label="desired")
    axes[0, i].plot(t_act, act_vals[i], color="steelblue", linewidth=1.0, alpha=0.85, label="actual")
    axes[0, i].set_title(titles[i])
    axes[0, i].set_ylabel(f"{titles[i]} (deg)")
    axes[0, i].legend(fontsize=8)

    err = np.interp(t_act, t_des, des_vals[i]) - act_vals[i]
    axes[1, i].plot(t_act, err, color="tomato")
    axes[1, i].axhline(0, color="k", linewidth=0.8)
    axes[1, i].set_title(f"{titles[i]} Error (desired − actual)")
    axes[1, i].set_ylabel("Error (deg)")

for row in axes:
    for ax in row:
        ax.set_xlabel("Time (s)")
        ax.grid(True, alpha=0.4)

plt.tight_layout()
fig.savefig("compare_excitation_pitch_roll.png", dpi=150)
print("Saved: compare_excitation_pitch_roll.png")
