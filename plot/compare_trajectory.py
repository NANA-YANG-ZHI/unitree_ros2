"""
Compare recorded rosbag data against the desired commands from walk_with_sin_height.

The bag must contain:
  /lf/sportmodestate          (unitree_go/msg/SportModeState)    — actual robot state
  /walk_sin_height/desired    (geometry_msgs/msg/PointStamped)   — desired commands
    point.x = commanded vx (m/s)
    point.y = 0
    point.z = desired absolute body height (body_height_0 + offset), m

Usage (source your ROS2 workspace first):
    python3 compare_trajectory.py <path_to_bag_folder>
"""

import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print("Usage: python3 compare_trajectory.py <bag_folder>")
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
DESIRED_TOPIC = "/walk_sin_height/desired"

for topic in [ACTUAL_TOPIC, DESIRED_TOPIC]:
    if topic not in type_map:
        print(f"ERROR: topic '{topic}' not found in bag.\nAvailable: {list(type_map.keys())}")
        sys.exit(1)

print(f"Actual  topic : {ACTUAL_TOPIC}")
print(f"Desired topic : {DESIRED_TOPIC}")

act_ts, x_act, y_act, body_height_act, vx_act, yaw_act = [], [], [], [], [], []
des_ts, vx_des, height_des = [], [], []

while reader.has_next():
    topic, data, ts_ns = reader.read_next()
    t_s = ts_ns * 1e-9

    if topic == ACTUAL_TOPIC:
        msg = deserialize_message(data, get_message(type_map[topic]))
        act_ts.append(t_s)
        x_act.append(msg.position[0])
        y_act.append(msg.position[1])
        body_height_act.append(msg.body_height)
        vx_act.append(msg.velocity[0])
        yaw_act.append(msg.yaw_speed)

    elif topic == DESIRED_TOPIC:
        msg = deserialize_message(data, get_message(type_map[topic]))
        des_ts.append(t_s)
        vx_des.append(msg.point.x)
        height_des.append(msg.point.z)

if not act_ts or not des_ts:
    print("ERROR: missing messages — did you record both topics?")
    sys.exit(1)

t0 = min(act_ts[0], des_ts[0])
t_act = np.array(act_ts) - t0
t_des = np.array(des_ts) - t0

x_act           = np.array(x_act);  x_act -= x_act[0]
y_act           = np.array(y_act);  y_act -= y_act[0]
body_height_act = np.array(body_height_act)
vx_act          = np.array(vx_act)
yaw_act         = np.array(yaw_act)
vx_des          = np.array(vx_des)
height_des      = np.array(height_des)

print(f"Actual : {len(t_act)} messages, duration {t_act[-1]:.1f} s")
print(f"Desired: {len(t_des)} messages, duration {t_des[-1]:.1f} s")

# ----------------------------------------------------------------
# Plot — 3×3 grid
#   row 0: pos X | pos Y | yaw         (actual only)
#   row 1: body height | vx | [hidden] (desired + actual)
#   row 2: height error | vx error | [hidden]
# ----------------------------------------------------------------
fig, axes = plt.subplots(3, 3, figsize=(15, 10))
fig.suptitle("Actual vs Desired — walk_with_sin_height", fontsize=13)

# --- Row 0: actual-only signals ---
axes[0, 0].plot(t_act, x_act, color="steelblue")
axes[0, 0].set_title("Position X (actual)")
axes[0, 0].set_ylabel("X (m)")

axes[0, 1].plot(t_act, y_act, color="steelblue")
axes[0, 1].set_title("Position Y (actual)")
axes[0, 1].set_ylabel("Y (m)")

axes[0, 2].plot(t_act, yaw_act, color="steelblue")
axes[0, 2].set_title("Yaw Speed (actual)")
axes[0, 2].set_ylabel("rad/s")

# --- Row 1: desired + actual ---
axes[1, 0].plot(t_des, height_des,      "k--", linewidth=1.2, label="desired")
axes[1, 0].plot(t_act, body_height_act, color="steelblue", linewidth=1.0, alpha=0.85, label="actual")
axes[1, 0].set_title("Body Height (absolute)")
axes[1, 0].set_ylabel("m")
axes[1, 0].legend(fontsize=8)

axes[1, 1].plot(t_des, vx_des, "k--", linewidth=1.2, label="desired")
axes[1, 1].plot(t_act, vx_act, color="steelblue", linewidth=1.0, alpha=0.85, label="actual")
axes[1, 1].set_title("Forward Velocity")
axes[1, 1].set_ylabel("Vx (m/s)")
axes[1, 1].legend(fontsize=8)

axes[1, 2].set_visible(False)

# --- Row 2: error signals ---
height_err = np.interp(t_act, t_des, height_des) - body_height_act
axes[2, 0].plot(t_act, height_err, color="tomato")
axes[2, 0].axhline(0, color="k", linewidth=0.8)
axes[2, 0].set_title("Height Error (desired − actual)")
axes[2, 0].set_ylabel("Error (m)")

vx_err = np.interp(t_act, t_des, vx_des) - vx_act
axes[2, 1].plot(t_act, vx_err, color="tomato")
axes[2, 1].axhline(0, color="k", linewidth=0.8)
axes[2, 1].set_title("Velocity Error (desired − actual)")
axes[2, 1].set_ylabel("Error (m/s)")

axes[2, 2].set_visible(False)

for row in axes:
    for ax in row:
        if ax.get_visible():
            ax.set_xlabel("Time (s)")
            ax.grid(True, alpha=0.4)

plt.tight_layout()
fig.savefig("compare_trajectory.png", dpi=150)
print("Saved: compare_trajectory.png")
