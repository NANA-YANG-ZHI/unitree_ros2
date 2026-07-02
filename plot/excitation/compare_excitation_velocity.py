"""
Compare recorded rosbag data against the desired commands from
excitation_velocity.

How to record (two terminals, both after sourcing your ROS2 workspace):
    # terminal 1
    ros2 bag record -o excitation_velocity_bag /lf/sportmodestate /excitation_velocity/desired
    # terminal 2 (start right after, or right before, terminal 1)
    ros2 run <your_package> excitation_velocity
    # Ctrl+C both once the node prints "Done."

The bag must contain:
  /lf/sportmodestate             (unitree_go/msg/SportModeState)    — actual robot state
  /excitation_velocity/desired   (geometry_msgs/msg/PointStamped)   — desired commands
    point.x = commanded vx (m/s)
    point.y = commanded vy (m/s)
    point.z = commanded vyaw (rad/s)

Usage (source your ROS2 workspace first):
    python3 compare_excitation_velocity.py <path_to_bag_folder>
"""

import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print("Usage: python3 compare_excitation_velocity.py <bag_folder>")
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
DESIRED_TOPIC = "/excitation_velocity/desired"

for topic in [ACTUAL_TOPIC, DESIRED_TOPIC]:
    if topic not in type_map:
        print(f"ERROR: topic '{topic}' not found in bag.\nAvailable: {list(type_map.keys())}")
        sys.exit(1)

print(f"Actual  topic : {ACTUAL_TOPIC}")
print(f"Desired topic : {DESIRED_TOPIC}")

act_ts, x_act, y_act, vx_act, vy_act, vyaw_act = [], [], [], [], [], []
des_ts, vx_des, vy_des, vyaw_des = [], [], [], []

while reader.has_next():
    topic, data, ts_ns = reader.read_next()
    t_s = ts_ns * 1e-9

    if topic == ACTUAL_TOPIC:
        msg = deserialize_message(data, get_message(type_map[topic]))
        act_ts.append(t_s)
        x_act.append(msg.position[0])
        y_act.append(msg.position[1])
        vx_act.append(msg.velocity[0])
        vy_act.append(msg.velocity[1])
        vyaw_act.append(msg.yaw_speed)

    elif topic == DESIRED_TOPIC:
        msg = deserialize_message(data, get_message(type_map[topic]))
        des_ts.append(t_s)
        vx_des.append(msg.point.x)
        vy_des.append(msg.point.y)
        vyaw_des.append(msg.point.z)

if not act_ts or not des_ts:
    print("ERROR: missing messages — did you record both topics?")
    sys.exit(1)

t0 = min(act_ts[0], des_ts[0])
t_act = np.array(act_ts) - t0
t_des = np.array(des_ts) - t0

x_act    = np.array(x_act);  x_act -= x_act[0]
y_act    = np.array(y_act);  y_act -= y_act[0]
vx_act   = np.array(vx_act)
vy_act   = np.array(vy_act)
vyaw_act = np.array(vyaw_act)
vx_des   = np.array(vx_des)
vy_des   = np.array(vy_des)
vyaw_des = np.array(vyaw_des)

print(f"Actual : {len(t_act)} messages, duration {t_act[-1]:.1f} s")
print(f"Desired: {len(t_des)} messages, duration {t_des[-1]:.1f} s")

# ----------------------------------------------------------------
# Plot — 3×3 grid
#   row 0: vx | vy | vyaw                    (desired + actual)
#   row 1: vx error | vy error | vyaw error
#   row 2: pos X (actual) | pos Y (actual) | [hidden]
# ----------------------------------------------------------------
fig, axes = plt.subplots(3, 3, figsize=(15, 10))
fig.suptitle("Actual vs Desired — excitation_velocity", fontsize=13)

des_vals  = [vx_des, vy_des, vyaw_des]
act_vals  = [vx_act, vy_act, vyaw_act]
val_titles = ["Forward Velocity", "Lateral Velocity", "Yaw Rate"]
val_ylabels = ["Vx (m/s)", "Vy (m/s)", "Vyaw (rad/s)"]

for i in range(3):
    axes[0, i].plot(t_des, des_vals[i], "k--", linewidth=1.2, label="desired")
    axes[0, i].plot(t_act, act_vals[i], color="steelblue", linewidth=1.0, alpha=0.85, label="actual")
    axes[0, i].set_title(val_titles[i])
    axes[0, i].set_ylabel(val_ylabels[i])
    axes[0, i].legend(fontsize=8)

    err = np.interp(t_act, t_des, des_vals[i]) - act_vals[i]
    axes[1, i].plot(t_act, err, color="tomato")
    axes[1, i].axhline(0, color="k", linewidth=0.8)
    axes[1, i].set_title(f"{val_titles[i]} Error (desired − actual)")
    axes[1, i].set_ylabel(f"Error ({val_ylabels[i].split(' ')[-1]})")

# --- Row 2: actual position, for context ---
axes[2, 0].plot(t_act, x_act, color="steelblue")
axes[2, 0].set_title("Position X (actual)")
axes[2, 0].set_ylabel("X (m)")

axes[2, 1].plot(t_act, y_act, color="steelblue")
axes[2, 1].set_title("Position Y (actual)")
axes[2, 1].set_ylabel("Y (m)")

axes[2, 2].set_visible(False)

for row in axes:
    for ax in row:
        if ax.get_visible():
            ax.set_xlabel("Time (s)")
            ax.grid(True, alpha=0.4)

plt.tight_layout()
fig.savefig("compare_excitation_velocity.png", dpi=150)
print("Saved: compare_excitation_velocity.png")
