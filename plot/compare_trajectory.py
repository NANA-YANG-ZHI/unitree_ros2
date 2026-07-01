"""
Compare recorded rosbag data against the desired trajectory from walk_with_sin_height.

The bag must contain both:
  /lf/sportmodestate   (unitree_go/msg/SportModeState)  — actual robot state
  /walk_sin_height/desired  (geometry_msgs/msg/PointStamped) — desired x, y, z_offset

Usage (source your ROS2 workspace first):
    python3 compare_trajectory.py <path_to_bag_folder>

Example:
    python3 compare_trajectory.py walk_sin_height_bag
"""

import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

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
# 1. Read bag
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

act_ts, x_act, y_act, z_act, vx_act, yaw_act = [], [], [], [], [], []
des_ts, x_des, y_des, z_des = [], [], [], []

while reader.has_next():
    topic, data, ts_ns = reader.read_next()
    t_s = ts_ns * 1e-9

    if topic == ACTUAL_TOPIC:
        msg = deserialize_message(data, get_message(type_map[topic]))
        act_ts.append(t_s)
        x_act.append(msg.position[0])
        y_act.append(msg.position[1])
        z_act.append(msg.body_height)
        vx_act.append(msg.velocity[0])
        yaw_act.append(msg.yaw_speed)

    elif topic == DESIRED_TOPIC:
        msg = deserialize_message(data, get_message(type_map[topic]))
        des_ts.append(t_s)
        x_des.append(msg.point.x)
        y_des.append(msg.point.y)
        z_des.append(msg.point.z)

if not act_ts or not des_ts:
    print("ERROR: missing messages — did you record both topics?")
    sys.exit(1)

# Shift both to t=0 from the earliest message in the bag
t0 = min(act_ts[0], des_ts[0])
t_act = np.array(act_ts) - t0
t_des = np.array(des_ts) - t0

x_act  = np.array(x_act);  x_act  -= x_act[0]   # start at origin
y_act  = np.array(y_act);  y_act  -= y_act[0]
z_act  = np.array(z_act)
vx_act = np.array(vx_act)
yaw_act = np.array(yaw_act)

x_des = np.array(x_des)
y_des = np.array(y_des)
z_des = np.array(z_des)

print(f"Actual : {len(t_act)} messages, duration {t_act[-1]:.1f} s")
print(f"Desired: {len(t_des)} messages, duration {t_des[-1]:.1f} s")

# ----------------------------------------------------------------
# 2. Plot comparison
# ----------------------------------------------------------------
fig, axes = plt.subplots(2, 3, figsize=(16, 8))
fig.suptitle("Actual vs Desired — walk_with_sin_height", fontsize=13)

def _plot(ax, t_d, des, t_a, act, ylabel, title):
    ax.plot(t_d, des, "k--", linewidth=1.2, label="desired")
    ax.plot(t_a, act, color="steelblue", linewidth=1.0, alpha=0.85, label="actual")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.4)

_plot(axes[0, 0], t_des, x_des, t_act, x_act, "X (m)",        "Forward Position")
_plot(axes[0, 1], t_des, y_des, t_act, y_act, "Y (m)",        "Lateral Position")
_plot(axes[0, 2], t_des, z_des, t_act, z_act, "Z offset (m)", "Body Height Offset")

# Forward velocity
vx_des_interp = np.interp(t_act, t_des, np.gradient(x_des, t_des))
axes[1, 0].plot(t_act, vx_act, color="steelblue", linewidth=1.0, label="actual vx")
axes[1, 0].plot(t_des, np.gradient(x_des, t_des), "k--", linewidth=1.2, label="desired vx")
axes[1, 0].set_xlabel("Time (s)")
axes[1, 0].set_ylabel("Vx (m/s)")
axes[1, 0].set_title("Forward Velocity")
axes[1, 0].legend(fontsize=8)
axes[1, 0].grid(True, alpha=0.4)

# Error: desired − actual (interpolate desired onto actual timestamps)
x_err = np.interp(t_act, t_des, x_des) - x_act
z_err = np.interp(t_act, t_des, z_des) - z_act

axes[1, 1].plot(t_act, x_err, color="tomato")
axes[1, 1].axhline(0, color="k", linewidth=0.8)
axes[1, 1].set_xlabel("Time (s)")
axes[1, 1].set_ylabel("Error (m)")
axes[1, 1].set_title("X Position Error (desired − actual)")
axes[1, 1].grid(True, alpha=0.4)

axes[1, 2].plot(t_act, z_err, color="tomato")
axes[1, 2].axhline(0, color="k", linewidth=0.8)
axes[1, 2].set_xlabel("Time (s)")
axes[1, 2].set_ylabel("Error (m)")
axes[1, 2].set_title("Height Offset Error (desired − actual)")
axes[1, 2].grid(True, alpha=0.4)

plt.tight_layout()
fig.savefig("compare_trajectory.png", dpi=150)
print("Saved: compare_trajectory.png")

# ----------------------------------------------------------------
# 3. 3D comparison
# ----------------------------------------------------------------
fig2 = plt.figure(figsize=(9, 6))
ax3d = fig2.add_subplot(111, projection="3d")
ax3d.plot(x_des, y_des, z_des, "k--", linewidth=1.5, label="desired")
ax3d.plot(x_act, y_act, z_act, color="steelblue", linewidth=1.5, label="actual")
ax3d.set_xlabel("X (m)")
ax3d.set_ylabel("Y (m)")
ax3d.set_zlabel("Z offset (m)")
ax3d.set_title("3D Trajectory Comparison")
ax3d.legend()
plt.tight_layout()
fig2.savefig("compare_trajectory_3d.png", dpi=150)
print("Saved: compare_trajectory_3d.png")
