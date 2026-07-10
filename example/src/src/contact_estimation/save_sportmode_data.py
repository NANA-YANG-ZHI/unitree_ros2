"""
Read /lf/sportmodestate (unitree_go/msg/SportModeState) from a recorded
rosbag2 folder and save its fields to a .npz file.

Needs rosbag2_py/rclpy (see bag_reader.py) -- run this inside the
Docker/WSL2 ROS2 environment, then copy the resulting .npz wherever you
want to inspect/plot it (loading it back only needs numpy).

No Pinocchio model is needed here (unlike read_lowstate_bag): sportmode
fields don't need the joint-name -> index map, so we read the topic
directly instead of going through read_lowstate_bag.

Usage:
    python save_sportmode_data.py [bag_path] [out_path]
"""

import sys
from pathlib import Path

import numpy as np

from bag_reader import SPORTMODE_TOPIC, _open_bag

DEFAULT_BAG_PATH = Path(__file__).resolve().parents[3] / "data" / "2026_07_07" / "usable_data" / "excitation_bag_v4"


def save_sportmode_data(bag_path, out_path=None):
    bag_path = Path(bag_path)
    reader, type_map, deserialize_message, get_message = _open_bag(str(bag_path))

    if SPORTMODE_TOPIC not in type_map:
        raise RuntimeError(f"'{SPORTMODE_TOPIC}' not found in bag '{bag_path}'. Available: {list(type_map.keys())}")

    t = []
    position, velocity = [], []
    body_height, yaw_speed = [], []
    mode, gait_type, progress = [], [], []
    foot_force = []
    foot_position_body, foot_speed_body = [], []

    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        if topic != SPORTMODE_TOPIC:
            continue
        msg = deserialize_message(data, get_message(type_map[topic]))
        t.append(ts_ns * 1e-9)
        position.append(np.array(msg.position, dtype=float))
        velocity.append(np.array(msg.velocity, dtype=float))
        body_height.append(msg.body_height)
        yaw_speed.append(msg.yaw_speed)
        mode.append(msg.mode)
        gait_type.append(msg.gait_type)
        progress.append(msg.progress)
        foot_force.append(np.array(msg.foot_force, dtype=float))
        foot_position_body.append(np.array(msg.foot_position_body, dtype=float))
        foot_speed_body.append(np.array(msg.foot_speed_body, dtype=float))

    if not t:
        raise RuntimeError(f"No '{SPORTMODE_TOPIC}' messages found in bag '{bag_path}'.")

    t = np.array(t)
    order_idx = np.argsort(t)
    t = t[order_idx] - t[order_idx][0]

    if out_path is None:
        out_path = bag_path.parent / f"{bag_path.name}_sportmode.npz"

    np.savez(
        out_path,
        t=t,
        position=np.array(position)[order_idx],                    # (N,3)
        velocity=np.array(velocity)[order_idx],                    # (N,3), world/odometry frame
        body_height=np.array(body_height)[order_idx],              # (N,)
        yaw_speed=np.array(yaw_speed)[order_idx],                  # (N,)
        mode=np.array(mode)[order_idx],                            # (N,)
        gait_type=np.array(gait_type)[order_idx],                  # (N,)
        progress=np.array(progress)[order_idx],                    # (N,)
        foot_force=np.array(foot_force)[order_idx],                # (N,4), [FR,FL,RR,RL]
        foot_position_body=np.array(foot_position_body)[order_idx],  # (N,12)
        foot_speed_body=np.array(foot_speed_body)[order_idx],      # (N,12)
    )
    print(f"Saved {len(t)} samples ({t[-1]:.1f}s) to {out_path}")
    return out_path


if __name__ == "__main__":
    bag_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BAG_PATH
    out_path = sys.argv[2] if len(sys.argv) > 2 else None
    save_sportmode_data(bag_path, out_path)
