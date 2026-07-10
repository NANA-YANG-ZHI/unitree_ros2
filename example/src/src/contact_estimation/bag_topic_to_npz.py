"""
Generic rosbag2 (sqlite3) -> .npz dumper: pick a single topic by name and
dump every field of every message on it to a flat dict of numpy arrays.
No message-type-specific knowledge required (unlike bag_reader.py /
save_sportmode_data.py, which know about LowState/SportModeState fields).

Needs rosbag2_py/rclpy/rosidl_runtime_py -- run inside the Docker/WSL2
ROS2 environment. Loading the resulting .npz back only needs numpy.

Usage:
    python bag_topic_to_npz.py <bag_path> <topic_name> [--out out.npz]

Example:
    python bag_topic_to_npz.py ./excitation_bag_v4 /lowstate
    python bag_topic_to_npz.py ./excitation_bag_v4 /lf/sportmodestate --out sportmode.npz
"""

import argparse
import re
from pathlib import Path

import numpy as np


def _flatten(prefix, value, out):
    """Recursively flattens a message_to_ordereddict() result into
    {dotted.field.name: value}, expanding arrays-of-submessages (e.g.
    LowState.motor_state[]) into one entry per index."""
    if isinstance(value, dict):
        for k, v in value.items():
            _flatten(f"{prefix}.{k}" if prefix else k, v, out)
    elif isinstance(value, (list, tuple)) and value and isinstance(value[0], dict):
        for i, v in enumerate(value):
            _flatten(f"{prefix}.{i}", v, out)
    else:
        out[prefix] = value


def read_topic_to_npz(bag_path, topic_name, out_path=None):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    if topic_name not in type_map:
        raise RuntimeError(f"Topic '{topic_name}' not found in bag '{bag_path}'. Available: {list(type_map.keys())}")

    msg_type = get_message(type_map[topic_name])
    t = []
    fields = {}  # field_name -> list of per-message values

    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, msg_type)
        flat = {}
        _flatten("", message_to_ordereddict(msg), flat)
        t.append(ts_ns * 1e-9)
        for k, v in flat.items():
            fields.setdefault(k, []).append(v)

    if not t:
        raise RuntimeError(f"No messages found on topic '{topic_name}' in bag '{bag_path}'.")

    t = np.array(t)
    order_idx = np.argsort(t)
    t = t[order_idx] - t[order_idx][0]

    arrays = {"t": t}
    for k, v in fields.items():
        arrays[k] = np.array(v)[order_idx]

    if out_path is None:
        safe_topic = re.sub(r"[^A-Za-z0-9_]+", "_", topic_name).strip("_")
        bag_path = Path(bag_path)
        out_path = bag_path.parent / f"{bag_path.name}_{safe_topic}.npz"

    np.savez(out_path, **arrays)
    print(f"Saved {len(t)} samples ({t[-1]:.1f}s) from '{topic_name}' to {out_path}")
    print(f"  Fields: {sorted(fields.keys())}")
    return out_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_path", help="Path to a rosbag2 folder, e.g. .../usable_data/excitation_bag_v4")
    parser.add_argument("topic_name", help="Topic to dump, e.g. /lowstate or /lf/sportmodestate")
    parser.add_argument("--out", default=None, help="Output .npz path (default: <bag_path>_<topic>.npz next to the bag)")
    args = parser.parse_args()

    read_topic_to_npz(args.bag_path, args.topic_name, args.out)
