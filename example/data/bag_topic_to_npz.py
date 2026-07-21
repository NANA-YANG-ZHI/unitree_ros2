"""
Generic rosbag2 (sqlite3) -> .npz dumper: pick a single topic by name and
dump every field of every message on it to a flat dict of numpy arrays.
No message-type-specific knowledge required (unlike npz_reader.py, which
knows about LowState/SportModeState fields).

A rosbag2 recording folder is usually split into multiple storage segments
(e.g. excitation_bag_v4_0.db3, excitation_bag_v4_1.db3, ... -- listed in
the bag's metadata.yaml, one entry per time rosbag2's max bagfile size was
hit during recording). This script treats each segment as its own
independent dataset: it opens segments one at a time (not the whole bag
folder as one continuous stream), so segment boundaries become hard cuts
rather than being stitched back together.

`t` is saved as the raw absolute epoch time (seconds) of each message, NOT
zeroed to the dump's own first sample. This is deliberate: a single call
only ever sees one topic, so it has no way to know the real time offset
between e.g. /lowstate and /sportmodestate. Any zeroing/alignment across
multiple topics from the same segment must happen downstream, where both
streams are actually available together (see contact_estimation/npz_reader.py
and input_data_gen4FeLaN/generate_input_data.py, which align on the true
overlap of the two streams' absolute time ranges).

Needs rosbag2_py/rclpy/rosidl_runtime_py/PyYAML -- run inside the Docker/WSL2
ROS2 environment. Loading the resulting .npz back only needs numpy.

Usage:
    python bag_topic_to_npz.py <bag_path> <topic_name> [--out-dir out_dir]

Example:
    python bag_topic_to_npz.py ./excitation_bag_v4 /lowstate
        -> npz_data/excitation_bag_v4_0_lowstate.npz
           npz_data/excitation_bag_v4_1_lowstate.npz
           npz_data/excitation_bag_v4_2_lowstate.npz
    python bag_topic_to_npz.py ./excitation_bag_v4 /lf/sportmodestate --out-dir ./out
"""

import argparse
import re
from pathlib import Path

import numpy as np
import yaml

# example/data/bag_topic_to_npz.py -> example/data/npz_data
DEFAULT_OUT_DIR = Path(__file__).resolve().parent / "npz_data"


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


def list_segments(bag_path):
    """Return the ordered list of segment storage file paths for a rosbag2
    recording folder, per its metadata.yaml `relative_file_paths`. If
    `bag_path` doesn't contain a metadata.yaml (e.g. it's already a single
    .db3 file), it's treated as its own single segment."""
    bag_path = Path(bag_path)
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.exists():
        return [bag_path]

    with open(metadata_path) as f:
        metadata = yaml.safe_load(f)
    relative_paths = metadata["rosbag2_bagfile_information"]["relative_file_paths"]
    return [bag_path / p for p in relative_paths]


def read_topic_to_npz(segment_path, topic_name, out_path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(segment_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    if topic_name not in type_map:
        raise RuntimeError(f"Topic '{topic_name}' not found in segment '{segment_path}'. Available: {list(type_map.keys())}")

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
        raise RuntimeError(f"No messages found on topic '{topic_name}' in segment '{segment_path}'.")

    t = np.array(t)
    order_idx = np.argsort(t)
    t = t[order_idx]  # absolute epoch seconds -- see module docstring

    arrays = {"t": t}
    for k, v in fields.items():
        arrays[k] = np.array(v)[order_idx]

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out_path, **arrays)
    print(f"Saved {len(t)} samples ({t[-1]:.1f}s) from '{topic_name}' ({segment_path.name}) to {out_path}")
    print(f"  Fields: {sorted(fields.keys())}")
    return out_path


def bag_topic_to_npz(bag_path, topic_name, out_dir=None):
    """Dump `topic_name` from every segment of the rosbag2 recording at
    `bag_path` to one .npz per segment, named <segment_stem>_<safe_topic>.npz
    in `out_dir` (default: npz_data/ next to this script)."""
    out_dir = Path(out_dir) if out_dir is not None else DEFAULT_OUT_DIR
    safe_topic = re.sub(r"[^A-Za-z0-9_]+", "_", topic_name).strip("_")

    out_paths = []
    for segment_path in list_segments(bag_path):
        out_path = out_dir / f"{segment_path.stem}_{safe_topic}.npz"
        out_paths.append(read_topic_to_npz(segment_path, topic_name, out_path))
    return out_paths


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_path", help="Path to a rosbag2 folder, e.g. .../usable_data/excitation_bag_v4")
    parser.add_argument("topic_name", help="Topic to dump, e.g. /lowstate or /lf/sportmodestate")
    parser.add_argument("--out-dir", default=None, help="Output directory (default: example/data/npz_data)")
    args = parser.parse_args()

    bag_topic_to_npz(args.bag_path, args.topic_name, args.out_dir)
