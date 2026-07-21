#!/usr/bin/env bash
# Batch-convert every topic of every bag under a bags directory (default:
# example/data/2026_07_21) to .npz via bag_topic_to_npz.py, using only
# segments *_0.db3 and *_1.db3 of each bag (segments _2 and beyond are
# skipped).
#
# Needs rosbag2_py/rclpy/rosidl_runtime_py/PyYAML -- run inside the
# unitree_ros2_dev container with ROS2/cyclonedds sourced:
#
#   docker exec -it unitree_ros2_dev bash
#   source /opt/ros/humble/setup.bash
#   source /workspace/cyclonedds_ws/install/setup.bash
#   /workspace/example/data/convert_bags_to_npz.sh
#
# Usage:
#   ./convert_bags_to_npz.sh [bags_dir] [out_dir]
#     bags_dir  default: example/data/2026_07_21 (sibling of this script)
#     out_dir   default: example/data/npz_data   (sibling of this script)
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAGS_DIR="${1:-$SCRIPT_DIR/2026_07_21}"
OUT_DIR="${2:-$SCRIPT_DIR/npz_data/2026_07_21}"
DUMP_SCRIPT="$SCRIPT_DIR/bag_topic_to_npz.py"

mkdir -p "$OUT_DIR"

ok_count=0
fail_count=0
skip_count=0

for bag_dir in "$BAGS_DIR"/*/; do
    [ -d "$bag_dir" ] || continue
    bag_name="$(basename "$bag_dir")"
    metadata="$bag_dir/metadata.yaml"
    if [ ! -e "$metadata" ]; then
        echo "== $bag_name: SKIP (no metadata.yaml) =="
        continue
    fi

    # Discover every topic in this bag from its metadata.yaml (same parsing
    # bag_topic_to_npz.py itself does), instead of hardcoding a topic list.
    topics="$(python3 -c "
import yaml
with open('$metadata') as f:
    meta = yaml.safe_load(f)
for t in meta['rosbag2_bagfile_information']['topics_with_message_count']:
    print(t['topic_metadata']['name'])
")"
    if [ -z "$topics" ]; then
        echo "== $bag_name: SKIP (no topics found in metadata.yaml) =="
        continue
    fi

    # Only segments _0 and _1 -- _2 and beyond are intentionally left out.
    segments=()
    for suffix in 0 1; do
        for f in "$bag_dir"*"_${suffix}.db3"; do
            [ -e "$f" ] && segments+=("$f")
        done
    done
    if [ "${#segments[@]}" -eq 0 ]; then
        echo "== $bag_name: SKIP (no *_0.db3 / *_1.db3 segment found) =="
        continue
    fi

    for segment in "${segments[@]}"; do
        seg_name="$(basename "$segment")"
        while IFS= read -r topic; do
            [ -z "$topic" ] && continue
            # Same safe_topic derivation as bag_topic_to_npz.py's
            # re.sub(r"[^A-Za-z0-9_]+", "_", topic_name).strip("_"), so the
            # collision check below matches its real output filename exactly.
            safe_topic="$(python3 -c "
import re, sys
print(re.sub(r'[^A-Za-z0-9_]+', '_', sys.argv[1]).strip('_'))
" "$topic")"
            stem="$(basename "$segment" .db3)"
            out_file="$OUT_DIR/${stem}_${safe_topic}.npz"

            if [ -e "$out_file" ]; then
                echo "== SKIP: $out_file already exists -- ${bag_name}/${seg_name} would overwrite output from a different source segment of the same stem name. =="
                skip_count=$((skip_count + 1))
                continue
            fi

            echo "== ${bag_name}/${seg_name}  topic=${topic} =="
            if python3 "$DUMP_SCRIPT" "$segment" "$topic" --out-dir "$OUT_DIR"; then
                ok_count=$((ok_count + 1))
            else
                echo "== ${bag_name}/${seg_name}  topic=${topic}: FAILED =="
                fail_count=$((fail_count + 1))
            fi
        done <<< "$topics"
    done
done

echo
echo "Done: $ok_count succeeded, $fail_count failed, $skip_count skipped (collisions)."
[ "$fail_count" -eq 0 ]
