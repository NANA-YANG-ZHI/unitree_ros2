#!/usr/bin/env bash
# Runs the data-generation half of the contact-estimation pipeline (force
# estimate + ground-truth contact comparison) over every bag in
# example/data/2026_07_07/usable_data. Raw per-topic npz dumps land in
# example/data/npz_data/; derived data (gt comparison, contact estimate)
# stays in plot/all_bags/ (kept separate from plot/contact_estimation/,
# which holds only code: plot_contact_estimate.py, debug/, validation/,
# tutorial/).
#
# Runs entirely in unitree_ros2_dev (source ROS2 + cyclonedds setup first).
# Per bag:
#   1. bag_topic_to_npz.py       -> <bag>_lowstate.npz, <bag>_sportmodestate.npz (example/data/npz_data/)
#   2. compute_gt_contact_signals.py -> <bag>_gt_contact_comparison.npz (plot/all_bags/)
#   3. run_contact_estimation.py -> <bag>_contact_estimate.npz (plot/all_bags/)
#
# See plot_all_bags.sh for the plotting half (runs in plot-tools-run).
#
# Usage:
#   ./plot/all_bags/generate_all_bags.sh
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PINOCCHIO_CONTAINER="unitree_ros2_dev"

USABLE_DATA_DIR="example/data/2026_07_07/usable_data"  # relative to repo root
NPZ_DATA_DIR="example/data/npz_data"   # relative to repo root (unitree_ros2_dev's /workspace)
OUT_DIR="plot/all_bags"   # relative to repo root (unitree_ros2_dev's /workspace)

BAGS=(excitation_bag_v4 excitation_bag_v5 excitation_bag_v6 excitation_bag_v7 excitation_bag_v8 excitation_bag_v9 excitation_bag_v95)

mkdir -p "${REPO_DIR}/${NPZ_DATA_DIR}" "${REPO_DIR}/${OUT_DIR}"

echo "== Per-bag lowstate/sportmode dump + gt comparison + force estimate, in ${PINOCCHIO_CONTAINER} =="
for BAG in "${BAGS[@]}"; do
    BAG_PATH="${USABLE_DATA_DIR}/${BAG}"

    if grep -q "name: /lf/sportmodestate" "${REPO_DIR}/${BAG_PATH}/metadata.yaml"; then
        SPORTMODE_TOPIC="/lf/sportmodestate"
    else
        SPORTMODE_TOPIC="/sportmodestate"
    fi

    echo "-- ${BAG} (sportmode topic: ${SPORTMODE_TOPIC}) --"
    docker exec -w /workspace "$PINOCCHIO_CONTAINER" bash -c "
        source /opt/ros/humble/setup.bash
        source /workspace/cyclonedds_ws/install/setup.bash
        cd example/data

        python3 bag_topic_to_npz.py '/workspace/${BAG_PATH}' /lowstate \
            --out '/workspace/${NPZ_DATA_DIR}/${BAG}_lowstate.npz'
        python3 bag_topic_to_npz.py '/workspace/${BAG_PATH}' '${SPORTMODE_TOPIC}' \
            --out '/workspace/${NPZ_DATA_DIR}/${BAG}_sportmodestate.npz'

        cd ../src/src/contact_estimation

        python3 compute_gt_contact_signals.py \
            '/workspace/${NPZ_DATA_DIR}/${BAG}_lowstate.npz' \
            '/workspace/${NPZ_DATA_DIR}/${BAG}_sportmodestate.npz' \
            --out '/workspace/${OUT_DIR}/${BAG}_gt_contact_comparison.npz'

        python3 run_contact_estimation.py \
            '/workspace/${NPZ_DATA_DIR}/${BAG}_lowstate.npz' \
            '/workspace/${NPZ_DATA_DIR}/${BAG}_sportmodestate.npz' \
            --out '/workspace/${OUT_DIR}/${BAG}_contact_estimate.npz'
    "
done

echo "Done. Raw per-topic npz saved under ${NPZ_DATA_DIR}/, derived data under ${OUT_DIR}/"
