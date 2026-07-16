#!/usr/bin/env bash
# Runs plot_force_vs_orientation.py (debug plot: est vs. raw force vs.
# roll/pitch/yaw) over every bag's <bag>_contact_estimate.npz already
# produced in plot/all_bags/ by run_all_bags.sh.
#
# Runs entirely in plot-tools-run (matplotlib only, no pinocchio/ROS2).
# plot-tools-run mounts only the repo's plot/ folder at /workspace, so
# paths passed to it are relative to plot/ (e.g. plot/all_bags -> all_bags).
#
# Usage:
#   ./plot/all_bags/plot_force_vs_orientation_all.sh
set -euo pipefail

PLOT_CONTAINER="plot-tools-run"
PLOT_OUT_DIR="all_bags"  # relative to plot/ (plot-tools-run's /workspace)

BAGS=(excitation_bag_v4 excitation_bag_v5 excitation_bag_v6 excitation_bag_v7 excitation_bag_v8 excitation_bag_v9 excitation_bag_v95)

for BAG in "${BAGS[@]}"; do
    echo "-- ${BAG} --"
    docker exec -w "/workspace/${PLOT_OUT_DIR}" "$PLOT_CONTAINER" python3 \
        /workspace/contact_estimation/debug/plot_force_vs_orientation.py \
        "${BAG}_contact_estimate.npz"
done

echo "Done. Plots saved under plot/${PLOT_OUT_DIR}/"
