#!/usr/bin/env bash
# Runs plot_estimator_vs_sensor_zoom.py over every bag's
# <bag>_contact_estimate.npz already produced in plot/all_bags/ by
# generate_all_bags.sh.
#
# Runs entirely in plot-tools-run (matplotlib only, no pinocchio/ROS2).
# plot-tools-run mounts only the repo's plot/ folder at /workspace, so
# paths passed to it are relative to plot/ (e.g. plot/all_bags -> all_bags).
#
# Usage:
#   ./plot/all_bags/plot_all_bags.sh [t0] [t1]
set -euo pipefail

PLOT_CONTAINER="plot-tools-run"
PLOT_OUT_DIR="all_bags"  # relative to plot/ (plot-tools-run's /workspace)

# Full-trial window by default -- bag durations range ~30-63s, so t1=60
# covers everything (shorter bags just end early, no error).
T0="${1:-0}"
T1="${2:-60}"

BAGS=(excitation_bag_v4 excitation_bag_v5 excitation_bag_v6 excitation_bag_v7 excitation_bag_v8 excitation_bag_v9 excitation_bag_v95)

for BAG in "${BAGS[@]}"; do
    echo "-- ${BAG} --"
    docker exec -w "/workspace/${PLOT_OUT_DIR}" "$PLOT_CONTAINER" python3 \
        /workspace/contact_estimation/validation/plot_estimator_vs_sensor_zoom.py \
        "${BAG}_contact_estimate.npz" --t0 "$T0" --t1 "$T1"
done

echo "Done. Plots saved under plot/${PLOT_OUT_DIR}/"
