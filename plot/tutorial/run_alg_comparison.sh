# #!/usr/bin/env bash
# # Compares the three ContactDetector injection-law variants (hg / sliding /
# # mixing -- see err_mapping_func in contact_detection.py) on the same bag,
# # and plots a short zoomed time window so the differences between them are
# # actually visible (a full-trial plot averages them out).
# #
# # Step 1 (pinocchio -- runs inside the unitree_ros2_dev container, after
# #   sourcing /opt/ros/humble/setup.bash and
# #   /workspace/cyclonedds_ws/install/setup.bash):
# #   run_contact_estimation.py once per --alg, saving
# #   plot/tutorial/<bag_name>_contact_estimate_<alg>.npz
# # Step 2 (matplotlib only -- runs inside the plot-tools-run container):
# #   plot_alg_comparison.py loads the three .npz files and saves
# #   plot/tutorial/<bag_name>_alg_comparison_zoom.png
# #
# # Both containers are expected to already be running. unitree_ros2_dev
# # mounts the whole repo at /workspace (see docker/run.sh); plot-tools-run
# # mounts only the repo's plot/ folder at /workspace, so paths passed to it
# # are relative to plot/ (e.g. plot/tutorial -> tutorial).
# #
# # Usage:
# #   ./plot/tutorial/run_alg_comparison.sh [bag_path] [t0] [t1]
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PINOCCHIO_CONTAINER="unitree_ros2_dev"
PLOT_CONTAINER="plot-tools-run"

BAG_PATH="${1:-example/data/2026_07_07/usable_data/excitation_bag_v95}"
T0="${2:-25}"
T1="${3:-28}"

OUT_DIR="plot/tutorial"          # relative to repo root (unitree_ros2_dev's /workspace)
PLOT_OUT_DIR="tutorial"          # same folder, relative to plot/ (plot-tools-run's /workspace)
BAG_NAME="$(basename "$BAG_PATH")"

# mkdir -p "${REPO_DIR}/${OUT_DIR}"

# echo "== Step 1/2: running contact estimator (hg / sliding / mixing) in ${PINOCCHIO_CONTAINER} =="
# for ALG in hg sliding mixing; do
#     echo "-- alg=${ALG} --"
#     docker exec -w /workspace "$PINOCCHIO_CONTAINER" bash -c "
#         source /opt/ros/humble/setup.bash
#         source /workspace/cyclonedds_ws/install/setup.bash
#         python3 example/src/src/contact_estimation/run_contact_estimation.py \
#             '${BAG_PATH}' --alg '${ALG}' \
#             --out '${OUT_DIR}/${BAG_NAME}_contact_estimate_${ALG}.npz'
#     "
# done

echo "== Step 2/2: plotting comparison in ${PLOT_CONTAINER} =="
docker exec -w /workspace "$PLOT_CONTAINER" python3 \
    "${PLOT_OUT_DIR}/plot_alg_comparison.py" \
    --bag-name "$BAG_NAME" --t0 "$T0" --t1 "$T1" --out-dir "$PLOT_OUT_DIR"

echo "Done. Data (.npz) and plot (.png) saved under ${OUT_DIR}/"
