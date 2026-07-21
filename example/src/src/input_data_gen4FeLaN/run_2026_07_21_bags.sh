#!/usr/bin/env bash
# Run generate_input_data.py over every lowstate/sportmodestate npz pair in
# example/data/npz_data/2026_07_21 (produced by
# example/data/convert_bags_to_npz.sh). Same structure as run_all_bags.sh,
# just pointed at this dataset's npz_data subfolder and topic naming
# (*_sportmodestate.npz -- these bags recorded /sportmodestate, not
# /lf/sportmodestate, but the _lf_ fallback is kept for safety).
#
# Run inside the unitree_ros2_dev container (needs pinocchio):
#   docker exec -it unitree_ros2_dev bash
#   /workspace/example/src/src/input_data_gen4FeLaN/run_2026_07_21_bags.sh
set -uo pipefail

CONDA_ENV_NAME="hybrid_ctrl"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
NPZ_DIR="$REPO_ROOT/example/data/npz_data/2026_07_21"
GEN_SCRIPT="$SCRIPT_DIR/generate_input_data.py"

# Only present in a conda-based setup (e.g. Anaconda Prompt) -- skip quietly
# inside a container where conda isn't installed.
if command -v conda >/dev/null 2>&1; then
    source "$(conda info --base)/etc/profile.d/conda.sh"
    conda activate "$CONDA_ENV_NAME"
fi

# Preflight disk-space check -- a prior batch run on this same disk died
# mid-write from running out of space (example/data/npz_data/2026_07_21's
# excitation_bag_11_1_lowstate.npz etc came out truncated/empty). Each
# generate_input_data.py call writes a felan_input_data/run_*.npz plus a
# plot/contact_estimation/contact_estimation_result_npz/*_contact_estimate.npz;
# warn loudly instead of repeating that failure silently.
MIN_FREE_KB=3000000  # 3 GB
avail_kb="$(df --output=avail -k "$REPO_ROOT" | tail -1 | tr -d ' ')"
if [ "$avail_kb" -lt "$MIN_FREE_KB" ]; then
    echo "WARNING: only $((avail_kb / 1024)) MB free on the filesystem holding $REPO_ROOT."
    echo "         A previous batch run died mid-write from exactly this. Free up space"
    echo "         before continuing, or outputs may be silently truncated/corrupt."
    echo
fi

fail_count=0
ok_count=0

for lowstate in "$NPZ_DIR"/*_lowstate.npz; do
    [ -e "$lowstate" ] || continue
    bag="$(basename "$lowstate" _lowstate.npz)"

    sportmode="$NPZ_DIR/${bag}_sportmodestate.npz"
    if [ ! -e "$sportmode" ]; then
        sportmode="$NPZ_DIR/${bag}_lf_sportmodestate.npz"
    fi
    if [ ! -e "$sportmode" ]; then
        echo "== $bag: SKIP (no matching sportmodestate npz) =="
        fail_count=$((fail_count + 1))
        continue
    fi

    echo "== $bag =="
    if python3 "$GEN_SCRIPT" "$lowstate" "$sportmode"; then
        ok_count=$((ok_count + 1))
    else
        echo "== $bag: FAILED =="
        fail_count=$((fail_count + 1))
    fi
done

echo
echo "Done: $ok_count succeeded, $fail_count failed/skipped."
[ "$fail_count" -eq 0 ]
