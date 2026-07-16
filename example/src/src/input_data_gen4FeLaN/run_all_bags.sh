#!/usr/bin/env bash
# Run generate_input_data.py over every lowstate/sportmodestate npz pair in
# example/data/npz_data. Intended for an Anaconda Prompt with bash hooks
# (`conda init bash`) -- activates the hybrid_ctrl env so pinocchio's native
# DLLs resolve correctly.
set -uo pipefail

CONDA_ENV_NAME="hybrid_ctrl"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
NPZ_DIR="$REPO_ROOT/example/data/npz_data"
GEN_SCRIPT="$SCRIPT_DIR/generate_input_data.py"

source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate "$CONDA_ENV_NAME"

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
    if python "$GEN_SCRIPT" "$lowstate" "$sportmode"; then
        ok_count=$((ok_count + 1))
    else
        echo "== $bag: FAILED =="
        fail_count=$((fail_count + 1))
    fi
done

echo
echo "Done: $ok_count succeeded, $fail_count failed/skipped."
[ "$fail_count" -eq 0 ]
