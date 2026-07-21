#!/usr/bin/env bash
# Copies run_*.npz from example/src/src/input_data_gen4FeLaN/felan_input_data/
# into this directory (plot/felan_input_data/) so plot_all_contact_states.py
# can see them -- the plot-tools-run container only mounts plot/ at
# /workspace, not the repo root, so the source directory isn't reachable
# from inside that container directly.
#
# Run on the host (or in unitree_ros2_dev, which mounts the whole repo) --
# not inside plot-tools-run, since the source path isn't mounted there:
#   /workspace/plot/felan_input_data/sync_run_npz.sh
#
# Safe to re-run: uses -n (no-clobber) so it never overwrites a file already
# here, e.g. a PNG or a manually-edited npz.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SRC_DIR="$REPO_ROOT/example/src/src/input_data_gen4FeLaN/felan_input_data"

cp -nv "$SRC_DIR"/run_*.npz "$SCRIPT_DIR"/
