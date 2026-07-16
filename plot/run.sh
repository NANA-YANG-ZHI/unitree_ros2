#!/usr/bin/env bash
# Start (or restart) the plot-tools container, mounting only this repo's
# plot/ folder at /workspace (matching the existing convention -- paths
# passed into this container are relative to plot/, e.g. plot/all_bags ->
# all_bags). Forwards X11 so plt.show() (TkAgg backend) can open a window
# on the host display; savefig-only scripts don't need this.
#
# If you're SSH'd into the remote machine, connect with `ssh -X` and make
# sure `xauth` is installed there; run `xhost +local:docker` on the remote
# machine once per login session before starting the container.
#
# Usage: ./plot/run.sh
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="plot-tools:latest"
CONTAINER_NAME="plot-tools-run"

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "Container '$CONTAINER_NAME' already exists, starting it."
    docker start "$CONTAINER_NAME" >/dev/null
else
    docker run -d \
        --name "$CONTAINER_NAME" \
        --network host \
        -e DISPLAY="${DISPLAY:-}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$REPO_DIR/plot":/workspace \
        -w /workspace \
        "$IMAGE_NAME"
fi

echo "Container '$CONTAINER_NAME' is running."
echo "Attach with: docker exec -it $CONTAINER_NAME bash"
