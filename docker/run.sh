#!/usr/bin/env bash
# Start (or restart) the unitree_ros2 dev container, then attach to it from
# VS Code with the Dev Containers extension: Command Palette ->
# "Dev Containers: Attach to Running Container..." -> unitree_ros2_dev
#
# GUI apps (rviz2, rqt) need an X server reachable from the container. If
# you're SSH'd into the remote machine, connect with `ssh -X` and make sure
# `xauth` is installed there; run `xhost +local:docker` on the remote
# machine once per login session before starting the container.
#
# Usage: ./docker/run.sh
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="unitree_ros2:humble"
CONTAINER_NAME="unitree_ros2_dev"

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "Container '$CONTAINER_NAME' already exists, starting it."
    docker start "$CONTAINER_NAME" >/dev/null
else
    docker run -d \
        --name "$CONTAINER_NAME" \
        --network host \
        --cap-add=SYS_PTRACE \
        --security-opt seccomp=unconfined \
        -e DISPLAY="${DISPLAY:-}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$REPO_DIR":/workspace \
        -w /workspace \
        "$IMAGE_NAME"
fi

echo "Container '$CONTAINER_NAME' is running."
echo "Attach with:      docker exec -it $CONTAINER_NAME bash"
echo "Or from VS Code:  Dev Containers -> Attach to Running Container -> $CONTAINER_NAME"
