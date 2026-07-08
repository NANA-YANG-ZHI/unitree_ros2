#!/usr/bin/env bash
# Build the unitree_ros2 dev image on the remote Linux machine.
# Usage: ./docker/build.sh
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="unitree_ros2:humble"

docker build \
    --build-arg USER_UID="$(id -u)" \
    --build-arg USER_GID="$(id -g)" \
    -t "$IMAGE_NAME" \
    -f "$REPO_DIR/docker/Dockerfile" \
    "$REPO_DIR"

echo "Built image $IMAGE_NAME"
