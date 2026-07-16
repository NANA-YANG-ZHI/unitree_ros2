#!/usr/bin/env bash
# Build the plot-tools image (numpy/matplotlib, no pinocchio/ROS2).
# Usage: ./plot/build.sh
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="plot-tools:latest"

docker build \
    -t "$IMAGE_NAME" \
    -f "$REPO_DIR/plot/Dockerfile" \
    "$REPO_DIR/plot"

echo "Built image $IMAGE_NAME"
