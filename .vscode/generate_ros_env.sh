#!/usr/bin/env bash
# Sources the ROS2 + workspace overlay setup files and dumps the resulting
# environment to .vscode/ros2.env in dotenv format, so launch.json can load
# it via "envFile". Re-run (via the "Generate ROS2 env for debugging" task,
# which fires automatically as a preLaunchTask) any time the underlay/overlay
# setup changes.
set -eo pipefail

# ROS2's setup.bash scripts reference unset variables internally, so nounset
# must be disabled while sourcing them.
set +u
source /opt/ros/humble/setup.bash
source /workspace/cyclonedds_ws/install/setup.bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
env | grep -v '^BASH_FUNC' | sort > "$SCRIPT_DIR/ros2.env"
