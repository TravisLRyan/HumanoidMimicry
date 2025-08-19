#!/bin/bash

# Exit if any command fails
set -e

set -euo pipefail
_safe_source() { if set -o | grep -q 'nounset *on'; then u=1; set +u; else u=0; fi; source "$1"; [ "$u" = 1 ] && set -u; }
[ -z "${ROS_DISTRO:-}" ] && { . /etc/os-release; [ "$VERSION_CODENAME" = "focal" ] && ROS_DISTRO=foxy || ROS_DISTRO=humble; }
[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ] && _safe_source "/opt/ros/$ROS_DISTRO/setup.bash"
[ -f "$HOME/humanoid_ws/install/setup.bash" ] && _safe_source "$HOME/humanoid_ws/install/setup.bash"


# Define paths
LIVOX_DIR="$HOME/humanoid_ws/src/livox_ros_driver2/"
HUMANOID_WS="$HOME/humanoid_ws/"

# Step 1: Build Livox with Humble flag
echo "Building project with ROS2 humble..."
cd "$LIVOX_DIR"
./build.sh humble

# Step 2: Source humanoid_ws setup
echo "Sourcing humanoid_ws setup..."
source "$HUMANOID_WS/install/setup.sh"

echo "Done."
