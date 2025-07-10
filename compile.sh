#!/bin/bash

# Exit if any command fails
set -e

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
