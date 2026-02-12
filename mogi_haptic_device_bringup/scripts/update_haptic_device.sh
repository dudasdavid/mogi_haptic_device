#!/usr/bin/env bash
set -euo pipefail

REPO="/home/david/ros2_ws/src/ROS2-lessons/mogi_haptic_device"
WS="/home/david/ros2_ws"

echo "=== Updating repo ==="
cd "$REPO"

# If you want it to always update the currently checked-out branch:
git status
git pull --rebase

echo
echo "=== Rebuilding workspace ==="
cd "$WS"

# Source ROS (adjust distro if needed)
source /opt/ros/jazzy/setup.bash

# Build (adjust packages-select if you only want this package)
colcon build --symlink-install

echo
echo "=== Done! ==="

# GUI popup if available; otherwise just keep terminal open
if command -v zenity >/dev/null 2>&1; then
  zenity --info --title="ROS2 Update" --text="Update + rebuild finished successfully."
else
  echo "Update + rebuild finished successfully."
fi

echo
echo "Press Enter to close..."
read