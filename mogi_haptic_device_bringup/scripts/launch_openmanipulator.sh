#!/usr/bin/env bash
set -e

# Optional: make script location stable
#SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- ROS env (choose one style) ---
# If using system install:
source /opt/ros/jazzy/setup.bash

# If using a workspace install:
source ~/ros2_ws/install/setup.bash

# --- App run ---
# (Optional) set domain id so they don't collide with others
export ROS_DOMAIN_ID=30

# (Optional) better logging
export RCUTILS_COLORIZED_OUTPUT=1

exec ros2 launch mogi_haptic_device_bringup bringup_openmanipulator.launch.py