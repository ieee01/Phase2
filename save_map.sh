#!/usr/bin/env bash
set -e

# Source ROS 2 Jazzy and CSTAM Workspace
source /opt/ros/jazzy/setup.bash
if [ -f "$(dirname "$0")/ros2_ws/install/setup.bash" ]; then
    source "$(dirname "$0")/ros2_ws/install/setup.bash"
fi

MAP_NAME="${1:-resto_arbi_map}"
TARGET_DIR="$(pwd)/ros2_ws/src/cstam_navigation/maps"
MAP_PATH="${TARGET_DIR}/${MAP_NAME}"

mkdir -p "$TARGET_DIR"

echo "=========================================================="
echo " 🗺️  Saving SLAM Map to: ${MAP_PATH}.yaml / .pgm"
echo "=========================================================="

# Method 1: Try SLAM Toolbox native service (direct & reliable)
echo "Attempting to save via SLAM Toolbox service..."
if ros2 service list | grep -q "/slam_toolbox/save_map"; then
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '${MAP_PATH}'}}"
    echo "✔ Successfully saved map via SLAM Toolbox service!"
    exit 0
fi

# Method 2: Fallback to nav2_map_server with simulation time enabled
echo "SLAM Toolbox service not detected; attempting via nav2_map_server map_saver_cli..."
ros2 run nav2_map_server map_saver_cli -f "${MAP_PATH}" --ros-args -p use_sim_time:=true

if [ -f "${MAP_PATH}.yaml" ]; then
    echo "✔ Successfully saved map: ${MAP_PATH}.yaml"
else
    echo "❌ Failed to save map. Ensure SLAM mapping is running and robot has moved."
    exit 1
fi
