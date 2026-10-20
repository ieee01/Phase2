#!/usr/bin/env bash
set -e

# Source ROS 2 Jazzy and CSTAM Workspace
source /opt/ros/jazzy/setup.bash
source "$(dirname "$0")/ros2_ws/install/setup.bash"

echo "=========================================================="
echo " 🐱 CSTAM SLAM Mapping All-in-One Demonstration Pipeline  "
echo " 1. Launching Gazebo Simulation (restaurant.world)        "
echo " 2. Spawning BellaBot with 2D LIDAR & Wheel Odometry     "
echo " 3. Starting SLAM Toolbox (Real-time Occupancy Grid)      "
echo " 4. Opening Pre-Configured RViz2 Visualizer               "
echo "=========================================================="
echo " In a second terminal, drive BellaBot with:               "
echo "   source /opt/ros/jazzy/setup.bash                       "
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard   "
echo "=========================================================="

ros2 launch cstam_navigation mapping_demo.launch.py "$@"
