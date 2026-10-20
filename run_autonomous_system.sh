#!/usr/bin/env bash
set -e

# Source ROS 2 Jazzy and CSTAM Workspace
source /opt/ros/jazzy/setup.bash
source "$(dirname "$0")/ros2_ws/install/setup.bash"

echo "=========================================================="
echo " 🚀 Launching CSTAM Complete 3D Autonomous Navigation Stack"
echo " 1. Gazebo Sim: restaurant.world with 24 Dining Tables     "
echo " 2. BellaBot 3D Robot with LIDAR and Wheel Odometry        "
echo " 3. Nav2 Perception: Costmaps with Obstacle Inflation     "
echo " 4. AMCL Localization: Auto-initialized at SE Dock       "
echo " 5. RViz2 3D Visualizer: Live LIDAR, Costmap & Path       "
echo "=========================================================="
echo " To send goals and watch 3D path planning & motion:       "
echo " - In RViz2: Click 'Nav2 Goal' on the top bar, then click "
echo "   any dining table on the map.                           "
echo " - Or run ./run_delivery_simulator.sh to open the Delivery "
echo "   Simulator Interface & test orders / auto-docking.      "
echo "=========================================================="

ros2 launch cstam_core cstam_system.launch.py launch_rviz:=true "$@"
