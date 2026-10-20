#!/bin/bash
# CSTAM: Launch Delivery Process Simulator Interface
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

if [ -f "ros2_ws/install/setup.bash" ]; then
    source ros2_ws/install/setup.bash
fi

export PYTHONPATH="$SCRIPT_DIR/ros2_ws/src/cstam_core:$PYTHONPATH"

echo "=========================================================="
echo " Starting CSTAM Delivery Process Simulator UI"
echo "=========================================================="

python3 delivery_simulator_ui.py "$@"
