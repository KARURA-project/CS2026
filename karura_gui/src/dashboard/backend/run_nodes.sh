#!/bin/bash
set -e  # Exit immediately if any command fails

# 1. Sourcing the base ROS 2 environment
echo "🔧 [1/4] Sourcing ROS 2 Humble environment..."
source /opt/ros/jazzy/setup.bash

# 2. Building the workspace
echo "🔨 [2/4] Building workspace with colcon..."
# This step is crucial to install the new node entry point
colcon build --symlink-install --packages-select karura_dashboard # Replace with your package name

# 3. Sourcing the workspace environment
echo "🌱 [3/4] Sourcing workspace environment..."
# This makes the newly built node executable visible to 'ros2 run'
source install/setup.bash

# 4. Running the MobilityNode
echo "🏃 [4/4] Launching the MobilityNode..."
# Syntax: ros2 run <package_name> <executable_name>
# The executable_name is the key from the 'console_scripts' in setup.py
ros2 run karura_dashboard mobility_node # Replace 'karura_dashboard' with your package name

