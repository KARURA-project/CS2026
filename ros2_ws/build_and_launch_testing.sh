#!/bin/bash
set -e  # exit immediately on error

echo "🔧 [1/4] Sourcing ROS 2 Humble environment..."
source /opt/ros/humble/setup.bash

echo "�� [2/4] Building workspace with colcon..."
colcon build --symlink-install

echo "🌱 [3/4] Sourcing workspace environment..."
source install/setup.bash

echo "🚀 [4/4] Launching multi-camera system..."
ros2 launch camera_gui pyside.launch.py
