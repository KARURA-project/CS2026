# #!/usr/bin/env bash
# set -e

# # READ ME FOR EXECUTION PLSSS
# # Usage:
# #   ./run_mobility.sh          # run without building
# #   ./run_mobility.sh --build  # build then run
# #
# # Optional env vars:
# #   ROS_DISTRO_PATH=/opt/ros/jazzy
# #   WS_ROOT=/home/<user>/Karura/control-station/CS2026
# #   PKG_NAME=karura_dashboard
# #   GUI_MODULE=dashboard.main_mobility

# ROS_DISTRO_PATH="${ROS_DISTRO_PATH:-/opt/ros/jazzy}"
# WS_ROOT="${WS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# PKG_NAME="${PKG_NAME:-karura_dashboard}"
# GUI_MODULE="${GUI_MODULE:-dashboard.main_mobility}"

# BUILD=0
# if [[ "${1:-}" == "--build" ]]; then
#   BUILD=1
# fi

# cleanup() {
#   # Kill background processes we started
#   if [[ -n "${JOY_PID:-}" ]] && kill -0 "$JOY_PID" 2>/dev/null; then
#     kill "$JOY_PID" 2>/dev/null || true
#   fi
# }
# trap cleanup EXIT INT TERM

# echo "[1/4] Sourcing ROS 2 environment: ${ROS_DISTRO_PATH}/setup.bash"
# source "${ROS_DISTRO_PATH}/setup.bash"

# cd "${WS_ROOT}"

# if [[ $BUILD -eq 1 ]]; then
#   echo "[2/4] Building workspace (package: ${PKG_NAME})"
#   colcon build --symlink-install --packages-select "${PKG_NAME}"
# else
#   echo "[2/4] Skipping build (use --build to compile)"
# fi

# echo "[3/4] Sourcing workspace overlay: install/setup.bash"
# source install/setup.bash

# echo "[4/4] Starting required ROS nodes"

# # Start joystick driver (publishes /joy)
# # NOTE: ensure 'ros-${ROS_DISTRO}-joy' is installed on the base station
# ros2 run joy joy_node &
# JOY_PID=$!

# # Give joy_node a moment to come up
# sleep 0.5

# # Start the Mobility GUI (recommended) - this should create MobilityBridge and start it
# python3 -m "${GUI_MODULE}"

#!/usr/bin/env bash
set -eo pipefail

ROS_DISTRO_PATH="/opt/ros/jazzy"

cleanup() {
  if [[ -n "${JOY_PID:-}" ]] && kill -0 "$JOY_PID" 2>/dev/null; then
    kill "$JOY_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[1/3] Source ROS 2"
set +u
source "${ROS_DISTRO_PATH}/setup.bash"
set -u

echo "[2/3] Start joy_node (publishes /joy)"
ros2 run joy joy_node &
JOY_PID=$!
sleep 0.5

echo "[3/3] Start Mobility GUI"

# Compute karura_gui/src so `python -m dashboard...` imports work
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"   # -> karura_gui/src
export PYTHONPATH="${SRC_DIR}:${PYTHONPATH:-}"

python3 -m dashboard.main_mobility

