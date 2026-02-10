#!/usr/bin/env bash
set -eo pipefail

ROS_DISTRO_PATH="/opt/ros/jazzy"

cleanup() {
  if [[ -n "${JOY_PID:-}" ]] && kill -0 "$JOY_PID" 2>/dev/null; then
    kill "$JOY_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[1/4] Source ROS 2"
set +u
source "${ROS_DISTRO_PATH}/setup.bash"
set -u

# Optional: avoid FastDDS SHM issues you saw earlier
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_SHM_TRANSPORT_DISABLED="${FASTDDS_SHM_TRANSPORT_DISABLED:-1}"

echo "[2/4] Start joy_node (publishes /joy)"
ros2 run joy joy_node &
JOY_PID=$!
sleep 0.5

echo "[3/4] Start teleop_twist_keyboard in a new terminal"
# Install if missing: sudo apt install ros-jazzy-teleop-twist-keyboard
gnome-terminal -- bash -lc "source ${ROS_DISTRO_PATH}/setup.bash; \
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; \
  export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}; \
  export FASTDDS_SHM_TRANSPORT_DISABLED=${FASTDDS_SHM_TRANSPORT_DISABLED}; \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel"

echo "[4/4] Start Mobility GUI"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
export PYTHONPATH="${SRC_DIR}:${PYTHONPATH:-}"

python3 -m dashboard.main_mobility
