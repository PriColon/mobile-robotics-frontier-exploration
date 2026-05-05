#!/usr/bin/env bash
# ROS 2 Jazzy environment loader — repo-aware version
# Usage: source open_ros_env.sh

# Guard: must be sourced
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "[ROS ERROR] Run with: source ${BASH_SOURCE[0]}"
  exit 1
fi

echo "[ROS] Loading ROS 2 Jazzy environment..."

# 1) Source global ROS install
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
else
  echo "[ROS ERROR] /opt/ros/jazzy/setup.bash not found. Is ROS 2 Jazzy installed?"
  return 1
fi

# 2) Resolve workspace relative to THIS script's location (repo root)
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$REPO_ROOT/ros2_ws"

if [ -f "$WS/install/setup.bash" ]; then
  echo "[ROS] Overlaying workspace: $WS"
  source "$WS/install/setup.bash"
else
  echo "[ROS] Workspace not built yet. Run from $WS:"
  echo "        cd $WS && colcon build"
fi

cd "$WS" || return 1

# 3) Middleware & discovery config
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_AUTOMATIC_DISCOVERY_RANGE
unset CYCLONEDDS_URI
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export ROS_DISCOVERY_SERVER="${ROS_DISCOVERY_SERVER:-192.168.1.18:11811}"
export ROS_DISCOVERY_SERVER=192.168.1.18:11811 #CHANGE IF ROBOT CHANGES WIFI
export ROS_SUPER_CLIENT=True

export PS1="(ROS2) $PS1"
echo "[ROS] Ready ✔  (workspace: $WS)"
