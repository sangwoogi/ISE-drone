#!/usr/bin/env bash
# Source this file in every ROS 2 shell:   source scripts/env.sh
# It deactivates conda (ROS 2 must use the system Python 3.12), sources ROS 2
# Jazzy and the project workspace, and exports the PX4 location.

# --- conda must not shadow the system python ---------------------------------
if command -v conda >/dev/null 2>&1; then
    while [ -n "${CONDA_PREFIX:-}" ]; do conda deactivate 2>/dev/null || break; done
fi

# --- paths -------------------------------------------------------------------
export ISE_DRONE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export ROS_WS="$ISE_DRONE_ROOT/ros2_ws"

# --- ROS 2 -------------------------------------------------------------------
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    echo "[env.sh] WARNING: /opt/ros/$ROS_DISTRO not found. Run scripts/setup/01_ros2_jazzy.sh" >&2
fi
if [ -f "$ROS_WS/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "$ROS_WS/install/setup.bash"
fi

# Single-machine DDS defaults. Change ROS_DOMAIN_ID when several simulators share a LAN.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# --- Gazebo / PX4 ------------------------------------------------------------
if [ -d "$PX4_DIR/Tools/simulation/gz" ]; then
    export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"
fi

echo "[env.sh] ROS_DISTRO=$ROS_DISTRO  PX4_DIR=$PX4_DIR  python=$(command -v python3)"
