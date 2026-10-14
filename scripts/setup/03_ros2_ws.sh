#!/usr/bin/env bash
# Phase 0 step 3: clone px4_msgs (same tag as PX4) + Micro-XRCE-DDS-Agent into ros2_ws/src
# and build the workspace with colcon. No sudo needed.
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/00_common.sh"
drop_conda_from_path

[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ] || die "ROS 2 $ROS_DISTRO not installed; run 01_ros2_jazzy.sh first"
# ROS setup scripts reference unset variables; relax `set -u` while sourcing.
set +u
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u

WS="$ISE_DRONE_ROOT/ros2_ws"
mkdir -p "$WS/src"

clone_or_check() {   # url dir tag
    if [ ! -d "$2/.git" ]; then
        log "Cloning $(basename "$2") $3"
        git clone -b "$3" --depth 1 "$1" "$2"
    else
        log "$(basename "$2") already present ($(git -C "$2" describe --tags --always 2>/dev/null))"
    fi
}
clone_or_check https://github.com/PX4/px4_msgs.git                 "$WS/src/px4_msgs"             "$PX4_VERSION"
clone_or_check https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$WS/src/Micro-XRCE-DDS-Agent" "$XRCE_AGENT_VERSION"

log "colcon build (px4_msgs generates ~200 messages; expect several minutes on first build)"
cd "$WS"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -n 5

log "Done. Use:  source $ISE_DRONE_ROOT/scripts/env.sh"
log "Agent:     MicroXRCEAgent udp4 -p 8888"
