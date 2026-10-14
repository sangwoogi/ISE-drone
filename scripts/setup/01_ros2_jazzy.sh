#!/usr/bin/env bash
# Phase 0 step 1: ROS 2 Jazzy desktop + dev tools + ros_gz (Jazzy pairs with Gazebo Harmonic) on Ubuntu 24.04.
# Idempotent. Requires sudo.
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/00_common.sh"
require_ubuntu_2404
drop_conda_from_path

log "Locale / universe repository"
sudo apt-get update -y
sudo apt-get install -y locales software-properties-common curl gnupg lsb-release
sudo locale-gen en_US en_US.UTF-8 >/dev/null
sudo add-apt-repository -y universe >/dev/null

if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
    log "Registering the ROS 2 apt source (ros2-apt-source package)"
    ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
    [ -n "$ROS_APT_SOURCE_VERSION" ] || die "Could not resolve ros-apt-source version (network?)"
    DEB="/tmp/ros2-apt-source.deb"
    curl -sSL -o "$DEB" \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
    sudo dpkg -i "$DEB" >/dev/null
    sudo apt-get update -y
else
    log "ros2-apt-source already registered"
fi

log "Installing ROS 2 $ROS_DISTRO desktop, build tools and Gazebo Harmonic bridge"
sudo apt-get install -y \
    "ros-$ROS_DISTRO-desktop" \
    "ros-$ROS_DISTRO-ros-gz" \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-pytest \
    python3-numpy \
    python3-yaml

# PX4 ROS 2 user guide pins these for px4_msgs generation.
log "Python helper packages for px4_msgs"
python3 -m pip install --user --break-system-packages -U "empy==3.3.4" pyros-genmsg setuptools >/dev/null

if [ ! -d /etc/ros/rosdep/sources.list.d ]; then
    sudo rosdep init >/dev/null 2>&1 || true
fi
rosdep update >/dev/null 2>&1 || warn "rosdep update failed (offline?); not fatal for Phase 1"

log "Done. Verify with:  source /opt/ros/$ROS_DISTRO/setup.bash && ros2 topic list"
