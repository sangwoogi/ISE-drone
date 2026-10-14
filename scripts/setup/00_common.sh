#!/usr/bin/env bash
# Shared helpers for setup scripts. Not meant to be run directly.
set -euo pipefail

ISE_DRONE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ISE_DRONE_ROOT
export PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export PX4_VERSION="${PX4_VERSION:-v1.17.0}"        # PX4 and px4_msgs must share this tag
export XRCE_AGENT_VERSION="${XRCE_AGENT_VERSION:-v2.4.3}"
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"

log()  { printf '\033[1;34m[setup]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[setup]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[setup]\033[0m %s\n' "$*" >&2; exit 1; }

require_ubuntu_2404() {
    # shellcheck disable=SC1091
    . /etc/os-release
    [ "${VERSION_ID:-}" = "24.04" ] || die "Ubuntu 24.04 required (found ${PRETTY_NAME:-unknown})"
}

# conda's python must never be first on PATH while building ROS packages.
drop_conda_from_path() {
    if [ -n "${CONDA_PREFIX:-}" ] || command -v conda >/dev/null 2>&1; then
        PATH="$(echo "$PATH" | tr ':' '\n' | grep -v -i 'conda' | paste -sd ':' -)"
        export PATH
        unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER PYTHONHOME 2>/dev/null || true
    fi
    log "python3 -> $(command -v python3) ($(python3 --version 2>&1))"
}
