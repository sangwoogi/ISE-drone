#!/usr/bin/env bash
# Phase 0 step 2: PX4-Autopilot ($PX4_VERSION) toolchain + Gazebo Harmonic + SITL build.
# PX4 lives in $PX4_DIR (default ~/PX4-Autopilot), outside the repo, inside the WSL filesystem.
# Requires sudo for Tools/setup/ubuntu.sh.
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/00_common.sh"
require_ubuntu_2404
drop_conda_from_path

if [ ! -d "$PX4_DIR/.git" ]; then
    log "Cloning PX4-Autopilot $PX4_VERSION into $PX4_DIR (shallow, with submodules)"
    git clone --recursive -b "$PX4_VERSION" --depth 1 --shallow-submodules \
        https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
else
    CUR="$(git -C "$PX4_DIR" describe --tags --exact-match 2>/dev/null || git -C "$PX4_DIR" rev-parse --short HEAD)"
    log "PX4 already present at $PX4_DIR ($CUR)"
    [ "$CUR" = "$PX4_VERSION" ] || warn "PX4 checkout ($CUR) differs from PX4_VERSION=$PX4_VERSION; px4_msgs must match it."
fi

log "Running PX4 Ubuntu setup script (toolchain + Gazebo Harmonic). This takes a while."
bash "$PX4_DIR/Tools/setup/ubuntu.sh" --no-nuttx

log "Building PX4 SITL (first build: several minutes)"
make -C "$PX4_DIR" px4_sitl

log "Done. Smoke test:  cd $PX4_DIR && make px4_sitl gz_x500   (or HEADLESS=1 make px4_sitl gz_x500)"
