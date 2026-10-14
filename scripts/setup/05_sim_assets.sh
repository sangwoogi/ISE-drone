#!/usr/bin/env bash
# Phase 3: make the repo's Gazebo assets visible to PX4 and pre-download Fuel models.
# PX4 spawns "file://${PX4_GZ_MODELS}/<model>/model.sdf" and loads "${PX4_GZ_WORLDS}/<world>.sdf"
# (gz_env.sh pins both to the PX4 tree), so our models/worlds are symlinked into it. No sudo.
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/00_common.sh"
PXM="$PX4_DIR/Tools/simulation/gz/models"; PXW="$PX4_DIR/Tools/simulation/gz/worlds"
[ -d "$PXM" ] && [ -d "$PXW" ] || die "PX4 gz model/world dirs not found under $PX4_DIR (run 02_px4.sh)"
for m in "$ISE_DRONE_ROOT"/simulation/models/*/; do
    n=$(basename "$m"); ln -sfn "$m" "$PXM/${n}"; log "model  $n -> $PXM/$n"
done
for w in "$ISE_DRONE_ROOT"/simulation/worlds/*.sdf; do
    n=$(basename "$w"); ln -sfn "$w" "$PXW/$n"; log "world  $n -> $PXW/$n"
done
log "Pre-downloading Fuel models used by indoor_v1 (cached in ~/.gz/fuel)"
set +u; source "/opt/ros/$ROS_DISTRO/setup.bash" 2>/dev/null; set -u
for m in Chair Table Sofa "Standing person" Bookshelf OfficeChairBlack BedsideTable; do
    if ls ~/.gz/fuel/fuel.gazebosim.org/openrobotics/models/ 2>/dev/null | grep -qi "^$(echo "$m" | tr ' ' '_')$\|^${m,,}$"; then
        log "  cached: $m"
    else
        timeout 300 gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/$m" >/dev/null 2>&1 && log "  downloaded: $m" || warn "  failed: $m (world still loads; the model will be missing)"
    fi
done
log "Done. Launch with:  ros2 launch drone_bringup simulation.launch.py vehicle:=gz_x500_rgbd world:=indoor_v1"
