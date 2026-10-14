#!/usr/bin/env bash
# Optional: QGroundControl Linux AppImage (runs inside WSL via WSLg, auto-connects to SITL).
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/00_common.sh"
DEST="$ISE_DRONE_ROOT/tools/qgc"
mkdir -p "$DEST"
APP="$DEST/QGroundControl.AppImage"
if [ ! -f "$APP" ]; then
    log "Downloading QGroundControl AppImage"
    curl -L -o "$APP" https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage
    chmod +x "$APP"
fi
sudo apt-get install -y libfuse2t64 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 >/dev/null
log "Run: $APP"
