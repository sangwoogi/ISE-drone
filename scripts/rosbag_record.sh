#!/usr/bin/env bash
# Record the common sensor + state topics (plan §16).  scripts/rosbag_record.sh <out_dir> [seconds=30]
set -euo pipefail
OUT="${1:?usage: rosbag_record.sh <out_dir> [seconds]}"; SEC="${2:-30}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u; source "$ROOT/scripts/env.sh"; source "$ROOT/ros2_ws/install/setup.bash"; set -u
rm -rf "$OUT"
echo "[rosbag] recording ${SEC}s -> $OUT"
timeout --signal=INT "$SEC" ros2 bag record -o "$OUT" --use-sim-time \
  /camera/color/image_raw /camera/depth/image_raw /camera/color/camera_info /camera/depth/camera_info \
  /tf /tf_static /drone/odom /drone/vehicle_state /fmu/out/vehicle_odometry /clock || true
ros2 bag info "$OUT" | sed -n '1,40p'
