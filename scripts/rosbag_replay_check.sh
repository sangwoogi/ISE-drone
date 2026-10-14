#!/usr/bin/env bash
# Replay a bag (no simulator running) and verify the common sensor topics come back at a sane rate.
#   scripts/rosbag_replay_check.sh <bag_dir>
set -euo pipefail
BAG="${1:?usage: rosbag_replay_check.sh <bag_dir>}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u; source "$ROOT/scripts/env.sh"; source "$ROOT/ros2_ws/install/setup.bash"; set -u
set -m; ros2 bag play "$BAG" --clock --loop > /dev/null 2>&1 & PP=$!; set +m
sleep 3
RC=0
for t in /camera/color/image_raw /camera/depth/image_raw /camera/color/camera_info /tf; do
  printf '%-30s ' "$t"
  r=$( { timeout 8 ros2 topic hz "$t" 2>/dev/null | grep -m1 average; } || true )
  echo "${r:-no data}"; [ -n "$r" ] || RC=1
done
kill -INT -- "-$PP" 2>/dev/null || true; sleep 1; kill -KILL -- "-$PP" 2>/dev/null || true
echo "[replay] $([ $RC -eq 0 ] && echo PASS || echo FAIL)"; exit $RC
