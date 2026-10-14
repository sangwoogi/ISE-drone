#!/usr/bin/env bash
# Phase 2 acceptance: simulation.launch.py (headless, no RViz) + action-driven sequences + cancel test.
#   scripts/run_phase2_test.sh [runs=5] [vehicle=gz_x500_depth]
set -euo pipefail
RUNS="${1:-5}"
VEHICLE="${2:-gz_x500_depth}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u; source "$ROOT/scripts/env.sh"; set -u
LOG_DIR="$ROOT/experiments/002_phase2_bringup"; mkdir -p "$LOG_DIR"

echo "[phase2] colcon build (px4_bridge, drone_bringup)"
(cd "$ROOT/ros2_ws" && colcon build --symlink-install --packages-select px4_bridge drone_bringup > "$LOG_DIR/colcon_build.log" 2>&1) \
    || { echo "[phase2] build failed, see $LOG_DIR/colcon_build.log"; exit 2; }
set +u; source "$ROOT/ros2_ws/install/setup.bash"; set -u

echo "[phase2] ros2 launch drone_bringup simulation.launch.py vehicle:=$VEHICLE headless:=true rviz:=false"
set -m
PYTHONUNBUFFERED=1 ros2 launch drone_bringup simulation.launch.py vehicle:="$VEHICLE" headless:=true rviz:=false > "$LOG_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!
set +m
cleanup() {
    echo "[phase2] stopping launch (pgid $LAUNCH_PID)"
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 15); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
    kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

for _ in $(seq 1 60); do ros2 topic list 2>/dev/null | grep -q '^/drone/vehicle_state$' && break; sleep 2; done
sleep 5
set +e   # the bring-up check is informational; a slow `topic hz` must not abort the run
{
  echo "# topics after launch ($(date))"; ros2 topic list | grep -E '^/(drone|px4_bridge|fmu/out/vehicle_(status|local_position|attitude))'
  echo "# rates"; for t in /drone/vehicle_state /drone/pose /drone/odom; do printf '%-22s ' $t; { timeout 8 ros2 topic hz $t 2>/dev/null | grep -m1 average; } || true; done
  echo "# tf"; timeout 8 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | grep -m1 -A2 Translation
  echo "# actions"; ros2 action list
} 2>&1 | tee "$LOG_DIR/bringup_check.txt"
set -e

python3 "$ROOT/tests/simulation/test_phase2_actions.py" --runs "$RUNS" 2>&1 | tee "$LOG_DIR/test_output.txt"
RC=${PIPESTATUS[0]}
echo "[phase2] exit=$RC"
exit "$RC"
