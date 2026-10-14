#!/usr/bin/env bash
# One-shot Phase 1 acceptance run: start SITL headless, fly N sequences, plot, stop SITL.
#   scripts/run_phase1_test.sh [runs=10] [vehicle=gz_x500]
set -euo pipefail
RUNS="${1:-10}"
VEHICLE="${2:-gz_x500}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# ROS setup scripts reference unset variables; relax `set -u` while sourcing.
set +u
# shellcheck disable=SC1091
source "$ROOT/scripts/env.sh"
set -u

LOG_DIR="$ROOT/experiments/001_phase1_flight"
mkdir -p "$LOG_DIR"

# ament_python copies launch files and python modules on install; rebuild so edits are live.
echo "[phase1] colcon build (px4_bridge, drone_bringup)"
(cd "$ROOT/ros2_ws" && colcon build --symlink-install --packages-select px4_bridge drone_bringup > "$LOG_DIR/colcon_build.log" 2>&1) \
    || { echo "[phase1] colcon build failed, see $LOG_DIR/colcon_build.log"; exit 2; }
set +u; source "$ROOT/ros2_ws/install/setup.bash"; set -u

echo "[phase1] launching SITL ($VEHICLE, headless)"
# Job control on: the launch gets its own process group, so a SIGINT to that group behaves
# exactly like Ctrl-C in a terminal (without it, background jobs ignore SIGINT entirely).
set -m
PYTHONUNBUFFERED=1 ros2 launch drone_bringup sitl.launch.py vehicle:="$VEHICLE" headless:=true > "$LOG_DIR/sitl.log" 2>&1 &
LAUNCH_PID=$!
set +m
cleanup() {
    echo "[phase1] stopping SITL (pgid $LAUNCH_PID)"
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 15); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
    kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "[phase1] waiting for PX4 telemetry on ROS 2 ..."
for _ in $(seq 1 60); do
    if ros2 topic list 2>/dev/null | grep -q '/fmu/out/vehicle_status'; then break; fi
    sleep 2
done
ros2 topic list | grep '^/fmu/' | sort > "$LOG_DIR/fmu_topics.txt"
echo "[phase1] $(wc -l < "$LOG_DIR/fmu_topics.txt") /fmu topics visible"

python3 "$ROOT/tests/simulation/test_flight_sequence.py" --runs "$RUNS" 2>&1 | tee "$LOG_DIR/test_output.txt"
RC=${PIPESTATUS[0]}
python3 "$ROOT/tools/plot_flight.py" "$LOG_DIR" || true
echo "[phase1] exit=$RC"
exit "$RC"
