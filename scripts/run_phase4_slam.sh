#!/usr/bin/env bash
# Phase 4 acceptance: indoor_v1 + RGB-D + RTAB-Map, scripted loop flight, ATE/RPE vs Gazebo ground truth.
#   scripts/run_phase4_slam.sh [odom_source=vo|vo_guess|px4] [laps=2] [run_name]
#   vo_guess = visual odometry with PX4 EKF as motion prior (TF odom -> odom_px4 -> base_link)
set -euo pipefail
ODOM="${1:-px4}"; LAPS="${2:-2}"; NAME="${3:-${ODOM}_$(date +%Y%m%d_%H%M%S)}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u; source "$ROOT/scripts/env.sh"; set -u
E="$ROOT/experiments/004_phase4_slam/$NAME"; mkdir -p "$E"
export DISPLAY="${DISPLAY:-:0}" PYTHONUNBUFFERED=1 GZ_IP=127.0.0.1

echo "[phase4] colcon build"
(cd "$ROOT/ros2_ws" && colcon build --symlink-install --packages-select px4_bridge drone_bringup sensor_adapter drone_slam > "$E/colcon_build.log" 2>&1) \
    || { echo "[phase4] build failed, see $E/colcon_build.log"; exit 2; }
set +u; source "$ROOT/ros2_ws/install/setup.bash"; set -u

PX4_TF=true; PX4_ODOM_FRAME=odom; VO_GUESS=false; ODOM_ARG=$ODOM
case "$ODOM" in
  vo) PX4_TF=false ;;
  vo_guess) ODOM_ARG=vo; VO_GUESS=true; PX4_ODOM_FRAME=odom_px4 ;;
esac
# slow, smooth flight for visual odometry (m/s, m/s^2)
PARAMS="NAV_DLL_ACT=0,COM_OF_LOSS_T=3,SIM_BAT_DRAIN=3600,SIM_BAT_MIN_PCT=90,MPC_XY_VEL_MAX=1.0,MPC_ACC_HOR=1.0,MPC_Z_VEL_MAX_UP=1.0,MPC_YAWRAUTO_MAX=45,MC_YAWRATE_MAX=40"
pids=()
start() { set -m; "$@" & pids+=($!); set +m; }
cleanup() {
    echo "[phase4] stopping (${pids[*]})"
    for p in "${pids[@]}"; do kill -INT -- "-$p" 2>/dev/null || true; done
    for _ in $(seq 1 20); do kill -0 "${pids[0]}" 2>/dev/null || break; sleep 1; done
    for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null || true; done
}
trap cleanup EXIT

echo "[phase4] simulation (gz_x500_rgbd, indoor_v1, headless, px4_tf=$PX4_TF)"
start ros2 launch drone_bringup simulation.launch.py vehicle:=gz_x500_rgbd world:=indoor_v1 headless:=true rviz:=false \
      use_sim_time:=true px4_tf:=$PX4_TF px4_odom_frame:=$PX4_ODOM_FRAME "px4_params:=$PARAMS" > "$E/launch_sim.log" 2>&1
for _ in $(seq 1 75); do ros2 topic list 2>/dev/null | grep -q '^/drone/vehicle_state$' && break; sleep 2; done; sleep 8
echo "[phase4] sensors"
start ros2 launch sensor_adapter gazebo_sensors.launch.py > "$E/launch_sensors.log" 2>&1
sleep 6
echo "[phase4] rtabmap (odom_source=$ODOM) + ground truth + recorder"
start ros2 launch drone_slam rtabmap.launch.py odom_source:=$ODOM_ARG vo_guess:=$VO_GUESS db_path:="$E/rtabmap.db" record_dir:="$E" > "$E/launch_slam.log" 2>&1
for _ in $(seq 1 45); do ros2 topic list 2>/dev/null | grep -q '^/ground_truth/pose$' && break; sleep 2; done; sleep 10
{ echo "# topics"; ros2 topic list | grep -E '^/(slam|ground_truth|camera/color/image_raw|drone/odom)' ;
  echo "# rates"; for t in /ground_truth/pose /slam/odom /camera/color/image_raw; do printf '%-26s ' $t; { timeout 8 ros2 topic hz $t 2>/dev/null | grep -m1 average; } || echo; done;
  echo "# tf"; { timeout 8 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -m1 -A1 Translation; } || true; } 2>&1 | tee "$E/bringup_check.txt"

python3 "$ROOT/tests/simulation/test_phase4_slam.py" --run-dir "$E" --laps "$LAPS" 2>&1 | tee "$E/test_output.txt"
RC=${PIPESTATUS[0]}
echo "[phase4] exit=$RC  (results in $E)"
exit "$RC"
