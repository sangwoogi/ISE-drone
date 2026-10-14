# 000 — Phase 0 environment verification (2026-10-14)

Machine: WSL2 (kernel 6.18.54), Ubuntu 24.04.4, 8 vCPU, 27 GB RAM, RTX 5060 Ti, WSLg.

| Check | Result | Evidence |
|---|---|---|
| ROS 2 Jazzy desktop + ros_gz | installed, 319 packages, `rclpy` import OK | `scripts/setup/01_ros2_jazzy.sh` log |
| PX4-Autopilot v1.17.0 SITL build | `build/px4_sitl_default/bin/px4` built (1141 steps) | `scripts/setup/02_px4.sh` log |
| Gazebo Harmonic | `gz sim` 8.15.0, X500 spawns in `default` world | `gazebo_x500_gui.png` (WSLg window capture) |
| px4_msgs v1.17.0 + Micro-XRCE-DDS-Agent v2.4.3 | colcon build OK (5 packages, 3 min) | `scripts/setup/03_ros2_ws.sh` log |
| PX4 → ROS 2 telemetry | `/fmu/out/vehicle_local_position_v1` at 50 Hz, `pre_flight_checks_pass: true` | `fmu_topics.txt` (topic list) |
| Launch | `ros2 launch drone_bringup sitl.launch.py headless:=true` brings up agent + PX4 + Gazebo, Ctrl-C stops all | this session |

Findings that shaped the code:
- Jazzy's Gazebo bridge package is `ros-jazzy-ros-gz` (no `-gzharmonic` suffix as in Humble).
- PX4 ≥ 1.16 appends `_v1` to versioned topics (`vehicle_status_v1`, `vehicle_local_position_v1`).
- Running `px4` without a TTY spawns the `pxh>` shell which spins on EOF (1 GB of log in 75 s) → launch uses `px4 -d` (daemon mode).
- `make px4_sitl gz_x500` ≡ `cd build/px4_sitl_default/rootfs && PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 ../bin/px4`.
- gz_x500 airframe sets `NAV_DLL_ACT=2` → "No connection to the GCS" blocks arming without QGC; `PX4_PARAM_*` env overrides are applied *before* the airframe file and are overwritten, so the launch applies overrides after boot with `px4-param set`.
- Background `ros2 launch` must be started with job control (`set -m`) and stopped with `kill -INT -- -<pgid>`; without job control the launch ignores SIGINT, and SIGTERM orphans PX4/Gazebo.
- WSL2 wall clock is stepped backwards (~0.3–0.8 s every ~10 s, kernel `tick` drifts off 10000). PX4 converts received timestamps with the XRCE timesync offset, so the offboard stream looked stale → `offboard_control_signal_lost` → RTL. SITL launch sets `COM_OF_LOSS_T=3`; the px4_bridge heartbeat timer runs on a steady clock.
