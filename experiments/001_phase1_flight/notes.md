# 001 — Phase 1 flight sequence acceptance (2026-10-14)

Setup: PX4 v1.17.0 SITL + Gazebo Harmonic 8.15 (headless) + Micro-XRCE-DDS-Agent v2.4.3 + ROS 2 Jazzy,
`px4_bridge.FlightController` (offboard position setpoints @ 10 Hz). Launched by `scripts/run_phase1_test.sh 10 gz_x500`.

Sequence per run: Arm → Takeoff 1.5 m → Hover 5 s → goto +2 m (East) → Hover 2 s → return_home → Hover 1 s → Land.

## Result: 10 / 10 consecutive runs passed (session 20261014_161541)

| run | total [s] | takeoff [s] | goto [s] | return [s] | land [s] | hover max err [m] | goto final err [m] | return final err [m] |
|---|---|---|---|---|---|---|---|---|
| 1 | 36.3 | 15.9 | 3.2 | 1.7 | 6.0 | 0.16 | 0.11 | 0.13 |
| 2 | 29.9 | 8.3 | 3.1 | 3.1 | 6.0 | 0.16 | 0.10 | 0.11 |
| 3 | 29.9 | 8.3 | 3.1 | 3.2 | 6.0 | 0.16 | 0.06 | 0.09 |
| 4 | 30.4 | 8.4 | 3.6 | 3.1 | 6.0 | 0.16 | 0.14 | 0.09 |
| 5 | 28.9 | 7.1 | 3.3 | 3.2 | 6.0 | 0.15 | 0.12 | 0.08 |
| 6 | 28.6 | 6.9 | 3.1 | 3.3 | 6.0 | 0.15 | 0.12 | 0.08 |
| 7 | 35.7 | 15.4 | 3.5 | 1.7 | 5.9 | 0.16 | 0.09 | 0.06 |
| 8 | 30.0 | 8.4 | 3.2 | 3.2 | 6.0 | 0.15 | 0.08 | 0.11 |
| 9 | 29.9 | 8.4 | 3.2 | 3.1 | 6.0 | 0.16 | 0.10 | 0.10 |
| 10 | 29.9 | 8.3 | 3.2 | 3.1 | 6.0 | 0.16 | 0.10 | 0.09 |

Raw data: `metrics.csv`, `trajectories/*.csv` (10 Hz position + setpoint samples), `summary.png`, `run05.png`,
`test_output.txt`, `sitl.log`, `fmu_topics.txt`.
Timing columns use the wall clock, which on this WSL2 host is stepped backwards (see below); runs 1 and 7 show
the artefact (takeoff "15 s", return "1.7 s").

## Failsafe check (`tests/simulation/test_failsafe.py`) — PASS
Offboard stream stopped mid-hover → PX4 left OFFBOARD after 2.36 s (COM_OF_LOSS_T = 3 s minus stream age) →
AUTO_RTL (offboard-loss action without RC falls back to RTL) → landed and disarmed. Output: `failsafe_output.txt`.
Note: once disarmed, nav_state reports OFFBOARD again (PX4 restores the user-intended mode while disarmed).

## gz_x500_depth — PASS (1 run, Gazebo GUI)
Same sequence with the depth-camera airframe: `SEQUENCE OK in 47.4 s` (`depth_demo_output.txt`).
Screenshots via WSLg: `gazebo_x500_depth_hover.png`, `gazebo_x500_depth_goto.png` (drone airborne, shadow on ground).
Real-time factor with the GUI ≈ 72 %, headless ≈ 100 %.

## Problems found and fixed on the way
1. First flights ended in AUTO_RTL seconds after arming. `failsafe_flags.offboard_control_signal_lost` was the
   trigger. Root cause: the WSL2 wall clock is stepped backwards by 0.3–0.8 s every ~10 s (kernel `tick` drifts
   away from 10000 µs and is corrected by the host time sync); PX4 converts incoming timestamps with the XRCE
   timesync offset, so the stream looked older than COM_OF_LOSS_T = 1 s. Mitigation: SITL launch sets
   `COM_OF_LOSS_T=3`; px4_bridge drives its 10 Hz heartbeat timer from a steady clock. Switching clocksource,
   stopping systemd-timesyncd or disabling `hv_utils.timesync_implicit` did not remove the steps (restored to
   original state afterwards).
2. `ros2 launch` started as a background job ignores SIGINT (non-interactive shells ignore SIGINT for `&` jobs);
   the runner uses `set -m` and `kill -INT -- -<pgid>` so Ctrl-C semantics reach PX4 and Gazebo.
3. ament_python copies launch files / modules on `colcon build --symlink-install`; the runner rebuilds
   `px4_bridge` + `drone_bringup` before every session.
