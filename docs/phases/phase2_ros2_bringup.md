# Phase 2 — ROS 2 + PX4 Interface (bringup)

**완료일** 2026-10-14 · **증거** `experiments/002_phase2_bringup/`

## 목적
계획서 §12. PX4를 ROS 2에서 완전히 제어하고 telemetry를 읽는다. 상위 노드는 `px4_msgs`를 직접 쓰지 않고
`px4_bridge`의 액션/토픽만 사용한다. `ros2 launch drone_bringup simulation.launch.py` 하나로
PX4 · Gazebo · DDS Agent · Bridge · RViz가 올라와야 한다.

## 사용자 결정
- cancel → **현재 위치 호버 유지**(OFFBOARD 유지, 결과 CANCELED). LAND는 cancel 거부.
- 수락 테스트: 액션 경로로만 시퀀스 **5회** + cancel 테스트.
- 기본 기체 `gz_x500_depth`.
- `/drone/odom`(nav_msgs/Odometry) 추가, TF/pose에 전체 자세.

## 구현 파일

| 파일 | 변경 |
|---|---|
| `px4_bridge/frames.py` | FRD→NED 쿼터니언을 FLU→ENU로 변환 (`quat_ned_frd_to_enu_flu`), 각속도 변환, 단위 테스트 7개 추가 |
| `px4_bridge/px4_interface.py` | `/fmu/out/vehicle_attitude` 구독 → `orientation_enu`; `hold_current_position()` |
| `px4_bridge/flight_controller.py` | `cancel_check` 폴링 → `FlightCancelled`(호버 유지), `hold()` |
| `px4_bridge/px4_bridge_node.py` | `/drone/odom` 발행, 전체 자세 TF, cancel→hold, LAND cancel 거부, 액션 결과 CANCELED |
| `drone_bringup/launch/simulation.launch.py` | SITL + Agent + px4_bridge(5 s 지연) + RViz + map→odom static, `use_sim_time` 인자 |
| `drone_bringup/config/drone.rviz` | TF, `/drone/pose`, 카메라/탐지/SLAM/semantic 마커 디스플레이 |
| `tests/simulation/test_phase2_actions.py` | ActionClient로 TAKEOFF→HOVER→GOTO→HOVER→RETURN_HOME→LAND N회, cancel→hold 검증, 텔레메트리 CSV |
| `scripts/run_phase2_test.sh` | 빌드 → launch(headless) → 점검(`bringup_check.txt`) → 테스트 |

## 실행
```bash
source scripts/env.sh
ros2 launch drone_bringup simulation.launch.py                       # GUI: Gazebo + RViz
ros2 action send_goal /px4_bridge/flight_command drone_interfaces/action/FlightCommand "{command: TAKEOFF, height: 1.5}" --feedback
ros2 action send_goal /px4_bridge/flight_command drone_interfaces/action/FlightCommand "{command: GOTO, target: {x: 2.0, y: 1.0, z: 1.5}}"
ros2 topic echo /drone/odom
scripts/run_phase2_test.sh 5                                          # 수락 테스트 (headless)
```

## 통과 조건과 결과

**통과 조건(§12)**: `ros2 launch drone_bringup simulation.launch.py` 하나로 PX4·Gazebo·DDS Agent·Bridge·RViz 기동.
**추가 기준(사용자 결정)**: 액션 경로로만 시퀀스 5회 연속 성공 + cancel→hold 검증.

### 단일 launch 기동 (`bringup_check.txt`)
- 토픽: `/drone/vehicle_state`, `/drone/pose`, `/drone/odom` 각 **20 Hz**; `/fmu/out/vehicle_attitude`, `vehicle_local_position_v1`, `vehicle_status_v1` 수신
- TF `odom → base_link` 발행 (전체 자세, RPY 확인)
- 액션 `/px4_bridge/flight_command` 등록
- GUI 실행 시 Gazebo Sim 창 + RViz(`drone.rviz`) 창 기동: `gazebo_gui_goto.png`, `rviz_goto.png`, `rviz_hover.png`
  (RViz: TF odom/base_link, 드론 pose 화살표, `/drone/odom` 궤적. 이미지 패널은 Phase 3 센서 연결 전까지 "No Image")

### 액션 경로 반복 (`metrics.csv`, `test_output.txt`, `summary.png`) — **5 / 5 PASS**

| run | total [s] | takeoff | hover | goto | return | land | goto err [m] | return err [m] |
|---|---|---|---|---|---|---|---|---|
| 1 | 39.5 | ok | ok | ok | ok | ok | 0.08 | 0.15 |
| 2 | 38.6 | ok | ok | ok | ok | ok | 0.10 | 0.11 |
| 3 | 41.1 | ok | ok | ok | ok | ok | 0.09 | 0.09 |
| 4 | 38.1 | ok | ok | ok | ok | ok | 0.11 | 0.10 |
| 5 | 41.7 | ok | ok | ok | ok | ok | 0.15 | 0.12 |

모든 단계가 액션 결과 `SUCCEEDED`; 텔레메트리는 `/drone/vehicle_state`만으로 기록(`telemetry_*.csv`).

### cancel → hold (`cancel_test.txt`) — **PASS**
GOTO(+3 m) 송신 1.5 s 후 cancel → 결과 `CANCELED`, 3 s 후 드리프트 0.38 m, `armed=True`, `nav=OFFBOARD`(호버 유지). 이후 LAND 정상.

## 알려진 문제 / 교훈
- `ros2 topic hz … | grep -m1`은 grep이 먼저 끝나 `timeout`이 124를 반환한다. `set -e -o pipefail` 스크립트에서는 점검 블록을 `set +e`로 감싼다.
- WSLg에서 RViz 창의 X11 이름은 `rviz2`(제목 표시줄과 다름). 캡처는 `xwininfo -root -tree`로 id를 찾아 `import -window`.
- rclpy `ServerGoalHandle.is_cancel_requested`는 속성이다. 콜백으로 넘길 때는 람다로 감싼다.
- `drone_bringup/setup.py`의 data_files glob이 `config/*.yaml`만 포함해 `drone.rviz`가 설치되지 않았고 RViz가 기본 설정(Grid만)으로 떴다. `*.rviz`를 glob에 추가. RViz 2 설정은 디스플레이마다 `Enabled: true`와 `Topic:{Value, Reliability Policy, ...}` 구조가 필요하다.
- 세션 중 WSL 연결이 끊겨도 산출물·빌드는 유지되며, 재접속 후 잔여 프로세스 없이 이어서 진행할 수 있었다.

## 설계 결정
- 액션 서버는 `ReentrantCallbackGroup` + `MultiThreadedExecutor`(4 threads): 실행 중에도 하트비트 타이머·텔레메트리 콜백이 돈다.
- FlightController는 `spin=False`로 노드에 내장되고, 진행 상황은 `progress_cb`로 액션 feedback에 연결된다.
- cancel은 blocking primitive 내부에서 폴링되며, 취소 즉시 setpoint를 현재 위치로 바꾼다(자세 setpoint 없음, 위치 모드).
- `/drone/odom`의 각속도는 0이다. PX4 `dds_topics.yaml`에서 `vehicle_angular_velocity`가 주석 처리되어 있어 발행되지 않는다(필요 시 PX4 빌드 설정 변경).
- RViz는 `output='log'`로 띄워 콘솔을 어지럽히지 않는다. 헤드리스 테스트는 `rviz:=false headless:=true`.
