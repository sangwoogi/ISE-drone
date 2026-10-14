# ISE-drone — Indoor Semantic Exploration Drone (software)

PX4 + Gazebo Harmonic + ROS 2 Jazzy 기반 실내 자율 Semantic Exploration 드론의 소프트웨어 스택.
전체 계획은 [`docs/indoor_semantic_exploration_drone_software_plan.md`](docs/indoor_semantic_exploration_drone_software_plan.md),
인터페이스 계약은 [`docs/ros_interfaces.md`](docs/ros_interfaces.md), **이어서 작업하려면 [`docs/HANDOFF.md`](docs/HANDOFF.md)** 를 먼저 읽는다.

## 현재 상태

| Phase | 내용 | 상태 |
|---|---|---|
| 0 | 개발 환경 (Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic / PX4 v1.17.0 / uXRCE-DDS) | **완료** — `experiments/000_phase0_env/` |
| 1 | PX4 + Gazebo 비행: `takeoff / hover / goto / return_home / land`, 10회 연속 시퀀스 | **완료** (10/10, failsafe OK, gz_x500_depth OK) — `experiments/001_phase1_flight/` |
| — | 전 Phase 베이스라인 골격 (계약·스텁 노드·launch, 전체 체인 스모크 통과) | **완료** — `docs/phases/baseline.md` |
| 2 | ROS 2 bringup: 단일 launch(PX4·Gazebo·Agent·Bridge·RViz), FlightCommand 액션 5/5, cancel→hold, `/drone/odom` | **완료** — `experiments/002_phase2_bringup/` |
| 3 | Sensor pipeline: 실내 world `indoor_v1`, 정합 RGB-D 모델 `x500_rgbd`, `/camera/*` 검증, rosbag 기록/재생, GT 추출 | **완료** — `experiments/003_phase3_sensors/` |
| 4 | SLAM: RTAB-Map(RGB-D) + PX4 EKF odometry, ATE 0.075 m / lost 0 / 2D·3D 지도, VO 모드 실험 포함 | **완료** — `experiments/004_phase4_slam/` |
| 5+ | YOLO, Semantic Map, LLM, Mission Manager (실구현) | 예정 |

## 저장소 구조

```
config/px4/            px4_bridge 파라미터(토픽명 등) 포인터
docs/                  계획서, 아키텍처, 인터페이스 계약, phases/ (Phase별 구현 기록)
experiments/           실험 결과 (metrics.csv, trajectories/, plots, notes.md)
ros2_ws/src/
  drone_interfaces/    msg/srv/action 계약 (SemanticObject, VehicleState, MissionStatus, QueryObjects, AskLLM, FlightCommand, Mission)
  px4_bridge/          px4_msgs를 캡슐화한 비행 API (FlightController) + px4_bridge_node (action, /drone/*, TF)
  drone_bringup/       sitl / simulation / full_simulation launch, RViz config
  sensor_adapter/      Gazebo(ros_gz_bridge) / 실카메라 → 공통 /camera/* 토픽          (Phase 3)
  drone_slam/          RTAB-Map launch, ground-truth 노드, ATE/RPE 평가                (Phase 4)
  object_detector/     DetectorBackend(mock|ultralytics|onnx) → vision_msgs Detection2DArray (Phase 5)
  object_tracker/      IoU tracker → /perception/tracks                                 (Phase 6)
  semantic_mapper/     bbox+depth+TF → 3D, association, relations, markers             (Phase 6)
  semantic_database/   SQLite + /semantic_database/query                               (Phase 6)
  llm_interface/       Ollama/mock backend, DB tools, QueryAgent, /llm/ask, CLI        (Phase 7)
  mission_manager/     상태기계 + 안전검증 + /mission/execute action                    (Phase 8)
  exploration/         frontier 검출 → /exploration/goal                                (Phase 8)
  system_monitor/      CPU/RAM/온도/토픽 Hz → /diagnostics, CSV
  px4_msgs/, Micro-XRCE-DDS-Agent/   외부 (setup 스크립트가 clone, git 미추적)
scripts/setup/         Phase 0 설치 스크립트 (01 ROS 2, 02 PX4+Gazebo, 03 colcon, 04 QGC)
scripts/env.sh         ROS 작업 쉘 환경 (conda 비활성화 + ROS/ws source)
tests/simulation/      Phase 1 반복 비행 테스트, failsafe 테스트
tools/                 plot_flight.py 등
```

## 설치 (Phase 0)

```bash
bash scripts/setup/01_ros2_jazzy.sh     # sudo, ROS 2 Jazzy desktop + ros_gz
bash scripts/setup/02_px4.sh            # sudo, PX4 v1.17.0 clone + toolchain + Gazebo Harmonic + SITL build
bash scripts/setup/03_ros2_ws.sh        # px4_msgs + XRCE agent clone, colcon build
bash scripts/setup/04_qgc.sh            # (선택) QGroundControl AppImage
bash scripts/setup/05_sim_assets.sh     # 저장소 모델/world를 PX4 트리에 링크 + Fuel 모델 캐시
```

> WSL2: 홈 디렉토리(Linux 파일시스템)에서 작업한다. Gazebo GUI는 WSLg로 표시되며, GUI 없이 돌리려면 `headless:=true`.
> conda가 쉘에 자동 활성화되어 있으면 ROS 2가 깨지므로 모든 ROS 쉘에서 `source scripts/env.sh`를 먼저 실행한다.
> WSL2의 wall clock은 주기적으로 되돌려진다(≈0.3–0.8 s). SITL launch는 `COM_OF_LOSS_T=3`을 기본 적용해 offboard 스트림이 '손실'로 오판되지 않게 한다 (`docs/ros_interfaces.md` §5).

## 실행 (Phase 1)

```bash
source scripts/env.sh

# 터미널 1: PX4 SITL + Gazebo + XRCE agent
ros2 launch drone_bringup sitl.launch.py                 # GUI
ros2 launch drone_bringup sitl.launch.py headless:=true  # GUI 없음

# 터미널 2: 비행 시퀀스 1회 (Arm → Takeoff 1.5 m → Hover 5 s → +2 m → Hover → Return → Land)
ros2 run px4_bridge flight_demo

# Phase 2 bringup: SITL + px4_bridge 노드 + RViz, 액션으로 비행 명령
ros2 launch drone_bringup simulation.launch.py
ros2 action send_goal /px4_bridge/flight_command drone_interfaces/action/FlightCommand "{command: TAKEOFF, height: 1.2}"

# 전체 스택 (mock detector / mock LLM / SLAM 제외 예시)
ros2 launch drone_bringup full_simulation.launch.py enable_slam:=false detector_backend:=mock llm_backend:=mock
ros2 action send_goal /mission/execute drone_interfaces/action/Mission "{action: TAKEOFF, param: 1.2}"
ros2 run llm_interface llm_cli "의자는 몇 개야?"

# Phase 1 통과 조건: 10회 연속 성공
python3 tests/simulation/test_flight_sequence.py --runs 10
python3 tools/plot_flight.py experiments/001_phase1_flight     # summary.png

# Failsafe 확인: 호버 중 offboard heartbeat 중단 → PX4가 OFFBOARD 이탈 + 자동 착륙
python3 tests/simulation/test_failsafe.py

# 위 전부를 한 번에 (SITL headless 자동 기동/종료)
scripts/run_phase1_test.sh 10
scripts/run_phase2_test.sh 5          # Phase 2: 액션 경로 5회 + cancel→hold
# Phase 3: 실내 world + RGB-D 센서
ros2 launch drone_bringup simulation.launch.py vehicle:=gz_x500_rgbd world:=indoor_v1 use_sim_time:=true
ros2 launch sensor_adapter gazebo_sensors.launch.py
python3 tests/simulation/test_phase3_sensors.py
scripts/run_phase4_slam.sh px4 2       # Phase 4: RTAB-Map + 2바퀴 평가 비행 (vo | vo_guess | px4)
```

## Mission API

```python
from px4_bridge.flight_controller import create_controller
fc = create_controller()
fc.takeoff(1.5); fc.hover(5.0)
fc.goto(2.0, 0.0, 1.5); fc.return_home(); fc.land()
```
ENU 좌표(m), yaw rad. NED 변환과 px4_msgs는 `px4_bridge` 내부에만 존재한다.

## 단위 테스트

```bash
cd ros2_ws && colcon test --packages-select px4_bridge && colcon test-result --verbose
```
