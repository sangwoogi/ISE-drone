# ISE-drone — Indoor Semantic Exploration Drone (software)

PX4 + Gazebo Harmonic + ROS 2 Jazzy 기반 실내 자율 Semantic Exploration 드론의 소프트웨어 스택.
전체 계획은 [`docs/indoor_semantic_exploration_drone_software_plan.md`](docs/indoor_semantic_exploration_drone_software_plan.md),
인터페이스 계약은 [`docs/ros_interfaces.md`](docs/ros_interfaces.md) 참조.

## 현재 상태

| Phase | 내용 | 상태 |
|---|---|---|
| 0 | 개발 환경 (Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic / PX4 v1.17.0 / uXRCE-DDS) | **완료** — `experiments/000_phase0_env/` |
| 1 | PX4 + Gazebo 비행: `takeoff / hover / goto / return_home / land`, 10회 연속 시퀀스 | **완료** (10/10, failsafe OK, gz_x500_depth OK) — `experiments/001_phase1_flight/` |
| 2+ | ROS 2 bringup, sensor adapter, SLAM, YOLO, Semantic Map, LLM, Mission Manager | 예정 |

## 저장소 구조

```
config/px4/            px4_bridge 파라미터(토픽명 등) 포인터
docs/                  계획서, 인터페이스 계약
experiments/           실험 결과 (metrics.csv, trajectories/, plots)
ros2_ws/src/
  px4_bridge/          px4_msgs를 캡슐화한 비행 API (FlightController)
  drone_bringup/       launch 파일 (sitl.launch.py)
  drone_interfaces/    커스텀 메시지 (Phase 5+)
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

# Phase 1 통과 조건: 10회 연속 성공
python3 tests/simulation/test_flight_sequence.py --runs 10
python3 tools/plot_flight.py experiments/001_phase1_flight     # summary.png

# Failsafe 확인: 호버 중 offboard heartbeat 중단 → PX4가 OFFBOARD 이탈 + 자동 착륙
python3 tests/simulation/test_failsafe.py

# 위 전부를 한 번에 (SITL headless 자동 기동/종료)
scripts/run_phase1_test.sh 10
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
