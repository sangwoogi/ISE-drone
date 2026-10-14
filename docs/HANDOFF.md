# ISE-drone 인수인계 문서 (HANDOFF)

> 이 문서는 **다른 개발자/LLM 에이전트가 이 저장소를 처음 받아 곧바로 이어서 작업**할 수 있도록 쓴 것이다.
> 2026-10-14 기준, Phase 0~4 + 전 Phase 베이스라인 골격이 완료되어 있고, 다음 작업은 Phase 5(객체 탐지)다.
> 상세 계획은 `docs/indoor_semantic_exploration_drone_software_plan.md`(이하 "계획서"), 아키텍처는 `docs/software_architecture.md`,
> 인터페이스 계약은 `docs/ros_interfaces.md`, Phase별 구현 기록은 `docs/phases/`를 본다.

---

## 0. 한 줄 요약

실내 자율 Semantic Exploration 드론의 소프트웨어를 **PC 시뮬레이션(PX4 SITL + Gazebo Harmonic + ROS 2 Jazzy)** 에서 먼저 완성하고,
같은 ROS 2 인터페이스를 유지한 채 Raspberry Pi 5로 단계 이식한다. 최종 사용자 경험: `"방 안에 가방이 있어?"` → Semantic DB 조회 → `"bag_01, (2.8, −0.9, 0.4) m"`.

---

## 1. 작업 방식 (사용자와 합의된 규칙)

1. **Phase 단위로 진행**한다. 각 Phase 시작 전에 설계 선택이 필요하면 **사용자에게 먼저 질문**(AskUserQuestion)하고, 구현이 끝나면
   **증거(스크린샷·그래프·CSV·JSON·영상)를 남기고 보고한 뒤 멈춘다**. 다음 Phase는 사용자가 지시할 때 시작한다.
2. 각 Phase 구현 내용은 `docs/phases/phaseN_*.md`에 기록한다(목적 → 사용자 결정 → 구현 파일 → 실행 → 통과 조건과 결과 → 설계 결정 → 알려진 문제).
3. 증거는 `experiments/NNN_phaseN_*/`에 둔다(`notes.md`/`report.json`, PNG, CSV). rosbag·`rtabmap.db`·로그는 git 제외.
4. sudo(apt 설치 등)는 사용자가 자율 사용을 허용했다. 비밀번호는 저장소/문서에 절대 남기지 않는다.
5. 커밋 메시지 끝에 `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>` 를 붙인다(사용자 git 설정은 user.name 미설정 → `git -c user.name=...` 사용).
6. 계획서의 핵심 원칙을 지킨다: Hardware-independent first / Sim·Real 인터페이스 동일 / Ground truth는 평가 전용 / LLM과 로보틱스 분리 /
   `px4_msgs`는 `px4_bridge` 안에서만 / Gate 건너뛰기 금지.

---

## 2. 개발 환경 (현재 머신)

| 항목 | 값 |
|---|---|
| OS | Ubuntu 24.04.4 **WSL2** (kernel 6.18), WSLg(DISPLAY=:0), 8 vCPU, 27 GB RAM, RTX 5060 Ti |
| ROS 2 | Jazzy (`/opt/ros/jazzy`), `ros-jazzy-ros-gz`, `ros-jazzy-vision-msgs`, `ros-jazzy-cv-bridge`, `ros-jazzy-rtabmap-ros` 0.23.13 |
| PX4 | v1.17.0, `~/PX4-Autopilot` (저장소 밖), `build/px4_sitl_default` 빌드됨 |
| Gazebo | Harmonic 8.15 (PX4 `ubuntu.sh`가 설치) |
| px4_msgs / XRCE Agent | v1.17.0 / v2.4.3, `ros2_ws/src/`에 clone(git 제외) |
| Python | 시스템 3.12 (ROS용). conda가 셸에 자동 활성화되므로 **반드시 `source scripts/env.sh`** |
| Fuel 모델 캐시 | `~/.gz/fuel/fuel.gazebosim.org/openrobotics/models/{chair,table,sofa,standing person,bookshelf,officechairblack,bedsidetable}` |

### 설치 스크립트 (멱등)
```bash
bash scripts/setup/01_ros2_jazzy.sh    # sudo: ROS 2 Jazzy desktop + ros_gz + pip(empy 3.3.4 등)
bash scripts/setup/02_px4.sh           # sudo: PX4 v1.17.0 clone + ubuntu.sh --no-nuttx + make px4_sitl
bash scripts/setup/03_ros2_ws.sh       # px4_msgs/Agent clone + colcon build 전체
bash scripts/setup/04_qgc.sh           # (선택) QGroundControl AppImage
bash scripts/setup/05_sim_assets.sh    # simulation/models,worlds → PX4 gz 트리 symlink + Fuel 캐시
```
버전은 `scripts/setup/00_common.sh`(`PX4_VERSION`, `XRCE_AGENT_VERSION`) 한 곳에서 관리한다.

### 모든 ROS 셸에서
```bash
source scripts/env.sh        # conda deactivate, PYTHONNOUSERSITE=1, ROS + ros2_ws source, PX4_DIR, GZ_SIM_RESOURCE_PATH
```
`PYTHONNOUSERSITE=1`이 중요하다: `~/.local`의 numpy 2.x/setuptools가 시스템 OpenCV·colcon과 충돌한다(아래 §8).

### 빌드
```bash
cd ros2_ws && colcon build --symlink-install --packages-skip microxrcedds_agent px4_msgs
```
ament_python 패키지는 `--symlink-install`이어도 launch/config/모듈이 **복사**되므로 수정 후 해당 패키지를 재빌드한다
(러너 스크립트들은 시작 시 자동 재빌드).

---

## 3. 저장소 구조

```
ISE-drone/
├── README.md                     상태표 + 실행 요약
├── docs/
│   ├── indoor_semantic_exploration_drone_software_plan.md   원 계획서(§ 번호 참조의 기준)
│   ├── software_architecture.md  패키지 구조도, Phase 매핑, 원칙 체크리스트
│   ├── ros_interfaces.md         ★ 토픽/서비스/액션/TF/프레임/파라미터 계약 (변경 시 drone_interfaces와 함께 수정)
│   ├── HANDOFF.md                이 문서
│   └── phases/                   phase0_environment, phase1_flight, baseline, phase2_ros2_bringup, phase3_sensor_pipeline, phase4_slam
├── config/px4/px4_bridge.yaml → ros2_ws/src/px4_bridge/config/px4_bridge.yaml (symlink)
├── scripts/
│   ├── env.sh                    환경 소스
│   ├── setup/00..05_*.sh         설치
│   ├── run_phase1_test.sh        Phase 1 수락(10회 시퀀스, headless)
│   ├── run_phase2_test.sh        Phase 2 수락(액션 경로 5회 + cancel)
│   ├── run_phase4_slam.sh        Phase 4 수락([px4|vo|vo_guess] [laps] [name])
│   ├── rosbag_record.sh / rosbag_replay_check.sh
├── simulation/
│   ├── models/x500_rgbd/         PX4 x500 + OakD-Lite-RGBD (컬러·깊이 640×480 @20 Hz, FOV 1.274, 정합)
│   ├── models/OakD-Lite-RGBD/    카메라 모듈 (sensor topics: rgb_camera, depth_camera; CameraInfo는 공용 /camera_info)
│   ├── models/ise_textures/      절차적 텍스처 PNG (벽 12개 고유 + carpet/brick/wood)
│   ├── worlds/indoor_v1.sdf      Room A(6×5) – 복도(4×2) – Room B(5×5), 가구(Fuel, static) + 원시도형 + 패널
│   ├── worlds/indoor_v1_objects.json   객체 명세(클래스 매핑)
│   └── worlds/indoor_v1_gt.json  시뮬레이션에서 추출한 GT 좌표(평가 전용)
├── ros2_ws/src/
│   ├── drone_interfaces/         msg: SemanticObject(+Array), VehicleState, MissionStatus · srv: QueryObjects, AskLLM · action: FlightCommand, Mission
│   ├── px4_bridge/               ★ px4_msgs를 쓰는 유일한 패키지 (아래 §4)
│   ├── drone_bringup/            sitl / simulation / full_simulation launch, drone.rviz
│   ├── sensor_adapter/           ros_gz_bridge 설정 + relay 노드 + 정적 카메라 TF
│   ├── drone_slam/               RTAB-Map launch/params, gt_pose_node, trajectory_recorder, slam_eval
│   ├── object_detector/          DetectorBackend(mock|ultralytics|onnx) + detector_node  [Phase 5 대상]
│   ├── object_tracker/           IoU tracker                                              [Phase 6]
│   ├── semantic_mapper/          projection / association / relations / node + markers     [Phase 6]
│   ├── semantic_database/        SQLite db.py + 노드(QueryObjects 서비스)                  [Phase 6]
│   ├── llm_interface/            backends(ollama|mock|cloud) + tools + QueryAgent + llm_node + llm_cli [Phase 7]
│   ├── mission_manager/          state_machine + safety_validator + Mission 액션 서버      [Phase 8]
│   ├── exploration/              frontier.py + exploration_node                           [Phase 8 V3]
│   ├── system_monitor/           CPU/RAM/온도/토픽 Hz → /diagnostics, CSV
│   ├── px4_msgs/, Micro-XRCE-DDS-Agent/   외부 (git 제외)
├── tests/
│   └── simulation/               test_flight_sequence.py(P1), test_failsafe.py(P1), test_phase2_actions.py, test_phase3_sensors.py, test_phase4_slam.py
├── tools/                        plot_flight.py, export_gt_objects.py
└── experiments/                  000_phase0_env, 001_phase1_flight, 002_phase2_bringup, 003_phase3_sensors, 004_phase4_slam/<run>
```
단위 테스트: 각 패키지 `test/` (pytest, 총 34개: frames 16, projection/association 6, db 3, state machine 5, frontier 1, llm mock 3).
```bash
source scripts/env.sh; for p in px4_bridge semantic_mapper semantic_database mission_manager exploration llm_interface; do (cd ros2_ws/src/$p && python3 -m pytest -q test); done
```

---

## 4. 핵심 설계 (코드를 읽기 전에 알아야 할 것)

### 4.1 비행 계층 `px4_bridge`
- `frames.py` — ENU↔NED 위치/yaw, FRD→NED 쿼터니언 → FLU→ENU 변환. **상위 계층은 전부 ENU.**
- `px4_interface.py` — px4_msgs 구독/발행 유일 지점. 10 Hz steady-clock 타이머로 `OffboardControlMode`+`TrajectorySetpoint` 스트림,
  `VehicleStatus/LocalPosition/Attitude/LandDetected/CommandAck` → `VehicleState`. `hold_current_position()`.
- `flight_controller.py` — **영구 Mission API** `takeoff(h) / land() / goto(x,y,z,yaw) / hover(t) / return_home()` (blocking, `FlightError`,
  `FlightCancelled`). Offboard 진입: 스트림 ≥10 msg → `DO_SET_MODE(1,6)` → `ARM`. 도착 판정: 오차<0.2 m ∧ 속도<0.2 m/s 0.5 s.
  `cancel_check` 콜백으로 취소 → 현재 위치 호버.
- `px4_bridge_node.py` — 액션 `/px4_bridge/flight_command`(동시 1개, cancel→hold, LAND cancel 거부), `/drone/vehicle_state`·`/drone/pose`·`/drone/odom` 20 Hz,
  TF `odom→base_link`(파라미터 `publish_tf`, `odom_frame`). `MultiThreadedExecutor` + `ReentrantCallbackGroup`.
- `teleop_node.py` — 키보드 데모(`ros2 run px4_bridge teleop`). `flight_demo_node.py` — 시퀀스 1회.

### 4.2 Launch 계층 `drone_bringup`
- `sitl.launch.py` — XRCE Agent + PX4(`build/px4_sitl_default/bin/px4 -d`, cwd rootfs, `PX4_SIM_MODEL=<vehicle>`, `HEADLESS`, `PX4_GZ_WORLD`) +
  **부팅 후 파라미터 적용 프로세스**(`px4-param`로 logger 기동을 기다린 뒤 set → `compare`로 검증, 재시도).
  기본 `px4_params`: `NAV_DLL_ACT=0,COM_OF_LOSS_T=3,SIM_BAT_DRAIN=3600,SIM_BAT_MIN_PCT=90`.
  저장소 모델(`gz_x500_rgbd` 등 airframe 파일이 없는 것)은 `PX4_SYS_AUTOSTART=4002`로 부팅하며, `simulation/models|worlds`를 PX4 트리에 symlink한다.
- `simulation.launch.py` — sitl + px4_bridge_node(5 s 지연) + RViz + map→odom static(옵션). 인자: `vehicle, world, headless, rviz, use_sim_time, px4_tf, px4_odom_frame`.
- `full_simulation.launch.py` — 전 스택. `enable_<stage>:=false`로 토글, `detector_backend`, `llm_backend`, `odom_source`(기본 px4), `use_sim_time` 기본 true.

### 4.3 센서 계층 `sensor_adapter`
- Gazebo 토픽 `/rgb_camera`, `/depth_camera`, `/camera_info`(두 센서 공용), `/depth_camera/points`, `/clock` → ros_gz_bridge → `*_gz` → relay 노드가
  frame_id를 `camera_optical_frame`으로 재작성하고 `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info`,
  `/camera/depth/camera_info`(컬러 info 재발행, `registered_depth`)로 발행. 정적 TF `base_link→camera_link`(0.132, 0, 0.261) → `camera_optical_frame`.
- 실카메라 이식 시 바꿀 곳은 **이 패키지뿐**이다.

### 4.4 SLAM 계층 `drone_slam`
- `rtabmap.launch.py` — `odom_source:=px4`(기본: PX4 EKF TF, rtabmap은 map→odom만) | `vo`(rgbd_odometry가 odom→base_link, 이때 `px4_tf:=false`) |
  `vo_guess:=true`(VO + PX4 prior, TF odom→odom_px4→base_link). `gt:=true`면 `gt_pose_node`가 `gz topic -e`로 GT를 `/ground_truth/pose`(frame `map_gt`)에 발행.
  `record_dir:=`면 `trajectory_recorder`가 est/odom/px4/gt CSV + `stats.json`(lost, loop closures).
- `rtabmap.yaml` — `Grid/Sensor=1`, 0.05 m 2D grid(`/slam/map`), 3D cloud(`/slam/pointcloud`), 엄격한 loop closure(LoopThr 0.15, MinInliers 25, OptimizeMaxError 0.3).

### 4.5 좌표계·시간
- TF: `map ─(RTAB-Map)→ odom ─(px4_bridge)→ base_link ─(static)→ camera_link → camera_optical_frame`. `map_gt`는 트리에 연결하지 않는다.
- 시뮬레이션 전체는 `use_sim_time:=true`(Gazebo `/clock`). Phase 1/2 전용 launch만 wall time.

---

## 5. Phase별 상태와 증거

| Phase | 상태 | 핵심 결과 | 증거 | 커밋 |
|---|---|---|---|---|
| 0 환경 | 완료 | ROS 2 Jazzy, PX4 v1.17.0, Gazebo 8.15, px4_msgs, `/fmu/out/*_v1` 50 Hz | `experiments/000_phase0_env/` | cf56f89 |
| 1 PX4+Gazebo 비행 | 완료 | 시퀀스 10/10, failsafe PASS, gz_x500_depth 1회 | `experiments/001_phase1_flight/` (metrics.csv, summary.png) | cf56f89 |
| 베이스라인 골격 | 완료 | 계약 + 11개 패키지 스텁, mock 탐지→DB→mock LLM 전 체인 스모크 통과 | `docs/phases/baseline.md` | 0e36fb1 |
| 2 ROS 2 bringup | 완료 | 단일 launch, FlightCommand 액션 5/5, cancel→hold, /drone/odom, RViz, 영상 | `experiments/002_phase2_bringup/` (gazebo_flight.mp4) | 50311a3, b9e37ba |
| 3 센서 파이프라인 | 완료 | indoor_v1 world, x500_rgbd, 컬러 19 Hz/깊이 17 Hz, 정합·동기·TF PASS, rosbag 기록/재생 PASS, GT 10개 | `experiments/003_phase3_sensors/` (sensor_report.json) | 22aaff0 |
| 4 SLAM | 완료 | PX4 EKF+RTAB-Map: ATE 0.075 m, lost 0, 2D/3D 지도. 순수 VO 실패(ATE 8 m, lost 67), vo_guess 0.79 m | `experiments/004_phase4_slam/px4_v3/` (trajectory.png, map.png, rviz_mapping_end.png, report.json) | ecd5d72 |
| 5 객체 탐지 | **다음** | 베이스라인: detector_node + backends(mock 동작, ultralytics 미검증) | – | – |
| 6 Semantic Mapping | 골격 | projection/association/relations/DB/markers 코드 + 단위 테스트, mock으로 chair_01 생성 확인 | – | – |
| 7 LLM | 골격 | Ollama backend 코드, mock backend로 DB-grounded 답변 테스트 통과 | – | – |
| 8 Mission/Exploration | 골격 | 상태기계·안전검증·Mission 액션(TAKEOFF/GOTO/GOTO_OBJECT/RETURN_HOME/LAND 동작 확인), frontier 함수 | – | – |
| R0~R7 Pi 이식 | 미착수 | system_monitor로 PC baseline 측정 가능 | – | – |

### 사용자 결정 이력
- 제어 경로 ROS 2 + px4_msgs(uXRCE-DDS), 설치는 WSL2 직접, 저장소 루트 `ISE-drone`.
- 베이스라인: 전체 골격 먼저 → Phase별 구현; Python 우선; `vision_msgs/Detection2DArray`; 명령은 ROS 2 Action.
- Phase 2: cancel = 호버 유지; 5회 반복; 기본 기체 gz_x500_depth; `/drone/odom` 추가.
- Phase 3: 실내 world 직접 제작; Fuel 가구 + 원시도형; 커스텀 정합 RGB-D 640×480 20 Hz.
- Phase 4: odometry 둘 다 구현(사용자는 VO 기본 선택) → **평가 결과로 px4 기본으로 변경, 사용자 확인 대기 중**; 2바퀴 경로; 2D+3D 지도, DB 보관.

---

## 6. 실행 치트시트

```bash
source scripts/env.sh
# 시뮬레이션 (GUI)
ros2 launch drone_bringup simulation.launch.py vehicle:=gz_x500_rgbd world:=indoor_v1 use_sim_time:=true
ros2 launch sensor_adapter gazebo_sensors.launch.py
ros2 launch drone_slam rtabmap.launch.py odom_source:=px4 record_dir:=/tmp/run1
ros2 action send_goal /px4_bridge/flight_command drone_interfaces/action/FlightCommand "{command: TAKEOFF, height: 1.2}" --feedback
ros2 action send_goal /px4_bridge/flight_command drone_interfaces/action/FlightCommand "{command: GOTO, target: {x: 2.0, y: 0.0, z: 1.2}, yaw: 0.0, use_yaw: true}"
ros2 run px4_bridge teleop
# 전체 스택 (mock)
ros2 launch drone_bringup full_simulation.launch.py enable_slam:=false detector_backend:=mock llm_backend:=mock
ros2 action send_goal /mission/execute drone_interfaces/action/Mission "{action: TAKEOFF, param: 1.2}"
ros2 service call /llm/ask drone_interfaces/srv/AskLLM "{question: '의자가 어디 있어?'}"
# 수락 테스트 (headless, 자동 기동/종료)
scripts/run_phase1_test.sh 10
scripts/run_phase2_test.sh 5
python3 tests/simulation/test_phase3_sensors.py          # 시뮬+센서 launch 필요
scripts/run_phase4_slam.sh px4 2 my_run
# 도구
python3 tools/plot_flight.py experiments/001_phase1_flight
python3 tools/export_gt_objects.py --world indoor_v1     # 시뮬 실행 중
scripts/rosbag_record.sh experiments/003_phase3_sensors/rosbag/x 30 ; scripts/rosbag_replay_check.sh <bag>
```
GUI 증거 캡처: `QT_QPA_PLATFORM=xcb`로 띄우고 `xwininfo -root -tree`에서 `"Gazebo Sim"`/`"rviz2"` 창 id → `import -window <id> out.png`.
Gazebo 카메라 추적: `gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --req 'data: "x500_rgbd_0"'` (GZ_IP=127.0.0.1).

---

## 7. 남은 구현 사항 (우선순위 순)

### Phase 5 — Object Detection (계획서 §17) ← 다음
- 질문할 것: 모델(yolov8n/s/m 또는 yolo11), 클래스 집합(COCO: person, chair, couch, dining table, tv, backpack, bottle, book…), GPU 목표 FPS,
  Fuel 가구 대상 평가 방식, 원시도형(박스/병)은 탐지 불가이므로 처리 방침.
- 할 일: `object_detector/backends/ultralytics_backend.py` 실검증(CUDA, conda `drone` env의 torch가 아닌 **시스템 python에 ultralytics 설치 필요** —
  `pip install --user ultralytics`는 `PYTHONNOUSERSITE=1`에 막히므로 venv(`--system-site-packages`) 또는 apt 경로 결정 필요), 모델 가중치 저장 위치(`models/`),
  `/perception/detections_image` RViz 확인, indoor_v1에서 클래스별 recall 측정(GT json의 class 매핑 사용), FPS/지연 기록(system_monitor),
  `docs/phases/phase5_detection.md`, 통과: 실시간 탐지 + 인터페이스 고정(§55).
- 주의: 컬러 카메라 640×480 20 Hz; detector_node는 `max_rate_hz`로 제한; 디버그 이미지는 구독자 있을 때만 그림.

### Phase 6 — Semantic Mapping (§18–23)
- `semantic_mapper`: 현재 컬러/깊이 해상도 비율 스케일 + 깊이 CameraInfo 공유 가정 → 실제 Phase 5 탐지로 3D 오차 측정(GT와 비교: 목표 평균 < 0.3 m),
  association 임계값/최소 관측 수 튜닝, 중복률 < 10 %, recall > 80 %, 관계(near/left/right…)의 기준 heading 결정, `/semantic_map/markers` RViz 확인.
- `semantic_database`: observations 로그 옵션, 평가 스크립트(`tools/eval_semantic_map.py` 신규: GT json vs DB).
- `object_tracker`: ByteTrack 교체 여부 결정(현재 greedy IoU).
- 평가용 스크립트 비행 경로는 `tests/simulation/test_phase4_slam.py`의 ROUTE 재사용.

### Phase 7 — LLM (§24–26)
- Ollama 모델 선택(`qwen2.5:7b` 가정, 한국어 응답 필요), `llm_interface/config`; `/llm/ask` 실모델 검증; what/where/how many/near 질의 세트와 채점;
  환각 방지 규칙(DB 결과만, 없으면 "확인하지 못했습니다") 유지; `structured_command` 생성(Phase 8 연동).

### Phase 8 — Mission Manager / Exploration (§27–30)
- `mission_manager`: EXPLORE/SEARCH_OBJECT/LOOK_AT_OBJECT 구현(현재 거부), `/exploration/goal` 소비, 안전 한계 재검토, ERROR 복구 경로, 중단 처리 테스트.
- `exploration`: frontier 선택(거리/정보량), 1.2 m 고정 고도 2D planner, 벽 충돌 회피(occupancy grid 기반 경로 검사) — 현재 GOTO는 직선 비행이라 문을 통과하려면
  waypoint를 문 중앙(y=0)에 두어야 한다(ROUTE 참고).
- `full_simulation.launch.py` end-to-end 테스트(§33): 이륙 → 경로 → SLAM → 탐지 → DB → 복귀 → 착륙 → LLM 질의.

### Pi 이식 (R0~R7, §34–44)
- R0 PC baseline: `system_monitor` CSV로 CPU/RAM/FPS/latency 기록(`experiments/0NN_pc_baseline/`). 이후 센서 드라이버 → px4_bridge → SLAM(VO/VIO 필요!) → detector(ONNX backend) → mapper → mission 순.

### 알려진 미해결/보류
- **순수 시각 odometry 불안정**(Phase 4): 실내 실기체에는 IMU 융합(VIO) 또는 다른 VO가 필요. R3에서 다룬다.
- RViz 이미지 패널은 센서 launch를 함께 띄워야 채워진다.
- `/drone/odom`의 각속도는 0(PX4가 `vehicle_angular_velocity`를 기본 발행하지 않음).
- 배터리 시뮬레이터가 긴 GUI 세션에서 멈춘 적이 있음(`px4-battery_simulator stop && start`로 복구).
- 컬러 카메라 rate가 테스트 노드 수신 기준으로는 adapter 집계보다 낮게 측정됨(best-effort 드롭).

---

## 8. 함정 모음 (반드시 읽을 것)

| 증상 | 원인 | 대처 |
|---|---|---|
| colcon: `option --uninstall not recognized` | `~/.local`의 최신 setuptools | `source scripts/env.sh`(PYTHONNOUSERSITE=1) 후 빌드; 필요 시 `rm -rf build/<pkg> install/<pkg>` |
| `numpy.core.multiarray failed to import` | 사용자 numpy 2.x vs 시스템 OpenCV | 동일 (PYTHONNOUSERSITE) |
| 백그라운드 `ros2 launch`가 Ctrl-C/SIGINT를 무시 | 비대화형 셸의 `&` 작업은 SIGINT 무시 | `set -m`으로 띄우고 `kill -INT -- -<pgid>`; SIGTERM은 자식을 고아로 만듦 |
| 스크립트가 영원히 멈춤 | bare `wait`가 launch까지 기다림 | 항상 `wait $PID` |
| `pkill -f`/`pgrep -f`가 자기 셸을 죽임 | 패턴이 자기 명령행과 매칭 | `pattern[x]` 트릭, 또는 PID로 kill |
| arming 거부 "No connection to the GCS" | gz_x500 기체 `NAV_DLL_ACT=2`, `PX4_PARAM_*`는 기체 파일에 덮임 | sitl.launch가 부팅 후 `px4-param set` + `compare` 검증 |
| 비행 중 갑자기 RTL(offboard loss) | WSL2 wall clock이 ~10 s마다 0.3~0.8 s 역행 → PX4가 스트림을 stale로 판정 | `COM_OF_LOSS_T=3`(SITL), 하트비트 타이머 steady clock. 호스트 쪽 근본 해결 못함 |
| "Battery unhealthy"로 재arming 거부 | 배터리 시뮬레이터 정지 / `SIM_BAT_DRAIN=60` | 모듈 재시작, launch 기본값 3600/90 |
| TF `extrapolation into the future` | 센서 스탬프가 20 Hz odom TF보다 수 ms 앞섬 | mapper가 50 ms 이내면 최신 TF 사용; TF 리스너는 **전용 보조 노드**에 `spin_thread=True` |
| 노드 SIGABRT `PyFloat_Check` | 메시지 float 필드에 int/numpy 대입 | `float()` 캐스팅 |
| `LookupError` 분기에 KeyError가 잡힘 | KeyError ⊂ LookupError | 전용 예외 클래스 사용 |
| RViz가 기본 화면(Grid만) | `.rviz`가 설치 glob에 없었음/키 형식(`Enabled`, `Topic:{Value…}`) | setup.py glob에 `*.rviz` |
| Gazebo CameraInfo가 안 들어옴 | 두 센서가 공용 `/camera_info`에 발행, 깊이 카메라는 info 없음 | bridge `/camera_info`, adapter가 깊이 info 재발행 |
| Fuel 모델이 바닥을 뚫고 떨어짐 | include 기본 동적 | `<static>true</static>` |
| ros_gz `Pose_V→TFMessage` 브리지 무응답 | 미지원/버그로 추정 | `gz topic -e` 서브프로세스 파싱(gt_pose_node) |
| RTAB-Map 지도 비어 있음 | 0.23에서 `Grid/FromDepth`→`Grid/Sensor`, 토픽 `map` | yaml/launch 반영됨 |
| loop closure가 궤적을 망침 | 동일 텍스처 반복(perceptual aliasing) | 벽별 고유 텍스처 + 엄격한 loop 파라미터 |
| 무거운 world에서 goto 타임아웃 | GUI 실행 시 RTF 0.3~0.7(wall clock 기준 타임아웃) | `goto_timeout_s=120`, 수락 테스트는 headless |
| GUI/헤드리스 동시에 두 시뮬레이션 | 포트 충돌 | 한 번에 하나만 |

---

## 9. 새 에이전트 시작 체크리스트

1. `git log --oneline | head`로 최신 커밋 확인, `README.md`·이 문서·`docs/phases/README.md`·`docs/ros_interfaces.md`를 읽는다.
2. `source scripts/env.sh && ros2 pkg list | grep -c drone_` (13개 패키지 중 drone_ 접두 3개) 로 환경 확인; 없으면 §2 설치 스크립트.
3. `scripts/run_phase2_test.sh 1`로 시뮬레이션+액션 경로가 살아 있는지 1회 확인(≈3분).
4. 사용자에게 다음 Phase(현재 Phase 5) 질문을 먼저 한 뒤 구현한다. 구현 후 `docs/phases/phase5_detection.md` + `experiments/005_*/` + 커밋 + 보고 후 멈춘다.
5. Phase 4의 기본 odometry 변경(vo→px4)은 사용자 확인 대기 중이다. 보고 시 다시 언급한다.
