# Baseline — 전체 Phase 골격 (2026-10-14)

**목적** 계획서 §5–7 "인터페이스를 먼저 정의"에 따라 Phase 2~8 전 패키지의 메시지 계약, 노드 골격, launch를 한 번에
만들어 토픽이 끝까지 연결되게 한다. 이후 각 Phase는 이 골격 안에서 알고리즘만 채운다.

**사용자 결정** 전체 골격 먼저 → Phase별 구현 / Python 우선 / `vision_msgs/Detection2DArray` / 명령 인터페이스는 ROS 2 Action.

## 계약 (`drone_interfaces`)

| 종류 | 이름 | 용도 |
|---|---|---|
| msg | `SemanticObject`, `SemanticObjectArray` | 계획서 §7 + size/relations |
| msg | `VehicleState` | px4_bridge가 ENU로 공개하는 기체 상태 (px4_msgs 비노출) |
| msg | `MissionStatus` | 상태기계 상태/현재 action/메시지 |
| srv | `QueryObjects` | class / near_object / near_point+radius / min_confidence |
| srv | `AskLLM` | 질문 → 답변 + structured_command + tool_calls 로그 |
| action | `FlightCommand` | TAKEOFF / LAND / GOTO / HOVER / RETURN_HOME (px4_bridge) |
| action | `Mission` | V1 TAKEOFF…RETURN_HOME, V2 GOTO_OBJECT/LOOK_AT_OBJECT, V3 EXPLORE/SEARCH_OBJECT (mission_manager) |

전체 토픽/서비스/TF 표는 `docs/ros_interfaces.md` §6, 패키지 구조는 `docs/software_architecture.md`.

## 패키지별 베이스라인 내용

| 패키지 | 구현된 것 | 단위 테스트 |
|---|---|---|
| px4_bridge | `px4_bridge_node`: FlightCommand 액션 서버(동시 1개, cancel→abort), `/drone/vehicle_state`·`/drone/pose` 20 Hz, TF odom→base_link. FlightController에 `spin=False`·progress 콜백 추가 | frames 9 |
| drone_bringup | `simulation.launch.py`(Phase 2: SITL+bridge+RViz+map→odom static), `full_simulation.launch.py`(전 스택, `enable_*` 토글), `config/drone.rviz` | — |
| sensor_adapter | ros_gz_bridge 설정(x500_depth), relay 노드(frame_id 재작성, color↔depth skew 측정), 정적 TF 2개, `gazebo_sensors.launch.py` | — |
| drone_slam | `rtabmap.launch.py`(RGB-D, PX4 odom 또는 rgbd_odometry), `gt_pose_node`(평가 전용), `slam_eval`(ATE/RPE, Umeyama) | — |
| object_detector | `DetectorBackend` 추상화: mock / ultralytics(YOLO) / onnx(R4 placeholder), `detector_node`(rate limit, 디버그 이미지) | — |
| object_tracker | 클래스별 greedy IoU 트래커 → `/perception/tracks` (`id=class#n`) | — |
| semantic_mapper | `projection.py`(중앙 영역 median depth, pinhole), `association.py`(class+거리, EMA), `relations.py`(near/left/right/front/behind), 노드(TF 변환, 마커) | 6 |
| semantic_database | `db.py`(objects/relations/observations 스키마, upsert/query/count/classes), 노드(구독+서비스) | 3 |
| llm_interface | backends(ollama HTTP tool-calling, mock 규칙기반, cloud placeholder), `tools.py`(find/count/list), `QueryAgent`, `llm_node`, `llm_cli`(오프라인 `--db` 모드) | 3 (mock DB + mock LLM) |
| mission_manager | `state_machine.py`(허용 전이표, ERROR 사유), `safety_validator.py`(고도/거리/객체 standoff), Mission 액션 서버 → FlightCommand 클라이언트, GOTO_OBJECT는 DB 조회 | 3 |
| exploration | `frontier.py`(free∧unknown 인접 셀, 클러스터링), `/slam/map` → `/exploration/goal` | 1 |
| system_monitor | psutil CPU/RAM/온도 + 토픽 Hz(raw 구독) → `/diagnostics`, CSV | — |

## 검증

- `colcon build` 13 패키지 성공, 단위 테스트 25개 통과.
- 통합 스모크 A: `simulation.launch.py` headless → `/drone/vehicle_state` 20 Hz, TF odom→base_link, 액션 TAKEOFF/HOVER/GOTO/LAND.
- 통합 스모크 B: `full_simulation.launch.py enable_slam:=false detector_backend:=mock llm_backend:=mock` → 노드 기동, `/camera/*` 수신, DB 서비스, mock LLM, Mission TAKEOFF/LAND.
- 결과: 아래 "스모크 결과" 절.

## 스모크 결과 (headless, 2026-10-14)

**A. `simulation.launch.py` (Phase 2 bringup)** — `/drone/vehicle_state` 20 Hz, TF odom→base_link 발행,
`ros2 action send_goal /px4_bridge/flight_command …`: TAKEOFF 1.2 m (final_error 0.15 m) → HOVER 2 s → GOTO (1,0,1.2)
(final_error 0.07 m) → LAND 모두 `success: true`.

**B. `full_simulation.launch.py enable_slam:=false detector_backend:=mock llm_backend:=mock`** — 15개 노드 기동
(sensor bridge/adapter, 정적 TF 2개, map→odom, px4_bridge, detector, tracker, mapper, database, mission_manager, llm, monitor, exploration).

| 확인 항목 | 결과 |
|---|---|
| `/clock` (Gazebo, use_sim_time) | ~110 Hz |
| `/camera/color/image_raw` / depth / camera_info | 3–4 Hz / 5–11 Hz / 13 Hz (colour 1920×1080 headless 렌더링이 병목 → Phase 3 결정) |
| `/perception/detections_2d` → `/perception/tracks` | 1–3 Hz, `id = chair#1` |
| `/perception/objects_3d` → `/semantic_map/objects` | mock chair → map (0.83, −0.10, −0.17) m, 41회 관측으로 `chair_01` 확정, 마커 2개 |
| `/semantic_database/query {class_name: chair}` | `1 objects`, `chair_01` |
| `/llm/ask "의자가 어디 있어?"` (mock backend) | `chair_01 확인: 위치 (0.83, -0.10, -0.17) m.` — DB 결과만 사용 |
| `/llm/ask` 객체 없을 때 | `현재 생성된 Semantic Map에서는 해당 객체를 확인하지 못했습니다.` |
| Mission TAKEOFF → GOTO → GOTO_OBJECT(없는 객체) → RETURN_HOME → LAND | HOVER → GOAL_REACHED → 거부(상태 유지 HOVER) → GOAL_REACHED → IDLE |
| `/diagnostics` | cpu/ram/온도 + 토픽 Hz 키 |
| 종료 (Ctrl-C) | 파이썬 트레이스백 0 |

mock 의자의 z가 음수인 것은 mock bbox가 지면(0.7 m 앞)을 가리키기 때문이며 투영 자체는 정상이다. 실제 객체 배치와 정확도 평가는 Phase 3/6.

## 골격 작성 중 발견·수정한 문제
1. **PX4 파라미터 적용 경쟁 조건**: `px4-param`은 rcS가 끝나기 전에도 응답하므로 기체 파일의 `set-default NAV_DLL_ACT 2`가 나중에 덮어썼다(간헐적 "No connection to the GCS" arming 거부). rcS 마지막에 뜨는 `logger` 상태를 기다린 뒤 set → `param compare`로 검증 → 재시도.
2. **numpy 충돌**: `~/.local`의 사용자 numpy 2.5가 시스템 OpenCV/cv_bridge(numpy 1.26 빌드)와 충돌해 detector/mapper가 죽음 → `scripts/env.sh`에 `PYTHONNOUSERSITE=1`.
3. system_monitor의 raw 구독은 타입 문자열이 아니라 `rosidl_runtime_py.utilities.get_message()`로 얻은 클래스를 넘겨야 한다.
4. `MockBackend`가 공통 팩토리 kwargs를 받지 못해 llm_node 기동 실패 → 생성자 수정.
5. **Mission LAND가 ABORTED(message `''`)**: 착륙 확인 콜백이 `landed()`로 `current_action`을 비운 직후 `finish()`가
   `DONE['']` → `KeyError('')`, 그리고 `KeyError`는 `LookupError`의 하위라 "MAP_UNAVAILABLE" 분기에 잡혔다. →
   `landed()`는 action 실행 중이면 무시, `finish()`는 빈 action이면 no-op, DB 조회 실패는 전용 `ObjectNotFound` 예외.
6. 검증 단계에서 거부된 목표(안전/없는 객체)가 ERROR 상태를 만들어 이후 RETURN_HOME이 거부됨 → `cancel()`로 이전 상태 복귀.
7. 카메라 스탬프(Gazebo sim time)와 TF(wall time) 불일치로 mapper TF 조회 실패 → 전 스택 `use_sim_time:=true` 기본값,
   `/clock`은 ros_gz_bridge가 발행.
8. 탐지 스탬프가 20 Hz odom TF보다 수 ms 앞서는 "extrapolation into the future" → 50 ms 이내면 최신 TF 사용.
   `TransformListener(spin_thread=True)`를 같은 노드에 붙이면 rcl abort(-6) → 전용 보조 노드에 부착.
9. 컬러(1920×1080)와 깊이(640×480) 해상도가 달라 bbox가 깊이 영상 밖 → 해상도 비율로 bbox·intrinsics 스케일(FOV 차이는 Phase 3/6에서 depth CameraInfo로 보정).
10. `SemanticObject.size`에 int를 넣으면 rosidl C 변환기 단언으로 SIGABRT → 메시지 변환 시 전부 float 캐스팅.
11. 백그라운드 `&` 작업은 SIGINT를 무시하고, `pkill -f`는 자기 셸을 매칭한다 → 프로세스 그룹 SIGINT, `[e]` 패턴 사용.

## Phase별 남은 질문 (각 Phase 시작 시 확인)
- Phase 3: 실내 benchmark world를 직접 만들지(§31), 어떤 객체 모델을 쓸지, 카메라 해상도/FPS.
- Phase 4: RTAB-Map odometry 소스(PX4 EKF vs rgbd_odometry), 평가 궤적.
- Phase 5: YOLO 모델 크기(n/s/m), 클래스 집합, FPS 목표.
- Phase 6: association 임계값, 관측 횟수 기준, 관계 생성 범위.
- Phase 7: Ollama 모델(예: qwen2.5:7b / llama3.1:8b), 한국어 응답 요구사항.
- Phase 8: 자율 탐색 포함 여부, 안전 한계값.
