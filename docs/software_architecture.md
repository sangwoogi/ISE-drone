# Software Architecture (baseline, 2026-10-14)

계획서 §4의 계층 구조를 ROS 2 패키지로 구현한 베이스라인이다. 모든 노드는 Python(rclpy)이고,
외부 C++ 패키지(RTAB-Map, ros_gz_bridge, PX4)는 바이너리로 사용한다.

```
                 ┌──────────────────────────────────────────────────┐
  User / CLI ───▶│ llm_interface   /llm/ask (AskLLM)                │  Phase 7
                 │   QueryAgent → tools(find/count/list) → DB only  │
                 └──────────────┬───────────────────────────────────┘
                                │ structured command (Phase 8)
                 ┌──────────────▼───────────────────────────────────┐
                 │ mission_manager   /mission/execute (Mission act.)│  Phase 8
                 │   state machine + safety validator               │
                 └──────────────┬───────────────────────────────────┘
                                │ /px4_bridge/flight_command (FlightCommand act.)
┌────────────────┐    ┌─────────▼────────────┐   ┌──────────────────────────┐
│ semantic_      │◀───│ px4_bridge           │   │ exploration (frontier)   │  Phase 8 V3
│ database       │    │  FlightController    │   │  /slam/map → /exploration/goal
│ /semantic_     │    │  /drone/vehicle_state│   └──────────────────────────┘
│  database/query│    │  TF odom→base_link   │
└──────▲─────────┘    └─────────┬────────────┘
       │ /semantic_map/objects  │ px4_msgs (uXRCE-DDS)   ← the ONLY place px4_msgs is used
┌──────┴─────────┐              ▼
│ semantic_mapper│          PX4 SITL / FC
│  projection    │
│  association   │   ┌────────────────┐   ┌─────────────────┐
│  relations     │◀──│ object_tracker │◀──│ object_detector │  Phase 5/6
│  markers       │   │ /perception/   │   │  DetectorBackend│
└──────▲─────────┘   │   tracks       │   │  mock|ultralytics|onnx
       │ TF map→odom └────────────────┘   └────────▲────────┘
┌──────┴─────────┐                                 │ /camera/color/image_raw
│ drone_slam     │◀────────────────────────────────┤ /camera/depth/image_raw
│  RTAB-Map      │                                 │ /camera/color/camera_info
│  gt_pose_node  │  (ground truth: evaluation only)│
│  slam_eval     │   ┌─────────────────────────────┴───┐
└────────────────┘   │ sensor_adapter                  │  Phase 3
                     │  ros_gz_bridge → *_gz → adapter │
                     │  (real camera driver later)     │
                     └─────────────────────────────────┘
                     system_monitor: /diagnostics + CSV (모든 Phase)
```

## 패키지 ↔ Phase

| 패키지 | Phase | 베이스라인 상태 | 실제 구현 시 할 일 |
|---|---|---|---|
| `drone_interfaces` | 전체 | msg/srv/action 계약 완성 | 필드 추가 시 여기만 수정 |
| `px4_bridge` | 1–2 | FlightController(검증됨) + `px4_bridge_node`(action, VehicleState, TF) | Phase 2에서 launch/RViz와 함께 검증 |
| `drone_bringup` | 2, 전체 | `sitl`, `simulation`, `full_simulation` launch, RViz config | 각 Phase에서 노드 추가 검증 |
| `sensor_adapter` | 3 | ros_gz_bridge 설정 + relay 노드 + 정적 TF | Gazebo 토픽명·카메라 extrinsic 확인, rosbag |
| `drone_slam` | 4 | rtabmap launch, gt_pose_node, slam_eval(ATE/RPE) | rtabmap_ros 설치, 파라미터 튜닝, baseline 측정 |
| `object_detector` | 5 | backend 추상화(mock/ultralytics/onnx), detector_node | GPU 추론 검증, 클래스 필터, FPS |
| `object_tracker` | 6 | IoU 기반 단순 tracker | ByteTrack 교체 여부 결정 |
| `semantic_mapper` | 6 | projection/association/relations/markers (단위 테스트) | 실제 depth·TF로 오차 측정 |
| `semantic_database` | 6 | SQLite 스키마, upsert/query, 서비스 | 로그 저장, 평가 스크립트 |
| `llm_interface` | 7 | Ollama/mock backend, tools, agent, CLI (mock 테스트) | 모델 선택, 프롬프트, 평가 질의 세트 |
| `mission_manager` | 8 | 상태기계·안전검증·Mission action (V1+V2) | EXPLORE/SEARCH 연결, 중단 처리 검증 |
| `exploration` | 8 V3 | frontier 검출(단위 테스트) + goal 노드 | planner/안전 검사 연동 |
| `system_monitor` | 전체 | CPU/RAM/온도/토픽 Hz → diagnostics, CSV | Pi 이식 벤치마크에 사용 |

## 원칙 체크리스트 (계획서 §2)
- Hardware-independent: `/camera/*`, `/drone/*` 공통 토픽만 상위가 본다. ✔
- Sim/Real 동일 인터페이스: `sensor_adapter`가 유일한 교체 지점. ✔
- Ground truth 분리: `gt_pose_node`는 `/ground_truth/*`만 발행, 어떤 노드도 구독하지 않는다. ✔
- LLM/로보틱스 분리: LLM → `AskLLM`/structured command → `mission_manager`(상태기계+안전검증) → `px4_bridge`. ✔
