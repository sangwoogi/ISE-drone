# ROS 2 Interface Contract

§1–5: Phase 1에서 **고정**한 PX4 비행 계층. §6: 베이스라인에서 정의한 전 Phase 공통 계약(`drone_interfaces`).
인터페이스를 바꿀 때는 이 문서와 `drone_interfaces`를 함께 수정한다.

## 1. 버전 고정

| 구성요소 | 버전 | 비고 |
|---|---|---|
| PX4-Autopilot | v1.17.0 | `~/PX4-Autopilot` (scripts/setup/02_px4.sh) |
| px4_msgs | v1.17.0 | PX4와 반드시 같은 태그 |
| Micro-XRCE-DDS-Agent | v2.4.3 | `MicroXRCEAgent udp4 -p 8888` |
| ROS 2 | Jazzy | Ubuntu 24.04 |
| Gazebo | Harmonic (8.x) | PX4 `ubuntu.sh`가 설치 |

## 2. PX4 ↔ ROS 2 토픽 (uXRCE-DDS)

PX4 ≥ 1.16은 `MESSAGE_VERSION > 0` 인 메시지의 토픽명에 `_v<N>` 접미사를 붙인다
(`src/modules/uxrce_dds_client/utilities.hpp::generate_topic_name`). v1.17.0 기준 실제 이름:

| 방향 | 토픽 | 메시지 | 용도 |
|---|---|---|---|
| PX4 → ROS | `/fmu/out/vehicle_status_v1` | `px4_msgs/VehicleStatus` | arming_state, nav_state |
| PX4 → ROS | `/fmu/out/vehicle_local_position_v1` | `px4_msgs/VehicleLocalPosition` | NED 위치/속도/heading, valid flags |
| PX4 → ROS | `/fmu/out/vehicle_land_detected` | `px4_msgs/VehicleLandDetected` | landed |
| PX4 → ROS | `/fmu/out/vehicle_attitude` | `px4_msgs/VehicleAttitude` | q (FRD→NED, w,x,y,z) → ENU/FLU 변환 |
| PX4 → ROS | `/fmu/out/vehicle_command_ack` | `px4_msgs/VehicleCommandAck` | 명령 수락/거부 |
| PX4 → ROS | `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | (Phase 3+에서 사용) |
| ROS → PX4 | `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | offboard heartbeat, `position=true`, 10 Hz |
| ROS → PX4 | `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | NED 위치 setpoint, 10 Hz |
| ROS → PX4 | `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | ARM/DISARM, DO_SET_MODE, NAV_LAND |

이름은 `ros2_ws/src/px4_bridge/config/px4_bridge.yaml` 의 `topics.*` 파라미터로 주입되며,
PX4 버전이 바뀌면 **이 파일만** 수정한다. `px4_msgs`를 import하는 코드는
`px4_bridge/px4_interface.py` 하나뿐이다.

### QoS

| 방향 | Reliability | Durability | Depth |
|---|---|---|---|
| `/fmu/out/*` 구독 | BEST_EFFORT | TRANSIENT_LOCAL | 1 |
| `/fmu/in/*` 발행 | RELIABLE (기본) | VOLATILE | 10 |

### Offboard 진입 순서 (PX4 공식 예제와 동일)

1. `OffboardControlMode` + `TrajectorySetpoint`를 10 Hz로 먼저 스트리밍 (≥ 10 msg)
2. `VehicleCommand(DO_SET_MODE, param1=1, param2=6)` → `nav_state == OFFBOARD` 확인
3. `VehicleCommand(COMPONENT_ARM_DISARM, param1=1)` → `arming_state == ARMED` 확인
4. 스트리밍이 2 Hz 아래로 떨어지면 PX4가 offboard-loss failsafe(기본: Land) 수행

## 3. 좌표계

| 계층 | 프레임 | 규칙 |
|---|---|---|
| Mission API / 상위 노드 | **ENU** (x East, y North, z Up), yaw = +x(East)에서 CCW [rad] | ROS REP-103 |
| PX4 | **NED** (x North, y East, z Down), yaw = North에서 CW | PX4 local frame |

변환은 `px4_bridge/frames.py` 에서만 수행한다.
`(E, N, U) ↔ (N, E, D)`, `yaw_ned = π/2 − yaw_enu` (wrap to (−π, π]).
자세: `q_enu_flu = q_NED→ENU · q_px4(FRD→NED) · q_FLU→FRD`, `q_NED→ENU = (w0, x√½, y√½, z0)`, `q_FLU→FRD = (0,1,0,0)`; 결과는 (x,y,z,w)로 발행.

Phase 1의 위치 기준은 PX4 local origin(시뮬레이션 시작점)이다. TF(`map/odom/base_link`)는 Phase 2~3에서 정의한다.
카메라 정적 TF(Phase 3): base_link → camera_link = (0.132, 0.0, 0.261) m, camera_link → camera_optical_frame = RPY(−π/2, 0, −π/2).

## 4. Mission API (영구 계약)

`px4_bridge.flight_controller.FlightController` — 모든 호출은 **blocking**, 실패 시 `FlightError`.

```python
takeoff(height)              # arming 지점 기준 height[m] 상승 (OFFBOARD 진입 + arm 포함)
land()                       # AUTO_LAND → landed & disarmed 까지 대기
goto(x, y, z, yaw=None)      # ENU 절대 좌표 [m], yaw [rad] (None = 현재 유지)
hover(duration)              # 현재 setpoint 유지 [s], 최대 이탈 거리 반환
return_home()                # arming 지점 상공(현재 고도)으로 복귀
```

도착 판정: 위치 오차 < 0.20 m **그리고** 속도 < 0.20 m/s 가 0.5 s 연속 유지.
최소 안전 범위: 상대 고도 0.5–5.0 m, home 기준 수평 15 m 이내 (Phase 8 Safety Validator가 대체).

Phase 8 Mission Manager의 action `TAKEOFF / LAND / HOVER / GOTO / RETURN_HOME` 과 1:1 대응한다.

## 5. Launch

```bash
ros2 launch drone_bringup sitl.launch.py [vehicle:=gz_x500|gz_x500_depth] [headless:=true] [world:=<name>]
```
PX4 SITL 바이너리(`build/px4_sitl_default/bin/px4 -d`, `make px4_sitl <vehicle>`과 동일한 환경)와 XRCE Agent를 띄운다.
Phase 2에서 bridge 노드/RViz를 추가한다.

### SITL 전용 파라미터 오버라이드

`px4_params:=NAME=VALUE,...` 인자는 PX4 rcS의 `PX4_PARAM_<NAME>` 환경변수 메커니즘으로 전달된다. 기본값:

| 파라미터 | 값 | 이유 |
|---|---|---|
| `NAV_DLL_ACT` | 0 | gz_x500 기체 기본값 2(datalink loss → RTL)는 GCS 미연결을 **pre-arm 실패**로 만든다. 자동 테스트는 QGC 없이 돌므로 끈다. 실기체에서는 다시 켠다. |
| `COM_OF_LOSS_T` | 3 | offboard 신호 손실 판정 시간[s] (기본 1). WSL2에서는 wall clock이 ~10초마다 최대 0.8초 역행하고, PX4는 수신 timestamp를 XRCE timesync 오프셋으로 변환하므로 1초 기준에서는 정상 스트림이 '손실'로 보여 RTL failsafe가 발생했다(`failsafe_flags.offboard_control_signal_lost`로 확인). 실기체(Pi, 안정적 시계)에서는 기본값으로 되돌린다. |

## 6. 전체 스택 계약 (baseline)

### 6.1 토픽

| 토픽 | 타입 | 발행 | 구독 | 비고 |
|---|---|---|---|---|
| `/camera/color/image_raw` | sensor_msgs/Image | sensor_adapter | detector, SLAM | frame `camera_optical_frame`, sensor QoS |
| `/camera/depth/image_raw` | sensor_msgs/Image | sensor_adapter | mapper, SLAM | 32FC1 m 또는 16UC1 mm |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | sensor_adapter | mapper, SLAM | x500_rgbd: 640×480, fx=fy=432.5, cx=320, cy=240 |
| `/camera/depth/camera_info` | sensor_msgs/CameraInfo | sensor_adapter | mapper | 시뮬레이션에서는 컬러 info 재발행(정합 RGB-D) |
| `/camera/depth/points` | sensor_msgs/PointCloud2 | ros_gz_bridge | RViz | 선택 |
| `/sensor_adapter/skew_ms` | std_msgs/Float32 | sensor_adapter | 평가 | depth−color 타임스탬프 차 |
| `/drone/vehicle_state` | drone_interfaces/VehicleState | px4_bridge | mission_manager, 평가 | 20 Hz, ENU, frame `odom` |
| `/drone/pose` | geometry_msgs/PoseStamped | px4_bridge | RViz | 전체 자세(FLU→ENU 쿼터니언) |
| `/drone/odom` | nav_msgs/Odometry | px4_bridge | SLAM(Phase 4), 평가 | frame odom, child base_link, 선속도 포함, 각속도 0 (PX4가 vehicle_angular_velocity를 기본 발행하지 않음) |
| `/ground_truth/pose` | geometry_msgs/PoseStamped | drone_slam/gt_pose_node | **평가 전용** | frame `map_gt` |
| `/slam/odom` | nav_msgs/Odometry | RTAB-Map(옵션) | | V1은 PX4 odom 사용 |
| `/slam/map` | nav_msgs/OccupancyGrid | RTAB-Map | exploration | |
| `/slam/pointcloud` | sensor_msgs/PointCloud2 | RTAB-Map | RViz | |
| `/perception/detections_2d` | vision_msgs/Detection2DArray | object_detector | tracker | class_id=string, score |
| `/perception/detections_image` | sensor_msgs/Image | object_detector | RViz | 디버그 |
| `/perception/tracks` | vision_msgs/Detection2DArray | object_tracker | semantic_mapper | `id = class#track` |
| `/perception/objects_3d` | drone_interfaces/SemanticObjectArray | semantic_mapper | 평가 | 프레임별 3D 관측 |
| `/semantic_map/objects` | drone_interfaces/SemanticObjectArray | semantic_mapper | semantic_database | 확정 객체(관측 ≥ N), 2 Hz |
| `/semantic_map/markers` | visualization_msgs/MarkerArray | semantic_mapper | RViz | |
| `/exploration/goal` | geometry_msgs/PoseStamped | exploration | mission_manager | frontier 목표 |
| `/mission/status` | drone_interfaces/MissionStatus | mission_manager | UI/LLM | 5 Hz |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | system_monitor | | CPU/RAM/온도/토픽 Hz |

### 6.2 서비스 / 액션

| 이름 | 타입 | 서버 | 설명 |
|---|---|---|---|
| `/px4_bridge/flight_command` | action `FlightCommand` | px4_bridge | TAKEOFF/LAND/GOTO/HOVER/RETURN_HOME, 동시 1개. **cancel → 현재 위치 호버 유지**(OFFBOARD 유지, 결과 CANCELED), LAND는 cancel 거부 |
| `/mission/execute` | action `Mission` | mission_manager | 상태기계가 허용하는 action만 수락, GOTO_OBJECT는 DB 조회+안전 검증 |
| `/semantic_database/query` | srv `QueryObjects` | semantic_database | class/near_object/near_point+radius/min_confidence |
| `/llm/ask` | srv `AskLLM` | llm_interface | DB 도구 호출로만 답변, tool_calls 로그 반환 |

### 6.3 TF 트리

```
map ──(SLAM, Phase 4; 그 전엔 static identity)──▶ odom ──(px4_bridge)──▶ base_link ──(static)──▶ camera_link ──(static)──▶ camera_optical_frame
map_gt (ground truth, 평가 전용, 트리에 연결하지 않음)
```

### 6.4 Semantic object 표현
`SemanticObject{id, class_name, position(map), size, confidence, first_seen, last_seen, observation_count, relations[]}`.
`relations` 문자열 형식 `"<relation>:<object_id>"`, V1 relation = near / left_of / right_of / in_front_of / behind.
SQLite 테이블: `objects`, `relations`, `observations` (`semantic_database/db.py`).

## 7. 시뮬레이션 자산 (Phase 3)

| 자산 | 위치 | 비고 |
|---|---|---|
| 기체 `gz_x500_rgbd` | `simulation/models/x500_rgbd`, `OakD-Lite-RGBD` | 컬러·깊이 640×480 @ 20 Hz, FOV 1.274; PX4 airframe 4002로 부팅 |
| world `indoor_v1` | `simulation/worlds/indoor_v1.sdf` | Room A – 복도 – Room B, 가구 7(Fuel) + 원시도형 3 + 색 패널 7 |
| GT | `simulation/worlds/indoor_v1_gt.json` | `tools/export_gt_objects.py`로 추출, **평가 전용** |
| Gazebo 토픽 | `/rgb_camera`, `/depth_camera`, `/depth_camera/points`, `/camera_info`(두 센서 공용), `/clock` | `sensor_adapter/config/gz_bridge_x500_rgbd.yaml` |
