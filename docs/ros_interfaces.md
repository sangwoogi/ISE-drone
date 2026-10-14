# ROS 2 Interface Contract — Phase 1 (PX4 flight layer)

이 문서는 Phase 1에서 **고정**한 토픽, QoS, 좌표계, Mission API를 기록한다.
이후 Phase에서 추가되는 인터페이스는 이 파일에 절을 늘려 기록한다.

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

Phase 1의 위치 기준은 PX4 local origin(시뮬레이션 시작점)이다. TF(`map/odom/base_link`)는 Phase 2~3에서 정의한다.

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
