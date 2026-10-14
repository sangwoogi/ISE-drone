# Indoor Autonomous Semantic Exploration Drone
## Software Development Plan

> **목표**  
> 실제 드론 하드웨어 없이 PC 환경에서 먼저 `PX4 + Gazebo + ROS 2 + SLAM + Object Detection + Semantic Mapping + LLM` 전체 파이프라인을 구현하고 검증한 뒤, 동일한 ROS 2 인터페이스를 유지하면서 연산 노드를 단계적으로 Raspberry Pi 5로 이식한다.
>
> 본 문서는 **소프트웨어 파트만** 다룬다. Flight Controller, 모터, ESC, 배터리, 프레임 설계 등 실제 하드웨어 제작은 별도 문서에서 다룬다.

---

# 1. 프로젝트의 소프트웨어 목표

최종 소프트웨어는 드론이 실내 공간을 탐색하면서 다음 정보를 생성하도록 한다.

1. 드론의 위치와 자세 추정
2. 실내 공간의 2D/3D 지도 생성
3. 카메라 영상에서 객체 탐지
4. 탐지된 객체의 3D 위치 계산
5. 동일 객체의 중복 탐지를 통합
6. 객체와 공간 정보를 Semantic Map으로 저장
7. Semantic Map을 자연어로 질의
8. 최종적으로는 자연어 명령을 안전한 이동 명령으로 변환
9. PC에서 검증한 일부 기능을 Raspberry Pi로 이식

최종 사용자 경험의 예시는 다음과 같다.

```text
사용자:
방 안에 가방이 있어?

시스템:
bag_01 detected
map position = (2.81, -0.92, 0.41)

LLM:
네. 가방 한 개를 확인했습니다.
출발 위치를 기준으로 약 2.8 m 앞, 왼쪽 약 0.9 m 지점에 있습니다.
```

향후에는 다음과 같이 확장한다.

```text
사용자:
가방 근처로 이동해.

LLM / Mission Parser
        ↓
go_to_object("bag_01")
        ↓
Semantic Map
        ↓
target = (2.81, -0.92, 1.0)
        ↓
Path / Safety Layer
        ↓
PX4 Position Setpoint
```

LLM이 모터 명령을 직접 생성하도록 만들지 않는다.

---

# 2. 핵심 개발 원칙

## 2.1 Hardware-independent first

초기 개발에서는 실제 센서를 전혀 사용하지 않는다.

다음 정보는 모두 Gazebo에서 얻는다.

- RGB image
- Depth image
- CameraInfo
- IMU
- Ground-truth pose
- PX4 odometry
- simulated vehicle state

따라서 초기 개발 대상은 실제 드론이 아니라 아래의 **가상 센서 인터페이스**이다.

```text
/camera/color/image_raw
/camera/depth/image_raw
/camera/camera_info
/fmu/out/vehicle_odometry
/tf
```

나중에 실제 카메라와 Flight Controller를 연결해도 상위 노드는 이 인터페이스를 최대한 유지한다.

---

## 2.2 Simulation과 Real Drone의 인터페이스를 동일하게 유지

가장 중요한 설계 원칙이다.

잘못된 구조:

```text
Gazebo 전용 코드
        ↓
프로젝트 완성
        ↓
실제 카메라 연결
        ↓
코드 대부분 재작성
```

권장 구조:

```text
               Sensor Adapter
                    │
       ┌────────────┴────────────┐
       │                         │
 Gazebo Adapter            Real Sensor Adapter
       │                         │
       └────────────┬────────────┘
                    │
              Common ROS Topics
                    │
         Perception / SLAM / AI
```

예를 들어 detector는 영상이 Gazebo에서 왔는지 Raspberry Pi Camera에서 왔는지 알 필요가 없어야 한다.

```text
/image_raw
    ↓
detector_node
    ↓
/detections
```

이 원칙을 지키면 Raspberry Pi 이식 작업은 알고리즘 재작성보다 **driver 교체 + 성능 최적화** 문제가 된다.

---

## 2.3 Ground Truth와 Estimated Pose를 분리

Simulation에서는 Gazebo가 실제 위치를 알고 있다.

그러나 SLAM을 평가할 때 Gazebo ground truth를 알고리즘 입력으로 사용하면 안 된다.

구조는 다음처럼 분리한다.

```text
Gazebo Ground Truth
       │
       └──────────→ Evaluation Only
                        │
                    ATE / RPE

Camera / IMU
       │
       ↓
SLAM / VIO
       │
       ↓
Estimated Pose
```

Ground Truth는 평가용으로만 사용한다.

---

## 2.4 LLM과 Robotics Logic을 분리

LLM은 다음 역할만 담당한다.

- 사용자 질문 해석
- DB 검색 요청 생성
- 검색 결과 자연어 설명
- 제한된 Mission API 호출

LLM에게 다음을 맡기지 않는다.

- motor PWM 생성
- attitude setpoint 생성
- collision avoidance
- 실시간 control loop
- raw SLAM pose 계산

권장 구조:

```text
LLM
 ↓
Structured Command
 ↓
Mission Manager
 ↓
Safety Validator
 ↓
Planner
 ↓
PX4
```

예:

```json
{
  "action": "go_to_object",
  "target": "backpack_01"
}
```

---

# 3. 권장 개발 환경

2026년 현재 PX4 공식 문서는 다음 조합을 권장한다.

```text
OS          : Ubuntu 24.04 LTS
ROS         : ROS 2 Jazzy LTS
Simulator   : Gazebo Harmonic
Autopilot   : PX4
Middleware  : uXRCE-DDS
Build       : colcon
Visualization: RViz2
```

가능하면 Desktop PC와 Raspberry Pi 모두 같은 Ubuntu/ROS 2 세대를 사용한다.

## 개발 PC

현재 PC의 RTX GPU는 다음 작업에 활용한다.

- YOLO inference
- 모델 학습/파인튜닝
- VLM/LLM 실험
- Gazebo simulation
- rosbag 분석
- offline SLAM evaluation

## WSL2 사용 시

WSL2에서도 일부 개발은 가능하지만 본 프로젝트는 Gazebo GUI, 네트워크, USB/Serial, ROS 2 DDS를 많이 사용한다.

따라서 권장 순서는 다음과 같다.

```text
초기 알고리즘 개발
    WSL2 사용 가능

PX4 + Gazebo + ROS2 통합
    Native Ubuntu 권장

실제 FC / Camera / Serial 연결
    Native Ubuntu 또는 Raspberry Pi 권장
```

가능하다면 프로젝트용 Ubuntu 24.04 환경을 별도로 마련한다.

---

# 4. 전체 소프트웨어 아키텍처

```text
┌──────────────────────────────────────────────────┐
│                   User Interface                 │
│           CLI / Web UI / Natural Language        │
└──────────────────────┬───────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────┐
│                  LLM Interface                   │
│  Query Parser / Function Calling / Response Gen  │
└──────────────────────┬───────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────┐
│                 Semantic Database                │
│ Object / Position / Relation / Confidence / Time │
└──────────────────────▲───────────────────────────┘
                       │
                       │ update
┌──────────────────────┴───────────────────────────┐
│                Semantic Mapper                   │
│  Detection + Depth + TF + Tracking + Association │
└──────────────▲────────────────────▲──────────────┘
               │                    │
          detections             map pose
               │                    │
┌──────────────┴───────┐   ┌────────┴──────────────┐
│ Object Detector      │   │ SLAM / VIO            │
│ YOLO                 │   │ RTAB-Map              │
└──────────────▲───────┘   └────────▲──────────────┘
               │                    │
               └─────────┬──────────┘
                         │
                 RGB / Depth / IMU
                         │
              ┌──────────┴──────────┐
              │ Sensor Interface    │
              └──────────▲──────────┘
                         │
            ┌────────────┴─────────────┐
            │                          │
      Gazebo Simulation           Real Sensors
            │                          │
            └────────────┬─────────────┘
                         │
                       ROS 2
                         │
                         ▼
                        PX4
```

---

# 5. ROS 2 패키지 구성

프로젝트 repository는 처음부터 실제 배포를 고려해 구성한다.

```text
semantic_exploration_drone/
│
├── README.md
├── LICENSE
│
├── docs/
│   ├── software_architecture.md
│   ├── ros_interfaces.md
│   ├── simulation.md
│   ├── slam.md
│   ├── semantic_mapping.md
│   ├── llm_interface.md
│   └── edge_migration.md
│
├── config/
│   ├── px4/
│   ├── slam/
│   ├── detector/
│   ├── semantic_mapper/
│   └── mission/
│
├── simulation/
│   ├── worlds/
│   ├── models/
│   └── scripts/
│
├── ros2_ws/
│   └── src/
│       │
│       ├── drone_interfaces/
│       │
│       ├── drone_bringup/
│       │
│       ├── px4_bridge/
│       │
│       ├── sensor_adapter/
│       │
│       ├── drone_slam/
│       │
│       ├── object_detector/
│       │
│       ├── object_tracker/
│       │
│       ├── semantic_mapper/
│       │
│       ├── semantic_database/
│       │
│       ├── exploration/
│       │
│       ├── mission_manager/
│       │
│       └── llm_interface/
│       │
│       └── system_monitor/
│
├── models/
│   └── README.md
│
├── tools/
│   ├── evaluation/
│   ├── rosbag/
│   └── visualization/
│
├── tests/
│   ├── unit/
│   ├── integration/
│   └── simulation/
│
└── docker/
```

---

# 6. ROS 인터페이스를 먼저 정의

실제 알고리즘 구현보다 먼저 message contract를 정의한다.

## 입력 센서

```text
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info

/imu/data

/tf
/tf_static
```

## PX4

예:

```text
/fmu/out/vehicle_odometry
/fmu/out/vehicle_status
/fmu/out/vehicle_local_position

/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
```

실제 topic 이름은 사용 중인 PX4/px4_msgs 버전에 맞춰 고정한다.

## Perception

```text
/perception/detections_2d
/perception/tracks
/perception/objects_3d
```

## SLAM

```text
/slam/odom
/slam/map
/slam/pointcloud
```

## Semantic Mapping

```text
/semantic_map/objects
/semantic_map/markers
```

## Mission

```text
/mission/goal
/mission/status
```

---

# 7. Custom ROS Message

초기부터 객체 표현을 표준화한다.

예:

```text
SemanticObject.msg

string id
string class_name

geometry_msgs/Point position

float32 confidence

builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen

uint32 observation_count
```

향후에는 다음을 추가할 수 있다.

```text
geometry_msgs/Vector3 size
geometry_msgs/Quaternion orientation

string[] relations

float32 localization_uncertainty
```

Semantic Object Array:

```text
SemanticObjectArray.msg

std_msgs/Header header
SemanticObject[] objects
```

---

# 8. TF Frame 규칙

처음부터 좌표계를 엄격히 관리한다.

권장 구조:

```text
map
 │
 └── odom
      │
      └── base_link
           │
           ├── camera_link
           │    ├── camera_left
           │    └── camera_right
           │
           └── imu_link
```

## 의미

### map

SLAM이 관리하는 global reference.

### odom

local continuous pose reference.

### base_link

드론 중심.

### camera_link

카메라 extrinsic 기준.

Semantic object의 위치는 최종적으로 모두 `map` frame에 저장한다.

---

# 9. 개발 단계 개요

전체 소프트웨어 개발은 아래 8단계로 나눈다.

```text
Phase 0
Development Environment
       ↓
Phase 1
PX4 + Gazebo
       ↓
Phase 2
ROS2 + Drone Interface
       ↓
Phase 3
SLAM
       ↓
Phase 4
Object Detection
       ↓
Phase 5
Semantic Mapping
       ↓
Phase 6
LLM Query
       ↓
Phase 7
Autonomous Mission
       ↓
Phase 8
Raspberry Pi Migration
```

각 Phase는 독립적인 통과 기준을 가진다.

---

# 10. Phase 0 — Development Environment

## 목표

모든 개발 환경을 reproducible하게 만든다.

## 설치 대상

```text
Ubuntu 24.04
ROS 2 Jazzy
Gazebo Harmonic
PX4
QGroundControl
Micro XRCE-DDS Agent
RViz2
RTAB-Map ROS
OpenCV
PyTorch
Ultralytics
SQLite
```

## 먼저 확인할 것

```bash
ros2 topic list
```

```bash
gz sim
```

```bash
cd PX4-Autopilot
make px4_sitl gz_x500
```

## 통과 조건

- PX4 SITL build 성공
- Gazebo에서 X500 등장
- QGroundControl에서 vehicle 연결
- ROS 2 example node 실행
- colcon workspace build 성공

이 조건이 만족되기 전에는 AI 코드를 작성하지 않는다.

---

# 11. Phase 1 — PX4 + Gazebo Flight Simulation

## 목적

실제 기체가 없어도 PX4 flight stack 사용법을 익힌다.

초기 vehicle:

```text
gz_x500
```

그 다음:

```text
gz_x500_depth
```

필요하면:

```text
gz_x500_vision
```

PX4 Gazebo에는 X500, visual odometry 버전, front-facing depth camera 버전이 제공된다.

## 구현할 기능

1. Arm
2. Takeoff
3. Hover
4. Position command
5. Land
6. Failsafe 확인

## 아직 하지 않을 것

- SLAM
- YOLO
- LLM
- 자율 탐색

먼저 비행 명령 계층만 분리해서 검증한다.

## Mission API 초안

```python
takeoff(height)
land()
goto(x, y, z, yaw)
hover(duration)
return_home()
```

이 API는 나중에도 유지한다.

## 통과 조건

Simulation에서 다음 sequence가 10회 연속 성공해야 한다.

```text
Arm
 ↓
Takeoff 1.5m
 ↓
Hover 5s
 ↓
Move 2m
 ↓
Hover
 ↓
Return
 ↓
Land
```

---

# 12. Phase 2 — ROS 2 + PX4 Interface

## 목적

PX4를 ROS 2에서 완전히 제어하고 telemetry를 읽는다.

PX4와 ROS 2 사이에는 uXRCE-DDS를 사용한다.

```text
PX4 uORB
   ↓
uXRCE-DDS Client
   ↓
Micro XRCE-DDS Agent
   ↓
ROS 2 DDS
   ↓
px4_msgs
```

## 반드시 구현할 노드

```text
px4_bridge
```

역할:

- vehicle state 수신
- odometry 수신
- offboard heartbeat
- trajectory setpoint publish
- arm/disarm
- takeoff/land abstraction

상위 node가 `px4_msgs` 내부 구조를 직접 사용하지 않도록 한다.

예:

```text
Mission Manager
      ↓

/mission/target_pose

      ↓
PX4 Bridge
      ↓
px4_msgs
```

## 이유

향후 PX4 message schema가 변경되더라도 `px4_bridge`만 수정할 수 있다.

## 통과 조건

ROS 2 node 하나로 다음 동작이 가능해야 한다.

```bash
ros2 launch drone_bringup simulation.launch.py
```

실행 후 자동으로:

```text
PX4
Gazebo
DDS Agent
Bridge
RViz
```

가 올라와야 한다.

---

# 13. Phase 3 — Sensor Pipeline

## 목적

SLAM과 detector가 실제 센서와 무관하게 작동하도록 한다.

Simulation Adapter를 만든다.

```text
Gazebo RGB
Gazebo Depth
Gazebo CameraInfo
       ↓
sensor_adapter
       ↓
Common Sensor Topics
```

출력:

```text
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
```

향후 Raspberry Pi에서는:

```text
libcamera / camera_ros
       ↓
real_sensor_adapter
       ↓
동일한 Common Sensor Topics
```

## 중요한 테스트

### Timestamp

RGB, Depth, CameraInfo timestamp가 일관적인지 확인.

### Frame ID

```text
camera_link
camera_optical_frame
```

이 올바르게 설정되는지 확인.

### Frequency

예:

```text
RGB   15~30 Hz
Depth 15~30 Hz
```

초기에는 15 Hz로 시작해도 충분하다.

## 통과 조건

RViz/OpenCV에서 RGB와 Depth가 동시에 정상 표시되고 rosbag recording/replay가 가능해야 한다.

---

# 14. Phase 4 — SLAM을 독립적으로 완성

SLAM은 detector와 결합하기 전에 먼저 완성한다.

초기 추천:

```text
RTAB-Map
```

RTAB-Map ROS 2는 RGB-D, stereo, LiDAR 입력과 loop closure를 지원하며 point cloud, occupancy map, OctoMap 출력이 가능하다.

## V1

Gazebo depth camera를 사용한다.

```text
RGB
+
Depth
+
CameraInfo
    ↓
RTAB-Map
    ↓
Odometry
+
3D Map
```

Stereo는 나중 문제로 미룬다.

이유:

RGB-D simulation은 depth ground truth에 가까워 SLAM 자체와 semantic mapping 로직을 먼저 검증하기 좋다.

## V2

Stereo camera simulation.

```text
Left
+
Right
    ↓
Stereo Depth
    ↓
Visual Odometry
    ↓
SLAM
```

## V3

실제 Raspberry Pi stereo.

---

# 15. SLAM 평가

Simulation의 가장 큰 장점은 ground truth를 알고 있다는 것이다.

평가:

```text
Estimated trajectory
         vs
Gazebo ground truth
```

Metrics:

### ATE

Absolute Trajectory Error

global trajectory accuracy.

### RPE

Relative Pose Error

local odometry drift.

추가 평가:

- loop closure success
- tracking lost count
- map completeness
- processing FPS

## 초기 목표

엄격한 논문 수준 목표가 아니라 engineering gate로 설정한다.

```text
Tracking lost = 0
5분 simulation continuous mapping 성공
Loop closure 정상 동작
Trajectory visually consistent
ATE 계산 pipeline 구축
```

정량 기준은 시뮬레이션 환경을 고정한 뒤 baseline을 측정하여 결정한다.

---

# 16. rosbag 중심 개발

실제 드론을 띄우면서 알고리즘을 매번 디버깅하면 개발 속도가 크게 떨어진다.

따라서 초기부터 rosbag을 적극 사용한다.

예:

```bash
ros2 bag record \
/camera/color/image_raw \
/camera/depth/image_raw \
/camera/color/camera_info \
/tf \
/tf_static \
/fmu/out/vehicle_odometry
```

개발 구조:

```text
Simulation / Real Flight
       ↓
rosbag
       ↓
Offline Replay
       ↓
SLAM / YOLO / Semantic Mapper 개발
```

향후 실제 비행 데이터도 동일한 방식으로 반복 재생할 수 있다.

---

# 17. Phase 5 — Object Detection

SLAM과 독립적으로 detector를 개발한다.

초기에는 COCO pretrained YOLO를 사용한다.

예시 클래스:

```text
person
chair
couch
bed
dining table
tv
backpack
bottle
book
```

V1의 목적은 detector accuracy 연구가 아니라 **Semantic Mapping pipeline 검증**이다.

따라서 초기에는 모델 학습을 하지 않아도 된다.

## Detector Node

입력:

```text
/camera/color/image_raw
```

출력:

```text
/perception/detections_2d
```

객체:

```text
class
confidence
bbox
timestamp
```

## 추후 Raspberry Pi 대비

node 내부를 다음처럼 abstraction한다.

```text
DetectorBackend
   │
   ├── PyTorchBackend
   ├── ONNXBackend
   └── TFLiteBackend / NCNNBackend
```

ROS interface는 동일하게 유지한다.

---

# 18. Phase 6 — 2D Detection을 3D Object로 변환

이 단계가 프로젝트의 핵심이다.

입력:

```text
2D bounding box
+
Depth
+
Camera Intrinsic
+
Camera Pose
```

출력:

```text
Object XYZ in map frame
```

## Step 1

bbox center:

```text
(u, v)
```

## Step 2

depth 추출.

단순 center pixel 하나보다 bounding box 내부의 robust depth를 사용한다.

추천:

```text
bbox center region
    ↓
invalid depth 제거
    ↓
median depth
```

## Step 3

pixel → camera coordinate.

Pinhole camera model:

```text
X = (u - cx) Z / fx
Y = (v - cy) Z / fy
Z = depth
```

## Step 4

TF transform:

```text
camera_optical_frame
        ↓
base_link
        ↓
odom
        ↓
map
```

최종:

```text
object position
(x_map, y_map, z_map)
```

---

# 19. Semantic Mapper

Semantic Mapper는 직접 구현해야 하는 핵심 모듈이다.

입력:

```text
/detections_2d
/depth
/tf
```

출력:

```text
/semantic_map/objects
```

내부 처리:

```text
Detection
    ↓
Depth association
    ↓
Camera XYZ
    ↓
TF transform
    ↓
Map XYZ
    ↓
Object association
    ↓
Semantic Database
```

---

# 20. 같은 객체를 여러 번 세는 문제

드론이 같은 의자를 100 frame 관찰할 수 있다.

이를 하나의 object로 통합해야 한다.

초기 association 기준:

```text
same class
+
3D distance < threshold
```

예:

```text
class == chair

distance(new_detection, chair_01)
< 0.5m

→ chair_01 update
```

그 다음 tracking 정보를 추가한다.

```text
ByteTrack
   +
3D Spatial Association
```

장기적으로는 appearance embedding을 추가할 수 있다.

```text
class
+
3D position
+
Re-ID feature
```

---

# 21. Object State

각 객체는 다음 정보를 가진다.

```json
{
  "id": "chair_01",
  "class": "chair",

  "position": [
    2.41,
    -1.02,
    0.44
  ],

  "confidence": 0.93,

  "observation_count": 31,

  "first_seen": "...",
  "last_seen": "..."
}
```

위치 업데이트는 단순 평균부터 시작한다.

```text
P_new =
alpha * P_detection
+
(1-alpha) * P_old
```

추후 uncertainty-aware fusion으로 확장 가능하다.

---

# 22. Semantic Database

처음에는 SQLite를 권장한다.

이유:

- Raspberry Pi에서 사용 가능
- 서버가 필요 없음
- SQL query 가능
- Python/C++ 모두 접근 가능
- 로그 저장 쉬움

Schema 예:

```text
objects

id
class_name
x
y
z
confidence
first_seen
last_seen
observation_count
```

relations:

```text
subject_id
relation
object_id
confidence
```

예:

```text
bottle_01
on
table_01
```

---

# 23. Spatial Relation 생성

단순 xyz만 저장하는 것보다 관계를 생성하면 LLM 활용성이 크게 올라간다.

V1 관계:

```text
near
left_of
right_of
in_front_of
behind
```

V2:

```text
on
under
inside
```

예:

```text
distance(bottle, desk) < threshold
+
bottle.z > desk.z

→ candidate relation
```

단, `on`, `inside`는 단순 좌표만으로 확정하기 어려우므로 초기에는 보수적으로 사용한다.

---

# 24. Phase 7 — LLM Interface

LLM은 Semantic DB 위에 올라간다.

구조:

```text
User
 ↓
LLM
 ↓
Tool / Function Call
 ↓
Semantic Database
 ↓
Structured Result
 ↓
LLM
 ↓
Natural Language
```

예:

```text
사용자:
물병이 어디 있어?
```

LLM function:

```json
{
  "function": "find_objects",
  "arguments": {
    "class_name": "bottle"
  }
}
```

DB:

```json
{
  "id": "bottle_01",
  "position": [2.4, 0.7, 0.8]
}
```

LLM:

```text
물병 한 개가 확인되었습니다.
출발 위치 기준 약 2.5m 떨어진 위치입니다.
```

---

# 25. LLM hallucination 방지

LLM에게 scene 정보를 기억하게 하지 않는다.

항상 DB를 source of truth로 사용한다.

금지:

```text
LLM memory:
"아까 가방 있었던 것 같은데..."
```

권장:

```text
User
 ↓
Database Query
 ↓
Result
 ↓
LLM Response
```

객체가 없다면:

```text
"현재 생성된 Semantic Map에서는 해당 객체를 확인하지 못했습니다."
```

라고 답하도록 한다.

---

# 26. 로컬 LLM 개발

초기에는 LLM을 PC에서 실행한다.

사용자가 이미 Ollama를 사용하고 있으므로 구조는 다음처럼 만들 수 있다.

```text
ROS2 llm_interface
      ↓ HTTP
Ollama Server
      ↓
Local LLM
```

다만 LLM 자체는 ROS node 내부에 강하게 결합하지 않는다.

```python
class LLMBackend:
    def query(...):
        ...

class OllamaBackend(LLMBackend):
    ...

class CloudBackend(LLMBackend):
    ...
```

이 구조를 사용하면 나중에 모델을 교체하기 쉽다.

---

# 27. Phase 8 — Mission Manager

LLM 질의가 안정된 뒤에 행동 기능을 추가한다.

지원할 action을 명확히 제한한다.

V1:

```text
TAKEOFF
LAND
HOVER
GOTO
RETURN_HOME
```

V2:

```text
GOTO_OBJECT
LOOK_AT_OBJECT
```

V3:

```text
EXPLORE
SEARCH_OBJECT
```

---

# 28. Mission Manager State Machine

LLM과 PX4 사이에 반드시 deterministic state machine을 둔다.

예:

```text
IDLE
 ↓
ARMING
 ↓
TAKEOFF
 ↓
HOVER
 ↓
NAVIGATING
 ↓
GOAL_REACHED
 ↓
HOVER
 ↓
LAND
```

Error state:

```text
TRACKING_LOST
COMMUNICATION_LOST
INVALID_GOAL
MAP_UNAVAILABLE
LOW_CONFIDENCE
```

LLM은 state transition 규칙을 우회할 수 없다.

---

# 29. Autonomous Exploration은 마지막 단계

처음부터 자율 탐색 알고리즘을 구현하지 않는다.

개발 순서:

```text
Manual/Scripted Trajectory
        ↓
Waypoint Navigation
        ↓
Semantic Mapping
        ↓
Autonomous Exploration
```

초기 Semantic Mapping에서는 미리 정해진 trajectory를 사용한다.

예:

```text
Waypoint 1
 → Waypoint 2
 → Waypoint 3
 → Waypoint 4
 → Home
```

이렇게 해야 SLAM/semantic mapping 문제와 exploration 문제를 분리할 수 있다.

---

# 30. Exploration V1

첫 번째 autonomous exploration은 frontier-based exploration으로 충분하다.

개념:

```text
Known Free Space
       |
       | frontier
       v
Unknown Space
```

반복:

```text
Map
 ↓
Frontier Detection
 ↓
Candidate Goal
 ↓
Safety Check
 ↓
Goto
 ↓
Map Update
 ↓
Repeat
```

드론은 3D 이동이 가능하지만 처음에는 고도를 고정한다.

```text
z = 1.2m
```

따라서 V1 exploration은 사실상 2D planner처럼 동작하도록 만든다.

그 다음 3D exploration으로 확장한다.

---

# 31. Simulation World를 직접 만든다

기본 Gazebo world만 사용하지 말고 작은 실내 benchmark를 만든다.

예:

```text
Room A
|
Hallway
|
Room B
```

객체:

```text
table
chair
backpack
bottle
box
person model
```

각 객체의 ground-truth position을 알고 있도록 한다.

그러면 Semantic Mapping 성능을 자동으로 평가할 수 있다.

---

# 32. Semantic Mapping 평가

Ground Truth:

```text
chair_GT = (2.0, 1.0, 0.5)
```

Prediction:

```text
chair_pred = (2.17, 0.91, 0.52)
```

Localization Error:

```text
sqrt(
 dx² +
 dy² +
 dz²
)
```

측정 항목:

```text
Object Detection Recall
Object Localization Error
Duplicate Object Rate
False Object Rate
Semantic Map Recall
```

목표 예:

```text
Simulation object localization mean error < 0.30m

Duplicate object rate < 10%

Semantic map object recall > 80%
```

정확한 threshold는 첫 baseline 결과를 얻은 뒤 조정한다.

---

# 33. End-to-End Simulation Test

PC 단계의 최종 성공 조건이다.

Launch:

```bash
ros2 launch drone_bringup full_simulation.launch.py
```

자동 실행:

```text
Gazebo
PX4
DDS Agent
Sensor Adapter
RTAB-Map
YOLO
Semantic Mapper
Semantic DB
Mission Manager
LLM Interface
RViz
```

Test Scenario:

```text
1. Drone takeoff

2. predefined indoor route

3. SLAM map generation

4. object detection

5. object 3D localization

6. semantic DB update

7. return

8. land
```

착륙 후:

```text
User:
의자는 몇 개 있었어?

System:
2개
```

```text
User:
가방은 어디 있어?

System:
방의 북서쪽 영역에서 하나를 확인했습니다...
```

여기까지 PC에서 성공한 뒤 Raspberry Pi migration을 시작한다.

---

# 34. Raspberry Pi 이식 원칙

**모든 것을 한 번에 Pi로 옮기지 않는다.**

PC에서 안정적으로 동작한 component를 하나씩 이동한다.

```text
Stage R0
Everything on PC

Stage R1
Sensor Driver on Pi

Stage R2
PX4 Bridge on Pi

Stage R3
SLAM on Pi

Stage R4
Detector on Pi

Stage R5
Semantic Mapper on Pi

Stage R6
Mission Manager on Pi

Stage R7
LLM 선택적 이식
```

---

# 35. R0 — PC-only Baseline

구성:

```text
Simulation / Sensors
       ↓
PC
 ├ PX4 communication
 ├ SLAM
 ├ Detector
 ├ Semantic Mapper
 ├ DB
 ├ LLM
 └ Mission Manager
```

이 상태의 성능을 반드시 기록한다.

기록:

```text
CPU
GPU
RAM
FPS
latency
SLAM processing time
detector processing time
semantic mapper time
```

이 결과가 Raspberry Pi migration baseline이다.

---

# 36. R1 — Raspberry Pi Sensor Server

첫 이식 대상은 카메라 interface다.

```text
Raspberry Pi
 └ Camera Driver
       │
       │ ROS2 / network
       ▼
PC
 ├ SLAM
 ├ Detector
 └ Semantic Mapper
```

목적:

실제 Raspberry Pi camera pipeline과 네트워크만 검증.

확인:

- image FPS
- timestamp
- frame drop
- bandwidth
- latency
- CameraInfo
- TF

---

# 37. R2 — PX4 Bridge를 Raspberry Pi로 이동

```text
Flight Controller
       │ UART
       ▼
Raspberry Pi
 ├ DDS Agent
 ├ PX4 Bridge
 └ Camera
       │
       ▼
PC
 ├ SLAM
 ├ Detector
 ├ Semantic Mapper
 └ LLM
```

이 단계부터 Pi가 companion computer 역할을 한다.

PC는 아직 AI server 역할을 한다.

---

# 38. R3 — SLAM을 Raspberry Pi로 이동

```text
Raspberry Pi
 ├ Camera
 ├ PX4 Bridge
 └ SLAM
      │
      ├ pose
      └ map
       │
       ▼
PC
 ├ Detector
 ├ Semantic Mapper
 └ LLM
```

평가:

```text
SLAM FPS
CPU load
RAM
temperature
tracking lost count
latency
```

PC baseline과 정확도를 비교한다.

---

# 39. R4 — Detector 이식

Detector backend를 경량화한다.

후보:

```text
PyTorch
   ↓
ONNX
   ↓
경량 inference backend
```

모델 후보:

```text
YOLO nano-class model
```

Pi에서 detector가 너무 느리면 architecture는 그대로 두고 detector만 PC에서 실행해도 된다.

즉 다음 hybrid 구조도 최종 프로젝트로 충분히 유효하다.

```text
Drone / Pi
 ├ Flight
 ├ Sensor
 └ SLAM

Wi-Fi

Ground PC
 ├ Heavy Detector
 ├ VLM
 └ LLM
```

최종 재난용 fully onboard system과 첫 실내 prototype을 동일시하지 않는다.

---

# 40. R5 — Semantic Mapper 이식

Semantic Mapper는 상대적으로 연산량이 작으므로 Pi에서 실행할 가능성이 높다.

```text
Raspberry Pi
 ├ SLAM
 ├ Detector
 ├ TF
 ├ Semantic Mapper
 └ SQLite DB
```

이 시점에는 드론 자체가 Semantic Map을 보유하게 된다.

PC가 끊겨도 semantic information은 Pi에 저장되어야 한다.

---

# 41. R6 — Mission Manager 이식

Mission Manager도 Pi에서 실행한다.

```text
Raspberry Pi

Camera
  ↓
SLAM
  ↓
Semantic Map
  ↓
Mission Manager
  ↓
PX4
```

LLM 연결이 끊겨도 현재 mission을 안전하게 중지할 수 있어야 한다.

---

# 42. R7 — LLM은 마지막

LLM은 반드시 Raspberry Pi에 올라갈 필요가 없다.

세 가지 deployment를 비교한다.

## Option A

```text
LLM on PC
```

가장 현실적인 첫 버전.

## Option B

```text
Small LLM on Raspberry Pi
```

offline language interface 연구용.

## Option C

```text
Hybrid

Pi
 └ Robotics

PC
 └ LLM/VLM
```

재난 상황에서 통신 단절까지 고려한다면 장기적으로 onboard 모델을 연구할 수 있다.

---

# 43. Edge Migration 통과 기준

각 node를 Pi로 옮길 때 다음을 비교한다.

| Metric | PC | Pi | 허용 기준 |
|---|---:|---:|---:|
| Input FPS | | | |
| Output FPS | | | |
| Mean latency | | | |
| P95 latency | | | |
| CPU | | | |
| GPU/NPU | | | |
| RAM | | | |
| Temperature | | | |
| Power | | | |
| Accuracy | | | |

단순히 "Pi에서 실행된다"를 성공 기준으로 사용하지 않는다.

---

# 44. 성능 측정 Node

초기부터 `system_monitor` 패키지를 만든다.

수집:

```text
node latency
topic frequency
CPU usage
RAM
temperature
dropped frame
```

실험 결과를 CSV로 저장한다.

```text
experiments/
  001_pc_baseline/
  002_pi_camera/
  003_pi_slam/
  004_pi_detector/
```

각 폴더:

```text
config.yaml
metrics.csv
notes.md
rosbag/
```

---

# 45. Testing 전략

## Unit Test

대상:

```text
Coordinate transform
DB query
Object association
Mission parser
Safety validator
```

예:

```python
def test_pixel_to_xyz():
    ...
```

## Integration Test

```text
Detector
+
Depth
+
TF
↓
Semantic Mapper
```

## Simulation Test

```text
Gazebo
+
PX4
+
Full ROS2 Stack
```

CI에서는 가능한 범위까지 자동화한다.

---

# 46. 반드시 독립 테스트해야 할 항목

## A. PX4 control

SLAM 없이 test.

## B. SLAM

PX4 mission logic 없이 rosbag으로 test.

## C. Detector

SLAM 없이 image dataset으로 test.

## D. Semantic Mapper

실제 detector 없이 synthetic detection message로 test.

## E. LLM

실제 drone 없이 mock semantic DB로 test.

이 구조가 중요하다.

예를 들어 LLM 개발 때문에 드론을 띄울 필요가 없어야 한다.

---

# 47. Mock Data를 적극 활용

Semantic Mapper 테스트용:

```json
{
  "class": "chair",
  "bbox": [320, 210, 500, 620],
  "confidence": 0.95
}
```

Depth:

```text
2.0m
```

Pose:

```text
camera = (1.0, 1.0, 1.2)
```

expected:

```text
chair map coordinate ≈ (...)
```

이런 fixture를 만들어 regression test를 수행한다.

---

# 48. Simulation → Real 전환 시 교체되는 것

최대한 아래 항목만 바뀌게 만든다.

| Simulation | Real |
|---|---|
| Gazebo RGB | Raspberry Pi Camera |
| Gazebo Depth | Stereo / Depth estimate |
| Gazebo IMU | Flight Controller IMU |
| SITL PX4 | Hardware PX4 |
| UDP DDS | UART DDS |
| Ground Truth | 없음 |

바뀌면 안 되는 것:

```text
Detector interface
SLAM output
SemanticObject
Database
LLM tools
Mission API
Evaluation framework
```

---

# 49. 실제 하드웨어가 오기 전에 구현 완료해야 할 것

최소한 다음은 모두 PC에서 끝내놓는다.

- [ ] PX4 SITL 실행
- [ ] Gazebo indoor world
- [ ] ROS 2 PX4 bridge
- [ ] Offboard control
- [ ] waypoint mission
- [ ] RGB/Depth sensor adapter
- [ ] rosbag record/replay
- [ ] RTAB-Map
- [ ] trajectory evaluation
- [ ] YOLO ROS node
- [ ] 2D detection message
- [ ] depth association
- [ ] camera → map transform
- [ ] SemanticObject message
- [ ] duplicate object association
- [ ] SQLite semantic DB
- [ ] RViz semantic marker
- [ ] natural language query
- [ ] mission state machine
- [ ] system monitoring
- [ ] full simulation launch
- [ ] repeatable end-to-end test

여기까지 완료되면 실제 하드웨어 통합은 새로운 프로젝트가 아니라 기존 software의 **sensor/transport backend 교체**가 된다.

---

# 50. 개발 우선순위

다음 순서를 지킨다.

## Priority 1 — Infrastructure

```text
PX4
ROS2
Gazebo
Topics
TF
rosbag
```

## Priority 2 — Localization

```text
Camera
Depth
Odometry
SLAM
```

## Priority 3 — Semantic Perception

```text
Detection
3D Localization
Tracking
Semantic DB
```

## Priority 4 — Language

```text
DB query
LLM
```

## Priority 5 — Autonomy

```text
Mission Manager
Planner
Exploration
```

## Priority 6 — Edge

```text
Raspberry Pi
Optimization
```

반대로 진행하지 않는다.

---

# 51. 10주 Software-only Roadmap

실제 하드웨어가 없어도 진행 가능한 기준이다.

## Week 1 — PX4 / ROS 2 기본 환경

목표:

```text
PX4 SITL
+
Gazebo
+
ROS2
```

결과:

- X500 spawn
- takeoff
- ROS 2 topic 확인
- PX4 telemetry 확인

---

## Week 2 — ROS2 Flight Interface

구현:

```text
px4_bridge
mission_manager
```

결과:

```text
takeoff()
goto()
hover()
land()
```

---

## Week 3 — Sensor Pipeline + rosbag

구현:

```text
sensor_adapter
camera topic
depth topic
TF
```

결과:

```text
Gazebo sensor → ROS2
```

rosbag 저장/재생 가능.

---

## Week 4 — SLAM

RTAB-Map 연동.

결과:

```text
camera
 ↓
odometry
 ↓
3D map
```

ATE/RPE 평가 script 작성.

---

## Week 5 — Object Detection

YOLO ROS node.

결과:

```text
image
 ↓
detections_2d
```

RViz detection visualization.

---

## Week 6 — Semantic Mapping

구현:

```text
bbox
+
depth
+
TF
 ↓
object xyz
```

결과:

```text
semantic object marker
```

---

## Week 7 — Object Association + Database

구현:

```text
tracking
3D association
SQLite
```

결과:

```text
chair_01
bottle_01
backpack_01
```

중복 객체 제거.

---

## Week 8 — LLM

구현:

```text
semantic DB
+
LLM function interface
```

질의:

```text
"가방이 어디 있어?"

"책상 근처에 뭐가 있어?"

"사람을 발견했어?"
```

---

## Week 9 — Autonomous Mission

구현:

```text
Waypoint
+
Mission State Machine
+
Safety Validator
```

선택적으로 frontier exploration prototype.

---

## Week 10 — Full Simulation Benchmark

완전 통합.

```text
Takeoff
 ↓
Explore
 ↓
Map
 ↓
Detect
 ↓
Semantic Map
 ↓
Return
 ↓
Land
 ↓
LLM Query
```

이 시점을 **Software V1 완료**로 정의한다.

---

# 52. Raspberry Pi Migration Roadmap

Software V1 이후 진행한다.

## Pi-1

Ubuntu / ROS2 환경 구축.

## Pi-2

ROS2 network 통신.

## Pi-3

Camera driver.

## Pi-4

PX4 DDS bridge.

## Pi-5

SLAM benchmark.

## Pi-6

Detector benchmark.

## Pi-7

Semantic mapper + DB.

## Pi-8

Mission Manager.

## Pi-9

Full edge benchmark.

---

# 53. 첫 Raspberry Pi 이식에서 목표로 하지 않을 것

다음은 처음부터 요구하지 않는다.

- Raspberry Pi 단독 대형 LLM
- Raspberry Pi 단독 VLM
- 30 FPS object detection
- photorealistic 3D reconstruction
- 완전 자율 3D exploration
- dynamic obstacle avoidance
- 재난 현장 수준 robustness

V1에서는 다음이면 충분하다.

```text
Stable flight
+
SLAM
+
Object localization
+
Semantic map
+
Natural language query
```

---

# 54. 장기 확장

Software V1 이후 다음 순서가 적절하다.

## V2 — Stereo VIO

```text
Stereo Camera
+
IMU
```

## V3 — Autonomous Exploration

```text
Frontier
+
Planner
```

## V4 — Scene Graph

```text
Object
+
Spatial Relation
+
Room
```

## V5 — VLM

```text
Open-vocabulary perception
```

## V6 — 3D Semantic Reconstruction

```text
Point cloud / Gaussian / Mesh
+
Semantic label
```

## V7 — Disaster Mission

```text
Victim
Hazard
Blocked path
Exit
Fire extinguisher
```

## V8 — Language-conditioned Mission Planning

```text
"2층 복도를 조사하고 사람이 있으면 위치를 보고해."
```

---

# 55. 최종 Software V1 성공 기준

Hardware 이식을 시작하기 위한 최소 Gate.

## Flight Simulation

- PX4 offboard flight sequence 반복 성공
- mission abort 가능
- automatic landing 가능

## ROS2

- single launch bringup
- topic/interface document 완료
- rosbag replay 가능

## SLAM

- 5분 이상 mapping
- tracking loss 없음 또는 원인 파악 가능
- trajectory evaluation 가능

## Perception

- 실시간 detection
- detector ROS interface 고정

## Semantic Mapping

- object 3D position 생성
- duplicate association
- DB 저장
- RViz visualization

## LLM

다음 질의 가능:

```text
what
where
how many
near what
```

LLM 응답은 Semantic DB에 존재하는 정보만 사용.

## System

다음 명령 하나로 전체 simulation이 올라온다.

```bash
ros2 launch drone_bringup full_simulation.launch.py
```

---

# 56. 최종 단계별 Gate 요약

| Gate | 반드시 확인할 것 | 다음 단계 |
|---|---|---|
| G0 | PX4/Gazebo 실행 | Flight |
| G1 | Offboard mission 안정 | Sensors |
| G2 | Sensor ROS interface 안정 | SLAM |
| G3 | SLAM 독립 동작 | Detection |
| G4 | Detection 독립 동작 | Semantic |
| G5 | Object 3D localization | Database |
| G6 | Semantic DB 신뢰성 | LLM |
| G7 | LLM DB-grounded query | Mission |
| G8 | Full simulation | Raspberry Pi |
| G9 | Pi sensor + PX4 bridge | Pi SLAM |
| G10 | Pi SLAM 안정 | Pi perception |
| G11 | Edge benchmark | Real flight integration |

중간 Gate를 건너뛰지 않는다.

---

# 57. 지금 바로 시작할 작업

첫 번째 작업 목표는 아래 하나로 제한한다.

```text
Ubuntu 24.04
    ↓
ROS 2 Jazzy
    ↓
PX4 SITL
    ↓
Gazebo Harmonic
    ↓
X500 Depth
    ↓
uXRCE-DDS
    ↓
ROS2
```

확인해야 하는 것은 세 가지다.

```text
1. Gazebo에서 드론이 정상적으로 뜨는가?

2. ROS2에서 PX4 vehicle state를 받을 수 있는가?

3. Gazebo camera/depth 데이터를 ROS2 topic으로 받을 수 있는가?
```

이 세 가지가 확인된 이후에만 RTAB-Map을 추가한다.

---

# 58. 첫 개발 세션 Checklist

```text
[ ] Ubuntu 24.04 준비

[ ] ROS 2 Jazzy 설치

[ ] PX4-Autopilot clone

[ ] PX4 SITL build

[ ] gz_x500 실행

[ ] gz_x500_depth 실행

[ ] px4_msgs workspace 구성

[ ] Micro XRCE-DDS Agent 실행

[ ] ros2 topic list 확인

[ ] vehicle_odometry 확인

[ ] Gazebo ↔ ROS2 sensor bridge 확인

[ ] RViz2 실행

[ ] Git repository 초기화

[ ] first_simulation 브랜치 또는 milestone 기록
```

---

# 59. 참고 공식 문서

본 계획은 아래 공식 문서를 기준으로 작성했다.

## PX4 ROS 2

PX4 ROS 2 User Guide  
https://docs.px4.io/main/en/ros2/user_guide

PX4 uXRCE-DDS  
https://docs.px4.io/main/en/middleware/uxrce_dds

PX4 ROS 2 Overview  
https://docs.px4.io/main/en/ros/ros2

## PX4 Simulation

PX4 Gazebo Vehicles  
https://docs.px4.io/main/en/sim_gazebo_gz/vehicles

PX4 Ubuntu Development Environment  
https://docs.px4.io/main/en/dev_setup/dev_env_linux

## RTAB-Map

RTAB-Map ROS 2  
https://github.com/introlab/rtabmap_ros/tree/ros2

RTAB-Map Odometry  
https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_odom

RTAB-Map Synchronization  
https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_sync

---

# 60. 핵심 결론

이 프로젝트의 소프트웨어는 처음부터 다음 구조를 지향한다.

```text
Hardware-specific Layer
        ↓
Stable ROS Interface
        ↓
SLAM
        ↓
Perception
        ↓
Semantic Mapping
        ↓
Semantic Database
        ↓
LLM
        ↓
Mission Manager
```

따라서 실제 드론이 완성될 때까지 기다릴 필요가 없다.

하드웨어가 준비되기 전에 PC Simulation에서 다음을 모두 구현한다.

```text
PX4 Flight
ROS2
SLAM
Object Detection
3D Object Localization
Semantic Mapping
LLM Query
Mission Logic
```

그 뒤에는

```text
Gazebo Sensor
     ↓
Real Sensor

PX4 SITL
     ↓
PX4 Hardware
```

두 부분을 교체하고, 연산량이 허용되는 ROS node부터 Raspberry Pi로 단계적으로 옮긴다.

이 방식의 목표는 단순히 실내에서 한 번 동작하는 데모를 만드는 것이 아니라, 이후 재난 탐사용 드론으로 확장할 수 있는 **모듈형 Physical AI software stack**을 만드는 것이다.
