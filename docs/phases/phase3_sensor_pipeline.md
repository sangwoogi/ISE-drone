# Phase 3 — Sensor Pipeline + rosbag

**완료일** 2026-10-14 · **증거** `experiments/003_phase3_sensors/`

## 목적
계획서 §13. SLAM과 detector가 센서 출처(Gazebo / 실카메라)와 무관하게 동작하도록 공통 센서 토픽
`/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info`(+ `/camera/depth/camera_info`)을
만들고, 타임스탬프·frame_id·주파수를 검증하며, rosbag 기록/재생이 되게 한다. 계획서 §31의 실내 벤치마크 world도 이 단계에서 만들었다.

## 사용자 결정
- 실내 world를 Phase 3에서 직접 제작 (`simulation/worlds/indoor_v1.sdf`).
- 객체는 Gazebo Fuel 가구/사람 모델 + 원시도형 보조.
- 카메라는 커스텀 모델 `x500_rgbd`: 컬러·깊이 모두 640×480, 동일 FOV(1.274 rad), 20 Hz (정합 RGB-D).

## 구현 파일

| 파일 | 내용 |
|---|---|
| `simulation/models/OakD-Lite-RGBD/` | PX4 OakD-Lite 복사본: `rgb`(640×480, topic `rgb_camera`)·`depth`(640×480 R_FLOAT32, topic `depth_camera`), FOV 1.274, 20 Hz |
| `simulation/models/x500_rgbd/` | x500 + OakD-Lite-RGBD (장착 위치 x500_depth와 동일 (0.12, 0.03, 0.242)) |
| `simulation/worlds/indoor_v1.sdf` | Room A(6×5 m) – 복도(4×2 m) – Room B(5×5 m), 벽 높이 2.5 m, 색 패널 7개(SLAM 특징), 원시도형 3개, Fuel 모델 7개(Table, Chair, Bookshelf, Standing person, Sofa, OfficeChairBlack, BedsideTable) |
| `simulation/worlds/indoor_v1_objects.json` / `indoor_v1_gt.json` | 객체 명세(클래스 매핑) / 시뮬레이션에서 추출한 GT 좌표 |
| `tools/export_gt_objects.py` | 실행 중인 Gazebo에서 `gz model -m <name> -p`로 GT 추출 (평가 전용) |
| `scripts/setup/05_sim_assets.sh` | 모델/world를 PX4 gz 트리에 symlink, Fuel 모델 사전 다운로드 |
| `drone_bringup/launch/sitl.launch.py` | 저장소 모델(`gz_x500_rgbd`)은 airframe 4002로 부팅(`PX4_SYS_AUTOSTART`), 자산 symlink 자동화, `world` 인자 |
| `drone_bringup/launch/simulation.launch.py`, `full_simulation.launch.py` | `world` 인자 전달, full_simulation 기본값 `gz_x500_rgbd` + `indoor_v1` |
| `sensor_adapter/config/gz_bridge_x500_rgbd.yaml` | ros_gz_bridge: `/rgb_camera`, `/depth_camera`, `/camera_info`, `/depth_camera/points`, `/clock` |
| `sensor_adapter/sensor_adapter_node.py` | frame_id 재작성, 깊이 CameraInfo 재발행(registered), 5 s 통계(Hz, skew) 로그 |
| `sensor_adapter/launch/gazebo_sensors.launch.py` | bridge + adapter + 정적 TF base_link→camera_link (0.132, 0, 0.261) → camera_optical_frame |
| `tests/simulation/test_phase3_sensors.py` | 15 s 수락 테스트: 주파수, frame_id, 정합 기하, 동기(skew), TF, 샘플 이미지 저장, `sensor_report.json` |
| `scripts/rosbag_record.sh`, `scripts/rosbag_replay_check.sh` | 기록(§16 토픽 세트) / 시뮬레이터 없이 재생 검증 |

## 실행
```bash
bash scripts/setup/05_sim_assets.sh                                        # 최초 1회 (symlink + Fuel 캐시)
source scripts/env.sh
ros2 launch drone_bringup simulation.launch.py vehicle:=gz_x500_rgbd world:=indoor_v1 use_sim_time:=true
ros2 launch sensor_adapter gazebo_sensors.launch.py                        # 다른 터미널
python3 tests/simulation/test_phase3_sensors.py --duration 15
scripts/rosbag_record.sh experiments/003_phase3_sensors/rosbag/sample 30
scripts/rosbag_replay_check.sh experiments/003_phase3_sensors/rosbag/sample   # 시뮬레이터 종료 후
python3 tools/export_gt_objects.py --world indoor_v1
```

## 통과 조건(§13)과 결과

| 조건 | 결과 (headless, use_sim_time) |
|---|---|
| RGB/Depth 주파수 15~30 Hz (초기 15 Hz) | adapter 5 s 창: 컬러 18.6~19.0 Hz, 깊이 17.4~17.6 Hz; 테스트 노드 수신 기준 14.5 / 10.7 Hz (best-effort 드롭 포함) |
| Timestamp 일관성 | 컬러–깊이 최근접 스탬프 차 median 0 ms (같은 렌더 틱), max 152 ms |
| Frame ID | 모든 메시지 `camera_optical_frame`; TF map→camera_optical_frame 조회 성공 |
| 정합 기하 | 컬러/깊이 640×480, K 동일 (fx=fy=432.5, cx=320, cy=240) |
| RViz/OpenCV에서 RGB·Depth 동시 표시 | `sample_color.png`, `sample_depth.png`(JET colormap), `rviz_sensors.png` |
| rosbag 기록/재생 | 15 s 기록 434 MB(mcap, 비압축 이미지), 재생 시 토픽 복원 — `rosbag_info.txt`, `rosbag_replay.txt` |
| 실내 world | `gazebo_indoor_v1.png`; GT 10개 객체 `indoor_v1_gt.json` |

`sensor_report.json`: `pass: true` (checks: rate_color, rate_depth, rate_camera_info, frame_ids, registered_geometry, sync, tf).

## 설계 결정
- Gazebo 카메라 센서는 `<topic>`을 지정해도 CameraInfo를 `/camera_info`에 발행하고(두 센서가 같은 토픽, 2×20 Hz), 깊이 카메라는 별도 info가 없다.
  두 카메라의 기하가 동일하므로 adapter가 컬러 info를 `/camera/depth/camera_info`로 재발행한다(`registered_depth: true`).
  실카메라에서는 드라이버가 각각 발행하면 자동으로 그것을 쓴다(1 s 이내 수신 시 재발행 중단).
- PX4는 모델을 `${PX4_GZ_MODELS}/<이름>/model.sdf`에서만 찾으므로 저장소 자산을 PX4 트리에 symlink한다(launch가 멱등 생성).
  저장소 모델은 airframe 파일이 없어 `PX4_SYS_AUTOSTART=4002`(x500_depth)로 부팅한다.
- 깊이 카메라 렌더링이 병목이라 GUI+RViz 동시 실행 시 RTF ≈ 0.3~0.7. 수락 테스트는 headless로 돌리고, FlightController의 wall-clock
  goto 타임아웃을 120 s로 늘렸다.
- rosbag은 git에서 제외(`experiments/**/rosbag/`). 압축이 필요하면 `--compression-mode file --compression-format zstd`.

## 알려진 문제 / 교훈
- 테스트 노드의 수신률이 adapter 집계보다 낮다(best-effort QoS, 파이썬 수신 지연). 주파수 판정은 adapter 로그와 함께 본다.
- 셸 스크립트의 bare `wait`는 백그라운드 launch까지 기다려 멈춘다. 반드시 `wait $PID`.
- 커스텀 world 로딩 시간이 길다(Fuel 메시 포함, 첫 기동 ~30 s).
