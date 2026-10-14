# Phase 0 — Development Environment

**완료일** 2026-10-14 · **증거** `experiments/000_phase0_env/`

## 목적
계획서 §10. PX4 SITL + Gazebo + ROS 2 + uXRCE-DDS가 한 머신(WSL2)에서 재현 가능하게 동작하도록 한다.

## 구성 (고정 버전)

| 구성요소 | 버전 | 설치 위치 / 스크립트 |
|---|---|---|
| Ubuntu | 24.04.4 (WSL2, kernel 6.18) | — |
| ROS 2 | Jazzy desktop + `ros-jazzy-ros-gz` | `scripts/setup/01_ros2_jazzy.sh` |
| PX4-Autopilot | v1.17.0, `px4_sitl_default` 빌드 | `~/PX4-Autopilot`, `scripts/setup/02_px4.sh` |
| Gazebo | Harmonic 8.15 (PX4 `ubuntu.sh`가 설치) | 02 |
| px4_msgs | v1.17.0 (PX4와 동일 태그 필수) | `ros2_ws/src/px4_msgs`, `scripts/setup/03_ros2_ws.sh` |
| Micro-XRCE-DDS-Agent | v2.4.3 | `ros2_ws/src/Micro-XRCE-DDS-Agent`, 03 |
| QGroundControl | AppImage (선택) | `scripts/setup/04_qgc.sh` |

`scripts/env.sh`: conda 비활성화 → ROS 2 → 워크스페이스 source → `PX4_DIR`, `GZ_SIM_RESOURCE_PATH` export.
모든 ROS 쉘은 이 파일을 먼저 source 한다.

## 통과 조건 (§10) 과 결과

| 조건 | 결과 |
|---|---|
| PX4 SITL build 성공 | `build/px4_sitl_default/bin/px4` (1141 steps) |
| Gazebo에서 X500 등장 | `gazebo_x500_gui.png` (Entity Tree에 `x500_0`) |
| QGroundControl 연결 | 설치 스크립트만 제공, 자동 테스트에는 불필요 (NAV_DLL_ACT=0, §Phase 1 참조) |
| ROS 2 node 실행 / colcon build | 5 패키지 빌드, `/fmu/out/vehicle_local_position_v1` 50 Hz 수신 (`fmu_topics.txt`) |

## 설계 결정
- PX4 소스는 저장소 밖(`~/PX4-Autopilot`)에 둔다. WSL 가이드 권장(리눅스 파일시스템)이며 1.8 GB.
- px4_msgs/Agent는 `ros2_ws/src`에 clone하되 git에서 제외(.gitignore). 버전은 `scripts/setup/00_common.sh`의
  `PX4_VERSION`, `XRCE_AGENT_VERSION` 한 곳에서 관리.
- Jazzy에서는 Gazebo 브리지 패키지가 `ros-jazzy-ros-gz`(Harmonic 쌍 고정)이다. `-gzharmonic` 접미사는 Humble용.

## 알려진 문제
- **WSL2 wall clock 역행**: 커널 `tick`이 10000 µs에서 벗어나고 호스트 동기화가 0.3~0.8 s씩 되돌린다.
  PX4가 수신 타임스탬프를 XRCE timesync 오프셋으로 변환하므로 offboard 스트림이 "손실"로 보일 수 있다.
  → Phase 1에서 `COM_OF_LOSS_T=3`(SITL 전용)과 steady-clock 타이머로 대응. 시계 소스 변경, timesyncd 중지,
  `hv_utils.timesync_implicit=0`은 효과가 없어 원복했다.
- Gazebo GUI 실행 시 RTF ≈ 72 %, headless ≈ 100 %. 자동 테스트는 `headless:=true`.
- `python3 ... px4` 를 TTY 없이 실행하면 `pxh>` 셸이 EOF에서 무한 출력한다(75 s에 1 GB). 항상 `px4 -d`.
