# Phase 1 — PX4 + Gazebo Flight Simulation

**완료일** 2026-10-14 · **증거** `experiments/001_phase1_flight/` · **태그** `first_simulation`

## 목적
계획서 §11. 실제 기체 없이 비행 명령 계층(Mission API)을 분리해 검증한다. SLAM/YOLO/LLM은 다루지 않는다.

## 구현 파일

| 파일 | 역할 |
|---|---|
| `ros2_ws/src/px4_bridge/px4_bridge/frames.py` | ENU↔NED 위치·yaw 변환 (px4_bridge 밖에서는 NED를 보지 않는다) |
| `ros2_ws/src/px4_bridge/px4_bridge/px4_interface.py` | **px4_msgs를 import하는 유일한 모듈.** 10 Hz offboard 스트림(steady clock 타이머), VehicleCommand 송신, 상태 미러링(`VehicleState`) |
| `ros2_ws/src/px4_bridge/px4_bridge/flight_controller.py` | Mission API `takeoff / land / goto / hover / return_home` (blocking, `FlightError`), 도착 판정, 최소 안전 범위 |
| `ros2_ws/src/px4_bridge/px4_bridge/flight_demo_node.py` | `ros2 run px4_bridge flight_demo` — 시퀀스 1회 |
| `ros2_ws/src/px4_bridge/config/px4_bridge.yaml` | 토픽명(PX4 v1.17 `_v1` 접미사), 허용 오차, 타임아웃 |
| `ros2_ws/src/drone_bringup/launch/sitl.launch.py` | XRCE Agent + PX4(`px4 -d`, `PX4_SIM_MODEL`) + 부팅 후 `px4-param` 오버라이드 |
| `tests/simulation/test_flight_sequence.py` | N회 반복 수락 테스트, `metrics.csv`, `trajectories/*.csv` |
| `tests/simulation/test_failsafe.py` | 호버 중 하트비트 중단 → failsafe 전환 확인 |
| `scripts/run_phase1_test.sh` | SITL headless 기동 → 테스트 → 그래프 → 종료 (원클릭) |
| `tools/plot_flight.py` | `summary.png`, `runNN.png` |
| `ros2_ws/src/px4_bridge/test/test_frames.py` | 프레임 변환 단위 테스트 (9 passed) |

## 실행
```bash
source scripts/env.sh
ros2 launch drone_bringup sitl.launch.py [vehicle:=gz_x500_depth] [headless:=true]
ros2 run px4_bridge flight_demo                              # 1회
scripts/run_phase1_test.sh 10                                # 수락 테스트 (SITL 자동 기동)
python3 tests/simulation/test_failsafe.py                    # failsafe
```

## 통과 조건 (§11) 과 결과
- 10회 연속 시퀀스 **10/10 PASS** (회당 29~36 s, goto 최종 오차 0.06~0.14 m, hover 최대 이탈 ≤ 0.16 m).
- Failsafe: 스트림 중단 2.36 s 후 OFFBOARD 이탈 → AUTO_RTL → 착륙·disarm. PASS.
- `gz_x500_depth` 1회 PASS (GUI, 스크린샷 2장).
- 그래프: `summary.png`(10회 궤적), `run05.png`(x/y/z vs setpoint).

## 설계 결정
- **제어 경로 = ROS 2 + px4_msgs(uXRCE-DDS)**, MAVSDK 미사용. Phase 2의 px4_bridge 노드로 그대로 이어진다.
- Offboard 진입 순서는 PX4 공식 예제와 동일: 스트림 ≥ 10 msg → `DO_SET_MODE(1,6)` → `ARM`.
- 도착 판정: 위치 오차 < 0.20 m 이고 속도 < 0.20 m/s 가 0.5 s 지속.
- `land()`는 `NAV_LAND` 명령으로 AUTO_LAND 전환 후 스트림을 끊는다(모드가 latch되므로 안전).
- 최소 안전 범위(상대 고도 0.5~5 m, 수평 15 m)는 Phase 8 Safety Validator가 대체한다.
- SITL 전용 파라미터: `NAV_DLL_ACT=0`(GCS 없이 arming), `COM_OF_LOSS_T=3`(WSL2 시계 역행 내성).
  `PX4_PARAM_*` 환경변수는 기체 파일의 `set-default`에 덮이므로 부팅 후 `px4-param set`으로 적용한다.

## 알려진 문제 / 교훈
- 백그라운드 `ros2 launch`는 `set -m` 하에 띄우고 `kill -INT -- -<pgid>`로 끝내야 PX4·Gazebo까지 정리된다.
- `colcon build --symlink-install`도 ament_python의 launch/모듈은 복사한다. 수정 후 반드시 재빌드(러너가 자동 수행).
- 측정된 소요시간은 wall clock 기준이라 시계 역행 시 왜곡될 수 있다(run 1, 7).
