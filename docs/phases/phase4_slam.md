# Phase 4 — SLAM (RTAB-Map)

**완료일** 2026-10-14 · **증거** `experiments/004_phase4_slam/px4_v3/` (통과 run), 비교 run `vo_v2`, `vo_guess_v2`, `px4_v2`

## 목적
계획서 §14–15. detector와 결합하기 전에 SLAM을 독립적으로 완성한다. V1은 RGB-D(컬러+깊이+CameraInfo) → RTAB-Map →
odometry + 3D 지도. Gazebo ground truth와 비교해 ATE/RPE 평가 파이프라인을 만든다.

## 사용자 결정
- odometry 입력은 **둘 다 구현**: `odom_source:=vo`(RTAB-Map rgbd_odometry) / `vo_guess`(VO + PX4 EKF 모션 prior) / `px4`(PX4 EKF TF).
  사용자는 vo를 기본으로 택했으나 평가 결과(아래)에 따라 **기본값을 px4로 변경**했다(vo/vo_guess는 실험용으로 유지).
- 평가 경로: Room A → 복도 → Room B → 복귀, 2바퀴, 1.2 m, 진행 방향으로 yaw.
- 지도 출력: 2D occupancy(0.05 m) `/slam/map` + 3D point cloud `/slam/pointcloud`; `rtabmap.db`는 실험 폴더에 보관.

## 구현 파일

| 파일 | 내용 |
|---|---|
| `drone_slam/launch/rtabmap.launch.py` | `odom_source` vo/px4, `rtabmap_slam/rtabmap`(map→odom, `--delete_db_on_start`), `gt:=true`면 Gazebo `/world/<w>/dynamic_pose/info`를 `tf2_msgs/TFMessage`로 브리지 + `gt_pose_node`, `record_dir`면 `trajectory_recorder` |
| `drone_slam/config/rtabmap.yaml` | F2M odometry, `Odom/ResetCountdown=1`, 2D grid 0.05 m (`Grid/FromDepth`, RayTracing), 탐지율 2 Hz |
| `drone_slam/drone_slam/gt_pose_node.py` | `x500_rgbd_0` 모델 pose → `/ground_truth/pose`(frame `map_gt`), **평가 전용** |
| `drone_slam/drone_slam/trajectory_recorder.py` | TF map→base_link(est), odom→base_link(odom), `/drone/odom`(px4), `/ground_truth/pose`(gt) → CSV; `/slam/odom_info.lost`로 tracking lost 횟수, `/slam/info.loop_closure_id`로 loop closure 횟수 → `stats.json` |
| `drone_slam/drone_slam/slam_eval.py` | ATE(RMSE/mean/max, Umeyama 정렬 옵션), RPE(1 s) |
| `tests/simulation/test_phase4_slam.py` | 13 waypoint × N바퀴 비행(FlightCommand, yaw=진행방향) → `/slam/map` PNG, ATE/RPE, `report.json`, `trajectory.png` |
| `scripts/run_phase4_slam.sh` | headless 시뮬레이션(`px4_tf:=false` for vo, 속도 제한 `MPC_XY_VEL_MAX=1.0`) + 센서 + rtabmap + 테스트 |
| `drone_bringup/launch/simulation.launch.py` | `px4_tf` 인자 (px4_bridge `publish_tf`) |
| `drone_bringup/launch/full_simulation.launch.py` | `odom_source` 인자, vo면 `px4_tf:=false` 자동 |

## 실행
```bash
scripts/run_phase4_slam.sh vo 2          # 기본: 시각 odometry, 2바퀴
scripts/run_phase4_slam.sh px4 2         # 비교: PX4 EKF odometry
python3 tests/simulation/test_phase4_slam.py --run-dir experiments/004_phase4_slam/<run> --skip-flight   # 재평가만
# 수동
ros2 launch drone_bringup simulation.launch.py vehicle:=gz_x500_rgbd world:=indoor_v1 use_sim_time:=true px4_tf:=false
ros2 launch sensor_adapter gazebo_sensors.launch.py
ros2 launch drone_slam rtabmap.launch.py odom_source:=vo record_dir:=/tmp/run1
```

## 통과 조건(§15 engineering gate)과 결과

게이트: tracking lost = 0, 3분 이상 연속 mapping, loop closure 동작, 궤적이 GT와 시각적으로 일치, ATE 파이프라인 구축.
수락 테스트 기준(`test_phase4_slam.py`): 비행 성공 ∧ ATE(est vs GT) ≤ 0.5 m ∧ lost = 0.

### 결과 표 (indoor_v1, 2바퀴 ≈ 190 s, 1.2 m, headless, sim time)

| run | odometry | 벽 텍스처 | lost | loop closures | ATE est [m] (max) | RPE 1 s [m] | ATE odom [m] | 지도 free/occ 셀 | 판정 |
|---|---|---|---|---|---|---|---|---|---|
| vo_run2 | VO | 단색 | 54 | 22 | 7.93 (19.3) | 1.98 | 8.00 | – (Grid 파라미터 오류) | FAIL |
| vo_v2 | VO | 고유 포스터 | 67 | 2 | 8.38 (18.6) | 1.17 | 9.57 | 12.8k / – | FAIL |
| vo_guess_v2 | VO + PX4 prior | 고유 포스터 | 88 | 18 | 0.79 (2.03) | 0.19 | 1.56 | 13.7k / – | FAIL |
| px4_v2 | PX4 EKF | 고유 포스터 | 0 | 55 | 0.68 (1.28) | 0.10 | 0.07 | 20.6k / 4.6k | FAIL (잘못된 loop closure) |
| **px4_v3** | PX4 EKF (EKF 신뢰 설정) | 고유 포스터 | **0** | 6 | **0.075 (0.22)** | **0.042** | 0.069 | 21.5k / 4.8k | **PASS** |

PX4 EKF 궤적 자체의 ATE는 모든 run에서 0.06~0.08 m로, GT 브리지·시간 동기·평가 코드가 올바름을 보여준다(sanity check).
`px4_v3/`: `trajectory.png`(GT·SLAM·EKF 궤적 일치), `map.png`(두 방 + 복도 + 가구 장애물), `report.json`, `stats.json`, `est/odom/px4/gt.csv`,
`rviz_mapping_*.png`(3D point cloud + occupancy grid), `rtabmap.db`(158 MB, git 제외).

### 해석
- **순수 RGB-D 시각 odometry는 이 환경에서 불안정**: 문 통과(깊이 불연속)와 선회에서 F2M 추적이 끊기고(`Odom/ResetCountdown`으로 재시작 → 큰 점프),
  텍스처를 추가해도 손실 횟수는 줄지 않았다. 실내 실기체라면 IMU 융합(VIO)이나 PX4 EKF와의 결합이 필요하다는 뜻이다.
- **PX4 prior(vo_guess)** 는 ATE를 8 m → 0.8 m로 줄이지만 손실 이벤트는 그대로다.
- **perceptual aliasing**: 같은 텍스처를 모든 벽에 쓰자 Room A/B가 서로 매칭되어 55개의 loop closure 중 다수가 틀렸고 ATE가 0.68 m로 악화됐다.
  벽마다 고유 텍스처 + `Rtabmap/LoopThr 0.15`, `Vis/MinInliers 25`, `RGBD/OptimizeMaxError 0.3`, EKF 분산 1e-4, `NeighborLinkRefining=false`로
  loop closure가 6개로 줄고 ATE 0.075 m.

## 설계 결정
- 시각 odometry 모드에서는 rgbd_odometry가 odom→base_link TF를 소유하고 px4_bridge는 TF를 끈다(`/drone/odom` 토픽은 계속 발행 → 비교용).
- GT는 `map_gt` 프레임으로만 발행하고 TF 트리에 붙이지 않는다(계획서 §2.3).
- `Odom/ResetCountdown=1`: tracking lost 시 1프레임 뒤 자동 재초기화(이벤트는 통계로 집계). 게이트는 lost=0.
- 오프보드 속도를 1 m/s로 제한해 시각 odometry에 유리한 완만한 비행을 만든다(SITL 전용 파라미터).

## 알려진 문제 / 교훈
- ros_gz_bridge의 `gz.msgs.Pose_V → tf2_msgs/TFMessage` 브리지는 생성은 되지만 메시지가 오지 않았다. `gt_pose_node`는 `gz topic -e`를 서브프로세스로 읽어 파싱한다.
- Fuel `<include>` 모델은 기본이 동적이라 사람 모델이 바닥을 뚫고 떨어졌다(z = −8 m). 모든 include에 `<static>true</static>`.
- RTAB-Map 0.23: `Grid/FromDepth` → `Grid/Sensor`(1 = depth); occupancy grid 토픽은 `map`(remap → `/slam/map`), `grid_map`이 아니다.
- `queue_size` → `sync_queue_size` 로 이름이 바뀌었다(경고만).
- 기록 노드는 통계를 5 s마다 flush 해야 테스트가 종료 전에 읽을 수 있다.
- 순수 VO 게이트(lost = 0)는 미달이며 Phase R3(Pi)에서 VIO 또는 다른 VO(예: 스테레오, IMU 융합)를 검토한다.
