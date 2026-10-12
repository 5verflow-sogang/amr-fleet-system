# amr_perception_core

- 담당: 연우
- 빌드 타입: ament_python
- 책임: Pinhole 3D 변환, LiDAR 군집화, KF 추적, 카메라 연관, TTC, 안전 판정, VO/ORCA (ROS 비의존 라이브러리)

## 공개 API

ROS 토픽을 직접 다루지 않는다. 공개 함수·클래스 목록을 여기에 적는다.

### 장애물 추적 (`amr_tracking` 노드가 사용)

| 모듈 | 이름 | 설명 |
| --- | --- | --- |
| `kalman` | `KalmanConfig(accel_noise_std, measurement_noise_std, initial_velocity_std)` | KF 잡음 파라미터 |
| `kalman` | `ConstantVelocityKF(position, timestamp, config)` | 등속 모델 KF. 상태 `[px, py, vx, vy]`, 위치 관측. `predict(t)`, `update(z, r=None)`, `mahalanobis_sq(z)`, `position`·`velocity`·`position_covariance`·`velocity_covariance`·`speed`·`heading` |
| `kalman` | `transition_matrix(dt)`, `process_noise(dt, σa)` | F, Q = σa² G Gᵀ (가속도 백색잡음 이산 모델) |
| `tracker` | `TrackerConfig(kalman, gate_mahalanobis_sq, confirm_hits, max_coast_time, dynamic_speed_threshold, dynamic_confirm_frames, dynamic_evidence_frames, radius_smoothing)` | 트랙 관리 파라미터 |
| `tracker` | `MultiTargetTracker(config)` | `step(t, measurements, radii=None, free_evidence=None)` → 확정 트랙 목록. 예측 → 연관(확정 트랙 우선, 마할라노비스 게이트, greedy) → 갱신 → 생성 → 삭제 → 동적 판정(속력 + 빈 공간 증거). `reset()` |
| `tracker` | `Track` | `track_id`, `kf`, `hits`, `last_update`, `confirmed`, `is_dynamic`, `radius`, `evidence_frames` |
| `clustering` | `ClusteringConfig`, `scan_to_points`, `cluster_scan`, `is_obstacle_candidate`, `merge_leg_pairs`, `detect_obstacles`, `transform_points_2d` | 스캔 군집화(adaptive breakpoint, 튄 점 하나 건너뛰기) → R1 크기 규칙 → 다리 묶기 |
| `free_space` | `FreeSpaceGrid(resolution, size, max_range, endpoint_margin, free_memory, hit_quiet, hit_clearance)` | 빈 공간 침입 증거(R4). `integrate(sensor_xy, directions, ranges, t)`, `evidence(points, t)` |
| `static_map` | `StaticMapMask(occupancy, resolution, origin, occupied_threshold, inflation)` | 정적 지도 대조(R2). `is_static(points)`, `best_offset(points, max_shift, step)` |
| `association` | `AssociationConfig`, `associate(tracks, detections, t, config)`, `ClassLabeler` | 카메라 검출 ↔ 트랙 연관, 트랙별 클래스 근거 누적(히스테리시스). `FixedDetection`, `TrackSnapshot`, `TrackLabel` |

### 충돌 위험·안전 (`amr_tracking`, `amr_safety` 노드가 사용)

| 모듈 | 이름 | 설명 |
| --- | --- | --- |
| `collision` | `footprint_clearance(points, length, width)` | 차체 사각형에서 점들까지 거리 |
| `collision` | `footprint_time_to_collision(rel_pos, rel_vel, radius, length, width, horizon, dt)` | 장애물 원이 차체 사각형에 닿기까지 TTC. 겹쳐 있으면 다가올 때만 0, 닿지 않으면 inf |
| `collision` | `path_hazard(points, v, w, length, width, front_margin, side_margin, horizon)` | 명령대로 움직일 때 진행 경로에 걸리는 점 (긴급정지) |
| `collision` | `time_to_collision(rel_pos, rel_vel, radius)`, `circumscribed_radius` | 두 원의 TTC 닫힌 식 |
| `safety` | `SafetyConfig`, `SafetyMonitor`, `ObstacleInfo`, `SafetyState` | 안전 단계 판정(ESTOP·CRITICAL·WARNING·NORMAL, 히스테리시스, E-Stop 래치, 센서 끊김). `update(...)`, `set_estop(bool)` |

- 측정은 고정 좌표계(odom 또는 map) 기준 위치여야 한다.
- 단위 테스트에 NEES(정규화 추정 오차 제곱) 일관성 검사가 있다: 필터와 같은 잡음 모델로 만든 궤적에서 평균 NEES ≈ 4(상태 차원).

### 객체 인식 3D 변환 (`amr_detection` 노드가 사용)

| 모듈 | 이름 | 설명 |
| --- | --- | --- |
| `projection` | `backproject_with_covariance(camera, u, v, z, sigma_px, sigma_z)` | 픽셀·깊이 → optical frame 위치와 3x3 공분산. Σ = J·diag(σ_px², σ_px², σ_z²)·Jᵀ. 옆 방향 σ ≈ σ_px·Z/f라 멀수록 커진다 |
| `projection` | `quaternion_to_matrix(x, y, z, w)` | TF 회전 → 3x3 행렬 (정규화 포함) |
| `projection` | `transform_with_covariance(R, t, p, Σ)` | p' = Rp + t, Σ' = RΣRᵀ (카메라 → map) |

## 파라미터

ROS 비의존 라이브러리이므로 파라미터는 함수/생성자 인자로 받는다. YAML 로딩은 이 라이브러리를 쓰는 노드 패키지가 담당한다.

## 실행

노드가 없는 라이브러리라 따로 실행하지 않는다. `amr_detection`(Pinhole 3D 변환), `amr_tracking`(군집화·추적·카메라 연관·TTC),
`amr_safety`(안전 판정)가 import해서 쓴다. 노드 실행은 각 패키지 README를 본다.

rclpy·ROS 메시지를 import하지 않고 numpy만 쓴다. 그래서 ROS 없이 pytest로도 테스트를 돌릴 수 있다.

```bash
cd src/perception/amr_perception_core
python3 -m pytest test          # numpy, pytest 필요
```

## 테스트

```bash
colcon test --packages-select amr_perception_core
```
