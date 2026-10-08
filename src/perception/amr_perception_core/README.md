# amr_perception_core

- 담당: 연우
- 빌드 타입: ament_python
- 책임: Pinhole 3D 변환, KF, TTC, VO/ORCA (ROS 비의존 라이브러리). 연구 2 알고리즘은 주제가 확정되면 이 패키지에 추가한다

## 공개 API

ROS 토픽을 직접 다루지 않는다. 공개 함수·클래스 목록을 여기에 적는다.

### 장애물 추적 (`amr_tracking` 노드가 사용)

| 모듈 | 이름 | 설명 |
| --- | --- | --- |
| `kalman` | `KalmanConfig(accel_noise_std, measurement_noise_std, initial_velocity_std)` | KF 잡음 파라미터 |
| `kalman` | `ConstantVelocityKF(position, timestamp, config)` | 등속 모델 KF. 상태 `[px, py, vx, vy]`, 위치 관측. `predict(t)`, `update(z, r=None)`, `mahalanobis_sq(z)`, `position`·`velocity`·`position_covariance`·`velocity_covariance`·`speed`·`heading` |
| `kalman` | `transition_matrix(dt)`, `process_noise(dt, σa)` | F, Q = σa² G Gᵀ (가속도 백색잡음 이산 모델) |
| `tracker` | `TrackerConfig(kalman, gate_mahalanobis_sq, confirm_hits, max_coast_time, dynamic_speed_threshold, dynamic_confirm_frames)` | 트랙 관리 파라미터 |
| `tracker` | `MultiTargetTracker(config)` | `step(t, measurements)` → 확정 트랙 목록. 예측 → 연관(확정 트랙 우선, 마할라노비스 게이트, greedy) → 갱신 → 생성 → 삭제. `reset()` |
| `tracker` | `Track` | `track_id`, `kf`, `hits`, `last_update`, `confirmed`, `is_dynamic` |

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

```bash
# TODO
```

## 테스트

```bash
colcon test --packages-select amr_perception_core
```
