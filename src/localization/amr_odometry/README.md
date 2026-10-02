# amr_odometry

- 담당: 지수
- 빌드 타입: ament_python
- 책임: 차동 구동 순기구학, 휠 오도메트리, LiDAR/Depth/IMU 전처리 필터

## 제공 인터페이스

`<ns>` = 로봇 네임스페이스 (기본 `amr_01`). 토픽은 상대 이름이고 네임스페이스는 launch가 붙인다.

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Publish | `/<ns>/odom` | `nav_msgs/msg/Odometry` | 휠 오도메트리, 50 Hz. 위치·속도 공분산 채움. frame `<ns>/odom` → `<ns>/base_footprint` |
| Publish | `/tf` | `tf2_msgs/msg/TFMessage` | `<ns>/odom` → `<ns>/base_footprint`. **`publish_tf:=true`일 때만** (기본 꺼짐, D4) |
| Subscribe | `/<ns>/joint_states` | `sensor_msgs/msg/JointState` | 좌우 바퀴 각도 (Gazebo에서 브리지, 1 kHz) |

- 인계 계약(명세서 8-2절): `/odom`은 공분산을 채워 민지(EKF)에게 넘긴다. EKF는 `vx`, `vy`, `vyaw`를 쓴다 ([localization.md](../../../docs/algorithms/localization.md)).
- TF: D4(A안)에 따라 `odom → base_footprint`는 `ekf_local`이 발행한다. 이 노드는 EKF 없이 단독으로 시험할 때만 TF를 낸다.
- **주의**: 지금은 Gazebo DiffDrive도 `/<ns>/odom`을 내보낸다. 같이 띄우면 섞이므로 그 전까지는 `odom_topic:=wheel/odom`으로 띄운다. DiffDrive 쪽 이름 변경은 센서 PR에서 한다.

전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

## 구성

| 파일 | 내용 |
| --- | --- |
| `kinematics.py` | 순기구학, 중점법 적분, Siegwart 공분산 전파 (ROS 비의존) |
| `encoder_model.py` | 엔코더 측정 모델: 4096틱 양자화, 계통 오차, 거리 비례 무작위 오차 (ROS 비의존) |
| `wheel_odometry_node.py` | `joint_states` 구독 → `odom` 발행 |

### 엔코더 측정 모델

Gazebo에는 엔코더 센서가 없어 실제 바퀴 각도에 오차를 넣는다 (PR #26 민지 리뷰).

1. **양자화**: 각도를 2π/4096 rad 단위로 내림 (바퀴 둘레 기준 0.15 mm)
2. **계통 오차**: 좌우 바퀴 반지름 배율 (1 + N(0, 0.005))을 실행마다 한 번 뽑아 고정
3. **무작위 오차**: 바퀴마다 이동 거리 Δs에 N(0, k²·|Δs|) → 누적 σ = k·√D, 갱신 주기와 무관

공분산은 같은 가정(바퀴 이동 거리 분산 k²·|Δs|)으로 Siegwart 모델로 전파한다. 계통 오차는 평균이 0이 아니라 공분산에 넣지 않는다.

`seed`를 같게 주면 계통 오차(반지름 배율)는 똑같이 재현된다. 무작위 오차는 sim 타이밍에 따라 틱이 넘어가는 시점이 달라 실행마다 조금씩 다르다.

## 파라미터

`config/odometry.yaml`. 파라미터 변경에 재빌드가 필요 없다.

| 이름 | 기본값 | 설명 |
| --- | --- | --- |
| `wheel_radius`, `wheel_separation` | 0.1, 0.34 m | `amr_description/urdf/properties.xacro`와 같은 값 |
| `left_wheel_joint`, `right_wheel_joint` | `left_wheel_joint`, `right_wheel_joint` | 네임스페이스 접두어 없이. 끝부분으로 찾는다 |
| `odom_topic` | `odom` | 출력 토픽 |
| `publish_tf` | `false` | D4: EKF가 TF를 발행 |
| `publish_rate` | 50 Hz | Gazebo `joint_states`(1 kHz)는 매번 적분하고 발행만 묶는다 |
| `twist_variance_floor` | 1e-4 | 정지 중 속도 공분산 하한 |
| `encoder.ticks_per_rev` | 4096 | 명세 |
| `encoder.wheel_radius_scale_stddev` | 0.005 | 계통 오차 σ |
| `encoder.distance_noise_k` | 0.01 m^½ | 무작위 오차. 바퀴당 10 m에서 1σ ≈ 3.2 cm |
| `encoder.seed` | 0 | 0이면 실행마다 무작위 |
| `encoder.noise_enabled` | `true` | `false`면 양자화만 |

#### k 값 (0.032 → 0.01로 낮춘 이유)

`distance_noise_k`는 **바퀴마다** 적용된다. #26 초안의 "k = 0.032면 10 m에 1σ ≈ 1%"는 바퀴 하나의 이동 거리
기준이라 맞지만, 로봇 위치는 좌우 바퀴 오차의 **차이**가 방향 오차가 되고 그게 옆 방향 오차로 쌓인다.
바퀴 간격이 0.34 m로 좁아 이 증폭이 크다.

10 m 직진 기준 계산값 (b = 0.34 m):

| k [m^½] | 바퀴 하나 거리 σ | 로봇 전진 σ | 로봇 방향 σ | 로봇 옆 방향 σ |
| --- | --- | --- | --- | --- |
| 0.032 (#26 초안) | 10.1 cm (1.0%) | 7.2 cm | 24.1° | 2.43 m (24%) |
| **0.01 (현재)** | 3.2 cm | 2.2 cm | 7.5° | 0.76 m (7.6%) |
| 0.003 | 0.95 cm | 0.67 cm | 2.3° | 0.23 m (2.3%) |

- 바퀴 하나 거리 k·√D, 로봇 전진 k·√(D/2), 방향 √2·k·√D / b, 옆 방향 (k/b)·√(2D³/3)
- 옆 방향 오차는 방향 오차가 누적된 결과라 거리의 1.5제곱으로 커진다.
- 0.01은 확정값이 아니라 출발점이다. "1%"를 로봇 위치 기준으로 잡으면 k ≈ 0.002~0.003이 필요하다.
  기준과 최종 값은 드리프트 실험(10 m 사각형 복귀)에서 민지와 정한다.

아래는 sim과 같은 6 m 경로를 Gazebo 없이 seed 300개로 돌린 결과다 (중앙값 / 95% 값).

| k | 계통 σ | 위치 오차 [cm] | 방향 오차 [°] | 공분산 방향 σ [°] |
| --- | --- | --- | --- | --- |
| 0.032 | 0.5% | 46.7 / 96.4 | 13.3 / 41.4 | 21.7 |
| **0.01** | **0.5%** | **16.5 / 35.6** | **6.5 / 17.5** | **6.0** |
| 0 | 0.5% | 8.7 / 23.9 | 5.0 / 13.2 | 0 |
| 0.01 | 0 | 14.0 / 30.0 | 3.8 / 12.4 | 6.0 |
| 0.005 | 0.2% | 8.0 / 16.4 | 2.9 / 7.9 | 3.0 |

- 계통 오차를 빼면(계통 σ 0) 방향 오차 중앙값 3.8°가 공분산 σ 6°의 정규분포 중앙값(|N(0, 6°)| ≈ 4.0°)과 맞는다.
- k를 낮춘 뒤에는 계통 오차(0.5%)의 몫이 크다. 계통 오차는 공분산에 들어가지 않으므로 EKF가 방향을 과신할 수 있다.

## 검증 (sim, 월드 v0, 6 m 주행: 직진 4 m → 제자리 90° → 곡선 2 m)

기준: Gazebo DiffDrive odom (같은 바퀴 각도로 계산한 이상적 오도메트리)

| 조건 | 위치 차이 | 방향 차이 | 공분산 방향 σ |
| --- | --- | --- | --- |
| `encoder_noise:=false` (양자화만) | 0.01 cm | 0.00° | 0.6° (하한) |
| k 0.01, `encoder_seed:=42` | 25.5 cm | −15.7° | 5.9° |
| k 0.01, `encoder_seed:=7` | 21.7 cm | −11.2° | 6.0° |

노이즈 포함 결과는 위 seed 300개 분포의 95% 범위 안이다.

- 발행 50.0 Hz (sim 시간), frame `amr_01/odom` → `amr_01/base_footprint`
- `publish_tf:=false`면 `/tf` 발행 노드에 `wheel_odometry`가 없고, `true`면 있다

## 실행

```bash
# sim이 떠 있는 상태에서 (DiffDrive odom과 겹치지 않게 지금은 wheel/odom으로)
ros2 launch amr_odometry odometry.launch.py odom_topic:=wheel/odom

ros2 launch amr_odometry odometry.launch.py publish_tf:=true        # EKF 없이 단독 시험
ros2 launch amr_odometry odometry.launch.py encoder_seed:=42        # 계통 오차 재현
ros2 launch amr_odometry odometry.launch.py encoder_noise:=false    # 이상적 엔코더
```

## 테스트

```bash
colcon test --packages-select amr_odometry
```

`test_kinematics.py`(직진·제자리 회전·원호 닫힌 해, 공분산 대칭·증가·주기 무관), `test_encoder_model.py`(양자화, ±π 넘김, 계통 오차 재현·분포, 누적 σ = k·√D가 주기와 무관).
