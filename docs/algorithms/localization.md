# 위치 추정 (AMCL · EKF)

> 상태: 초안 (2주차). 담당: 민지. 설정 항목과 설계 근거(공식 문서·선행연구)를 정리했고, 설정 파일 초안은 `amr_localization/config/`(`ekf.yaml`, `amcl.yaml`)에 있다. 튜닝값과 실측 근거는 실제 로봇(센서 플러그인 이후)으로 채운다. 인용은 [참고 문헌](#6-참고-문헌)의 번호다.
> 지도 생성(slam_toolbox)은 지수 담당이며 별도 문서로 다룬다. 분업은 [명세서 8-2절](../project-spec.md#8-2-매핑위치추정-분업-지수--민지)을 따른다.

## 1. 구성

| 노드 | 패키지 | 입력 | 출력 |
| --- | --- | --- | --- |
| AMCL | `nav2_amcl` | `map`, `scan`, TF | `amcl_pose` (PoseWithCovarianceStamped) |
| `ekf_local` | `robot_localization` | `odom`, `imu/data` | `odometry/local`, `odom→base_footprint` |
| `ekf_global` | `robot_localization` | `odom`, `imu/data`, `amcl_pose` | `odometry/filtered` (KPI 측정 대상), `map→odom` |

실행: `ros2 launch amr_localization localization.launch.py` (지도 서버, AMCL, EKF 두 개). 인자는 패키지 README 참고.

### TF 발행 구성 (D4, A안 확정)

`odom→base_footprint`와 `map→odom`은 각각 한 노드만 발행해야 한다. 원 명세서의 "EKF(Odom+IMU+AMCL)"를 만족하는 후보 두 가지를 비교해 A안으로 확정했다(#34, [api.md 2-1절](../api.md#2-1-tf-발행-주체-d4-a안-ekf-두-개)).

| 안 | `odom→base_footprint` | `map→odom` | 설정 |
| --- | --- | --- | --- |
| A. EKF 두 개 (확정) | `ekf_local` | `ekf_global` (AMCL 보정 포함) | 오도메트리 노드 `publish_tf: false`, AMCL `tf_broadcast: false` |
| B. EKF 한 개 + AMCL | EKF (odom+IMU) | AMCL | AMCL 결과가 EKF에 들어가지 않아 명세 요구와 어긋남 |

A안은 `odom` 프레임이 연속적(점프 없음)이라는 조건을 지키면서, AMCL 보정은 `map→odom`에만 반영한다. robot_localization 공식 문서가 권장하는 구성과 같다: `world_frame`을 `map`으로 둔 인스턴스는 `map→odom`을 발행하고, `odom→base_footprint`는 다른 노드(또 하나의 robot_localization 인스턴스여도 된다)가 발행하되 **그 인스턴스는 전역 데이터(AMCL 등)를 넣지 않는다** [2]. AMCL의 `tf_broadcast: false`는 AMCL이 `map→odom`을 발행하지 않게 하는 옵션이다 [4]. EKF의 `base_link_frame`은 URDF 구조(`base_footprint → base_link`)에 맞춰 `base_footprint`로 둔다.

## 2. EKF (`robot_localization`)

### 2-1. config 매트릭스

센서마다 15개 bool로 "어떤 상태 변수를 이 센서에서 받을지"를 정한다. 순서는 아래와 같다.

```
[x,     y,      z,
 roll,  pitch,  yaw,
 vx,    vy,     vz,
 vroll, vpitch, vyaw,
 ax,    ay,     az]
```

| 입력 | 켤 항목 (초안) | 이유 |
| --- | --- | --- |
| `odom0` (휠 오도메트리) | `vx`, `vy`, `vyaw` | 휠 오도메트리는 누적 위치보다 속도가 믿을 만하다. 차동구동은 옆으로 못 움직이므로 `vy=0`을 측정으로 넣어 제약한다 |
| `imu0` | `vyaw` (필요 시 `ax`) | 자이로 각속도가 회전 추정을 보강한다. 절대 yaw는 자기장 센서가 없으면 드리프트하므로 넣지 않는다 |
| `pose0` (AMCL, `ekf_global`만) | `x`, `y`, `yaw` | 지도 기준 절대 위치로 누적 오차를 잡는다 |

- 위 `odom0`·`imu0` 설정은 Nav2 공식 튜토리얼의 설정과 같다 [3]. robot_localization 문서도 오도메트리가 위치와 속도를 모두 주면 속도를 쓰라고 권한다 [2].
- 같은 상태 변수를 두 입력이 절대값으로 동시에 주면 EKF가 과신한다(명세서 8-2절 책임 판별표의 "EKF가 AMCL 보정을 무시"). 절대값은 한 입력에만 준다.
- 공통 설정: `two_d_mode: true`(평면 주행), `frequency: 50.0`(KPI: 50Hz 이상, 패키지 기본값은 30Hz [3]), `use_sim_time: true`.

### 2-2. 공분산 설정 근거

원 명세서는 "센서별 공분산 설정 근거"를 설명할 수 있기를 요구한다. 실제로 robot_localization 사용자 대부분이 공분산을 감·시행착오·기본값 재사용으로 정하고, 이것이 드리프트나 필터 발산의 원인이 된다는 지적이 있다 [9]. 그래서 값마다 출처를 남긴다.

| 항목 | 값의 출처 |
| --- | --- |
| 센서 측정 공분산 (R) | 지수의 센서 노이즈 모델(LiDAR σ=0.03m, IMU 바이어스·노이즈)과 드리프트 실험 결과. 오도메트리·IMU 메시지의 공분산 필드로 들어온다 |
| 공정 노이즈 (Q, `process_noise_covariance`) | 실측 튜닝. 변경마다 RMSE와 NEES(필터가 자기 오차를 제대로 알고 있는지)를 기록한다 [10] |

robot_localization 문서의 규칙을 따른다 [2].

- 어떤 값을 무시하려고 공분산을 크게 부풀리지 않는다. config 매트릭스에서 false로 끈다.
- 공분산을 0으로 두지 않는다. 0이면 내부에서 아주 작은 값으로 바뀌어 성능이 나빠진다.
- Q가 센서 분산에 비해 클수록 필터가 측정값을 빨리 따라간다. 튜닝 방향을 정할 때 쓴다.

#### 실측 (10/6)

우리 로봇 전체 스택에서 입력마다 "보고한 분산"과 "정답 대비 실제 오차의 분산"을 비교했다. 주행은 3-1절과 같고, 엔코더 seed 42·7, AMCL은 3절 튜닝값, 지도 v0이다. EKF 출력은 NEES(오차ᵀ·P⁻¹·오차, x·y 2차원)로 본다. 필터가 자기 오차를 제대로 알면 평균이 2다 [10].

| 입력·출력 | 실제 오차 σ | 보고 σ (중앙값) | 실제 분산 / 보고 분산 | 판단 |
| --- | --- | --- | --- | --- |
| 휠 오도메트리 `v` | 0.021 m/s | 0.018 m/s | 1.3~1.4 | 맞음 |
| 휠 오도메트리 `ω` | 0.118 rad/s | 0.088 rad/s | 1.8 | 약간 과신 |
| IMU `ω` (`imu/data`) | 0.0012 rad/s | 0.0002 rad/s | 약 37 | 과신. 공분산이 센서 노이즈 σ만 쓰고 저역통과 필터 지연을 넣지 않았다 |
| AMCL `amcl_pose` 위치 | x 2.7 cm, y 1.1 cm | x 7.6 cm, y 3.3 cm | NEES 0.9 | 보수적 |
| `ekf_global` 위치 (Q 기본) | x 3.0~3.6 cm, y 1.2~1.3 cm | x 12.5 cm, y 11 cm | NEES 0.2 | 매우 보수적 |

- 정확도 오차는 대부분 지도 해상도 편향(y −2.5 cm, 3-1절)이라 분산으로는 드러나지 않는다. 위 σ는 편향을 뺀 흔들림이다.
- IMU `ω`는 과신하지만 실제 오차가 0.0012 rad/s로 작아 방향 정확도에는 영향이 작다(`ekf_local` 방향 오차 σ 0.003 rad). 공분산은 지수의 `imu_filter`가 채우므로 필터 지연분을 더하는 것을 지수와 정한다.

**공정 노이즈 Q**: 기본값에서 `ekf_global`의 NEES가 0.2라 공분산이 실제 오차보다 σ 기준 3~4배 크다. 이대로면 "공분산이 커지면 위치를 잃은 것"이라는 Kidnapped 감지 기준(4절)을 EKF 공분산으로 세울 수 없다. Q를 두 EKF 모두 같은 비율로 줄여 봤다.

| Q | NEES 평균 (seed 42 / 7) | 정지·직진·회전 RMSE cm (seed 42) | 같음 (seed 7) |
| --- | --- | --- | --- |
| 기본 (×1) | 0.2 / 0.2 | 4.2 · 3.9 · 3.8 | 6.0 · 5.5 · 4.7 |
| ×0.1 | 0.8 / 0.7 | 6.9 · 6.2 · 3.8 | 4.6 · 4.0 · 3.9 |
| ×0.01 | 4.5 / — | 9.0 · 7.8 · 5.6 | — |

- ×0.01은 NEES 4.5로 과신하고 정확도도 나빠진다(EKF가 휠 오도메트리를 믿고 AMCL 보정을 늦게 따라가 통로 방향 x 오차가 쌓인다).
- ×0.1은 NEES가 기대값 2에 가장 가깝다. 정확도는 seed 42에서 나쁘고 seed 7에서 좋아 방향이 일정하지 않다.
- **같은 설정·같은 seed에서도 실행마다 RMSE가 최대 2 cm 다르다**(AMCL 파티클 무작위성. Q 기본, seed 7: 4.1 → 6.0 cm). Q 차이는 이 흔들림보다 작아서 실행 1~2회로는 고를 수 없다.
- 결정: 지금은 기본값을 `ekf.yaml`에 명시만 하고, 하네스 v1(N회 반복 + 신뢰구간)으로 ×1과 ×0.1을 비교해 정한다. KPI 판정도 단일 실행이 아니라 N회 반복의 신뢰구간으로 한다(명세서 1-4절 "하네스로 N회 반복 신뢰구간 제시").

### 2-3. 예측-업데이트 단계

TODO(검증 후, 학교 GPU 서버 확보 뒤): 예측(운동 모델로 상태·공분산 전파)과 업데이트(측정과의 차이를 칼만 이득으로 반영) 단계를 이 시스템의 입력 기준으로 설명한다.

## 3. AMCL (`nav2_amcl`)

| 파라미터 | 의미 | Nav2 기본값 | 튜닝값 | 근거 |
| --- | --- | --- | --- | --- |
| `min_particles` / `max_particles` | 파티클 수 범위. 불확실할수록 많이 쓴다(KLD-sampling [6]) | 500 / 2000 | TODO | TODO |
| `alpha1` | 회전 운동에서 생기는 회전 노이즈 | 0.2 | TODO | TODO |
| `alpha2` | 직진 운동에서 생기는 회전 노이즈 | 0.2 | TODO | TODO |
| `alpha3` | 직진 운동에서 생기는 직진 노이즈 | 0.2 | TODO | TODO |
| `alpha4` | 회전 운동에서 생기는 직진 노이즈 | 0.2 | TODO | TODO |
| `alpha5` | 옆 방향 노이즈 (옴니휠 전용, 차동구동은 미사용) | 0.2 | — | — |
| `laser_model_type` | 스캔-지도 비교 모델 | `likelihood_field` | TODO | TODO |
| `update_min_d` / `update_min_a` | 필터를 갱신할 최소 이동 거리(m)·회전(rad) | 0.25 / 0.2 | TODO | TODO |
| `resample_interval` | 몇 번 갱신마다 리샘플링할지 | 1 | TODO | TODO |
| `recovery_alpha_slow` / `recovery_alpha_fast` | 측정 가능도가 급락하면 무작위 파티클을 주입한다. 0이면 꺼짐 | 0.0 / 0.0 | TODO (권장 0.001 / 0.1 [4]) | Kidnapped 복구에 필수. 기본값은 꺼져 있다 |

- 회전 시에만 오차가 커지면 먼저 지수가 바퀴 간격 L을 확인하고, 그다음 `alpha1`·`alpha2`를 조정한다(명세서 8-2절).
- 위 기본값과 의미는 Nav2 Humble 소스 코드와 문서에서 확인했다 [4][5].
- 파티클 수는 상황에 따라 바뀐다. 불확실할 때 늘리고 확실해지면 줄이는 KLD-sampling 방식이다 [6]. AGV 대상 튜닝 실험에서는 alpha를 낮추면 파티클이 모이고 높이면 퍼졌으며, max 파티클을 늘려도 초기 위치 찾기가 좋아지지 않았다 [7].

## 4. Kidnapped Robot 복구

TODO(3주차). 설계 방향:

1. 감지: AMCL 공분산 급증, EKF와 AMCL 위치의 괴리, 스캔-지도 일치도 하락 중 하나 이상
2. 복구: AMCL `reinitialize_global_localization` 서비스로 전역 재초기화 (지도 전체에 파티클을 고르게 뿌린다 [5])
3. 수렴: 먼저 `request_nomotion_update` 서비스로 로봇을 움직이지 않고 필터 갱신을 반복한다 [5]. 그래도 공분산이 기준 이하로 모이지 않으면 제자리 회전으로 스캔을 모은다
4. 제자리 회전에는 `cmd_vel`이 필요하므로 사용 우선순위를 이진(제어)·지수(BT)와 합의한다. 3단계에서 회전 없이 끝나는 경우가 많을수록 합의할 부분이 줄어든다

주의: AMCL은 비슷한 구조가 반복되는 환경에서 잘못된 위치로 수렴하기 쉽다 [8]. 60×40m 물류센터의 똑같은 선반 통로가 이 경우라서, 복구 테스트에 "다른 통로로 수렴하는지"를 넣는다.

## 5. 정확도 측정 (RMSE)

- 산출: Ground Truth − EKF 추정의 RMSE와 Max Error, 시나리오당 100개 시점 이상
- KPI: 정지 3cm · 직선 5cm · 회전 8cm
- 로그 포맷: [tools/harness/README.md](../../tools/harness/README.md)의 `localization.csv`
- TODO(2주차): GT 토픽(지수, `amr_gazebo`)과 추정 토픽을 연결하는 측정 노드

## 6. 참고 문헌

1. T. Moore, D. Stouch, "A Generalized Extended Kalman Filter Implementation for the Robot Operating System," IAS-13, 2014. — robot_localization 원 논문
2. robot_localization 문서 — [state_estimation_nodes](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/doc/state_estimation_nodes.rst), [preparing_sensor_data](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/doc/preparing_sensor_data.rst)
3. Nav2 문서, [Smoothing Odometry using Robot Localization](https://github.com/ros-navigation/docs.nav2.org/blob/rolling/docs/configuration_and_development/first_time_robot_setup_guide/odom/setup_robot_localization.md)
4. Nav2 문서, [Configuring AMCL](https://github.com/ros-navigation/docs.nav2.org/blob/rolling/docs/configuration_and_development/configuration_guide/others/configuring_amcl.md)
5. Nav2 Humble 소스, [nav2_amcl/src/amcl_node.cpp](https://github.com/ros-navigation/navigation2/blob/humble/nav2_amcl/src/amcl_node.cpp) — 파라미터 기본값, 서비스
6. D. Fox, "Adapting the Sample Size in Particle Filters Through KLD-Sampling," IJRR, 2003.
7. "An Extended Analysis on Tuning the Parameters of Adaptive Monte Carlo Localization ROS Package in an Automated Guided Vehicle" — AMCL 파라미터 22개 중 17개의 영향 분석
8. "An improved Localization of Mobile Robotic System Based on AMCL Algorithm" — 반복 구조 환경에서의 kidnapping 문제
9. B. F. Abaza, "AI-Driven Dynamic Covariance for ROS 2 Mobile Robot Localization," Sensors, 2025.
10. Q·R 튜닝과 NEES 기반 일관성 평가 — 예: "Noise Covariance Identification Based Adaptive UKF with Application to Mobile Robot Systems"
