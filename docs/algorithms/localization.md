# 위치 추정 (AMCL · EKF)

> 상태: 초안 (1주차). 담당: 민지. 설정 항목과 문서 구조만 잡았다. 튜닝값과 근거는 지도 인계(10/11) 뒤 실제 로봇으로 채운다.
> 지도 생성(slam_toolbox)은 지수 담당이며 별도 문서로 다룬다. 분업은 [명세서 8-2절](../project-spec.md#8-2-매핑위치추정-분업-지수--민지)을 따른다.

## 1. 구성

| 노드 | 패키지 | 입력 | 출력 |
| --- | --- | --- | --- |
| AMCL | `nav2_amcl` | `map`, `scan`, TF | `amcl_pose` (PoseWithCovarianceStamped) |
| `ekf_local` | `robot_localization` | `odom`, `imu/data` | `odom→base_link` 후보 |
| `ekf_global` | `robot_localization` | `odom`, `imu/data`, `amcl_pose` | `odometry/filtered`, `map→odom` 후보 |

### TF 발행 구성 (2주차에 지수와 확정)

`odom→base_link`와 `map→odom`은 각각 한 노드만 발행해야 한다. 원 명세서의 "EKF(Odom+IMU+AMCL)"를 만족하는 후보는 두 가지다.

| 안 | `odom→base_link` | `map→odom` | 설정 |
| --- | --- | --- | --- |
| A. EKF 두 개 (권장) | `ekf_local` | `ekf_global` (AMCL 보정 포함) | 오도메트리 노드 `publish_tf: false`, AMCL `tf_broadcast: false` |
| B. EKF 한 개 + AMCL | EKF (odom+IMU) | AMCL | AMCL 결과가 EKF에 들어가지 않아 명세 요구와 어긋남 |

A안은 `odom` 프레임이 연속적(점프 없음)이라는 조건을 지키면서, AMCL 보정은 `map→odom`에만 반영한다. 확정 결과는 [api.md](../api.md) TF 트리에 기록한다.

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

- 같은 상태 변수를 두 입력이 절대값으로 동시에 주면 EKF가 과신한다(명세서 8-2절 책임 판별표의 "EKF가 AMCL 보정을 무시"). 절대값은 한 입력에만 준다.
- 공통 설정: `two_d_mode: true`, `frequency: 50.0`(KPI: 50Hz 이상), `use_sim_time: true`.

### 2-2. 공분산 설정 근거

TODO(3주차): process noise와 센서별 공분산 값, 그리고 그 근거(지수의 센서 노이즈 모델·드리프트 실험 결과)를 적는다. 원 명세서는 "센서별 공분산 설정 근거"를 설명할 수 있기를 요구한다.

### 2-3. 예측-업데이트 단계

TODO(1주차 검증 후): 예측(운동 모델로 상태·공분산 전파)과 업데이트(측정과의 차이를 칼만 이득으로 반영) 단계를 이 시스템의 입력 기준으로 설명한다.

## 3. AMCL (`nav2_amcl`)

| 파라미터 | 의미 | Nav2 기본값 | 튜닝값 | 근거 |
| --- | --- | --- | --- | --- |
| `min_particles` / `max_particles` | 파티클 수 범위. 불확실할수록 많이 쓴다 | 500 / 2000 | TODO | TODO |
| `alpha1` | 회전 운동에서 생기는 회전 노이즈 | 0.2 | TODO | TODO |
| `alpha2` | 직진 운동에서 생기는 회전 노이즈 | 0.2 | TODO | TODO |
| `alpha3` | 직진 운동에서 생기는 직진 노이즈 | 0.2 | TODO | TODO |
| `alpha4` | 회전 운동에서 생기는 직진 노이즈 | 0.2 | TODO | TODO |
| `alpha5` | 옆 방향 노이즈 (옴니휠 전용, 차동구동은 미사용) | 0.2 | — | — |
| `laser_model_type` | 스캔-지도 비교 모델 | `likelihood_field` | TODO | TODO |
| `update_min_d` / `update_min_a` | 필터를 갱신할 최소 이동 거리(m)·회전(rad) | 0.25 / 0.2 | TODO | TODO |
| `resample_interval` | 몇 번 갱신마다 리샘플링할지 | 1 | TODO | TODO |
| `recovery_alpha_slow` / `recovery_alpha_fast` | 측정 가능도가 급락하면 무작위 파티클을 주입한다. 0이면 꺼짐 | 0.0 / 0.0 | TODO | Kidnapped 복구와 연결 |

- 회전 시에만 오차가 커지면 먼저 지수가 바퀴 간격 L을 확인하고, 그다음 `alpha1`·`alpha2`를 조정한다(명세서 8-2절).
- 위 기본값은 Nav2 Humble 문서 기준이며, 1주차 TurtleBot3 검증에서 실제 로드된 값으로 다시 확인한다.

## 4. Kidnapped Robot 복구

TODO(3주차). 설계 방향:

1. 감지: AMCL 공분산 급증, EKF와 AMCL 위치의 괴리, 스캔-지도 일치도 하락 중 하나 이상
2. 복구: AMCL `reinitialize_global_localization` 서비스로 전역 재초기화
3. 수렴: 제자리 회전으로 스캔을 모아 공분산이 기준 이하가 되면 종료
4. `cmd_vel` 사용 우선순위는 이진(제어)·지수(BT)와 합의한다

## 5. 정확도 측정 (RMSE)

- 산출: Ground Truth − EKF 추정의 RMSE와 Max Error, 시나리오당 100개 시점 이상
- KPI: 정지 3cm · 직선 5cm · 회전 8cm
- 로그 포맷: [tools/harness/README.md](../../tools/harness/README.md)의 `localization.csv`
- TODO(2주차): GT 토픽(지수, `amr_gazebo`)과 추정 토픽을 연결하는 측정 노드
