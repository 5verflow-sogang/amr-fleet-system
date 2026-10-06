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
- 하네스 v1(#77)로 AMCL 튜닝값(#75)과 함께 10회씩 비교했다. 평균 [95% CI] (cm):

| Q | 정지 | 직진 | 회전 |
| --- | --- | --- | --- |
| ×1 (기본) | 4.2 [3.6~4.8] | 3.5 [3.2~3.9] | 4.0 [3.8~4.2] |
| ×0.1 | 4.7 [4.4~5.0] | 4.2 [3.9~4.6] | 3.9 [3.6~4.2] |

- **결정: 기본값(×1) 유지.** ×0.1은 직진이 나쁘고 CI가 겹치지 않는다. 정확도가 KPI라 NEES보다 우선한다.
- 그 대신 EKF 공분산은 실제보다 σ 기준 3~4배 크다는 점을 알고 쓴다. Kidnapped 감지(4절)는 EKF 공분산 대신 AMCL 공분산(NEES 0.9로 실제에 가깝다)과 EKF·AMCL 위치 괴리로 판단한다.
- KPI 판정은 단일 실행이 아니라 N회 반복의 신뢰구간으로 한다(명세서 1-4절 "하네스로 N회 반복 신뢰구간 제시").

### 2-3. 예측-업데이트 단계

TODO(검증 후, 학교 GPU 서버 확보 뒤): 예측(운동 모델로 상태·공분산 전파)과 업데이트(측정과의 차이를 칼만 이득으로 반영) 단계를 이 시스템의 입력 기준으로 설명한다.

## 3. AMCL (`nav2_amcl`)

| 파라미터 | 의미 | Nav2 기본값 | 튜닝값 | 근거 |
| --- | --- | --- | --- | --- |
| `min_particles` / `max_particles` | 파티클 수 범위. 불확실할수록 많이 쓴다(KLD-sampling [6]) | 500 / 2000 | 500 / 2000 | 유지. 바꿔 보지 않았다 |
| `alpha1` | 회전 운동에서 생기는 회전 노이즈 | 0.2 | 0.05 | 아래 "alpha" |
| `alpha2` | 직진 운동에서 생기는 회전 노이즈 | 0.2 | 0.05 | 같음 |
| `alpha3` | 직진 운동에서 생기는 직진 노이즈 | 0.2 | 0.05 | 같음 |
| `alpha4` | 회전 운동에서 생기는 직진 노이즈 | 0.2 | 0.05 | 같음 |
| `alpha5` | 옆 방향 노이즈 (옴니휠 전용, 차동구동은 미사용) | 0.2 | — | — |
| `laser_model_type` | 스캔-지도 비교 모델 | `likelihood_field` | `likelihood_field` | 유지 |
| `sigma_hit` | 스캔 끝점과 가장 가까운 장애물 사이 거리 오차의 σ | 0.2 | 0.05 | 아래 "sigma_hit" |
| `z_hit` / `z_rand` | 측정 모델 혼합 비율 (맞는 점 / 무작위 점) | 0.5 / 0.5 | 0.9 / 0.1 | 정적 환경. 동적 장애물 환경에서 다시 확인 |
| `max_beams` | 갱신에 쓰는 빔 수 (720빔 중) | 60 | 180 | 2° 간격. CPU 사용률은 아직 재지 않았다 |
| `update_min_d` / `update_min_a` | 필터를 갱신할 최소 이동 거리(m)·회전(rad) | 0.25 / 0.2 | 0.1 / 0.1 | 보정 빈도 약 2배 (같은 주행에서 `amcl_pose` 59 → 113건) |
| `resample_interval` | 몇 번 갱신마다 리샘플링할지 | 1 | 1 | 유지 |
| `recovery_alpha_slow` / `recovery_alpha_fast` | 측정 가능도가 급락하면 무작위 파티클을 주입한다. 0이면 꺼짐 | 0.0 / 0.0 | TODO (권장 0.001 / 0.1 [4]) | Kidnapped 복구에 필수. 기본값은 꺼져 있다 |

- 회전 시에만 오차가 커지면 먼저 지수가 바퀴 간격 L을 확인하고, 그다음 `alpha1`·`alpha2`를 조정한다(명세서 8-2절).
- 위 기본값과 의미는 Nav2 Humble 소스 코드와 문서에서 확인했다 [4][5].
- 파티클 수는 상황에 따라 바뀐다. 불확실할 때 늘리고 확실해지면 줄이는 KLD-sampling 방식이다 [6]. AGV 대상 튜닝 실험에서는 alpha를 낮추면 파티클이 모이고 높이면 퍼졌으며, max 파티클을 늘려도 초기 위치 찾기가 좋아지지 않았다 [7].

**alpha**: Nav2 차동구동 모델에서 alpha는 분산 비율이라 노이즈 표준편차는 이동량의 √α배다 [5]. 엔코더 모델(#35: 바퀴당 Δs에 N(0, 0.01²·|Δs|), 반지름 배율 σ 0.5%)로 계산하면 0.1 m 이동당 직진 표준편차는 약 3%다. 기본값 0.2(약 45%)는 이보다 15배 커서 파티클이 필요 이상으로 퍼진다. 0.05(약 22%)는 여전히 7배 여유가 있다. AMCL이 받는 `odom→base_footprint`는 `ekf_local`(휠 + IMU 자이로) 출력이라 방향 오차는 휠만 쓸 때보다 더 작다.

**sigma_hit**: 끝점 거리 오차는 LiDAR 노이즈 σ 0.03 m, 지도 칸 양자화 0.05/√12 ≈ 0.014 m, 지도 v0의 물체 위치 오차(중앙값 0.025 m, amr_mapping README)를 합쳐 약 0.04 m다. 기본값 0.2 m는 이보다 5배 커서 스캔이 위치를 거의 바로잡지 못했다.

### 3-1. 튜닝 실험 (10/6)

우리 로봇 전체 스택(시뮬레이션 → 휠 오도메트리·센서 필터 → `ekf_local`·AMCL·`ekf_global`)에서 같은 주행을 반복했다. 주행: 정지 → 직진 3 m → 제자리 한 바퀴 → 반 바퀴 → 돌아오는 직진 3 m → 정지 (0.5 m/s, 0.5 rad/s, RTF 1.0). 측정은 `eval_node`로 `odometry/filtered`(EKF 전역)와 정답을 비교했다. 엔코더 seed 42·7·123 세 번.

| EKF 전역 RMSE (cm), seed 42 / 7 / 123 | 정지 (3) | 직진 (5) | 회전 (8) |
| --- | --- | --- | --- |
| 지도 v0, 기본값 | 5.5 / 4.7 / 5.5 | 5.0 / 4.2 / 4.9 | 7.1 / 7.5 / 6.6 |
| 지도 v0, 튜닝값 | 4.2 / 4.1 / 3.3 | 3.9 / 4.1 / 3.6 | 3.8 / 3.5 / 3.6 |
| 월드 기준 이상 지도 0.025 m, 튜닝값 | 2.1 / 2.2 / 3.1 | 2.1 / 3.2 / 2.7 | 2.4 / 2.1 / 2.5 |

같은 설정·seed에서도 실행마다 최대 2 cm 흔들려서(AMCL 파티클 무작위성), 하네스 v1(#77)로 조건마다 10회 반복해 다시 쟀다. 이 표는 처음 움직인 뒤부터 센 값이라(첫 움직임 전 오차 0 구간 제외) 위 표보다 정지 값이 크다.

| 하네스 `loc_basic` 10회, 평균 [95% CI] (cm) | 정지 (3) | 직진 (5) | 회전 (8) |
| --- | --- | --- | --- |
| 지도 v0, 기본값 | 7.2 [6.3~8.4] | 5.4 [4.7~6.4] | 7.8 [7.2~8.4] |
| **지도 v0, 튜닝값** | **4.2 [3.6~4.8]** | **3.5 [3.2~3.9]** | **4.0 [3.8~4.2]** |
| 이상 지도 0.025 m, 튜닝값 | 2.9 [2.4~3.5] | 2.4 [2.0~2.8] | 2.8 [2.6~3.0] |

**100개 시점 정확도 테스트 (작업 6)**: 위 "지도 v0, 튜닝값" 10회의 실행마다 구간별 시점 수와 Max다(KPI는 RMSE로 판정, Max는 보고만).

| 구간 | 시점 수 (실행당) | RMSE 평균 (cm) | Max 평균 (범위) (cm) |
| --- | --- | --- | --- |
| 정지 | 496~499 | 4.2 | 4.9 (3.7~8.4) |
| 직진 | 647~649 | 3.5 | 6.0 (4.4~8.8) |
| 회전 | 954~956 | 4.0 | 5.4 (4.8~5.9) |

세 구간 모두 100개 이상이다. 원자료(`localization.csv`, `localization_summary.json`, 궤적 TUM)는 하네스 실행 폴더(`tools/harness/results/loc_basic-amcl_tuned_*/eval_ekf/`)에 있다. 지도 v1을 받으면 작업 9에서 seed 1~N으로 다시 잰다.

- 기본값에서 AMCL은 직진 중 방향이 −0.03 → −0.06 rad로 틀어졌다. 같은 시간 `ekf_local`의 방향 오차는 0.005 rad 수준이라, 오도메트리가 아니라 스캔 보정이 약한 것이 원인이었다.
- 튜닝값은 직진·회전 오차를 줄였지만(회전은 절반), 지도 v0에서 **정지는 여전히 3 cm를 넘는다.** 남는 오차는 y 방향 약 −2.5 cm의 일정한 편향이다.
- 이 편향은 지도 칸(0.05 m)의 절반 크기다. 오도메트리를 오차 없는 값(DiffDrive)으로 바꿔도 남고(지도 v0 정지 3.1 cm, 월드 기준 이상 지도 0.05 m에서 3.5 cm), 이상 지도를 0.025 m로 만들면 −1.0 cm로 줄어 세 구간 모두 기준 안에 든다. 지도 해상도가 정지 KPI의 한계로 보인다.
- 지도 v0 자체는 문제가 아니다. 정답 위치에서 찍은 스캔(빔 600여 개)은 지도 v0과 이상 지도 모두에 거의 전부 한 칸(5 cm) 안으로 겹쳤다. LiDAR 장착 위치·TF·`map_origin_in_world`도 맞다.
- 이상 지도는 월드 SDF의 충돌 박스를 LiDAR 높이에서 잘라 만들었다. 장애물 안쪽까지 채우면 AMCL likelihood field에서 끝점이 장애물 안으로 들어가도 감점이 없어 위치가 장애물 쪽으로 밀린다(같은 조건 14 cm). SLAM 지도처럼 표면 테두리만 남겨야 한다.
- 측정 주의: 주행 전 대기 구간(모든 추정기 오차 0)은 정지 RMSE를 낮추므로 표는 첫 움직임 5초 전부터 계산했다. 하네스 v1에 반영한다.

남은 것: 지도 v1 해상도(지수와 인계 조건 논의), 동적 장애물 환경에서 `z_hit`·`z_rand` 재확인, `max_beams` 180의 CPU 사용률, 100개 시점 테스트.

## 4. Kidnapped Robot 복구

> 상태: 착수 (10/6). 감지는 동작한다. 가까운 이동(약 2 m)은 국소 재초기화로 복구했고, 먼 이동(다른 통로)은 아직 못 찾는다. 완성(10/11)은 아래 "다음 단계".

노드: `ros2 run amr_localization kidnap_recovery --ros-args -r __ns:=/amr_01 -p use_sim_time:=true` (`kidnap_recovery_node.py`, 로직은 `kidnap/`).

### 4-1. 감지: 스캔-지도 일치율

AMCL은 위치를 잃어도 스스로 알리지 않는다. 그래서 EKF 전역 자세(`odometry/filtered`)에서 스캔 끝점을 지도에 찍고, 점유 셀과 0.1 m 이내에 떨어지는 끝점의 비율(일치율)을 2 Hz로 잰다. 일치율이 `lost_ratio`(0.5) 아래로 3번 연속(1.5초) 떨어지면 위치를 잃은 것으로 본다.

- 근거: 정답 자세에서 스캔 빔 600여 개가 지도에 거의 다 한 칸(5 cm) 안으로 겹친다(3-1절). 정상 주행 중 일치율은 0.98~1.00이었다.
- AMCL 공분산(1번 안)이나 EKF·AMCL 괴리(2번 안)와 달리 지도와 직접 비교하므로, AMCL이 엉뚱한 곳을 확신하는 경우도 잡는다.
- 지도 거리장은 numpy만으로 계산한다(체비쇼프 거리 팽창, 0.5 m까지).

### 4-2. 복구 절차

| 단계 | 동작 | 끝 조건 |
| --- | --- | --- |
| 1차 시도: 국소 재초기화 | 마지막으로 일치율이 높았던 자세 주변에 파티클을 넓게 뿌린다(AMCL `initialpose`, 위치 σ 2.5 m, 방향 σ 0.8 rad) | 수렴 또는 `settle_s` 10초 |
| 2차 시도부터: 전역 재초기화 | AMCL `reinitialize_global_localization` (지도 전체) | 같음 |
| 각 시도 안 | 움직이지 않고 `request_nomotion_update` 반복 → 미수렴이면 탐색(`rotate_s` 15초): 제자리 회전 0.5 rad/s, 또는 `explore_forward`면 앞 ±20°가 1 m 넘게 비었을 때 0.2 m/s 전진 | 수렴하면 정상 추적으로 |
| 실패 | `max_attempts`(3) 넘으면 `failed`. 상위(BT·사람)가 처리 | — |

수렴: 일치율 ≥ 0.8이고 AMCL 위치 표준편차 ≤ 0.15 m가 3번 연속. 일치율 조건이 있어 AMCL이 엉뚱한 곳을 확신해도 복구로 치지 않는다.

### 4-3. 순간 이동 실험 (10/6, 지도 v0, AMCL 튜닝값)

Gazebo `set_pose`로 로봇을 옮겼다(정지 상태).

| 이동 | 파티클 최대 | 감지 | 결과 |
| --- | --- | --- | --- |
| 먼 곳: A–B 통로 (2, 0) → B–C 통로 (11, −7), 북향 | 2,000 | 1.4초 (일치율 1.00 → 0.13) | 실패. 전역 재초기화가 다른 통로로 수렴(18~25 m 오차, 한 번은 방향 180° 반대) |
| 같음 | 10,000 | 3.4초 | 실패. 통로(y)는 맞혔지만 같은 통로 안에서 x가 15.6 m 어긋남 |
| 가까운 곳: 1.5 m 앞·1.2 m 옆·+29°, 국소 σ 1 m | 10,000 | 1.4초 (0.07) | 1차 국소 재초기화로 3초 만에 방향 오차 0.003 rad, 옆 방향 오차 6 cm까지 바로잡음. 통로 방향 x 1.3~1.4 m 오차는 남음(일치율 0.65). 이후 전역 재초기화는 다른 곳으로 수렴 |
| 같음 + 전진 탐색 (`explore_forward`) | 10,000 | 1.5초 | 실패. 1.6 m 전진했지만 추정도 나란히 움직여 x 1.3~1.5 m 오차가 그대로. 파티클이 이미 틀린 x에 모여 정답 근처에 파티클이 없다 |
| **같음, 국소 σ 2.5 m** | 10,000 | 1.5초 | **복구 9.5초** (움직이지 않음). 위치 오차 0.10 m, 방향 0.014 rad, 일치율 0.85 → 판정 기준 초안 통과 (1회) |

- 감지는 세 번 모두 2~4초 안에 됐고, 순간 이동 전 정상 주행에서 잘못 감지한 적은 없었다.
- 잘못 수렴했을 때 일치율이 0.3~0.65에 머물러, 노드가 "복구됨"으로 잘못 판정하지 않았다.
- 가까운 이동은 국소 재초기화 범위가 실제 어긋남(1.9 m)을 넉넉히 덮어야 풀렸다. σ 1 m로는 정답 근처 파티클이 너무 적었다.
- **먼 이동 실패 원인: 반복 구조.** 랙이 7.5 m 간격으로 같아서, 한 자리에서 보는 스캔으로는 통로 방향 위치가 잘 구분되지 않는다(정상 추적에서도 AMCL 오차가 x 2.7 cm, y 1.1 cm로 통로 방향이 크다, 2-2절). 제자리 회전은 새 정보를 거의 주지 않는다. 앞서 적은 위험 [8]이 실제로 나타났다.

### 4-4. 성공 판정 기준 (초안)

| 항목 | 기준 | 이유 |
| --- | --- | --- |
| 감지 | 순간 이동 후 5초 안에 `lost` | 실측 1.4~3.4초 |
| 오감지 | 정상 주행(하네스 `loc_basic` N회)에서 0회 | 복구 동작이 주행을 끊으므로 |
| 복구 | 감지 후 60초 안에 위치 오차 ≤ 0.3 m, 방향 오차 ≤ 0.1 rad를 3초 이상 유지 | 0.3 m는 좁은 통로(2.6 m)에서 로봇 양옆 여유(약 1.1 m)보다 충분히 작은 값으로 잡은 초안. 이진(경로 추종)과 확인한다. 60초는 회전 포함 한 번의 시도 시간의 약 2배 |
| 성공률 | 순간 이동 N회(시작·도착 위치 무작위) 중 복구 비율과 95% CI | 작업 8 |

### 4-5. 다음 단계 (작업 8, 10/11)

1. **움직이며 찾기**: 시제품(`explore_forward`)은 국소 σ 1 m 실패 상황에서 효과가 없었다(파티클 부족이 먼저). 전역 재초기화 뒤 파티클이 퍼진 상태에서 통로를 따라 움직이면 랙 끝·기둥이 보여 도움이 될 수 있어 먼 이동에서 다시 시험한다. `cmd_vel`을 쓰므로 사용 우선순위를 이진(제어)·지수(BT)와 정한다.
2. **외부 단서**: 전역 재초기화 대신 Fleet 서버가 아는 마지막 작업 위치, 도크 ArUco 마커(지수)로 후보를 좁힌다.
3. 오감지 0회 확인: 하네스 `loc_basic`에 이 노드를 켜고 N회.
4. 순간 이동 N회 성공률 (하네스 시나리오로 만든다).

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
