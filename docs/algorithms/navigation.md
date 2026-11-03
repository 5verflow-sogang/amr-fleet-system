# 경로 계획 · 모션 제어 (A* · DWA · Costmap)

> 상태: 진행 중 (2주차). 담당: 이진. Costmap(1절)·DWA(4절) 설정·구현 근거를 정리한다.
> A* 알고리즘 설명(3절)은 아직 TODO. Pure Pursuit/Stanley/PID와 Gazebo CTE 측정은 5절. 경로 계획·모션
> 제어 KPI 판정(#158)은 8절.
> 파라미터 파일은 [amr_bringup/config/nav2_params.yaml](../../src/bringup/amr_bringup/config/nav2_params.yaml)이다.

## 1. Costmap

Nav2 Costmap은 global(정적 맵 전체)과 local(로봇 주변 롤링 윈도우) 두 인스턴스로 나뉘고,
각각 3개 레이어를 쌓아 최종 비용 격자를 만든다. 레이어는 `plugins` 목록 순서대로 위에 덧쓰인다.

| 레이어 | 역할 | 이 프로젝트에서의 입력 |
| --- | --- | --- |
| Static Layer | SLAM이 만든 정적 맵(`map.pgm`/`map.yaml`)을 그대로 비용 격자에 반영한다 | `map` 토픽(지수 `amr_mapping` 산출물), `map_subscribe_transient_local: true`로 최신 맵을 늦게 구독해도 받는다 |
| Obstacle Layer | 센서로 실시간 감지한 장애물(정적+동적)을 마킹·클리어링한다 | LiDAR `scan` (상대 토픽). 명세상 Depth Camera도 있으나 1주차는 LiDAR 단일 소스로 시작 |
| Inflation Layer | 장애물 주변에 비용 그라디언트를 씌워 로봇이 벽에 너무 붙어 계획하지 않게 한다 | 위 두 레이어의 결과 + `footprint` |

global/local 모두 3레이어 구성을 쓰기로 한 이유: 명세서 1-4절이 "Costmap 3레이어"를
필수 항목으로 명시하고, local costmap도 static layer를 두면 롤링 윈도우 가장자리에서
아직 스캔하지 못한 영역을 정적 맵 정보로 미리 채울 수 있어 계획 실패를 줄인다.

### 1-1. 로봇 풋프린트와 좁은 통로 통과

로봇 스펙(원 명세서): 600×400×300mm. 중심 기준 x=±0.3m, y=±0.2m 사각형 풋프린트를 쓴다.

```
footprint: [[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]
```

Nav2는 이 풋프린트에서 두 반경을 자동 계산한다.

- **inscribed radius (약 0.2m, 폭 방향이 더 좁아서 이 값으로 결정)**: 이 거리 안쪽은
  로봇이 어떤 방향을 향해도 장애물과 겹치므로 통과 불가(치명 비용, `INSCRIBED_INFLATED_OBSTACLE=253`)로 처리한다.
- **circumscribed radius (약 0.36m, 대각선 방향)**: 이 거리 밖에서는 로봇이 어떤 방향을
  향하든 안전하다고 본다.

명세의 "좁은 통로(로봇 폭+20cm)" 요구는 통로 폭 0.6m(=0.4m+0.2m) 통과를 뜻한다. 통로
중심을 따라가면 각 벽까지 남는 여유는 0.1m로, 로봇의 inscribed radius(0.2m)보다 좁다.
즉 **통로를 통과할 수 있는지는 inflation 파라미터가 아니라 풋프린트 폭 자체가 결정**하고,
inflation은 그 안에서 얼마나 벽에 붙어도 되는지의 "선호도" 그라디언트만 만든다.

- `inflation_radius`: 비용이 0으로 떨어지는 최대 거리. 1주차 기본값은 global 0.55m,
  local 0.5m(롤링 윈도우 3×3m에 맞춰 약간 좁게).
- `cost_scaling_factor` (기본 3.0): inscribed radius 밖에서 비용이 `exp(-cost_scaling_factor
  × (distance - inscribed_radius))`로 감쇠하는 속도. 값이 클수록 벽에서 조금만 멀어져도
  비용이 빨리 낮아져, 좁은 통로에서도 경로가 통로 중앙으로 과도하게 몰리지 않는다.

### 1-1-1. 확정된 좁은 통로 좌표 (2주차, `amr_gazebo/worlds/warehouse.sdf`)

지수의 물류센터 월드(PR #19)에 이진이 통로 1곳을 더해 총 2곳을 확정했다. 좌표·근거는
[amr_gazebo/README.md](../../src/simulation/amr_gazebo/README.md#좁은-통로-②-추가-검증-2주차)에 정리했다.

| 통로 | 위치 | 폭 × 길이 | 용도 |
| --- | --- | --- | --- |
| ① | `rack_b3`–`rack_b4` 사이, x −5.0~−4.4, y −2.6~−1.4 | 0.6 × 1.2 m | 짧은 관통형 틈 — 진입/통과 자체가 되는지 |
| ② | `rack_lane_divider`–`rack_b4` 사이, x −4.4~0.6, y −1.4~−0.8 | 0.6 × 5.0 m | 여러 미터를 좁은 폭으로 유지 주행하는지 |

50쌍 경로 성공률 98% KPI는 이 두 통로를 모두 지나는 시작/목표 쌍을 섞어 검증한다
(통로 ①만으로는 단일 지점 편향이 생기기 때문에 ②를 추가했다).

TODO(2주차 계속): 위 좌표로 실제 `inflation_radius`·`cost_scaling_factor`를 재조정하고,
50쌍 성공률 98% KPI를 측정한다.

### 1-2. Obstacle Layer: 센서 업데이트 주기 · 장애물 유지 시간

원 명세서가 요구하는 두 설정을 `scan` observation source에 명시했다.

| 파라미터 | 값 | 의미 |
| --- | --- | --- |
| `expected_update_rate` | 0.1초 | LiDAR 스펙(10Hz 이상)을 만족하는 최대 주기. 실제 발행이 이보다 느려지면 Nav2가 경고를 낸다 |
| `observation_persistence` | 0.3초 | 마지막 관측 후 장애물 마킹을 유지하는 시간. 0.0이면 매 스캔마다 최신 관측만 남는다 |

`observation_persistence`는 트레이드오프가 있다: 너무 짧으면 스캔 사이 빈틈에서 장애물이
깜빡이며 사라지고, 너무 길면 최대 1.5m/s로 움직이는 동적 장애물이 지나간 자리에 "유령
장애물"이 남아 불필요한 재계획을 유발한다(0.3초 기준 최대 잔상 약 0.45m). 1주차는 0.3초로
시작하고, 2주차 동적 장애물 회피 실험(연구 2, 4장)에서 재계량한다.

`raytrace_max_range`(3.0m)·`obstacle_max_range`(2.5m)는 LiDAR 최대 사거리(25m)보다 훨씬
짧게 잡았다. Costmap 갱신 비용을 통로 스케일(로봇 근처 몇 m)에 맞추기 위한 관례적 값이며,
지수 월드의 실제 통로·개활지 크기가 정해지면 재검토한다.

### 1-3. Global vs Local 갱신 주기

| 항목 | Global | Local | 이유 |
| --- | --- | --- | --- |
| `update_frequency` | 1.0Hz | 5.0Hz | Global은 정적 맵 기반이라 느려도 되고, Local은 동적 장애물 회피가 걸려 있어 빠르게 갱신해야 한다 |
| `rolling_window` | false | true | Global은 맵 전체 고정, Local은 로봇을 따라다니는 3×3m 창 |

## 2. 1주차 기준선 Planner / Controller

A*·DWA 직접 구현과 비교할 baseline으로 Nav2 내장 플러그인을 그대로 붙였다.

| 서버 | 플러그인 | 역할 |
| --- | --- | --- |
| `planner_server` | `nav2_navfn_planner/NavfnPlanner` | 원 명세서가 요구하는 "A* vs NavFn" 비교의 NavFn 쪽. `use_astar: false`로 Dijkstra 모드를 쓴다 |
| `controller_server` | `dwb_core::DWBLocalPlanner` | "DWA 직접구현 vs TEB" 비교의 대조군 자리에 임시로 둔 것. 속도·가속 한계만 로봇 스펙(2.0m/s, 1.0m/s², 1.5rad/s)에 맞췄고 critic 가중치는 Nav2 공식 튜토리얼 기본값이다 |

TODO(2주차): `amr_planning_core`의 A*를 `amr_nav2_plugins`로 감싸 `planner_plugins`에
추가하고, NavFn과 경로 길이·계획 소요 시간·장애물 여유 거리를 비교한다. DWA 직접구현도
같은 방식으로 `controller_plugins`에 추가해 DWB와 비교한다.

## 3. A* (예정)

TODO(2주차): `amr_planning_core`의 A* 알고리즘 설명(격자 표현, 8방향 이동, 휴리스틱
선택 근거, 셀 비용 가중치), 경로 평활화 방법을 여기에 작성한다.

## 4. DWA (Dynamic Window Approach)

> 담당: 이진. `amr_planning_core::DwaPlanner` (`dwa.hpp`/`dwa.cpp`). **이번 주 범위는
> 정적 장애물 회피까지다** — 동적 장애물·Nav2 플러그인 연결·TEB 비교는 3주차(4-5절).

### 4-1. 4단계

1. **동적 윈도우 계산** (`computeDynamicWindow`): 현재 속도 (v, w)에서 가속 한계×dt만큼만
   벗어날 수 있다고 보고, 그 범위를 다시 로봇 최대 속도로 자른다. 지금 속도가 한계 밖으로 한 주기 변화
   이상 벗어나 있으면(처음 속도가 `max_v`보다 높음, w가 `-max_w`보다 낮음 등) 창이 뒤집히므로, 한계 쪽으로
   최대로 줄인 값(위로 벗어났으면 지금 − acc·dt, 아래로 벗어났으면 지금 + acc·dt) 하나만 남긴다.
2. **속도 샘플링 + 궤적 시뮬레이션** (`simulateTrajectory`): 윈도우 안의 (v, w) 조합마다
   차동구동 전진 운동학으로 `sim_time` 동안 `dt` 간격으로 궤적을 찍는다.
3. **비용 함수** (`evaluateCost`): heading(목표 방향) + clearance(장애물 거리) +
   velocity(속도)를 각각 [0, 1]로 정규화해 가중합한다.
4. **충돌 궤적 제외 후 최소 비용 선택** (`computeVelocityCommand`): 궤적 위 모든 점에서
   사각 풋프린트가 치명 비용 셀과 겹치면 그 (v, w)는 후보에서 뺀다. 남은 후보 중 비용이
   가장 낮은 (v, w)를 반환한다. 전부 충돌하면 `kNoValidTrajectory`. (허용 속도 조건을 켜면 4-7절)

### 4-2. 파라미터 (`DwaConfig`, 기본값)

| 파라미터 | 값 | 근거 |
| --- | --- | --- |
| `max_v` / `min_v` | 2.0 / 0.0 m/s | `nav2_params.yaml` `FollowPath.max_vel_x`와 동일(로봇 스펙) |
| `max_w` | 1.5 rad/s | `FollowPath.max_vel_theta`와 동일 |
| `acc_v` | 1.0 m/s² | `FollowPath.acc_lim_x`와 동일 |
| `acc_w` | 3.2 rad/s² | `FollowPath.acc_lim_theta`와 동일 |
| `dt` | 0.1 s | 제어 주기 |
| `sim_time` | 2.0 s | 궤적 예측 길이. 짧게 잡으면 장애물을 더 가까이 와서야 "미래 충돌"로 감지해서 막판에야 급히 피하려다, 동적 윈도우(한 스텝에 꺾을 수 있는 각속도, `acc_w*dt`뿐)가 모자라 못 피하는 상황이 생긴다 — 테스트 중 실제로 발견해서 `sim_time`은 줄이지 않기로 했다(계산량은 샘플 개수로 조절) |
| `v_samples` | 20 (스택 10) | `DwaConfig` 기본은 `FollowPath.vx_samples`와 같은 20. 창고 스택(`nav2_params.yaml`)은 10 — 아래 "샘플 수" |
| `w_samples` | 30 (스택 15) | `DwaConfig` 기본 30. 창고 스택은 15 — 아래 "샘플 수". 연구 2는 20 x 30 그대로 |
| `footprint_half_length` / `half_width` | 0.3 / 0.2 m | 로봇 스펙(600×400mm)과 동일. **외접원이 아니라 사각형으로 충돌 검사**한다 — 외접원 지름(0.72m)으로 하면 0.6m 좁은 통로(1-1절)를 항상 "막힘"으로 오판한다 |
| `lethal_cost_threshold` | 253 | `AStarConfig`와 동일한 nav2 costmap 관례 |
| `heading_weight` / `clearance_weight` / `velocity_weight` | 0.8 / 0.1 / 0.1 | 4-3절 |
| `goal_tolerance` | 0.15 m | 전역 경로의 다음 지점 추종이 목적이라 각도 허용오차는 두지 않았다(최종 도킹 정밀도는 범위 밖) |
| `use_admissible_velocity` | true | 4-7절. false면 이전 동작(호라이즌 안 충돌 궤적 전부 제외)과 같다 |
| `use_goal_slowdown` | true | 4-7절. 최종 목표까지 직선 거리로 `v <= sqrt(2·acc_v·d_goal)` |
| `lookahead_distance` / `adaptive_lookahead` | 0.5 m / true | 4-8절. 경로 버전의 heading 기준점 |
| `rotate_in_place_min_angle` / `rotate_in_place_exit_angle` | 0.785 / 0.524 rad (45° / 30°) | 4-9절. 제자리 회전 모드 들어가기·나오기(경로 버전). exit > min이면 min으로 줄인다 |
| `rotate_in_place_clearance` | 0.05 m | 4-9절. 회전 모드의 제자리 회전 샘플 풋프린트 여유. 안전 노드 `side_margin`과 같아야 한다 |
| `avoid_mode_enabled` | false (창고 스택 yaml true) | 4-10절. TTC 회피 모드 전체. 연구 2는 false 고정 |
| `avoid_margin_add` / `avoid_weight_scale` / `avoid_rollout_time` | 0.4 m / 2.0 / 3.0 s | 4-10절. 회피 모드에서 안전 여유 더하기, 동적 가중치 곱, 궤적 시뮬레이션 길이 |
| `avoid_ttc_enter_s` / `avoid_exit_hold_s` / `avoid_risk_timeout_s` (플러그인) | 3.0 / 0.5 / 0.3 s | 4-10절. 켜기 TTC, 끄기 전 유지 시간, collision_risks 끊김 판정 |

**샘플 수 (10/27, 창고 스택 `nav2_params.yaml` 10 x 15):** `computeVelocityCommand` 한 번 계산 시간을 재고
(x86 8코어, 다른 부하 없음, 3 x 3 m local costmap + 통로 벽·inflation, 90° 꺾이는 경로, max_v 0.5, 300회 중앙값),
Gazebo에서 통로 ①(목표 yaw +90°)을 설정마다 3번씩 돌렸다(실행 전마다 `fastdds shm clean`).

| v x w | 순항, 지금 빌드(최적화 없음) | 순항, `-O2` | 동적 장애물 2개 (없음 / `-O2`) | 제자리 회전 (없음 / `-O2`) | 통로 ① 3회 (시간, 복구) |
| --- | --- | --- | --- | --- | --- |
| 20 x 30 = 600 | 66 ms (p95 87) | 25 ms (p95 42) | 67 / 25 ms | 3.4 / 1.3 ms | 3/3 성공, 111·109·131 s, 복구 3·3·6 |
| 15 x 20 = 300 | 32 ms (p95 37) | 12 ms (p95 14) | 39 / 12 ms | 2.3 / 0.9 ms | — |
| 10 x 15 = 150 | 17 ms (p95 22) | 6 ms (p95 6.5) | 17 / 6 ms | 1.7 / 0.7 ms | 3/3 성공, 97·106·89 s, 복구 1·3·0 |

- 계산 시간은 샘플 수에 거의 비례한다. 지금 빌드(`CMAKE_BUILD_TYPE` 없음, 최적화 없음)에서 20 x 30은 20 Hz 제어 주기
  (50 ms)를 넘어 Gazebo에서 "Control loop missed its desired rate" 경고가 실행마다 1,000번 넘게 났다.
- 10 x 15는 통로 ①에서 20 x 30보다 나쁘지 않았다(평균 97 s / 117 s, 복구 1.3 / 4회). 직선 5 m·출발 방향 180°도 둘 다
  모두 성공했다. 실물(라즈베리파이)에서도 돌아야 해 작은 쪽으로 정했다.
- `w_samples`가 홀수면 [min_w, max_w]를 고르게 나눌 때 가운데(지금 각속도, 흔히 0)가 샘플에 들어간다.
- Release 빌드(`-O2`)면 2.6배 빨라진다 - 팀 빌드 설정이라 이번에는 바꾸지 않았다(위 표에 결과만 남긴다).

### 4-3. 비용 함수 가중치 근거와 "제자리 정지" 함정

처음엔 heading 0.8 / clearance 0.1 / velocity 0.1로 "충돌 궤적은 이미 하드 필터로
제외되니, 남은 후보 중에서는 목표 방향이 가장 중요하다"는 근거로 정했다.

그런데 폐루프 테스트(`ClosedLoopReachesGoalWithOneObstacle`)를 만들면서 **로봇이
장애물 앞에서 멈춘 채 영원히 못 움직이는 현상**을 실제로 발견했다. 원인: 정지(v≈0)
궤적은 위치가 거의 안 바뀌므로, 로봇이 마침 목표 쪽을 보고 있으면 heading 비용이
0이 되어버린다. 그 결과 "제자리에 멈춰서 목표를 바라만 보기"가 실제로 전진하는
어떤 후보보다도 싸 보이는 국소 최소(local minimum)가 됐다 — 속도 0.025 m/s까지
떨어지다 결국 0에서 고정되는 것을 테스트 로그로 확인했다.

고친 방법: `evaluateCost`에서 `|v|`가 거의 0인 궤적은 heading 비용을 계산하지 않고
최댓값(1.0)으로 고정한다. "멈춰 있는 것"은 목표를 향해 가고 있다고 볼 근거가 없다고
보는 것이다. 수정 전/후 동일 시나리오를 Python으로 재현해 비교했다 — 수정 전에는
영원히 멈추거나(그 자리에서 제자리 회전만 반복) 끝났고, 수정 후에는 316스텝(약
31.6초) 만에 장애물을 피해 목표에 도달했다. 가중치 비율(0.8/0.1/0.1) 자체는 바꾸지
않았다 — 문제는 가중치가 아니라 "정지도 heading이 좋을 수 있다"는 정의 쪽이었다.

### 4-4. 좌표계

`RobotState`의 (x, y)는 `Grid`의 로컬 좌표계(미터, 셀 (0,0)의 모서리가 원점)다.
`Grid`가 원점 정보를 들고 있지 않은 설계(`grid.hpp`, A*와 공유)와 같은 이유로,
odom/map 프레임 ↔ 이 좌표계 변환은 core가 아니라 3주차 `amr_nav2_plugins`가 맡는다.

### 4-5. 범위 변화 (2주차 → 3주차)

| 구분 | 2주차 | 3주차 |
| --- | --- | --- |
| 장애물 | 정적 장애물만(`Grid` 비용값) | 동적 장애물(4-6절)도 비용 함수·충돌 판정에 반영 |
| 연결 | ROS 비의존 core만(`amr_planning_core`) | `amr_nav2_plugins::DwaController`로 `nav2_core::Controller` 감싸 `controller_server`에 `DWA`로 추가(1주차 DWB baseline은 비교용으로 남겨 둔다) |
| 비교 | — | DWB·TEB와 CTE·정지 횟수·경로 길이로 비교(원 명세 "DWA vs TEB") — 아직 TODO |

### 4-6. 동적 장애물 (3주차)

`perception/obstacles`(`TrackedObstacleArray`)의 pose·twist·radius를 `DynamicObstacle`
(x, y, vx, vy, radius)로 바꿔 `computeVelocityCommand`에 추가 인자로 넘긴다
(`DwaConfig.predict_dynamic_obstacles`). 궤적 롤아웃의 각 시각 t마다 장애물을
`x+vx·t, y+vy·t`로 등속 예측해 사각 풋프린트와의 거리를 구하고, 안전 여유
(`dynamic_obstacle_safety_margin`) 안이면 그 궤적을 충돌로 걸러내고, 그 밖이면
거리에 따라 줄어드는 소프트 비용(`dynamic_weight`, 영향 반경
`dynamic_obstacle_influence_radius`)을 더한다. 동적 장애물을 안 넘기면(빈 벡터,
기본값) 2주차 정적 장애물 전용 결과와 완전히 같다 — `predict_dynamic_obstacles:
false`면 장애물이 관측 위치에 멈춰 있다고 본다(순간 스냅샷).

지금 자세에서 이미 안전 여유 안에 있는 장애물은 **지금보다 가까워지거나 닿는 궤적만** 충돌로 본다(#216).
전에는 궤적 첫 점부터 여유 안이라 멀어지는 궤적까지 모두 버려져, 가로지르는 장애물 앞에 0.2 m 안으로
선 로봇이 장애물이 비킬 때까지 0~0.01 m/s로 서 있었다(연구 2 S1 진단 로그: 한 주기 600개 샘플 중
236개가 첫 점 동적 충돌, 나머지는 그 장애물의 costmap 치명 셀). 정면 장애물처럼 앞으로 가면 가까워지는
경우는 그대로 막는다.

`amr_nav2_plugins::DwaController`가 이 장애물 목록을 만들어 넘기고, 전역 경로에서
로봇 앞 `lookahead_distance`만큼의 지점을 "다음 목표"로 골라 DWA에 준다. Gazebo
월드 v0 + Nav2 bringup으로 실제 주행을 확인했다(`amr_bringup/launch/
nav2_dwa_demo.launch.py`, `use_static_map`은 없어졌다 - 바로 아래 참고).

처음엔 목표가 수락되고 BT가 시작된 뒤 `cmd_vel`이 전혀 나오지 않아 "NavFn이 큰
지도(60x40m)에서 멈춘다"고 보고 global costmap을 로봇 중심 rolling_window(지도
없이)로 바꿔 돌아갔었다. 그런데 진짜 원인은 지도 크기가 아니라 `obstacle_layer`가
구독하는 토픽 이름이 실제 Gazebo LiDAR 토픽(`scan_raw`)과 달라 costmap이 영영
`isCurrent()`가 안 되고 `planner_server`·`controller_server`가 그걸 기다리며
조용히 멈춰 있던 것이었다 - 토픽을 맞추고 나니(+ bt_navigator가 찾는 컨트롤러
이름 불일치, 최종 접근 시 목표 각도까지 회전하지 않던 버그도 같이 고쳐서)
static_layer(지도 v0) 그대로도 바로 된다. rolling_window는 그래서 다시 뺐다.

### 4-7. 허용 속도 (Fox 1997)

원래 DWA의 허용 속도 조건 `v <= sqrt(2·acc_v·dist)`를 넣었다(`use_admissible_velocity`, 기본 true).
`dist`는 궤적을 따라 장애물(Grid 치명 셀·동적 장애물 안전 여유)에 닿기 전까지 이동 거리로,
이미 하는 충돌 검사 루프에서 닿기 직전 점까지 센 개수로 구한다(`|v|·dt·free_steps`, 보수적).
장애물에 안 닿는 궤적은 호라이즌 밖을 모르므로 끝(`|v|·sim_time`)에 장애물이 있다고 본다.

이전에는 `sim_time` 안에서 닿는 궤적을 전부 뺐다. 이러면 남는 궤적은 항상 `dist >= |v|·sim_time`이라
`|v| <= 2·acc_v·sim_time`(기본값 4 m/s > `max_v`)인 한 허용 조건이 늘 성립해서 `acc_v`는 동적 윈도우
에만 쓰이고, 질량(가속 한계)을 틀리게 줘도 결과가 거의 안 변했다. 그래서 원래 DWA처럼 닿는 궤적도
"acc_v로 감속해 장애물 앞에 설 수 있으면" 남긴다(heading은 닿기 직전 점에서 본다, 첫 점부터 닿으면
뺀다). 다만 안 닿는 궤적이 하나라도 있으면 그중에서만 고르고, 닿는 궤적은 안 닿는 궤적이 하나도 없을
때(이전에는 `kNoValidTrajectory`로 바로 정지)만 고른다. 닿는 궤적까지 같은 비용으로 겨루게 했더니
heading 비중(0.8)이 압도적이라 벽으로 곧장 가다 막판에 못 꺾어 `ClosedLoopReachesGoalWithOneObstacle`,
`FrontObstacleTurnsAway`, `FrontApproachingObstacleIsAvoidedOnlyWithPrediction`이 실패했다.
그래서 질량 정보는 "모든 궤적이 호라이즌 안에서 장애물에 닿는" 상황(정면이 막혀 감속·정지해야 할 때)의
감속 프로파일에 반영된다. 감속 한계는 `acc_v`를 그대로 쓴다.

허용 속도 조건은 2초 충돌 지평 때문에 대부분 효과가 없다 - 안 닿는 궤적은 `dist = |v|·sim_time`이라
`acc_v >= v/(2·sim_time)`이면 항상 만족한다(기본 `max_v` 2.0이면 `acc_v >= 0.5`).

**목표 앞 감속** (`use_goal_slowdown`, 기본 true): 최종 목표(전역 경로 끝, 지역 waypoint가 아님)까지
직선 거리 `d_goal`에서 샘플 선속도를 `v <= sqrt(2·acc_v·d_goal)`로 제한한다(`computeVelocityCommand`의
`final_goal` 인자. 경로를 받는 버전은 경로 마지막 점을 쓴다). 동적 윈도우 최저 속도가 이보다
높으면 최저 속도 하나만 샘플링한다. #216부터 상한은 연속 시간 `sqrt(2·acc_v·d_goal)`이 아니라 이번 제어 주기를 넣은 `|v|·dt + v²/(2·acc_v) <= d_goal`의 최대 속도(`goalStoppingVelocity`)다 - 연속 시간 식은 매 주기 그 속도로 `dt`만큼 더 가는 만큼 상한보다 늦게 줄어, 최대로 감속해도 목표를 지나쳤다(연구 2 S2 25 kg 30/30 지나침, `GoalSlowdownStopsBeforeGoalAccountingForControlPeriod`). 감속은 `acc_v` 그대로 쓰고, `xy_goal_tolerance` 안에서는 지금처럼
플러그인이 v=0으로 끊고 `rotateToGoalYaw`로 돈다.

다만 이것도 기존 heading 항에 가려지는 경우가 많다. 궤적 끝이 목표를 넘으면 heading 비용이 최대(1.0)라
이미 `v <= d_goal/sim_time`으로 줄이고 있었고, 이 값이 `sqrt(2·acc_v·d_goal)`보다 작은 구간
(`d_goal < 2·acc_v·sim_time²`, 기본 8 m)에서는 그쪽이 먼저 걸린다. 목표 앞 감속이 heading보다 먼저
시작하려면 `v > 2·acc_v·sim_time`이어야 하는데, 그 속도는 허용 속도 조건이 빼므로 둘 다 켜져 있으면
목표 앞 감속은 실제로 걸리지 않는다(`GoalSlowdownStartsEarlierWithSmallerAcceleration`는 그래서 허용
속도 조건을 끄고 작은 `acc_v`로 확인한다). 경로를 받는 버전에서 `adaptive_lookahead`를 켜면 경로 끝에서
heading을 끝점 앞에서 재므로 이 가림이 없어진다(4-8절).

### 4-8. heading 기준점 (`adaptive_lookahead`)

**원인:** 플러그인은 전역 경로에서 로봇 앞 `lookahead_distance`(0.5 m) 점 하나를 골라 DWA 목표로
넘겼고, heading 비용은 궤적 끝에서 그 점을 향하는 각도였다. 궤적 끝이 그 점을 지나치면 방향이
뒤집혀 heading 비용이 최대(1.0, 가중치 0.8)가 되므로, 속도 비용(가중치 0.1)으로는 못 이기고
`v <= lookahead_distance/sim_time = 0.25 m/s`로 묶였다. Gazebo 목표 (6,0)에서 최고 속도가 0.246 m/s였다.
같은 이유로 4-7절의 허용 속도·목표 앞 감속도 이 속도에서는 거의 안 걸렸다(목표 3 cm 앞에서야 걸림).

**바꾼 것:** 코어가 전역 경로(점 목록)를 받는 `computeVelocityCommand` 버전을 추가하고, 플러그인이
경로를 통째로 넘긴다. heading 기준점은 궤적마다 경로를 따라 `max(lookahead_distance, |v|·sim_time)`
앞의 점이다(로봇에 가장 가까운 경로 점부터 누적, 경로 끝을 넘으면 마지막 점 - 기존
`selectLookaheadWaypoint`와 같은 규칙인 `pathPointAhead`). `lookahead_distance`는 최솟값으로 남아
느릴 때(`|v|·sim_time <= 0.5 m`)는 기존과 같은 점을 쓴다 - #165 좁은 통로 진입 동작 유지. 궤적이 경로
끝점을 지나치면 끝점보다 `goal_tolerance`만큼 앞의 궤적 점에서 heading을 잰다 - 지나친 궤적 끝에서
재면 다시 방향이 뒤집혀 목표 앞에서 `v <= d_goal/sim_time`으로 묶이고 목표 앞 감속이 가려진다. 도착
판정(`kGoalReached`)은 기존처럼 `lookahead_distance` 앞 점 기준이다. 점 하나만 넘기는 기존 버전은
그대로다(경로가 없으면 그 점 기준). `adaptive_lookahead: false`면 경로 버전도 기존 플러그인 흐름과
결과가 같다(`AdaptiveLookaheadDisabledMatchesFixedWaypointCall`).

**#216에서 고친 것(연구 2에서 1.5 m/s로 달리다 0.15 m/s로 떨어져 기어가 시간 초과):** 위 규칙은 기준점을
궤적 길이(`|v|·sim_time`)만큼 앞에 두어, 빠른 샘플일수록 궤적 끝과 기준점이 거의 같은 곳(수 cm 차)에
있었다. 궤적 끝에서 기준점을 향하는 방향이 경로 점 간격(0.05 m)과 옆 오차 몇 mm에 따라 크게 튀어, 빠른
샘플의 heading 비용만 들쭉날쭉 커지고(0.2~0.3) 속도 비용(최대 0.1)으로는 못 이겨 속도가
`lookahead_distance/sim_time` 근처로 떨어졌다 - 그보다 느리면 기준점이 고정 lookahead 점이라 다시
앞에 있게 된다. 진단 로그(샘플별 기준점·궤적 끝·비용 항)로 확인했다. 그래서 기준점을 **궤적 끝보다
`lookahead_distance` 앞**(`|v|·sim_time + lookahead_distance`)에 둔다 - 모든 샘플에서 끝 → 기준점
거리가 같아 heading 비용을 같은 조건으로 비교한다. 기준점이 경로 끝점으로 잘리면 끝점보다
`lookahead_distance` 앞인 궤적 점에서 heading을 잰다(전에는 끝점을 지나치는 궤적만 `goal_tolerance`
앞에서 쟀다) - 같은 이유로 목표 앞 몇 m에서 다시 기어가던 것을 막는다. 정지(v=0)에서는 기준점이 전과
같이 `lookahead_distance` 앞이다. 로봇에 가장 가까운 경로 점 찾기는 경로 전체를 보므로 원인이 아니었다.

### 4-9. 제자리 회전 (`rotate_in_place_min_angle`·`rotate_in_place_exit_angle`)

**원인 (10/14 하연 정리):** v≈0 샘플은 정지 함정(4-3절) 때문에 heading 비용을 1.0으로 고정했다. 그래서
제자리 회전(v=0, w≠0)은 어느 쪽으로 돌든 0.8(heading)+0.1(속도) = 0.9라, 안 닿는 전진 샘플이 하나라도
있으면 졌다. 목표가 뒤(180°)면 v=0.1, w=0.32 호(2초 뒤 0.64 rad 돎)가 0.8·(π−0.64)/π + 0.095 ≈ 0.73으로
이겨, 제자리에서 돌지 않고 목표 반대쪽으로 크게 돌며 나갔다. 전진이 전부 막힌 좁은 통로에서만 회전을
골랐고, 그때는 회전 샘플끼리 heading이 같아 도는 방향이 샘플 순서·clearance로 정해졌다.

**바꾼 것:** 경로를 받는 `computeVelocityCommand`에서, 지금 자세에서 heading 기준점(`lookahead_distance`
앞)까지 방향 오차가 `rotate_in_place_min_angle`(기본 45°, Nav2 RPP `rotate_to_heading_min_angle`과 같음)보다
크면 **제자리 회전 모드**에 들어가고 `rotate_in_place_exit_angle`(기본 30°) 이하가 되면 나온다 - 경계
근처에서 회전/전진이 매 주기 바뀌며 떨지 않게 하는 히스테리시스다(`RotateInPlaceModeHasHysteresis`). 모드
상태는 플래너가 주기 사이에 들고 있다(`rotatingInPlace()`).

- 모드에서는 v≈0 샘플도 **회전한 뒤 방향**(궤적 끝 yaw)으로 heading 비용을 매긴다. 정지(w=0)는 지금 오차
  그대로라 유리하지 않다. 모드 밖(오차 ≤ 문턱)에서는 지금처럼 1.0이라 정지 함정 방지는 그대로다.
- 먼저 최대 감속 속도(`window.min_v`, 서 있으면 0)만 샘플링해 돈다. 고른 회전 후보는 **궤적 끝 방향 오차가
  지금보다 `acc_w·dt·sim_time/2`(기본 0.32 rad) 이상 줄 때만** 채택하고, 아니면(좁은 통로 안처럼 회전이 막힘)
  전체 윈도우로 다시 고른다 - 못 도는데 서 버리지 않게. 기준은 서 있다가 한 주기 최대 각가속(`acc_w·dt`)으로
  `sim_time` 동안 돌 때 줄어드는 오차의 절반이다. w 샘플은 지금 각속도 둘레라 `state.w`≠0이면 정확한 0이 없어,
  양쪽이 막혀도 거의 안 도는 샘플(w≈0.01)이 "회전"으로 남는다 - |w|>0만 보면 그걸 채택해 그 자리에
  섰다(#244 연우 리뷰, `BlockedRotationFallsBackWithoutStationaryHeadingTrap`). 전체 윈도우 최선과 비용으로
  겨루는 안은 `velocity_weight` 때문에 전진 호가 이겨 출발 때 다시 1.6 m 호로 돌았다
  (`RotatesInPlaceWhenStartingFarOffHeading`).
- 다시 고르는 전체 윈도우 탐색은 평소와 같이 채점한다(정지 heading 1.0) - 회전 모드 채점이면 "목표 보고 서
  있기"가 전진보다 싸진다.
- v≈0 **회전**(|w|>0) 샘플이 궤적 중간에 닿으면 버린다. 사각 풋프린트는 v=0이어도 매 스텝 yaw가 바뀐 자세로
  검사하지만, 허용 속도 조건(4-7절)은 v=0이면 늘 "설 수 있음"이라 회전 감속(`acc_w`)을 안 보고 닿는 회전을
  [1] 후보로 남겼다. 대각선 0.72 m > 좁은 통로 0.6 m라 통로 안에서 크게 돌면 벽에 닿는다
  (`RotationSamplesInsideNarrowCorridorAreFiltered`). 정지(w≈0)는 남긴다 - 사람이 서 있는 로봇 쪽으로 걸어오면
  (예측 켬) 정지 궤적도 예측 궤적 중간에 닿는데, 그 자리에 서 있기는 후보여야 한다
  (`StationarySampleSurvivesApproachingDynamicObstacle`).
- 회전 모드의 회전 후보는 풋프린트 반길이·반폭을 `rotate_in_place_clearance`(0.05 m)만큼 키워 검사한다. 안전
  노드(`collision.path_hazard`, `side_margin` 0.05 m)는 제자리 회전 중 모서리가 0.05 m 안으로 가까워지면 막는데,
  DWA는 오차가 30° 아래로 내려가야 모드에서 나와 계속 회전을 내고 안전 노드는 계속 막아 progress_checker까지
  멈춘다(`RotateInPlaceKeepsClearanceFromStaticObstacle`). 회전 모드 밖 v≈0 회전과 전체 윈도우 탐색에는 안 쓴다 -
  전진은 장애물 0.05 m 앞까지 다가가므로 거기서 모든 회전을 키워 검사하면 정지만 남아 영영 못
  돈다(`ClosedLoopReachesGoalWithOneObstacle`). 지금 자세에서 이미 여유 안이면 그 주기는 키우지 않는다.
- 동적 장애물이 `safety_margin + influence_radius`(1.2 m) 안이면 회전 모드에 들어가지 않는다 - 사람을 피하느라
  경로에서 벗어나 오차가 45°를 넘었을 때 서서 경로 쪽(사람이 오는 쪽)으로 돌지 않게(`NoRotateInPlaceNearDynamicObstacle`).
- 점 하나로 부르는 버전(먼 목표)에는 안 쓴다 - 장애물을 돌아가는 동안 그 점까지 오차가 커져 회전이 회피와
  싸웠다(`ClosedLoopReachesGoalWithOneObstacle`). 경로의 기준점은 전역 경로가 이미 장애물을 돌아간다.
- 최종 접근(목표 도착 뒤 방향 맞추기)은 바뀌지 않았다. 플러그인이 `xy_goal_tolerance` 안이면 DWA 대신
  `rotateToGoalYaw`로 돈다 - 도착 뒤 90° 돌아야 하는 목표에서 제자리로 돌아 허용오차 안에 서는 것을 테스트로
  확인했다(`FinalApproachRotatesToGoalYawAfterArriving`).
- 연구 2(`experiments/research2`)는 제자리 회전을 끄고(`rotate_in_place_min_angle` 0) 회전 샘플 여유도 0이다
  (`DwaParams`, 논문 값). C API(`dwa_c_api.cpp`)는 호출마다 플래너를 새로 만들어, 켜더라도 히스테리시스 상태가
  안 이어진다.

### 4-10. TTC 회피 모드 (`avoid_mode_enabled`)

I17(원 명세 7장 동적 장애물 회피) 측정용이다. 평소 DWA는 동적 장애물을 안전 여유 0.2 m·`sim_time` 2.0 s 안에서만
본다. 사람이 빠르게 다가오는 짧은 동안만 더 멀리·더 크게 피하게 한다.

**진입·해제 (플러그인 `AvoidModeSwitch`, `amr_nav2_plugins/avoid_mode.hpp`)**
- `perception/collision_risks`(로봇 네임스페이스)의 TTC 중 **움직이는 트랙(`is_dynamic`)만** 본다. `CollisionRisk`에는
  `is_dynamic`이 없어 같은 추적 결과(`perception/obstacles`)의 `track_id`로 짝짓는다(`toDynamicObstacles`와 같은
  기준). 정적 트랙(랙 면 등)은 이미 costmap에 있다
- 켜기: 최소 TTC < `avoid_ttc_enter_s`(3.0 s)
- 끄기: TTC ≥ 3.0 s(또는 inf, 충돌 경로 아님)가 `avoid_exit_hold_s`(0.5 s) 동안 이어질 때. 그 사이 한 번이라도
  위험하면 처음부터 다시 센다 - 문턱 근처에서 켜고 끄기가 떨지 않게
- 상태는 `dwa/avoid_mode`(`std_msgs/Bool`, transient local)로 바뀔 때 발행하고(활성화 때 처음 상태 한 번), 켜고 꺼질
  때 INFO 로그를 남긴다
- 코어(`DwaPlanner::setAvoidMode`)는 "회피 모드 여부"만 받아 설정을 바꾼다. 토픽·시간은 플러그인이 맡는다

**collision_risks가 끊기면 지금 상태를 유지한다** (`avoid_risk_timeout_s` 0.3 s, 경고 로그는 2 s 스로틀). 켜져 있으면
끄지 않는다 - 끊긴 사이 위험이 왔는지 모르고, 회피 중에 추적이 잠깐 끊겼다고 여유를 줄이면 가장 가까울 때 덜 피한다.
다시 오면 안전한 TTC가 0.5 s 동안 새로 이어져야 끈다. 꺼져 있으면 켜지 않는다 - 추적 노드를 안 띄우는 스택
(fleet_stack 등)은 처음부터 collision_risks가 없어, 끊김을 위험으로 보면 늘 회피 모드가 된다. 그래서 한 번도 안 받았으면
끊김으로 보지 않는다(경고도 없음).

**회피 모드에서 바뀌는 것**
- `dynamic_obstacle_safety_margin` += `avoid_margin_add` **0.4 m**: 정면 최악 기준. 통신 100 ms + 추적 100 ms = 0.2 s
  지연 동안 상대속도 2.0 m/s(로봇 0.5 + 명세 동적 장애물 최대 1.5)로 다가오는 거리다. 옆으로 건너는 경우(상대속도 약
  1.6 m/s)면 0.32 m라, 정면 기준이 더 크다
- `dynamic_weight` *= `avoid_weight_scale` **2.0**: 초기값이다. 튜닝은 튜닝용 시드(1001~)로만 하고, I17 본 측정
  (시드 1~30) 전에 고정한다
- 궤적 시뮬레이션 길이 = `avoid_rollout_time` **3.0 s**: TTC 3 s 안의 장애물을 충돌 검사가 덮게 한다(평소 2.0 s면
  2~3 s 뒤 만날 장애물은 궤적 밖이다). **궤적 길이만** 바꾼다 - heading 기준점 거리(|v|·`sim_time`, 4-8절), 비용을
  매기는 궤적 끝(`sim_time` 시점), 제자리 회전 채택 기준(`acc_w·dt·sim_time/2`, 4-9절)은 평소 `sim_time` 2.0 s
  그대로다. `sim_time` 자체를 3.0으로 바꾸면 기준점이 1.5배 멀어져 모드가 바뀌는 순간 목표 방향이 튄다
  (`AvoidModeKeepsHeadingTargetWithoutObstacles`: 장애물이 없으면 모드와 상관없이 명령이 같다)

**계산 시간** (창고 스택 10×15, `computeVelocityCommand` 1회, 평균 ms (최대 ms), 3회 실행의 범위. x86 8코어, 3×3 m local
costmap, max_v 0.5, 300회씩. 최악은 회전 모드에서 회전이 거부돼 전체 윈도우를 다시 탐색하는 경우, 4-9절)

| 경우 | 평소 (최적화 없음) | 회피 모드 (최적화 없음) | 평소 (-O2) | 회피 모드 (-O2) |
| --- | --- | --- | --- | --- |
| 순항 | 14.9–15.9 (21–24) | 16.1–16.7 (23–29) | 5.6–6.0 (6.7–12) | 6.1–6.4 (7.4–14) |
| 동적 장애물 2개 | 15.3–16.2 (19–26) | 16.3–17.2 (18–29) | 5.8–6.0 (9.2–10) | 6.2–6.6 (7.9–16) |
| 최악 (회전 거부) | 12.1–12.7 (16–19) | 14.5–15.3 (18–21) | 4.5–4.7 (6.4–14) | 5.5–5.8 (10–16) |

회피 모드는 궤적이 1.5배(20 → 30점)지만 평균 5~25%만 는다 - 늘어난 점은 대부분 3×3 m local costmap 밖이라(순항 0.5 m/s ×
3 s = 1.5 m) 칸을 안 본다. 최악도 최적화 없는 빌드 최대 29 ms로 20 Hz 제어 주기(50 ms) 안이다.

**연구 2**는 `avoid_mode_enabled`를 false로 고정한다(`DwaParams`, C API로 넘김, `test_avoid_mode_is_off_paper_value`).

## 5. Pure Pursuit · Stanley · PID

> 담당: 이진. ROS 비의존 코어 `amr_control/include/amr_control/core/`, 노드 `path_tracker_node`
> (`plan` + `odom` → `cmd_vel_nav`). 파라미터는 [amr_control/config/path_tracker.yaml](../../src/navigation/amr_control/config/path_tracker.yaml).
> Nav2 `controller_server` 플러그인화는 아직 안 했다.

### 5-0. 공통: 가장 가까운 점과 CTE

경로는 점을 잇는 폴리라인으로 본다. 로봇 위치 p를 각 선분 [a, b]에 투영하고(t는 0~1로 자름)
가장 가까운 점 p\*를 고른다. 진행 방향 단위벡터 u = (b − a)/|b − a|에 대해

```
CTE = sign(u × (p − p*)) · |p − p*|      (경로 진행 방향 왼쪽 +, 오른쪽 −)
```

되돌아오거나 교차하는 경로에서 한참 앞·뒤 구간으로 튀지 않도록, 직전에 투영된 선분부터
`search_window`(2 m) 앞까지만 찾는다. 기록용 CTE는 두 방식 모두 **로봇 중심** 기준이다.

### 5-1. Pure Pursuit

경로에서 호 길이로 lookahead L_d 앞의 점을 목표로 잡고, 로봇을 지나며 그 점에 닿는 원호의
곡률을 쓴다. 목표점의 로봇 좌표를 (x_l, y_l), 거리를 L이라 하면

```
κ = 2·y_l / L²,   ω = v·κ
L_d = clamp(lookahead_time · |v|, min_lookahead, max_lookahead)    (adaptive_lookahead: true)
```

lookahead를 속도에 비례시키는 이유: 짧으면 저속에서 정확하지만 고속에서 진동하고, 길면
안정적이지만 곡선 안쪽을 깎는다. 목표점을 경로 **위**에서 고르므로 원호에서는 정상상태
오차가 이론상 0이다(시뮬레이션 R = 1 m에서 0.7 cm, 경로 점 간격 때문).

| 파라미터 | 값 | 근거 |
| --- | --- | --- |
| `lookahead_time` | 1.0 s | 0.5 m/s 순항에서 L_d = 0.5 m |
| `min_lookahead` / `max_lookahead` | 0.3 / 1.5 m | 하한은 풋프린트 반 길이(0.3 m). 상한은 로봇 최고속 1.5 m/s 근처에서 1초 앞 |
| `lookahead_distance` | 0.6 m | `adaptive_lookahead: false`일 때 비교용 고정값 |

### 5-2. Stanley

원 논문(Thrun et al., 2006)은 앞바퀴 축 중심을 기준점으로 조향각 δ를 정한다. 차동구동은
앞바퀴가 없으므로 로봇 중심에서 d(`front_offset`) 앞의 점을 기준점으로 쓴다.
기준점의 투영으로 heading 오차 θ_e(경로 방향 − 로봇 방향)와 CTE e를 구하면

```
δ = θ_e − atan( k·e / (v_s + v) ),   |δ| ≤ max_steer
ω = v·tan(δ) / d
```

ω 식은 기준점의 속도 방향이 로봇 방향에서 δ만큼 꺾이도록 고른 값이다(atan2(ω·d, v) = δ).
축거 d인 자전거 모델과 같은 식이라, 직선에서 기준점 오차가 대략 exp(−k·t)로 줄어드는 원 논문
성질이 남는다. 대신 원호에서는 기준점이 경로를 따라가므로 **중심이 안쪽으로 약 d²/(2R)만큼
붙는다**(R = 1 m에서 4.5 cm, 시뮬레이션 3.9 cm). 곡선 KPI 10 cm 안이지만, R이 0.5 m 아래로
가면 d를 줄여야 한다.

| 파라미터 | 값 | 근거 |
| --- | --- | --- |
| `cte_gain` (k) | 2.0 | 1 m 옆 출발에서 1.5 m 안에 5 cm로 들어오고 넘침 없음 (단위 테스트 시뮬레이션) |
| `softening_speed` (v_s) | 0.2 m/s | 출발 직후 v ≈ 0에서 atan 항이 튀지 않게 |
| `front_offset` (d) | 0.3 m | 풋프린트 앞 끝 |
| `max_steer` | 1.2 rad | tan이 발산하기 전에 자름 |

### 5-3. 공통 속도 프로파일 · 정지

| 단계 | 식 / 규칙 | 파라미터 |
| --- | --- | --- |
| 순항 | v = min(`desired_speed`, `max_linear_speed`) | 0.5 / 2.0 m/s |
| 끝 감속 | v ≤ max(`min_approach_speed`, √(2·`max_linear_decel`·남은 호 길이)) | 0.05 m/s, 1.0 m/s² |
| 가속 제한 | v ≤ 직전 명령 + `max_linear_accel`·dt | 1.0 m/s² |
| 각속도 한계 | \|v·κ\| > `max_angular_speed`면 v = `max_angular_speed`/\|κ\| (곡률은 유지) | 1.5 rad/s |
| 제자리 회전 | \|경로 방향 − 로봇 방향\| > `rotate_in_place_angle`면 v = 0, ω = ±`rotate_in_place_speed` | 1.0 rad, 0.8 rad/s |
| 정지 | 끝점과 `goal_tolerance` 안이거나 마지막 선분에서 끝점을 지나치면 (0, 0), 새 경로까지 유지 | 0.1 m |

로봇 한계(2.0 m/s, 1.0 m/s², 1.5 rad/s)는 `nav2_params.yaml` FollowPath와 같다. 재계획으로
경로가 바뀌어도 직전 명령 속도는 이어 쓴다(1 Hz 재계획마다 멈칫하지 않게).

### 5-4. CTE 기록 (`cte.csv`)

`cte_csv_path`를 주면 제어 주기마다 하네스 형식([tools/harness/README.md](../../tools/harness/README.md) 로그 포맷)으로 쓴다:
`timestamp, planned_x, planned_y, actual_x, actual_y, cte, segment, robot`.

- `timestamp`: odom `header.stamp`(시뮬레이션 시간), `planned_*`: 로봇 중심의 투영점, `actual_*`: 로봇 위치(경로 프레임)
- `cte`: 명세서 정의(수직 거리)대로 부호 없는 값. 부호는 코어 `PathProjection::cte`에 있다
- `segment`: 투영점 앞뒤 0.25 m로 잰 경로 곡률이 `curve_curvature_threshold`(0.1 1/m, 반지름 10 m) 이상이면 `curve`, 아니면 `straight`

### 5-5. 단위 테스트 (`test/test_path_tracker.cpp`)

차동구동 모델(속도 1차 지연 0.1 s, 20 Hz)로 두 방식 모두 같은 시나리오를 돌린다.

| 시나리오 | 판정 |
| --- | --- |
| 직선, 0.5 m 옆 출발 | 3 m 지난 뒤 \|CTE\| < 5 cm |
| 직선, ±1 m 옆 + 바깥쪽으로 0.3 rad 틀어 출발 | 4 m 지난 뒤 \|CTE\| < 5 cm, 반대쪽 넘침 < 10 cm |
| 원호 R = 1, 2, 5 m (반원) | 전 구간 \|CTE\| < 10 cm |
| 3 m 직선 끝 | 끝점 0.13 m 안 정지, 끝점을 3 cm 넘게 지나치지 않음, 이후 계속 (0, 0) |
| 반대 방향 출발 / R = 0.25 m 급커브 / 재계획 | 제자리 회전 후 추종 / \|ω\| ≤ 1.5 rad/s / 속도 유지 |

Gazebo 1회 측정은 5-8절. 남은 것: 위치추정(EKF/AMCL)을 켠 하네스 시나리오로 반복 측정하고 Pure Pursuit과
Stanley를 비교한다(`meta.json` `controller`). S-curve 프로파일은 그 다음.

### 5-6. speed_limit 연동

적용 위치는 `amr_control`(`path_tracker_node`)이 아니라 `amr_zone_client`다 — Pure
Pursuit·Stanley·PID·DWA 등 어떤 컨트롤러를 쓰든 그 출력이 전부 이 노드를 거쳐 나가므로,
컨트롤러마다 따로 챙기지 않고 한 곳에서만 반영하면 된다(#69·#70).

`amr_zone_client`는 `safety/status`(`amr_msgs/msg/SafetyStatus`)의 `speed_limit`을 받아
구역 진입 상한과 `min(구역 상한, speed_limit)`으로 합치고, 상한이 낮아질 때만 1.0 m/s²
(구역 진입 감속과 같은 `decel_mps2`) 이내로 부드럽게 따라간다. 값 정의(`inf`/0.0/WARNING
등)는 [docs/api.md](../api.md)를 참조한다.

`safety/status`가 끊기면 제한 없음으로 본다. 최종 게이트가 `amr_safety`라서다.

### 5-7. 선속도 PID

추종기가 낸 선속도(목표)를 odom 선속도가 따라가도록 명령을 보정한다. 코어 `Pid`(`pid.hpp`),
노드에서 `velocity_pid.enabled`로 켠다(**기본 끔** — 끄면 추종기 출력을 그대로 낸다).

```
u = v_ref + kp·e + ki·∫e dt − kd·d(v_meas)/dt,   e = v_ref − v_meas,   u ∈ [0, max_linear_speed]
```

- **앞먹임(v_ref)**: Gazebo DiffDrive나 실제 모터 드라이버가 이미 바퀴 속도를 맞추므로 PID는 그
  바깥에서 남는 오차(적재 질량·바닥 마찰로 덜 나가는 몫)만 메운다. 게인이 모두 0이면 u = v_ref다.
- **측정값 미분**: 목표가 계단으로 바뀔 때 미분항이 튀지 않게(derivative kick) 측정값을 미분하고,
  odom 잡음 때문에 1차 저역통과(`derivative_filter_tau`)를 거친다.
- **anti-windup**: 출력이 상·하한에 걸린 채 오차가 그 방향으로 더 미는 동안은 적분하지 않는다
  (conditional integration). 적분항은 `integral_limit`으로도 자른다.
- 제자리 회전·정지·odom 끊김이면 PID 상태를 비운다(멈춘 동안 적분이 쌓이지 않게).
- ω는 보정하지 않는다. PID가 맞추려는 것은 실제 선속도가 v_ref가 되는 것이라, 그때 실제 곡률이 ω/v_ref로 추종기 의도와 같다.

| 파라미터 | 값 | 근거 |
| --- | --- | --- |
| `kp` | 0.5 | 1차 플랜트(시정수 0.2 s, 정상상태 이득 0.8) 계단 응답에서 넘침 없이 2% 정착 1.9 s (`test_pid`) |
| `ki` | 1.0 [1/s] | 이득 0.8 플랜트의 정상상태 오차(앞먹임만이면 20%)를 없앤다 |
| `kd` | 0.0 | 플랜트가 1차라 필요 없고, odom 미분 잡음만 더한다 |
| `integral_limit` | 0.3 m/s | 순항 0.5 m/s에서 플랜트 이득 0.63까지 메울 수 있는 크기 (0.5·(1/0.63 − 1) ≈ 0.3) |
| `derivative_filter_tau` | 0.05 s | 제어 주기(0.05 s)와 같게. `kd`를 켤 때만 의미 있다 |

게인은 단위 테스트 플랜트로 정한 초기값이다. Gazebo DiffDrive는 바퀴 속도를 이미 맞추고 있어서
(5-8절 측정은 PID 끔) 적재 질량을 바꾼 실측 비교로 다시 정한다.

### 5-8. Gazebo CTE 측정 (1회, 10/7)

[experiments/cte_baseline](../../experiments/cte_baseline/README.md). 월드 v0, 로봇 1대, Nav2 없이
고정 경로를 퍼블리셔로 줬다. Pure Pursuit 기본 파라미터, PID 끔, 0.5 m/s.

- 경로(odom 프레임, 출발 (0, 1.5) 월드): 직선 3 m → 왼쪽 반원 R = 1 m → 돌아오는 직선 3 m (점 간격 5 cm)
- 추종기 입력은 DiffDrive의 이상적 odom(`diff_drive/odom`), EKF·AMCL 없음. `cmd_vel_nav`를 `cmd_vel`에 바로 이었다
- CTE 두 가지: `cte.csv`(추종기가 본 odom 위치 기준)와 GT(`ground_truth/odom`을 경로에 다시 투영)

| 기준 | 구간 | 샘플 | 평균 [cm] | RMSE [cm] | 최대 [cm] |
| --- | --- | --- | --- | --- | --- |
| `cte.csv` (odom) | 직선 | 230 | 0.28 | 0.69 | 2.40 |
| `cte.csv` (odom) | 곡선 | 137 | 0.69 | 1.06 | 2.39 |
| GT | 직선 | 1233 | 9.68 | 14.55 | 31.04 |
| GT | 곡선 | 647 | 3.60 | 4.12 | 8.00 |

GT 직선이 큰 이유를 경로 순서별로 나눠 보면 추종기가 아니라 **odom 누적 오차**다.

| 구간 | CTE odom 평균 / 최대 [cm] | CTE GT 평균 / 최대 [cm] | odom–GT 거리 평균 / 최대 [cm] |
| --- | --- | --- | --- |
| 직선 1 (곡선 전) | 0.05 / 1.26 | 0.04 / 1.22 | 0.00 / 0.05 |
| 곡선 | 0.62 / 2.39 | 3.60 / 8.00 | 4.81 / 11.68 |
| 직선 2 (곡선 후) | 0.64 / 2.40 | 19.53 / 31.04 | 21.09 / 31.96 |

- 추종기 자체(odom 기준)는 직선·곡선 모두 최대 2.4 cm로 KPI(직선 5 cm·곡선 10 cm) 안이다. odom과 GT가
  같은 직선 1에서는 GT CTE도 1.2 cm다.
- 반원을 도는 동안 DiffDrive odom의 방향이 GT보다 약 5° 틀어지고(바퀴 미끄러짐), 그 뒤 직선 2에서 위치
  차이가 32 cm까지 벌어진다. 추종기는 틀린 odom 위의 경로를 정확히 따라가므로 GT CTE가 그만큼 커진다.
- 그래서 **명세 CTE KPI는 odom만으로는 판정할 수 없다.** 위치추정(EKF + AMCL, map 프레임 경로)을 켠 상태에서
  GT 기준으로 재야 한다. `cte.csv`의 `actual_*`도 그때는 추종기가 쓴 추정 위치라, KPI 판정에는 GT 기준
  재계산(`analyze.py`)을 같이 본다.
- 1회 측정이라 신뢰구간은 없다. 하네스 시나리오로 만들어 반복할 때 정한다.

### 5-9. Gazebo CTE 측정 (위치추정 켬, 1회, 10/10)

5-8절의 결론(위치추정을 켠 상태에서 GT 기준으로 재야 한다)에 따라 다시 측정했다.
[experiments/cte_baseline](../../experiments/cte_baseline/README.md)의 `run_localized.sh`. 월드·지도는
그대로(월드 `warehouse.sdf`, 지도 v1 `warehouse_v1`, #96), Pure Pursuit 기본값·PID 끔·0.5 m/s도 5-8절과
같다. 추가된 조건:

- **위치추정**: `amr_localization` EKF 둘 + AMCL (`amr_bringup system.launch.py`, D4 A안). 추종기 odom은 둘 중 하나:
  - **local**: `odometry/local`(ekf_local, `<ns>/odom` 기준) - 경로도 `<ns>/odom` 프레임
  - **filtered**: `odometry/filtered`(ekf_global, `map` 기준, AMCL 보정) - 경로도 `map` 프레임
    (`path_tracker_node`는 경로 프레임이 odom 메시지 프레임과 다르면 TF로 돌아가 버려서, 두 프레임을
    맞춰야 실제로 그 추정을 쓴다 - `robotPose()` 참고)
- **GT 비교 보정**: #98 지도 정렬(`amr_mapping` README "지도 정렬"절)이 찾은 `map_origin_in_world` 보정값
  `[-0.0571, 1.5167, 0.00013]`(v1 0.05 m)을 GT 재투영에 적용/미적용 둘 다 본다(시뮬레이션은 한 번만 돌리고
  `analyze.py --correction`으로 같은 `gt.csv`에서 두 버전을 낸다)

| odom 소스 | #98 보정 | 구간 | 평균 [cm] | 최대 [cm] | KPI(직선 5·곡선 10 cm) |
| --- | --- | --- | --- | --- | --- |
| local | 미적용 | 직선 | 0.64 | 3.11 | 충족 |
| local | 미적용 | 곡선 | 2.02 | 2.89 | 충족 |
| filtered | 미적용 | 직선 | 2.22 | **8.60** | **미충족** |
| filtered | 미적용 | 곡선 | 2.43 | 5.42 | 충족 |
| filtered | 적용 | 직선 | 2.07 | 4.58 | 충족 |
| filtered | 적용 | 곡선 | 2.85 | 4.18 | 충족 |

#98 보정은 map→world 보정이라 map 기준(filtered)에만 적용한다.

- **5-8절(위치추정 없음, GT 직선 최대 31 cm) 대비**: 위치추정을 켜면 EKF가 odom 누적 오차(특히 반원을 돈
  뒤 방향이 틀어지는 것)를 IMU·AMCL로 계속 보정해서, local 3.11 cm·filtered(#98 보정) 4.58 cm로 거의
  10배 줄었다 - 둘 다 KPI(직선 5 cm) 안이다.
- **filtered는 #98 보정이 KPI 충족을 가른다.** SLAM 지도가 월드보다 약 6 cm 밀려 그려진 만큼이 AMCL
  추정(map 프레임)에 그대로 들어가, 보정 없이 GT와 비교하면 직선 최대 8.60 cm로 KPI를 넘긴다. 보정하면
  4.58 cm로 충족한다. **AMCL 기반(map 프레임) CTE KPI 판정에는 #98 보정이 필요하다.**
  local(odom 프레임, map과 무관)은 보정 여부와 무관하게 충족한다 - 보정은 지도-월드 정렬 문제라
  odom 프레임 추정과는 원래 상관없다(참고로 같이 재 둔 것이다).
- 경로 순서별로 보면 filtered·미보정에서도 직선 1(곡선 전) 최대 1.25 cm, 직선 2(곡선 후) 최대 8.60 cm로
  5-8절과 같은 비대칭(곡선을 돈 뒤 더 틀어짐)이 남아 있다 - AMCL이 있어도 완전히는 못 지운다. #98 보정
  적용 뒤에는 1(2.95)·2(4.58)로 격차가 줄어든다.
- local 실행의 odom(ekf_local)–GT 거리가 직선 1에서 한때 최대 100 cm까지 튀었다(평균은 21.5 cm, 나머지
  구간은 2~3 cm) - CTE 자체(odom·GT 둘 다 경로에 가깝게 투영됨, 최대 3 cm대)에는 영향이 없어 경로를
  따라간 위치가 살짝 앞서거나 뒤처진 것으로 보인다. IMU 바이어스 보정(시작 2 s 정지) 직후 과도 상태로
  추정 - filtered 실행에서는 재현되지 않았다(직선 1 최대 1.49 cm). 반복 측정 전까지는 참고로만 둔다.
- 1회 측정이라 신뢰구간은 없다.

## 6. 구역 진입 처리 (Zone Client)

`amr_zone_client`(이진)가 `ZoneRequest`/`ZoneGrant`로 `amr_fleet_manager`의 구역 허가 서버(#57)와
주고받는다. 평소 상태 기계(`IDLE`→`APPROACHING`→`INSIDE`/`BLOCKED`, 재시도·반납 규칙)는
[amr_zone_client README '동작 규칙' 절](../../src/navigation/amr_zone_client/README.md#동작-규칙-상태-흐름)에
있다. 여기서는 교착 해소(6-1)와 경로에서 잠깐 빠진 구역 처리(6-2)를 다룬다.

### 6-1. 교착 해소 (`GRANT_REVOKED`, 10/11 결정)

원 명세 7장·`amr_fleet_core.traffic.zones`의 교착 해소(대기 그래프 순환 탐지 → 우선순위 낮은 로봇의
허가 회수)에서, 회수당한 로봇이 실제로 "풀리는" 쪽 - 클라이언트 처리다. 서버가 `GRANT_REVOKED`
(`REASON_DEADLOCK_RESOLUTION`)를 보내면:

- **`APPROACHING`, 응답 시점에 아직 구역 밖**: 들어가지 않고 `BLOCKED`로 간다. `REJECTED`와 다른 점은
  이미 허가를 쥐고 있었다는 것 - 다음 틱에 `TYPE_RELEASE`를 한 번 보낸 뒤 `BLOCKED`의 재시도
  규칙(`retry_period_s` 뒤 새 `request_id`)을 그대로 따른다.
- **`APPROACHING`, 응답 시점에 이미 구역 안**(요청을 보내고 응답이 오기까지 한 틱 사이에 경계를
  넘어선 경우): `BLOCKED`가 아니라 `BACKING`으로 간다(#137 하연 두 번째 리뷰) - 반납하고 재시도해도
  이미 안에 있어서 의미가 없다.
- **`INSIDE`(이미 구역 안)**: 새 상태 `BACKING`으로 가서 **`cmd_vel_nav`를 완전히 무시**하고 직접
  후진 속도를 낸다(`backing_speed_mps`, 기본 0.15 m/s). `decel_mps2`로 가속하고, 구역 경계를 벗어난
  뒤 `backing_margin_m`(기본 0.3 m) 남았을 때부터 같은 감속으로 줄여 정확히 그 지점에서 0이 된다 -
  `path_tracker`의 정지 속도 프로파일(`speed_cap_for_stop`)을 "남은 후진 거리"에 그대로 재사용한
  것이다(5-0절과 같은 식). 닿으면 정지 → `TYPE_RELEASE` → `IDLE`.
- **다른 구역 잠금(#137 하연 두 번째 리뷰)**: 위 세 경우 모두, 그 구역이 복구를 마치고 다시 `IDLE`로
  돌아갈 때까지 다른 구역은 전부 건드리지 않는다 - 대기 중이던 요청은 그 즉시 반납하고 `IDLE`로
  돌리며, `IDLE`인 구역은 새로 트리거하지 않는다. 교착 해소 하나에 집중한다.
- **재전송(#137 하연 두 번째 리뷰)**: `BACKING`·`FAILED` 중에도 `resend_period_s`마다 같은
  `request_id`로 `TYPE_ACCESS`를 계속 보낸다 - 서버가 `last_seen`을 갱신해 구역을 다른 로봇에게
  넘기지 않게 한다. 서버는 어차피 같은 `GRANT_REVOKED`로 답할 뿐이고, `on_grant`가 `phase`로 걸러
  그냥 무시하므로 상태·속도에는 영향이 없다.
- **안전장치(fail-closed)**: 후진이 `backing_max_distance_m`(기본 6 m)나 `backing_timeout_s`(기본
  45 s)를 넘으면 그 순간 속도를 0으로 묶고 ERROR를 한 번 남긴 뒤 새 상태 `FAILED`로 바뀐다(#137
  지수 리뷰 - 10/11 결정은 `BACKING`에 계속 머무는 것이었는데, 그러면 상태 토픽만 보고는 "아직
  정상적으로 후진 중"인지 "한도를 넘어 멈춰 섰는지" 구분이 안 됐다). `FAILED`가 되면 반납·재시도
  없이 그대로 멈춰 선다(재전송은 계속한다) - 계속 후진하다 다른 사고로 이어지는 것보다, 서버가
  자리를 계속 쥐고 있더라도 멈춰서 수동 확인을 받는 쪽이 안전하다고 봤다. 두 기본값은 `zones.yaml`
  에서 가장 긴 구역(`narrow_02`, 5.0 m)을 기준으로 뒀다(#137 하연 두 번째 리뷰) - 거리는 그 길이를
  다 빠져나오고 `backing_margin_m` 여유를 더한 6.0 m, 시간은 `backing_speed_mps`(0.15 m/s)로 다
  빠져나오는 시간(≈33.3 s)에 여유를 더한 45 s다. 위 재전송 덕분에 더는 서버
  `release_timeout_s`(#124 `fleet_manager.yaml` 기본 30 s)보다 작아야 할 필요가 없다 - 재전송이
  `last_seen`을 계속 갱신하므로 서버가 "반납이 안 온다"고 구역을 넘겨버릴 일이 없다.
- TF 조회 실패는 평소 규칙(fail-closed, 속도 0)과 같다 - `BACKING` 중에도 그 틱만 0을 내고, 코어
  상태(후진 거리·경과 시간)는 그대로 둬 TF가 돌아오면 이어서 재개한다.

상태 토픽 `zone_client/state`(`std_msgs/String`, 바뀌면 즉시 + 1 Hz)로 `idle`/`approaching`/
`waiting`/`blocked`/`inside`/`backing`/`failed` 7가지를 낸다. `waiting`은 `Phase`가 아니라
`APPROACHING` 중 일부만의 라벨이다 - **허가가 없고 경계까지 남은 거리가
`stop_margin_m + waiting_margin_m`(기본 0.5 + 0.15 = 0.65 m) 이하일 때만** `waiting`이고, 그보다
멀면 (속도 상한만 걸린) `approaching`이다(#137 지수 리뷰 - 원래는 요청을 보내는
`trigger_distance_m`(3 m) 순간부터 바로 `waiting`이어서, 아직 속도만 깎이는 중인데도 "멈춰
기다리는 중"처럼 보였다). `waiting_margin_m`은 지수의 두 번째 리뷰로 0.5 → 0.15까지 더 줄였다 -
그 지점의 속도 상한(`speed_cap_for_stop`)이 거의 0(≈0.55 m/s)이어야 라벨에 맞다고 봤다. 구역이
여러 개면 가장 "급한" 것 하나로 요약하고, 우선순위는
`failed > backing > blocked > waiting > inside > approaching > idle`이다 - 예를 들어 구역 A에서
`inside`, 구역 B에서 `waiting`이면 전체 상태는 `waiting`이다.

**현재 한계**: 서버(`amr_fleet_core.traffic.zones.ZoneManager`)가 아직 교착을 감지하지 않아(순환 탐지
미구현) 실제로 `GRANT_REVOKED`를 보내는 경로가 없다 - 이 처리는 `test_core.py` 단위 테스트(합성
`on_grant` 호출)로만 확인했고, 서버가 실제로 보내는 end-to-end 확인은 서버 쪽 구현이 생긴 뒤에 한다.
`backing_margin_m`(0.3 m)은 경계에서 딱 그만큼만 나가고 멈춘다 - 로봇 앞끝이 경계에 거의 닿은 채로
서 있을 수 있어서, 양보받은 로봇이 지나가는 길을 다시 막는 경우가 생길 수 있다(#137 하연 두 번째
리뷰 지적). margin을 키울지 BT가 옆으로 비켜서게 할지는 10/12 회의에서 정한다 - 아직
`backing_margin_m` 자체는 그대로 뒀다.

### 6-2. 경로에서 잠깐 빠진 구역 (10/15 하연·이진, 10/18 하연 수정)

#66에서 정한 "경로에서 빠지면 즉시 반납"을 바꿨다. 교착 유도 실험(fleet_deadlock, 통로 ②를 서쪽·동쪽 구역으로
나눔)에서 상대 로봇이 동쪽 구역을 막자 Nav2가 잠깐 북쪽으로 돌아가는 경로를 냈고, 존 클라이언트가 동쪽 구역을
"경로에서 빠짐"으로 보고 반납·`IDLE`로 돌리면서 정지 상한이 사라져, 로봇이 허가(QUEUED) 없이 경계 0.34 m 안까지
들어갔다.

- 아직 들어가지 않은 구역(`APPROACHING`·`BLOCKED`)이 경로에서 빠져도, 로봇이 그 구역에서 `trigger_distance_m`
  (3 m, 직선 거리) 안이면 바로 반납하지 않는다. 같은 `request_id`로 계속 요청해 대기열 순서를 지키고, 정지
  상한은 로봇 → 구역 사각형 직선 거리로 건다(경로 거리보다 짧거나 같아 보수적이다).
- `drop_grace_s`(기본 3 s, `zone_client.yaml`) 넘게 계속 빠져 있거나 로봇이 trigger 거리 밖으로 멀어지면 그때
  반납(`BLOCKED`는 쥔 자리가 없어 반납 없이)하고 `IDLE`로 돌아간다.
- 그 전에 경로가 `lookahead_m`(기본 `trigger_distance_m + 2 m`) 안에서 다시 그 구역을 지나면 그대로
  이어간다(새 `request_id` 없음). `INSIDE` 구역은 바뀌지 않는다(`lookahead_m`과 무관하게 경계를 벗어나면
  반납).
- **"경로에 있음/없음" 판정 자체를 `lookahead_m`까지만 본다(10/18 하연 수정)**: 처음엔 #66 원래 결정대로
  `APPROACHING`·`BLOCKED`도 경로 전체(끝까지)에서 구역을 찾았는데, 그러면 뒤쪽 경유점에서나 다시 지나는
  먼 재방문까지 "경로에 있음"으로 잡혀 `drop_grace_s` 카운트가 영원히 시작되지 않거나(#189, "먼 재방문이
  계속돼 반납 안 되는 문제") 이미 세고 있던 카운트가 그 먼 재방문 때문에 매번 취소되는 문제(#177 회귀)가
  둘 다 있었다. `lookahead_m` 밖의 재방문은 "경로에 없음"으로 보게 좁히자 두 문제가 함께 풀렸다 -
  `drop_grace_s`가 이미 "잠깐 벗어남"을 봐주므로, 판정 범위를 줄여도 #66→#177이 고치려던 문제(우회 중
  반납)는 그대로 막힌다. 별도의 "다시 잡힌 뒤 일정 시간 유지돼야 카운트를 취소" 히스테리시스
  (`on_path_confirm_s`, 10/16 임시 수정)는 이 수정으로 더는 필요 없어 뺐다 - 판정 자체가 가까운 거리로
  좁혀져, 한 틱이라도 `lookahead_m` 안에서 잡히면 바로 신뢰할 수 있다.
- 함께 고친 것: 속도 상한은 `cmd_vel_nav`가 들어올 때만 적용돼, Nav2가 0이 아닌 명령을 낸 뒤 발행을 멈추면
  상한이 0이 돼도 마지막 명령이 남았다. 상한이 0으로 바뀌는 순간 `cmd_vel_zone_limited`에 0을 한 번 낸다.

## 7. 로컬 컨트롤러 비교 (DWA vs TEB)

> 상태: 하네스 시나리오 `nav_compare`로 1차 측정 (10/12, 조합마다 3회). 원 명세 "DWA 직접구현 vs TEB".
> 실행·원자료·해석은 [experiments/teb_compare](../../experiments/teb_compare/README.md).

### 7-1. TEB 도입 (#110)

TEB(`teb_local_planner`)는 Humble apt 패키지가 없어 이미지가 `/opt/teb_ws` underlay에 소스로 빌드한다
(SHA 고정: `humble-devel` 630a22e, `costmap_converter` 0.1.2(9565858) + 로거 수정 b8a8179 - `ros2` 브랜치 최신은
Jazzy부터 있는 `cv_bridge.hpp`를 써서 Humble에서 안 된다). 10/8 초안에서 대안으로 적었던 MPPI는 TEB가 이렇게
들어와 쓰지 않는다. `nav2_params.yaml`의 `teb` 블록은 예제 값에 로봇 스펙(풋프린트 0.6 × 0.4 m 다각형, 2.0 m/s,
1.5 rad/s, 1.0 m/s²)을 넣었고, 동적 장애물은 `costmap_converter::CostmapToDynamicObstacles`(costmap 배경 차분 +
추적)로 본다. 데모 `nav2_dwa_demo.launch.py controller:=teb`로 DWA와 바꿔 띄운다.

### 7-2. 결과 (1차, 10/12)

| 컨트롤러 | 시나리오 | 성공/전체 | Nav2 도착 | 충돌 | 도착 시간 [s] (성공만) | Nav2 도착 시간 [s] (충돌 포함) | 최소 장애물 거리 [cm] | 경로 길이 [m] | CTE 평균 [cm] | CTE 최대 [cm] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| DWA | clear | 3/3 | 3/3 | 0/3 | 9.5 (9.2~9.8) | 9.5 (9.2~9.8) | — | 3.14 (3.14~3.14) | 1.3 (1.2~1.4) | 3.9 (3.7~4.2) |
| DWA | obstacle | 0/3 | 3/3 | 3/3 | — | 39.5 (33.0~43.7) | 0.0 (0.0~0.0) | 4.44 (3.67~5.18) | 30.8 (22.2~39.3) | 138.2 (96.3~169.4) |
| TEB | clear | 3/3 | 3/3 | 0/3 | 6.8 (5.6~8.8) | 6.8 (5.6~8.8) | — | 3.36 (3.32~3.42) | 2.7 (2.3~3.5) | 8.1 (5.6~13.0) |
| TEB | obstacle | 0/3 | 3/3 | 3/3 | — | 7.2 (7.2~7.3) | 0.0 (0.0~0.0) | 4.01 (3.97~4.05) | 7.1 (5.3~8.4) | 28.3 (27.4~29.0) |

평균(최소~최대). 목표 map (0,0) → (3.0, 1.5), 지도 v1, 위치추정 없음(map→odom 항등). obstacle은 경로 중점을 남북으로
왕복하는 장애물 1개(0.5 m/s, 반지름 0.25 m), 장애물이 북쪽 끝에서 출발할 때 목표를 보내 교차 시점을 맞췄다.
성공 = Nav2 SUCCEEDED이고 충돌(풋프린트-장애물 1 cm 미만) 없음. 도착 시간은 성공한 실행만, Nav2 도착 시간은 충돌한 실행도 포함한다(하네스 `nav.arrival_s`·`nav.reached_s`).

- 장애물 없으면 둘 다 3/3. TEB가 30% 빠르고(속도 한계는 같음 - 시간 최적화 대 DWA의 heading 가중치 0.8) DWA가 경로에 더 붙는다(CTE 최대 3.9 vs 8.1 cm)
- 왕복 장애물에는 둘 다 3/3 접촉. TEB는 앞질러 지나가다, DWA는 선 앞에서 기다리다 되돌아온 장애물에 맞는다. DWA는
  장애물 GT를 직접 받는데도(인식이 DWA에 유리) 못 피했다. 두 컨트롤러의 등속 예측이 왕복 장애물의 반환을 못 보는 것으로
  보이며, 이 시나리오로는 둘을 가릴 수 없다 - 한 번 가로지르기·멈췄다 가기로 나눠 다시 잰다
- 3회라 신뢰구간은 내지 않았다. 원자료는 로컬 `tools/harness/results/`(git 제외)

### 7-3. 비교 지표

모든 지표는 **GT**(`ground_truth/odom`)로 잰다. 추정 위치(odom·AMCL) 기준으로 재면 위치추정 오차가 섞인다
(5-8절에서 odom 기준 2.4 cm가 GT 기준 31 cm였다).

| 지표 | 정의 | 단위 | 기록 |
| --- | --- | --- | --- |
| 성공 여부 | `NavigateToPose` 결과 SUCCEEDED이고 충돌 0. 실패는 timeout·abort·충돌로 나눠 센다 | — | 실행마다 |
| 도착 시간 | 목표 수락 시각 → SUCCEEDED 결과 시각 (시뮬레이션 시간). 성공한 실행만 평균(`nav.arrival_s`). 충돌했어도 Nav2가 도착한 실행의 시간은 Nav2 도착 시간(`nav.reached_s`)으로 따로 낸다 | s | 실행마다 |
| 최소 장애물 거리 | 주행 중 매 GT 시점에서 **사각 풋프린트(0.6×0.4 m)** 와 가장 가까운 장애물 사이 거리의 최솟값. 동적 장애물은 각자의 GT(#58)와 충돌 반지름. 정적 장애물(월드 SDF의 랙·벽·기둥)은 아직 안 잰다. 1 cm 미만이면 충돌로 본다(Gazebo에서 닿은 몸체는 겹치지 않고 0.1~1 mm로 붙는다) | m | 실행마다 최솟값(`nav_result.json`) + 시계열(`nav_series.csv`) |
| 경로 길이 | GT 위치 증분의 합. 같은 실행의 전역 경로 길이(NavFn) 대비 비율도 같이 | m, 비 | 실행마다 |
| CTE | GT 위치를 **그 시각의 전역 경로**(map 프레임)에 투영한 거리의 평균·최대. 직선·곡선 구간 분리는 아직 안 했다 | m | 시점마다 |

- CTE는 장애물 회피로 일부러 벗어난 구간까지 섞인다. 회피가 없는 구간(로봇 주변 1 m 안에 장애물이 없는
  시점)만 따로 집계하는 건 아직 안 했다(`nav_series.csv`로 다시 계산할 수 있다).
- 보조 지표(여유 있으면): 정지 횟수(선속도 < 0.01 m/s가 0.5 s 이상), 가가속도(jerk) RMS — 5절 PID·연구 2와 공유.
- 최소 장애물 거리는 LiDAR 최소 거리로 대신하지 않는다. 스캔은 풋프린트 모양을 모르고 노이즈가 있다.

### 7-4. 같은 시나리오 조건

| 항목 | 고정값 |
| --- | --- |
| 월드·지도 | 창고 월드 v0, 지도 v1(나오면). 위치추정 EKF + AMCL (map 프레임 경로) |
| 시작·목표 | 고정 쌍 목록(직선 위주, 곡선 포함, 좁은 통로 ①·② 통과 포함). 쌍마다 모든 컨트롤러가 같은 시작 자세로 출발 |
| 전역 계획 | NavFn 고정(재계획 주기 같게). 컨트롤러만 바꾼다 |
| Costmap | 같은 `nav2_params.yaml` costmap 설정(풋프린트·inflation 동일) |
| 속도·가속 한계 | 같은 값(최대 선속도·각속도·가속도). 비교 속도는 0.5 m/s 상한으로 맞추는 안 — 컨트롤러마다 순항 속도가 다르면 도착 시간이 속도 설정 비교가 된다 |
| 제어 주기·목표 판정 | `controller_frequency` 20 Hz, goal checker 같은 허용오차 |
| 장애물 | 정적만 / 동적 포함(#58) 두 조건. 동적은 실행마다 같은 seed 목록 |
| 반복 | 조건마다 N회(하네스 seed 규칙). 실행 단위 부트스트랩 신뢰구간 |
| 기록 | 하네스 `meta.json`의 `controller`에 컨트롤러 이름 |

1차 측정(7-2)에서 지킨 것: 지도 v1, NavFn 고정, 같은 costmap·풋프린트·속도 한계·제어 주기·goal checker, 장애물
없음/있음 두 조건, 하네스 `meta.json` `params.controller`. 아직 안 한 것: 위치추정(EKF+AMCL) 켜기, 여러 시작·목표
쌍(좁은 통로·곡선), 0.5 m/s 상한 맞추기(지금은 둘 다 2.0 m/s 한계라 도착 시간에 순항 속도 차이가 들어간다), 정적
장애물 최소 거리, N회 반복 신뢰구간.

## 8. 경로 계획 · 모션 제어 KPI 판정 (#158, 10/16)

원 명세 4장(경로 계획)·5장(모션 제어) 필수 KPI의 1회 판정이다. 민지가 하네스 시나리오 `nav_kpi`(아래
8-1절)로 GCP에서 돌렸고(#158), 판정은 **path_tracker(Pure Pursuit, `controller=pp`) 기준**으로 하고
Nav2(DWA, `controller=nav2`)는 참고로만 본다 - CTE KPI(5장)는 애초에 Pure Pursuit이 맞는 몫이라 그걸로
판정하기로 했다(10/13 결정, `nav_kpi.yaml` 머리말과 같음). 7-4절 "같은 시나리오 조건"은 컨트롤러끼리의
비교이고, 여기는 KPI 통과 여부를 가리는 판정이라 축이 다르다.

### 8-1. 측정 조건

- 하네스 `nav_kpi`(`tools/harness/scenarios/nav_kpi.yaml`), GCP, RTF(real-time factor) 1.0, 50쌍 1회
  실행(반복 없음 - 신뢰구간 없음, 5-8·5-9절과 같은 한계)
- seed 1, 지도 v1 0.025 m(`warehouse_v1_0025`), 위치추정 EKF + AMCL(map 프레임 경로)
- 순항 속도 0.5 m/s(DWA도 `config_patch`로 같게 맞춤, `DWA.max_v: 0.5`), 가감속 1.0 m/s²
- 원자료(GCP 버킷): `runs/nav_kpi-nav_pp/…_38607783484/`(PP), `runs/nav_kpi-nav_nav2/…_38612048525/`(DWA)
- 측정 구역(map 기준, #207·10/16 합의): x −10~15 m, y −15~5 m —
  `src/localization/amr_mapping/tools/draw_kpi_zone.py`의 `ZONE`(지도 휨이 작은 구역, 3-1절 그림과 같음).
  `cte.csv`에는 위치가 없어, 같은 쌍의 `gt_track.csv`에서 시각이 가장 가까운 점으로 구역 안/밖을 가른다 -
  `gt_track.csv`는 5 Hz, `cte.csv`는 10 Hz(`cte_hz`)라 최악 ~0.05 s 어긋나는데, 순항 0.5 m/s 기준 최대
  ~2.5 cm 위치 오차다. 재계산 스크립트: `experiments/nav_kpi/recompute_zone_cte.py`

### 8-2. 정의 (10/13 하연·이진 합의, `experiments/nav_kpi/navkpi_core.py`·`nav_kpi.py` 구현)

| 지표 | 정의 |
| --- | --- |
| 계획 성공률 | 출발·목표 50쌍 중 `ComputePathToPose`가 경로를 낸 비율 |
| 재계획 | 1초(`replan_period_s`)마다 로봇의 현재 위치에서 같은 목표로 `planner_server`(`ComputePathToPose`)를 다시 불러 걸린 시간(ms) - P95·최대로 본다 |
| 예측 오차 | 사다리꼴(순항 속도까지 못 올라가는 짧은 경로는 삼각형) 속도 프로파일로 낸 예측 시간(경로 길이 ÷ 순항 속도 0.5 m/s + 가감속 1.0 m/s² 보정) 대비 \|실제 − 예측\| / 실제. 도착한 쌍만 평균·중앙을 낸다 |
| CTE | GT 위치를 그 시각의 전역 경로에 투영한 수직 거리. 투영점의 경로 곡률이 0.2 1/m(`curve_kappa`) 넘으면 곡선, 아니면 직선으로 나눠 평균(5-0절 CTE 정의와 같음) |
| 복구 제외 | `NavigateToPose` 복구 동작(spin·backup·wait)이 도는 중과 끝난 뒤 2초(`recovery_grace_s`)는 CTE·예측 오차 양쪽에서 뺀다 |

### 8-3. 결과 (10/16, #158; 측정 구역 기준 재계산은 10/26)

| 지표 | 기준 | PP (판정) | 측정 구역 기준 (PP) | DWA (참고, 0.5 m/s 상한) |
| --- | --- | --- | --- | --- |
| 계획 성공률 | ≥ 98% | 50/50 (100%) | — | 50/50 (100%) |
| 도착 | — | 50/50 (100%) | — | 44/50 (88%) |
| 재계획 P95 / 최대 | ≤ 500 ms | 15 / 20 ms | — | 15 / 935 ms |
| 예측 오차 평균 / 중앙 | ≤ 15% | 5.3% / 4.5% | — | 58.5% / — |
| CTE 직선 평균 | ≤ 5 cm | **6.3 cm** | **3.71 cm** (n=944/4359) | 6.2 cm |
| CTE 곡선 평균 | ≤ 10 cm | 6.7 cm | 3.18 cm (n=1554/5962) | 6.5 cm |

측정 구역 기준 n은 "구역 안 샘플 수/도착한 쌍의 전체 샘플 수"(8-1절 정의, `pair_success`만). 50쌍 중 이
구역을 지나는 쌍은 16개다. 계획 성공률·재계획·예측 오차는 구역을 나눌 대상이 아니라(위치가 아니라 쌍·시도
단위 지표) "—"로 둔다.

50쌍 1회, CI 없음(8-1절 한계와 같음).

### 8-4. 판정

**측정 구역 기준(PP, 판정)으로 5개 모두 통과** - CTE 직선 3.71 cm·곡선 3.18 cm 둘 다 기준 안이다(10/16
결정대로 측정 구역을 명시하는 B안, 8-5절). 구역 밖까지 포함한 전체 CTE 직선은 **6.3 cm로 기준(5 cm)을
넘는다** - 원인은 추종기 오차가 아니라 구역 밖(창고 서쪽·북쪽 끝)의 지도 휨(#183, 8-5절)이라, 전체 값은
참고로 남기고 구역 기준으로 판정한다. 계획 성공률·재계획·예측 오차는 여유 있게 통과하고(재계획 최대
20 ms는 기준의 1/25, 예측 오차 5.3%는 기준의 1/3), 구역 기준 CTE 곡선(3.18 cm)도 기준의 1/3 수준이다.

DWA(참고)는 예측 오차(58.5%, 기준의 약 4배)·도착률(88%, 기준이 요구하는 98%에 못 미침)·재계획
최대(935 ms, 기준 초과)가 크게 벗어난다 - 10/13 합의대로 판정에는 쓰지 않지만, 원인은 8-5절에 남긴다.

### 8-5. 미달 항목 원인

- **CTE 직선 6.3 cm (PP)**: 지도 휨(#183, 민지). 지도 정렬(B안, #98, amr_mapping README "지도 정렬"절)
  뒤에도 시작점 근처는 위치 오차 2~5 cm지만 창고 서쪽·북쪽 끝은 14~22 cm까지 벌어진다 - B안(강체 정합
  1점 보정)은 지도 전체를 한 번에 옮기는 보정이라, 지도 안쪽이 휘어 있는 것 자체는 못 고친다. 경로
  따라가기 자체(추종기 오차, odom·GT 둘 다 경로에 가까운 5-8·5-9절 결과와 같은 성격)는 1.5 cm로
  KPI(5 cm) 안이다 - 6.3 cm의 대부분이 추종이 아니라 위치추정이 보는 지도가 휜 데서 온다.
  **10/16 결정**: 재매핑은 논문 진행 중에 할 여유가 없어 B안(지도 휨이 작은 측정 구역을 명시해 그
  안에서 KPI를 판정)으로 가고, 재매핑은 논문 제출 뒤로 미룬다.
- **DWA 예측 오차 58.5% (참고)**: 방향 기준점을 고정해 두는 lookahead 때문에 실제 최고 속도가 약
  0.25 m/s로 묶여 있었다(명령 0.5 m/s의 절반) - 예측은 0.5 m/s 기준인데 실제로는 그 속도를 못 내서
  오차가 커졌다. #188로 고쳤고(Gazebo 실측 0.246 → 0.391 m/s), 머지 후 재측정할 예정이다.

