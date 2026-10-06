# 경로 계획 · 모션 제어 (A* · DWA · Costmap)

> 상태: 진행 중 (2주차). 담당: 이진. Costmap(1절)·DWA(4절) 설정·구현 근거를 정리한다.
> A* 알고리즘 설명(3절), Pure Pursuit/Stanley/PID(5절)는 아직 TODO.
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
| ② | `rack_lane_divider`–B열 사이, x −2.5~2.5, y −1.4~−0.8 | 0.6 × 5.0 m | 여러 미터를 좁은 폭으로 유지 주행하는지 |

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
   벗어날 수 있다고 보고, 그 범위를 다시 로봇 최대 속도로 자른다.
2. **속도 샘플링 + 궤적 시뮬레이션** (`simulateTrajectory`): 윈도우 안의 (v, w) 조합마다
   차동구동 전진 운동학으로 `sim_time` 동안 `dt` 간격으로 궤적을 찍는다.
3. **비용 함수** (`evaluateCost`): heading(목표 방향) + clearance(장애물 거리) +
   velocity(속도)를 각각 [0, 1]로 정규화해 가중합한다.
4. **충돌 궤적 제외 후 최소 비용 선택** (`computeVelocityCommand`): 궤적 위 모든 점에서
   사각 풋프린트가 치명 비용 셀과 겹치면 그 (v, w)는 후보에서 뺀다. 남은 후보 중 비용이
   가장 낮은 (v, w)를 반환한다. 전부 충돌하면 `kNoValidTrajectory`.

### 4-2. 파라미터 (`DwaConfig`, 기본값)

| 파라미터 | 값 | 근거 |
| --- | --- | --- |
| `max_v` / `min_v` | 2.0 / 0.0 m/s | `nav2_params.yaml` `FollowPath.max_vel_x`와 동일(로봇 스펙) |
| `max_w` | 1.5 rad/s | `FollowPath.max_vel_theta`와 동일 |
| `acc_v` | 1.0 m/s² | `FollowPath.acc_lim_x`와 동일 |
| `acc_w` | 3.2 rad/s² | `FollowPath.acc_lim_theta`와 동일 |
| `dt` | 0.1 s | 제어 주기 |
| `sim_time` | 2.0 s | 궤적 예측 길이. 짧게 잡으면 장애물을 더 가까이 와서야 "미래 충돌"로 감지해서 막판에야 급히 피하려다, 동적 윈도우(한 스텝에 꺾을 수 있는 각속도, `acc_w*dt`뿐)가 모자라 못 피하는 상황이 생긴다 — 테스트 중 실제로 발견해서 `sim_time`은 줄이지 않기로 했다(계산량은 샘플 개수로 조절) |
| `v_samples` | 20 | `nav2_params.yaml` `FollowPath.vx_samples`와 동일 |
| `w_samples` | 30 | `FollowPath.vtheta_samples`(20)보다 많다 — 회전 각속도 해상도가 좁은 통로 회피 성공률에 더 민감하다고 보고 촘촘히 샘플링했다. TODO(3주차): 실측 KPI로 `nav2_params.yaml`과 맞출지 재검토 |
| `footprint_half_length` / `half_width` | 0.3 / 0.2 m | 로봇 스펙(600×400mm)과 동일. **외접원이 아니라 사각형으로 충돌 검사**한다 — 외접원 지름(0.72m)으로 하면 0.6m 좁은 통로(1-1절)를 항상 "막힘"으로 오판한다 |
| `lethal_cost_threshold` | 253 | `AStarConfig`와 동일한 nav2 costmap 관례 |
| `heading_weight` / `clearance_weight` / `velocity_weight` | 0.8 / 0.1 / 0.1 | 4-3절 |
| `goal_tolerance` | 0.15 m | 전역 경로의 다음 지점 추종이 목적이라 각도 허용오차는 두지 않았다(최종 도킹 정밀도는 범위 밖) |

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

### 4-5. 이번 주 범위와 3주차 범위

| 구분 | 이번 주(2주차) | 3주차 |
| --- | --- | --- |
| 장애물 | 정적 장애물만(`Grid` 비용값) | `amr_msgs/TrackedObstacleArray` 기반 동적 장애물을 비용 함수에 반영(연구 2, 4장의 AKF·확률적 TTC와 연결) |
| 연결 | ROS 비의존 core만(`amr_planning_core`) | `amr_nav2_plugins`에서 `nav2_core::Controller`로 감싸 `controller_server`에 꽂는다(1주차 DWB baseline을 교체) |
| 비교 | — | DWB·TEB와 CTE·정지 횟수·경로 길이로 비교(원 명세 "DWA vs TEB") |

## 5. Pure Pursuit · Stanley · PID (예정)

TODO(3주차 이후): Pure Pursuit/Stanley 적응형 look-ahead 설계, PID 게인 튜닝 과정을
작성한다.

### 5-1. speed_limit 연동

제어 쪽은 `safety/status`(`amr_msgs/msg/SafetyStatus`)의 `speed_limit`을 받아 명령
선속도를 `min(명령, speed_limit)`으로 제한하고, 1.0 m/s² 이내로 감속한다. 값 정의
(`inf`/0.0/WARNING 등)는 [docs/api.md](../api.md)를 참조한다.

`safety/status`가 끊기면 제한 없음으로 본다. 최종 게이트가 `amr_safety`라서다.
