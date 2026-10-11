# amr_planning_core

- 담당: 이진
- 빌드 타입: ament_cmake (C++17)
- 책임: A*, 경로 평활화, DWA 핵심 로직 (ROS 비의존 라이브러리)

## 공개 API

ROS 토픽을 직접 다루지 않는다. 격자와 셀 좌표만 다루는 순수 C++ 라이브러리다.

### `amr_planning_core::Grid` (`grid.hpp`)

2D 점유 격자의 읽기 전용 뷰. 비용 버퍼(`uint8_t*`)를 **소유하지 않는다** — 2주차에
`amr_nav2_plugins`에서 nav2 costmap의 `getCharMap()` 버퍼를 복사 없이 그대로 감싸기
위한 설계다. `plan()` 호출이 끝날 때까지 버퍼는 호출자가 살려둬야 한다.

```cpp
amr_planning_core::Grid grid(width, height, resolution, costmap_buffer_ptr);
grid.inBounds({x, y});
grid.cost({x, y});  // 범위 밖이면 std::out_of_range
```

### `amr_planning_core::Cell` (`grid.hpp`)

격자 좌표 `{int x; int y;}`. `==`, `!=` 연산자 제공.

### `amr_planning_core::AStarConfig` (`astar.hpp`)

튜닝 파라미터 struct. 하드코딩 없이 생성자 인자로 넘긴다.

| 필드 | 기본값 | 설명 |
| --- | --- | --- |
| `heuristic` | `kOctile` | `kEuclidean` \| `kOctile` |
| `lethal_cost_threshold` | `253` | 이 값 이상인 셀은 통과 불가 (nav2 관례: `INSCRIBED_INFLATED_OBSTACLE=253`, `LETHAL_OBSTACLE=254`) |
| `cost_weight` | `0.0` | 셀 비용을 이동 비용에 반영하는 가중치. 0이면 순수 거리 기반 |
| `prevent_corner_cutting` | `true` | 대각선 이동 시 모서리 뚫기 방지 |

### `amr_planning_core::AStarPlanner` (`astar.hpp`)

```cpp
amr_planning_core::AStarConfig config;
config.cost_weight = 2.0;
amr_planning_core::AStarPlanner planner(config);

const amr_planning_core::PlanResult result = planner.plan(grid, start_cell, goal_cell);
if (result.status == amr_planning_core::PlanStatus::kSuccess) {
  // result.path: std::vector<Cell>, start부터 goal까지 순서대로
}
```

`PlanStatus`: `kSuccess`, `kStartOutOfBounds`, `kGoalOutOfBounds`, `kStartOccupied`,
`kGoalOccupied`, `kNoPathFound`.

### `amr_planning_core::DwaPlanner` (`dwa.hpp`)

DWA(Dynamic Window Approach) 로컬 컨트롤러 코어. 정적 장애물(`Grid`)은 하드
필터(충돌 궤적 제외)로, 동적 장애물(`DynamicObstacle`, 3주차)은 등속 예측 위치를
하드 필터와 소프트 비용 둘 다에 반영한다. 자세한 설계는
[docs/algorithms/navigation.md](../../../docs/algorithms/navigation.md#4-dwa-dynamic-window-approach) 4절 참고.

`RobotState`의 `(x, y)`는 `Grid`와 같은 로컬 좌표계(미터, 셀 `(0,0)`의 모서리가
원점)다 — `Grid`가 원점을 들고 있지 않은 설계와 같은 이유로, odom/map 프레임
변환은 이 라이브러리를 쓰는 노드(`amr_nav2_plugins`)가 담당한다.

```cpp
amr_planning_core::DwaConfig config;  // 기본값은 로봇 스펙 + nav2_params.yaml과 맞춤
amr_planning_core::DwaPlanner planner(config);

const amr_planning_core::RobotState state{/*x=*/1.0, /*y=*/1.0, /*yaw=*/0.0, /*v=*/0.5, /*w=*/0.0};
const amr_planning_core::Point2D goal{3.0, 1.0};

const amr_planning_core::DwaCommand command = planner.computeVelocityCommand(state, goal, grid);
if (command.status == amr_planning_core::DwaStatus::kOk) {
  // command.v, command.w를 다음 제어 주기에 낸다
}

// 동적 장애물(perception/obstacles 기반)은 네 번째 인자(기본값 빈 벡터)로 넘긴다.
// 빈 벡터면 위 호출과 결과가 완전히 같다(2주차 정적 장애물 전용 동작과 동일).
std::vector<amr_planning_core::DynamicObstacle> obstacles{{/*x=*/2.0, /*y=*/1.0,
  /*vx=*/-0.5, /*vy=*/0.0, /*radius=*/0.3}};
planner.computeVelocityCommand(state, goal, grid, obstacles);
```

`DwaStatus`: `kOk`, `kGoalReached`(목표까지 `goal_tolerance` 이내), `kNoValidTrajectory`
(모든 후보가 충돌해 낼 수 있는 속도가 없음).

충돌 검사는 로봇 풋프린트를 **외접원이 아니라 사각형**(`footprint_half_length`,
`footprint_half_width`)으로 한다 — 외접원(지름 0.72m)으로 하면 0.6m 좁은 통로를
항상 "막힘"으로 오판한다.

| 필드 | 기본값 | 설명 |
| --- | --- | --- |
| `max_v` / `min_v` | `2.0` / `0.0` m/s | 로봇 최대/최소 속도 |
| `max_w` | `1.5` rad/s | 로봇 최대 각속도 |
| `acc_v` / `acc_w` | `1.0` m/s² / `3.2` rad/s² | 가속 한계(동적 윈도우 계산에 사용) |
| `dt` / `sim_time` | `0.1` s / `2.0` s | 제어 주기 / 궤적 예측 길이 |
| `v_samples` / `w_samples` | `20` / `30` | 속도/각속도 샘플 개수 |
| `footprint_half_length` / `footprint_half_width` | `0.3` / `0.2` m | 로봇 풋프린트 반폭(600×400mm) |
| `lethal_cost_threshold` | `253` | `AStarConfig`와 동일한 치명 비용 임계값 |
| `heading_weight` / `clearance_weight` / `velocity_weight` | `0.8` / `0.1` / `0.1` | 비용 함수 가중치(근거는 navigation.md 4-3절) |
| `goal_tolerance` | `0.15` m | 이 거리 이내면 `kGoalReached` |
| `predict_dynamic_obstacles` | `true` | 동적 장애물을 등속 예측할지(`false`면 관측 위치에 정지한 것으로 본다) |
| `dynamic_obstacle_safety_margin` | `0.2` m | 장애물 표면에서 이 안쪽이면 충돌 |
| `dynamic_obstacle_influence_radius` | `1.0` m | 소프트 비용이 미치는 거리(표면 기준) |
| `dynamic_weight` | `0.3` | 동적 장애물 거리비용 가중치(나머지 세 가중치와 합이 1일 필요는 없다) |

## 파라미터

ROS 비의존 라이브러리이므로 파라미터는 `AStarConfig` 생성자 인자로 받는다. YAML 로딩은
이 라이브러리를 쓰는 노드 패키지(`amr_nav2_plugins`)가 `amr_bringup/config`에서 담당한다.

## 실행

노드가 아닌 라이브러리라 단독 실행 파일은 없다. 다른 패키지에서 링크해서 쓴다.

## 테스트

```bash
colcon test --packages-select amr_planning_core
```
