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

## 파라미터

ROS 비의존 라이브러리이므로 파라미터는 `AStarConfig` 생성자 인자로 받는다. YAML 로딩은
이 라이브러리를 쓰는 노드 패키지(`amr_nav2_plugins`)가 `amr_bringup/config`에서 담당한다.

## 실행

노드가 아닌 라이브러리라 단독 실행 파일은 없다. 다른 패키지에서 링크해서 쓴다.

## 테스트

```bash
colcon test --packages-select amr_planning_core
```
