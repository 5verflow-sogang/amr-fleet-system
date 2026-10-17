# amr_fleet_core

- 담당: 하연
- 빌드 타입: ament_python
- 책임: 작업 할당(최소거리/부하균형/Hungarian), 구역 허가, 경로 충돌 예측, 교착 탐지 (ROS 비의존 라이브러리)

## 공개 API

ROS 토픽을 직접 다루지 않는다. 공개 함수·클래스 목록을 여기에 적는다.

### 작업 주인 (`claims.TaskClaims`)

같은 작업을 두 로봇이 맡았을 때 누가 계속할지 정한다. COMPLETED를 보고한 로봇, 아니면 `TaskStatus.start_time`(수락 시각)이 가장 이른 로봇이 주인이다. RobotState가 `timeout_s` 동안 끊기거나 그 뒤 RobotState가 다른 작업을 하면 그 로봇의 주장은 뺀다. 서버(`amr_fleet_manager`)·가짜 로봇·경매 노드(`amr_fleet_decentralized`)가 같이 쓴다 ([architecture.md 5-2절](../../../docs/architecture.md#5-2-하이브리드-폴백-연구-1-비교군-1016-하연-설계)).

| 메서드 | 뜻 |
| --- | --- |
| `observe(task_id, robot_id, state, start, now)` | `TaskStatus` 1건 |
| `robot_state(robot_id, current_task_id, now)` | `RobotState` 1건 |
| `owner(task_id, now, exclude='')` | 주인 robot_id 또는 None |
| `rival(task_id, robot_id, start, now)` | robot_id(수락 시각 start, 수락 전이면 None)보다 먼저 맡은 다른 로봇 |
| `completed(task_id, robot_id)` | 그 로봇이 COMPLETED를 보고했는지 |

### 작업 할당 (설계 초안)

> 상태: 알고리즘 3종 구현 완료(2주차). 기본 알고리즘 선택과 비교 실험은 아래 미결 참고. 호출 흐름은 [architecture.md 5-1절](../../../docs/architecture.md#5-1-작업-요청--할당--이동--완료-보고-중앙집중-모드) 4~5번이다.

#### 설계 원칙

- 세 알고리즘은 **같은 시그니처**를 쓴다. `amr_fleet_manager`는 파라미터 하나로 알고리즘을 바꾼다.
- 입력은 `amr_msgs`가 아니라 이 패키지의 dataclass로 받는다. msg → dataclass 변환은 `amr_fleet_manager`가 한다.
- 알고리즘은 **후보 로봇 거르기를 하지 않는다**. 할당 가능한 로봇만 넘겨받는다. 거르기는 별도 함수 `filter_candidates()`가 한다.
- 같은 입력이면 항상 같은 결과를 낸다 (하네스 반복 실험 재현성). 비용이 같으면 `robot_id`, `task_id`의 사전순으로 정한다.
- 한 번 호출에 로봇 1대는 작업 최대 1개를 받는다.

#### 모듈 구성 (안)

```
amr_fleet_core/assignment/
├── types.py          TaskInfo, RobotInfo, AssignmentResult
├── cost.py           euclidean_cost (기본 비용 함수)
├── common.py         filter_candidates, sort_by_priority, check_unique_ids
├── nearest.py        assign_nearest       (최소거리)
├── load_balance.py   assign_load_balance  (부하균형)
├── hungarian.py      assign_hungarian     (Hungarian)
└── __init__.py       ALGORITHMS = {"nearest": ..., "load_balance": ..., "hungarian": ...}
```

#### 입력

좌표는 `map` 좌표계 (m), 시각은 초 단위 실수다.

**`TaskInfo`** (출처: `amr_msgs/msg/Task`)

| 필드 | 타입 | 설명 |
| --- | --- | --- |
| `task_id` | `str` | 작업 ID |
| `task_type` | `int` | `Task.TYPE_*` |
| `pickup_xy` | `tuple[float, float] \| None` | 픽업 위치. CHARGE·RETURN은 `None` |
| `dropoff_xy` | `tuple[float, float]` | 목적지 |
| `priority` | `int` | 0~255, 클수록 급함 |
| `deadline` | `float \| None` | 마감 시각. 없으면 `None` |
| `created` | `float` | 요청 시각 |

속성 `start_xy`는 로봇이 처음 가야 할 곳이다: `pickup_xy`가 있으면 그것, 없으면 `dropoff_xy`.

**`RobotInfo`** (출처: `amr_msgs/msg/RobotState` + 서버 집계)

| 필드 | 타입 | 설명 |
| --- | --- | --- |
| `robot_id` | `str` | 예: `"amr_01"` |
| `xy` | `tuple[float, float]` | 현재 위치 |
| `battery_percent` | `float` | 0~100 |
| `mode` | `int` | `RobotState.MODE_*` |
| `load` | `int` | 부하균형용 누적 할당 수. `RobotState`에 없는 값이라 서버가 센다 |

**비용 함수** `CostFn = Callable[[RobotInfo, TaskInfo], float]`

- 기본값 `euclidean_cost`: 로봇 → `start_xy` 직선 거리.
- 픽업 → 목적지 구간은 어느 로봇이 맡아도 같으므로 할당 비용에 넣지 않는다. 총 이동거리 지표는 `TaskStatus.distance_traveled`로 따로 잰다.
- 나중에 Nav2 경로 길이 비용으로 바꿀 수 있도록 인자로 받는다.

#### 출력

**`AssignmentResult`**

| 필드 | 타입 | 설명 |
| --- | --- | --- |
| `assignments` | `dict[str, str]` | `task_id → robot_id` |
| `unassigned` | `list[str]` | 이번에 할당 못 한 `task_id`. 서버 큐에 남아 다음 호출로 넘어간다 |
| `total_cost` | `float` | 할당된 짝의 비용 합. 알고리즘 간 최적성 비교에 쓴다 |

#### 함수

```python
def filter_candidates(robots: Sequence[RobotInfo], min_battery_percent: float) -> list[RobotInfo]:
    """mode == IDLE 이고 battery_percent >= min_battery_percent 인 로봇만 남긴다."""

def assign_xxx(
    tasks: Sequence[TaskInfo],
    robots: Sequence[RobotInfo],
    cost_fn: CostFn = euclidean_cost,
) -> AssignmentResult: ...
```

- 작업이나 로봇이 비어 있으면 빈 결과를 돌려준다.
- `task_id` 또는 `robot_id`가 중복되면 `ValueError`를 낸다.

#### 알고리즘별 동작

세 알고리즘 모두 작업을 먼저 **우선순위 순서**로 정렬한다: `priority` 내림차순 → `deadline` 오름차순 (`None`은 맨 뒤) → `created` 오름차순 → `task_id`.

| 알고리즘 | 키 | 동작 | 복잡도 |
| --- | --- | --- | --- |
| 최소거리 | `nearest` | 우선순위 순으로 작업을 하나씩 꺼내, 남은 로봇 중 비용이 가장 작은 로봇에 맡긴다 (탐욕) | O(T·R) |
| 부하균형 | `load_balance` | 우선순위 순으로 작업을 꺼내, 남은 로봇 중 `load`가 가장 작은 로봇에 맡긴다. `load`가 같으면 비용이 작은 쪽 | O(T·R) |
| Hungarian | `hungarian` | 우선순위 순 상위 `min(T, R)`개 작업을 고른 뒤, 비용 합이 최소가 되게 한 번에 매칭한다 (`scipy.optimize.linear_sum_assignment`) | O(R³) |

- Hungarian에서 작업을 먼저 자르는 이유: 작업이 로봇보다 많을 때 비용만 보면 가까운 작업만 뽑혀 급한 작업이 굶는다.
- T = 대기 작업 수, R = 후보 로봇 수. Hungarian도 작업을 R개로 자르므로 R×R 행렬이고, 로봇 10대 기준 수십 µs다.
- 부하균형의 `load`는 함수 안에서 바꾸지 않는다. 할당 후 증가는 서버가 한다.

#### 호출 측 (`amr_fleet_manager`) 책임

| 항목 | 내용 |
| --- | --- |
| 호출 시점 | 새 작업이 들어올 때, 로봇이 IDLE로 돌아올 때 (`TaskStatus` COMPLETED·FAILED) |
| 입력 | 대기 큐의 작업 전체와 `filter_candidates()`를 거친 로봇 |
| 파라미터 (YAML) | `assignment.algorithm` (`nearest` / `load_balance` / `hungarian`), `assignment.min_battery_percent` |
| 후처리 | 결과마다 `TaskAssignment(ASSIGN)` 발행, 해당 로봇 `load += 1` |

#### 단위 테스트 (`test/test_assignment.py`)

- 탐욕이 최적이 아닌 2×2 예제에서 `hungarian.total_cost < nearest.total_cost`
- 작업 > 로봇, 로봇 > 작업, 빈 입력
- 비용이 같을 때 결과가 사전순으로 고정되는지
- 우선순위가 높은 작업이 로봇 부족 시에도 먼저 할당되는지

#### 미결

| 쟁점 | 초안 |
| --- | --- |
| 기본 알고리즘 | 3종 구현 후 같은 시나리오로 비교 실험(총 이동거리·작업 완료 시간)해 결정. 실험 설계는 민지와 합의. 연구 1은 이 값으로 고정 |
| 비용 함수 | 직선 거리로 시작, Nav2 경로 길이는 성능 보고 결정 |
| 배터리 하한 | 20% (파라미터) |
| `load`의 정의 | 누적 할당 수. 누적 이동거리로 바꿀지 검토 |
| 할당된 작업 재조정 | 하지 않는다. 한 번 맡긴 작업은 완료·실패·취소 전까지 유지 |

### 구역 허가 (`traffic/zones.py`)

> 상태: 정원·lease·우선순위 대기열·교착 해소 2종(허가 회수 / 회피) 구현(3주차).

`ZoneRequest`/`ZoneGrant`의 서버 측 판단만 한다. 메시지 변환과 발행은 `amr_fleet_manager`가 한다.

```python
zm = ZoneManager([ZoneSpec('narrow_01', capacity=1)], lease_s=5.0, release_timeout_s=30.0,
                 deadlock_strategy='revoke')  # 또는 'avoid'
zm.request(robot_id, zone_id, request_id, now, priority=0) -> list[Decision]  # 요청한 로봇에 대한 답 1건
zm.release(robot_id, zone_id, now) -> list[Decision]              # 자리가 나서 허가받은 대기 로봇의 답
zm.expire(now) -> list[Decision]                                  # 주기 호출. 만료 정리 후 대기 로봇 허가
zm.resolve_deadlocks(now) -> list[Decision]                       # 주기 호출. 교착마다 허가 1건 회수(REVOKED)
zm.wait_for() -> dict[str, dict[str, str]]                        # wait-for 그래프 [기다리는 로봇][점유 로봇] = 구역
```

| 상황 | 답 |
| --- | --- |
| 모르는 `zone_id` | `REJECTED` |
| 이미 허가받은 로봇이 다시 요청 (연장) | `GRANTED`, `lease_expiry = now + lease_s` |
| 자리가 있음 | `GRANTED` |
| 정원이 참 | `QUEUED` (`REASON_OCCUPIED`, `holder_robot_id` = 먼저 들어간 로봇), 대기열에 넣는다 |
| 정원이 차 있고 더 급한 로봇이 앞에서 기다림 | `QUEUED` (`REASON_PRIORITY_YIELD`, `holder_robot_id` = 그 로봇) |
| 점유 로봇이 반납 | 대기열에서 차례인 로봇(아래 순서)에게 그 로봇의 마지막 `request_id`로 `GRANTED` |
| 교착 (서로 상대가 점유한 구역을 기다림) | 우선순위가 가장 낮은 로봇에게 `REVOKED` (`REASON_DEADLOCK_RESOLUTION`, `holder_robot_id` = 양보받는 로봇). 반납 전까지는 구역을 넘기지 않는다 |
| 회수당한 로봇이 다시 요청 | 같은 `REVOKED` (연장하지 않는다. 회수 통보가 유실돼도 다시 알 수 있다) |
| 점유 로봇의 lease가 지남 | **넘기지 않는다.** 통로 안에서 만료되면 로봇은 빠져나간 뒤 반납한다 (이진 합의) |
| 점유 로봇이 `release_timeout_s` 동안 요청도 반납도 없음 | 죽은 것으로 보고 풀어 대기 로봇에게 넘긴다 |
| 대기 로봇이 `lease_s` 동안 다시 요청하지 않음 | 대기열에서 뺀다 (답 없음) |
| 반납 | 반납한 로봇에게는 답하지 않는다 |

- `now`는 로봇이 `lease_expiry`와 비교하는 시계와 같아야 한다. `amr_fleet_manager`는 노드 시계(`use_sim_time`이면 시뮬레이션 시간)를 넘긴다. 실제 시간으로 재면 시뮬레이터가 느릴 때 서버가 로봇보다 먼저 허가를 풀어 두 대가 동시에 들어갈 수 있다.
- 대기 순서(교차로 우선순위): 로봇이 맡은 작업의 `priority` 높은 순 → 대기열에 처음 들어온 시각 → `robot_id`. `ZoneRequest`에는 우선순위가 없어 `amr_fleet_manager`가 작업 장부에서 찾아 넘긴다 (작업이 없으면 0).

### 교착 탐지 (`deadlock/`)

구역 점유·대기로 wait-for 그래프를 만든다. 간선 A → B는 "A가 기다리는 구역을 B가 점유 중"이다. 순환이 있으면 교착이다 (예: 좁은 통로 양 끝에서 마주 본 두 대가 각자 한 구역을 점유하고 상대 구역을 기다림).

```python
find_deadlocks(edges) -> list[list[str]]       # 크기 2 이상인 강한 연결 요소(SCC). 그 안의 로봇은 모두 어떤 순환 위에 있다
choose_victim(robots, priority) -> str         # 물러날 로봇: 우선순위 가장 낮은 로봇, 같으면 robot_id가 가장 뒤
would_deadlock(edges, waiter, holders) -> bool # waiter가 holders를 기다리게 되면 순환이 닫히는지
```

해소 1 (허가 회수, `ZoneManager.resolve_deadlocks`):

1. 교착 묶음마다 물러날 로봇을 고르고, 묶음 안의 다른 로봇이 기다리는 그 로봇의 점유 구역 하나를 회수한다.
2. 회수당한 로봇은 구역에서 물러나 `TYPE_RELEASE`를 보낸다 (로봇 쪽 처리는 `amr_zone_client`). 기다리던 구역도 반납해 대기열에서 빠진다.
3. 서버는 반납이 오면 대기 순서대로 다음 로봇에게 허가한다. 회수당한 로봇이 죽어 반납이 없으면 `release_timeout_s` 뒤 풀린다.
4. 회수 중인 점유는 그래프에서 빼서 같은 교착을 두 번 풀지 않는다.

해소 2 (회피, `deadlock_strategy='avoid'`):

1. 구역이 차서 대기열에 넣기 전에, 이 대기로 순환이 닫히는지(`would_deadlock`) 본다.
2. 닫히면 대기열에 넣지 않고 `REJECTED` (`REASON_DEADLOCK_RESOLUTION`, `holder_robot_id` = 점유 로봇)로 답한다. 로봇은 그 구역에 들어가지 않고 경로를 다시 계획한다 (`ZoneGrant.msg` REJECTED 규칙).
3. 순환이 아예 생기지 않으므로 이미 들어간 구역에서 물러날 일이 없다. 대신 우회로가 없으면 그 로봇은 다시 요청할 때까지 돌아가야 한다.
4. 대기열 승격(자리가 나서 기다리던 로봇이 점유자가 됨)으로 순환이 생기는 드문 경우를 위해 해소 1도 같이 돈다. 거부 판단은 대기열에 넣을 때의 점유 로봇만 보므로, 한 로봇이 여러 구역을 함께 기다리다 그중 하나로 승격되면 순환이 생길 수 있다. 이때는 `avoid`에서도 구역 안 로봇이 `REVOKED`를 받는다 (`test_avoid_still_revokes_cycle_from_queue_promotion`).

| | 해소 1: 허가 회수 (`revoke`, 기본) | 해소 2: 회피 (`avoid`) |
| --- | --- | --- |
| 시점 | 순환이 생긴 뒤 (탐지 → 회복) | 순환이 생기기 전 (회피) |
| 대상 | 순환 안에서 우선순위가 가장 낮은 로봇 | 순환을 닫게 될 요청을 보낸 로봇 |
| 로봇 동작 | 점유 구역에서 물러나 반납 | 다음 구역에 들어가지 않고 경로 재계획 |
| 비용 | 후진 거리, 이미 진행한 이동을 되돌림 | 우회 거리, 우선순위와 무관하게 늦게 요청한 쪽이 양보 |

## 파라미터

ROS 비의존 라이브러리이므로 파라미터는 함수/생성자 인자로 받는다. YAML 로딩은 이 라이브러리를 쓰는 노드 패키지가 담당한다.

## 실행

```bash
# TODO
```

## 테스트

```bash
colcon test --packages-select amr_fleet_core
# ROS 없이 (scipy·pytest. 그림까지 보려면 matplotlib)
PYTHONPATH=src/fleet/amr_fleet_core python3 -m pytest src/fleet/amr_fleet_core/test
```

### 알고리즘 그림 (`viz.py`)

`FLEET_VIZ=<폴더>`로 테스트를 돌리면 대표 테스트가 알고리즘 상태를 PNG로 남긴다 (`<폴더>/<테스트 파일>/<이름>.png`). 설정하지 않으면 그리지 않고 matplotlib도 부르지 않는다.

```bash
FLEET_VIZ=/tmp/fleet_viz PYTHONPATH=src/fleet/amr_fleet_core python3 -m pytest src/fleet/amr_fleet_core/test
open /tmp/fleet_viz
```

| 함수 | 그림 | 남기는 테스트 |
| --- | --- | --- |
| `draw_assignment(ax, robots, tasks, result)` | 로봇(원)·작업 시작 지점(사각형), 할당(로봇 → 작업 곡선 화살표)과 비용, 픽업 → 목적지(점선), 할당 못 한 작업(회색 X) | `test_assignment`: `greedy_vs_hungarian`, `algorithms`(세 알고리즘 나란히) |
| `draw_zones(ax, manager, decisions)` | 구역마다 상자, 안에 점유 로봇, 아래에 대기열(허가 순서). `decisions`의 회수(REVOKED)는 주황 빗금 | `test_zones`: 2대·3대 교착 회수, 회피 거부, 대기열 승격 교착 |
| `draw_wait_for(ax, edges, victim)` | wait-for 그래프. 교착 순환은 빨강, 물러날 로봇은 빨간 테두리, 화살표에 구역 이름 | `test_zones`, `test_deadlock`: 꼬리 달린 순환, 따로 떨어진 순환 |

- 그림 함수는 `ax`(matplotlib Axes)만 받아 그린다. 이 모듈을 import해도 matplotlib을 부르지 않으므로 라이브러리 본체는 matplotlib에 의존하지 않는다. 노드에서 쓰려면 그 패키지가 matplotlib을 의존성에 넣는다.
- 새 테스트에 그림을 남기려면 `viz` fixture(`test/conftest.py`)를 인자로 받아 `viz('이름', lambda ax: draw_...(ax, ...), ...)`로 부른다. 함수를 여러 개 주면 왼쪽부터 한 장에 나란히 그린다.
- `test_viz.py`가 CI에서 그림 함수를 한 번씩 그려 깨지지 않았는지 확인한다.
- 그림 글자는 한글 글꼴 문제를 피하려고 영어로 쓴다.
