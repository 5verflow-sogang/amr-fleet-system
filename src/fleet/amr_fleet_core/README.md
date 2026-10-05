# amr_fleet_core

- 담당: 하연
- 빌드 타입: ament_python
- 책임: 작업 할당(최소거리/부하균형/Hungarian), 구역 허가, 경로 충돌 예측, 교착 탐지 (ROS 비의존 라이브러리)

## 공개 API

ROS 토픽을 직접 다루지 않는다. 공개 함수·클래스 목록을 여기에 적는다.

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

> 상태: 정원·lease·대기열 구현(3주차). 교차로 우선순위(`REASON_PRIORITY_YIELD`)와 교착 해소(`GRANT_REVOKED`)는 아직 없다.

`ZoneRequest`/`ZoneGrant`의 서버 측 판단만 한다. 메시지 변환과 발행은 `amr_fleet_manager`가 한다.

```python
zm = ZoneManager([ZoneSpec('narrow_01', capacity=1)], lease_s=5.0, release_timeout_s=30.0)
zm.request(robot_id, zone_id, request_id, now) -> list[Decision]  # 요청한 로봇에 대한 답 1건
zm.release(robot_id, zone_id, now) -> list[Decision]              # 자리가 나서 허가받은 대기 로봇의 답
zm.expire(now) -> list[Decision]                                  # 주기 호출. 만료 정리 후 대기 로봇 허가
```

| 상황 | 답 |
| --- | --- |
| 모르는 `zone_id` | `REJECTED` |
| 이미 허가받은 로봇이 다시 요청 (연장) | `GRANTED`, `lease_expiry = now + lease_s` |
| 자리가 있음 | `GRANTED` |
| 정원이 참 | `QUEUED` (`REASON_OCCUPIED`, `holder_robot_id` = 먼저 들어간 로봇), 대기열 끝에 넣는다 |
| 점유 로봇이 반납 | 대기열 맨 앞 로봇에게 그 로봇의 마지막 `request_id`로 `GRANTED` |
| 점유 로봇의 lease가 지남 | **넘기지 않는다.** 통로 안에서 만료되면 로봇은 빠져나간 뒤 반납한다 (이진 합의) |
| 점유 로봇이 `release_timeout_s` 동안 요청도 반납도 없음 | 죽은 것으로 보고 풀어 대기 로봇에게 넘긴다 |
| 대기 로봇이 `lease_s` 동안 다시 요청하지 않음 | 대기열에서 뺀다 (답 없음) |
| 반납 | 반납한 로봇에게는 답하지 않는다 |

- `now`는 로봇이 `lease_expiry`와 비교하는 시계와 같아야 한다. `amr_fleet_manager`는 노드 시계(`use_sim_time`이면 시뮬레이션 시간)를 넘긴다. 실제 시간으로 재면 시뮬레이터가 느릴 때 서버가 로봇보다 먼저 허가를 풀어 두 대가 동시에 들어갈 수 있다.
- 대기 순서는 먼저 요청한 순서(FIFO)다.

## 파라미터

ROS 비의존 라이브러리이므로 파라미터는 함수/생성자 인자로 받는다. YAML 로딩은 이 라이브러리를 쓰는 노드 패키지가 담당한다.

## 실행

```bash
# TODO
```

## 테스트

```bash
colcon test --packages-select amr_fleet_core
```
