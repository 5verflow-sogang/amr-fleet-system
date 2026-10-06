# amr_bt

- 담당: 지수
- 빌드 타입: ament_cmake (C++17)
- 책임: BehaviorTree.CPP 작업 수행 노드와 서브트리, 작업 관리

> 상태: 착수(3주차 작업 5). Fleet 서버와 작업을 주고받고 BT로 단계를 진행한다.
> **이동(`MoveTo`)과 싣기·내리기(`Dwell`)는 아직 자리채움**이다. 정해진 시간만 기다리고 실제로 움직이지 않는다.
> 작업 9에서 Nav2 `navigate_to_pose`와 `docking/dock`(Dock.action, 3회 재시도)으로 바꾼다.

## 제공 인터페이스

`<prefix>` = `server_topic_prefix`: `/fleet`(서버 원본, 기본), 장애 주입기 중계본이면 `fleet`(→ `/<ns>/fleet/...`).

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Subscribe | `<prefix>/task_assignments` | `amr_msgs/msg/TaskAssignment` | 자기 `robot_id`만 받는다 |
| Subscribe | `<prefix>/heartbeat` | `amr_msgs/msg/FleetHeartbeat` | `epoch`가 바뀌면 명령 번호 기준 초기화 |
| Publish | `/fleet/task_assignment_acks` | `amr_msgs/msg/TaskAssignmentAck` | 할당·취소마다. 재전송이면 같은 Ack를 다시 |
| Publish | `/fleet/task_status` | `amr_msgs/msg/TaskStatus` | 상태·단계가 바뀔 때 |
| Publish | `/fleet/robot_states` | `amr_msgs/msg/RobotState` | `state_rate_hz`(2 Hz) |

QoS는 Fleet 서버와 같은 RELIABLE depth 10 (api.md 6절이 정해지면 맞춘다).
전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

## 동작

### 수신 규칙 (`include/amr_bt/assignment_filter.hpp`)

`TaskAssignment.msg` 주석과 가짜 로봇의 `robot_sim.AssignmentFilter`(amr_fleet_manager)와 같다. 다른 도메인 패키지는 import하지 않으므로 C++로 다시 썼다.

| 받은 명령 | 처리 |
| --- | --- |
| 처음 보는 `assignment_id` (마지막보다 큼) | 처리하고 Ack. 작업 중에 새 할당이 오면 `RESULT_REJECTED` + `ASSIGN_ALREADY_BUSY` |
| 마지막으로 처리한 번호 (재전송) | 다시 처리하지 않고 같은 Ack만 |
| 마지막보다 작은 번호 (뒤늦게 옴) | 무시 (Ack 없음) |
| `ACTION_CANCEL` | 하던 작업이면 **Ack보다 먼저** BT를 멈추고(halt) FAILED(`canceled`)를 보내고 작업을 비운다 → 바로 뒤 새 할당을 거부하지 않는다. 아니면 수락만. 취소 코드는 `ErrorInfo`에 생기면 채운다 (지금 NONE, 가짜 로봇과 같음) |
| heartbeat `epoch` 변경 (서버 재시작) | 번호 기준 초기화 |

수락하면 바로 `TaskStatus(PENDING)`를 보내고 `RobotState.current_task_id`를 채운다. 서버는 Ack가 유실돼도 이 값으로 받은 것을 확인한다.

### 작업 사이클 (`behavior_trees/task_cycle.xml`)

```
Sequence
├── WaitForTask                    수락한 작업이 생길 때까지 RUNNING
└── Fallback
    ├── Sequence
    │   ├── IfThenElse(IsDelivery) → move_to_pickup → MoveTo → loading → Dwell
    │   ├── move_to_dropoff → MoveTo
    │   ├── IfThenElse(IsDelivery) → unloading → Dwell
    │   └── FinishTask             COMPLETED
    └── FailTask                   FAILED (실패, 그때 단계와 원인 코드 ctx.fail(code, message))
```

- 배송(`TYPE_DELIVERY`): `move_to_pickup` → `loading` → `move_to_dropoff` → `unloading` → COMPLETED
- 충전·복귀(`TYPE_CHARGE`·`TYPE_RETURN`): `move_to_dropoff` → COMPLETED
- 단계 이름은 [architecture.md 5-1절](../../../docs/architecture.md)의 `TaskStatus.phase`
- 끝나면 처음으로 돌아가 다음 작업을 기다린다

### 아직 없는 것

- 실제 이동·도킹 (작업 9): `MoveTo` → Nav2 `navigate_to_pose`, `Dwell` 앞에 `docking/dock` + 3회 재시도·대체 작업
- `TaskStatus.distance_traveled` (지금 0), 가상 배터리 모델 (지금 `battery_percent` 고정)
- 구역 허가(`ZoneRequest`/`ZoneGrant`), 분산 모드 전환 (heartbeat 끊김)
- `RobotState.pose`는 `map` → `<robot_id>/base_footprint` TF. 위치 추정(map → odom)이 없으면 원점을 보내고 10초마다 경고

## 파라미터

`config/amr_bt.yaml`:

| 이름 | 기본값 | 설명 |
| --- | --- | --- |
| `robot_id` | `""` | 비우면 네임스페이스(`/amr_01` → `amr_01`) |
| `server_topic_prefix` | `/fleet` | launch `relayed:=true`면 `fleet` |
| `tick_hz` | `10.0` | BT tick 주기 |
| `state_rate_hz` | `2.0` | `RobotState` 주기 (1~5 Hz) |
| `global_frame` / `base_frame` | `map` / `""` | `RobotState.pose` TF. `base_frame`을 비우면 `<robot_id>/base_footprint` |
| `stub_move_s` / `stub_dwell_s` | `5.0` / `2.0` | 자리채움 이동·싣기 시간 [s] |
| `battery_percent` | `100.0` | 가상 배터리 모델 전까지 고정 |
| `tree_file` | `behavior_trees/task_cycle.xml` | BT XML |

## 실행

```bash
# 터미널 1: Fleet 서버
ros2 launch amr_fleet_manager fleet_manager.launch.py
# 터미널 2: 로봇 1대 (sim 없이 돌릴 때는 use_sim_time:=false)
ros2 launch amr_bt amr_bt.launch.py namespace:=amr_01 use_sim_time:=false
# 터미널 3: 작업 요청
ros2 topic pub --once /fleet/task_requests amr_msgs/msg/Task \
  "{task_id: task_0001, task_type: 0, item_type: 1, pickup_pose: {position: {x: 5.0, y: 5.0}}, dropoff_pose: {position: {x: 20.0, y: 5.0}}}"
ros2 topic echo /fleet/task_status

# 장애 주입기를 거칠 때
ros2 launch amr_fault_injector fault_injector.launch.py delay_ms:=200.0 loss_pct:=5.0
ros2 launch amr_bt amr_bt.launch.py relayed:=true use_sim_time:=false
```

Fleet 서버와 확인한 결과 (`stub_move_s` 3, `stub_dwell_s` 1, 작업 2건: 배송 → 충전):

| 경로 | 할당 → Ack | 재전송 | 결과 |
| --- | --- | --- | --- |
| 직접 (`/fleet`) | 10 ms 안 | 없음 | 두 작업 모두 PENDING → 단계별 IN_PROGRESS → COMPLETED |
| 장애 주입기 (지연 200 ms, 손실 40 %) | 2.2 s, 1.2 s | 2회, 1회 (같은 `assignment_id`) | 같음. 재전송은 한 번만 수락 |

## 테스트

```bash
colcon test --packages-select amr_bt
```

- `test_assignment_filter`: 새 명령·재전송·뒤늦은 명령, epoch 초기화
- `test_task_cycle`: ROS 없이 가짜 시계로 트리를 돌려 배송·충전 단계 순서, 연속 작업, 취소(즉시 FAILED, 취소 직후 새 작업 수락, 취소 뒤 IN_PROGRESS 없음, 다른 작업 취소 무시, 싣기 중 취소), 실패 오류 코드 전달
