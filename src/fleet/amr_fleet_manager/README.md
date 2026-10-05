# amr_fleet_manager

- 담당: 하연
- 빌드 타입: ament_python
- 책임: 중앙집중 Fleet 서버: 작업 할당, Traffic Management, 교착 해소

> 상태: 최소판(3주차). 작업 접수 → 할당 → Ack·재전송, heartbeat·작업표 방송, `assignments.csv` 기록, 구역 허가(정원·lease·대기열)까지 구현했다. 교차로 우선순위, 교착 해소, 작업 취소, 실패 작업 재할당은 아직 없다.

노드 두 개를 제공한다.

| 실행 파일 | 역할 |
| --- | --- |
| `fleet_manager` | Fleet 서버 |
| `fake_robot` | 가짜 로봇. Nav2·BT 없이 Fleet 메시지만 주고받는다. `amr_bt`가 완성되기 전에 서버·장애 주입기·하네스를 끝까지 돌려볼 때 쓴다 |

## 제공 인터페이스

### `fleet_manager`

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Subscribe | `/fleet/task_requests` | `amr_msgs/msg/Task` | 새 작업. `task_id`가 비면 서버가 만든다. 이미 받은 `task_id`는 무시 |
| Subscribe | `/fleet/robot_states` | `amr_msgs/msg/RobotState` | 할당 후보 판단 (`mode`, `battery_percent`, `pose`). `current_task_id`로 Ack 유실도 확인 |
| Subscribe | `/fleet/task_assignment_acks` | `amr_msgs/msg/TaskAssignmentAck` | 재전송 중지. 거부면 작업을 큐로 되돌린다 |
| Subscribe | `/fleet/task_status` | `amr_msgs/msg/TaskStatus` | 작업 상태 갱신. COMPLETED·FAILED면 로봇을 비우고 다음 할당 |
| Subscribe | `/fleet/zone_requests` | `amr_msgs/msg/ZoneRequest` | 구역 진입 요청·연장·반납 |
| Publish | `/fleet/task_assignments` | `amr_msgs/msg/TaskAssignment` | 할당 명령. Ack가 없으면 같은 `assignment_id`·`header.stamp`로 재전송 |
| Publish | `/fleet/heartbeat` | `amr_msgs/msg/FleetHeartbeat` | `heartbeat_period_s`마다. `mode`는 항상 CENTRALIZED |
| Publish | `/fleet/task_table` | `amr_msgs/msg/FleetTaskTable` | `task_table_period_s`마다. 받은 작업 전부 (끝난 작업 포함) |
| Publish | `/fleet/zone_grants` | `amr_msgs/msg/ZoneGrant` | 요청에 대한 답, 대기 로봇 차례가 오면 |

### `fake_robot`

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Subscribe | `<prefix>/task_assignments` | `amr_msgs/msg/TaskAssignment` | 자기 `robot_id`만 처리 |
| Subscribe | `<prefix>/heartbeat` | `amr_msgs/msg/FleetHeartbeat` | `epoch`가 바뀌면 명령 번호 기준 초기화 |
| Publish | `/fleet/task_assignment_acks` | `amr_msgs/msg/TaskAssignmentAck` | 명령마다 |
| Publish | `/fleet/task_status` | `amr_msgs/msg/TaskStatus` | PENDING → IN_PROGRESS(`move_to_pickup` → `loading` → `move_to_dropoff` → `unloading`) → COMPLETED |
| Publish | `/fleet/robot_states` | `amr_msgs/msg/RobotState` | `state_rate_hz`마다 |

`<prefix>`는 `server_topic_prefix` 파라미터다. 기본 `/fleet`(서버 원본), 장애 주입기를 거치면 `fleet`(네임스페이스 안 상대 이름 → `/amr_0N/fleet/...`).

작업은 픽업·목적지까지 직선으로 `speed_mps`로 가고 싣기·내리기에 `dwell_s`를 쓴다. 이미 작업 중이면 `ASSIGN_ALREADY_BUSY`로 거부한다.

전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

## 동작 규칙

### 할당 시점

새 작업이 들어올 때, `RobotState`를 받을 때, 작업이 끝나거나 거부·재전송 포기로 큐에 돌아올 때 할당 알고리즘을 부른다. 큐에 작업이 없거나 후보 로봇이 없으면 부르지 않는다 (`assignments.csv`에도 남지 않는다).

후보 로봇은 다음을 모두 만족해야 한다.

- `robot_timeout_s` 안에 `RobotState`가 왔다
- 서버가 맡긴 작업이 없다 (완료·실패 보고 전까지는 `RobotState.mode`가 IDLE이어도 후보가 아니다)
- `RobotState.mode == IDLE`이고 배터리가 `assignment.min_battery_percent` 이상 (`amr_fleet_core.filter_candidates`)

### 확인과 재전송

| 상황 | 서버 동작 |
| --- | --- |
| Ack(수락) | 재전송 중지 |
| Ack(거부) | 작업을 큐로 되돌리고 다시 할당 |
| Ack 유실, 그러나 그 로봇의 `RobotState.current_task_id` 또는 `TaskStatus`가 그 작업 | 받은 것으로 보고 재전송 중지 |
| `resend.interval_s` 동안 아무 확인 없음 | 같은 메시지를 재전송 |
| 재전송 `resend.max_resends`번 후에도 확인 없음 | 작업을 큐로 되돌리고 다시 할당 (새 `assignment_id`) |

- Ack 유실 확인에 `RobotState.last_assignment_id`를 쓰지 않는 이유: 서버가 재시작하면 번호가 1부터 다시 시작해, 로봇이 새 epoch를 보기 전에 보낸 큰 번호를 확인으로 잘못 볼 수 있다.
- 재전송·heartbeat·로봇 생존 판정은 실제 시간(단조 시계)으로 잰다. 메시지의 `header.stamp`는 노드 시계(`use_sim_time`이면 시뮬레이션 시간)다.
- 하네스 관측 마감의 Ack 여유 시간은 기본값으로 `interval_s × max_resends` = 5초다 ([하네스 README](../../../tools/harness/README.md) "관측 마감").

### 로봇 측 수신 규칙 (`amr_bt` 참고)

`fake_robot`의 [robot_sim.py](amr_fleet_manager/robot_sim.py) `AssignmentFilter`가 [TaskAssignment.msg](../../interfaces/amr_msgs/msg/TaskAssignment.msg) 주석의 규칙을 그대로 구현한 참고본이다.

| 받은 `assignment_id` | 처리 |
| --- | --- |
| 마지막으로 처리한 번호보다 큼 | 처리하고 Ack |
| 마지막으로 처리한 번호와 같음 (재전송) | 다시 처리하지 않고 같은 내용의 Ack만 다시 보낸다 |
| 마지막으로 처리한 번호보다 작음 (지연으로 뒤바뀜) | 무시 |
| `FleetHeartbeat.epoch`가 바뀜 | 기준을 0으로 초기화 |

### 구역 허가

판단 규칙은 [amr_fleet_core README](../amr_fleet_core/README.md) "구역 허가" 절에 있다. 서버는 구역 목록 파일([config/zones.yaml](config/zones.yaml))의 `id`·`capacity`만 쓴다.

- `lease_expiry`는 노드 시계 기준이다. 로봇은 구역 안에 있는 동안 `traffic.lease_s`(5초)보다 짧은 주기로 같은 `request_id`를 다시 보내 연장한다.
- 대기 중인 로봇도 같은 주기로 다시 요청한다. `lease_s` 동안 요청이 없으면 대기열에서 빠진다.
- lease가 지나도 반납이 올 때까지 구역을 다른 로봇에 주지 않는다. 통로 안에서 만료되면 로봇은 멈추지 않고 빠져나간 뒤 반납한다(이진, 10/5). 요청도 반납도 `traffic.release_timeout_s`(30초) 동안 없으면 로봇이 죽은 것으로 보고 푼다.
- 로봇 측(이진): `/fleet/zone_requests`로 요청, 네임스페이스 안 `fleet/zone_grants`로 답을 받는다. 경계 3 m 앞에서 요청, 허가가 없으면 경계 0.5 m 앞에서 정지, 1초마다 연장, 벗어나면 반납.
- 구역은 좁은 통로 두 곳(`narrow_01`, `narrow_02`)으로 확정했다. 교차로는 1차 연동 뒤 이진이 월드 좌표로 골라 추가한다.

### epoch

서버가 뜰 때 `epoch_file`의 값에 1을 더해 쓴다. 하네스가 서버를 강제종료하고 다시 띄우면 epoch가 바뀌어 로봇이 명령 번호 기준을 초기화한다. 경로를 비우면 항상 1이다.

## 파라미터

`fleet_manager` ([config/fleet_manager.yaml](config/fleet_manager.yaml))

| 이름 | 기본값 | 설명 |
| --- | --- | --- |
| `server_id` | `fleet_manager` | heartbeat·작업표의 서버 이름 |
| `assignment.algorithm` | `nearest` | `nearest` / `load_balance` / `hungarian`. launch 인자 `algorithm` |
| `assignment.min_battery_percent` | `20.0` | 할당 후보 배터리 하한 (%) |
| `robot_timeout_s` | `3.0` | 이 시간 동안 `RobotState`가 없으면 후보에서 뺀다 (s) |
| `resend.interval_s` | `1.0` | Ack 없을 때 재전송 간격 (s) |
| `resend.max_resends` | `5` | 재전송 한도. 첫 발송은 세지 않는다 |
| `heartbeat_period_s` | `0.2` | heartbeat 주기 (s). 연구 1 실험 변수 |
| `task_table_period_s` | `1.0` | 작업표 방송 주기 (s) |
| `traffic.lease_s` | `5.0` | 구역 허가 유효 시간 (노드 시계, s) |
| `traffic.release_timeout_s` | `30.0` | 점유 로봇이 이 시간 동안 요청·반납이 없으면 구역을 푼다 (노드 시계, s) |
| `traffic.zones_file` | `''` | 구역 목록 파일. 비우면 모든 구역 요청을 거부. launch 인자 `zones_file` (기본 `config/zones.yaml`) |
| `tick_s` | `0.05` | 재전송·구역 만료 확인 주기 (s) |
| `epoch_file` | `/tmp/amr_fleet_manager/epoch` | 서버 재시작 번호 파일 |
| `assignments_csv` | `''` | 할당 호출 기록 경로. 비우면 기록 안 함. launch 인자 `assignments_csv` |

`assignments.csv`의 컬럼은 [하네스 README](../../../tools/harness/README.md#지표-csv)를 따른다. `timestamp`는 노드 시계(`use_sim_time`이면 시뮬레이션 시간), `cost_fn`은 지금 `euclidean`뿐이다.

`fake_robot` ([config/fake_robots.yaml](config/fake_robots.yaml))

| 이름 | 기본값 | 설명 |
| --- | --- | --- |
| `robot_id` | `amr_01` | launch가 네임스페이스와 같게 넣는다 |
| `start_xy` | `[0.0, 0.0]` | 시작 위치 (map, m). yaml에 로봇별로 둔다 |
| `speed_mps` | `1.0` | 직선 이동 속도 |
| `dwell_s` | `1.0` | 싣기·내리기 시간 |
| `battery_percent` | `100.0` | 고정값 (방전 없음) |
| `state_rate_hz` | `2.0` | `RobotState` 주기 |
| `step_hz` | `10.0` | 이동 계산 주기 |
| `server_topic_prefix` | `/fleet` | 서버 → 로봇 토픽 앞부분. launch 인자 `relayed:=true`면 `fleet` |

## 실행

```bash
# 터미널 1: 서버
ros2 launch amr_fleet_manager fleet_manager.launch.py algorithm:=hungarian
# 터미널 2: 가짜 로봇 3대 (amr_01~03)
ros2 launch amr_fleet_manager fake_robots.launch.py count:=3
# 터미널 3: 작업 요청
ros2 topic pub --once /fleet/task_requests amr_msgs/msg/Task \
  "{task_id: task_0001, task_type: 0, item_type: 1, pickup_pose: {position: {x: 5.0, y: 5.0}}, dropoff_pose: {position: {x: 20.0, y: 5.0}}}"
ros2 topic echo /fleet/task_status
```

### 장애 주입기를 거쳐 돌리기

```bash
ros2 launch amr_fleet_manager fleet_manager.launch.py assignments_csv:=/tmp/run1/assignments.csv
ros2 launch amr_fault_injector fault_injector.launch.py delay_ms:=200.0 loss_pct:=5.0 seed:=1
ros2 launch amr_fleet_manager fake_robots.launch.py count:=5 relayed:=true
```

하네스는 이 상태에서 `/fleet/*`, `/<robot>/fleet/*`를 rosbag2로 녹화하면 `response_time.csv`(Ack 컬럼)·`fleet_latency.csv`의 원자료가 된다. 가짜 로봇은 `/<robot>/odom`을 내지 않으므로 응답 시간(`response_time`) 자체는 실제 로봇 스택에서만 잰다.

## 테스트

```bash
colcon test --packages-select amr_fleet_manager
```

- `test/test_dispatcher.py`, `test/test_robot_sim.py`: rclpy 없이 할당·재전송·수신 규칙·이동 모델을 확인한다.
- `test/test_fake_fleet_launch.py`: 서버와 가짜 로봇 1대를 띄워 요청 → 할당 → Ack → 완료, 구역 허가 → 대기 → 반납 후 넘김을 확인한다.

## 미결

| 쟁점 | 초안 |
| --- | --- |
| 구역 목록 | 좁은 통로 2곳 확정. 교차로는 이진이 월드 좌표로 추가, 지도 v1(10/9) 뒤 map 좌표로 확정 |
| 서버 재시작 직후 구역 | 새 서버는 구역 장부가 비어 있어, 통로 안에서 빠져나오는 로봇이 있어도 다른 로봇을 허가할 수 있다. 재시작 뒤 한동안 허가를 보류하거나 `ZoneRequest`에 "구역 안" 표시가 필요 |
| 경로 충돌 예측 | 이진: Nav2 전역 경로 `/<ns>/plan` (`nav_msgs/Path`, map 프레임)을 그대로 구독 |
| 교차로 우선순위 | 대기열이 FIFO다. 급한 작업을 가진 로봇 먼저(`REASON_PRIORITY_YIELD`)는 이후 |
| 교착 해소 | 구역 대기 그래프에서 순환 탐지 → `GRANT_REVOKED` 등 2가지 이상 |
| 가짜 로봇의 구역 요청 | `fake_robot`은 구역을 요청하지 않는다 |
| 작업 취소 | `ACTION_CANCEL` 발행 경로 없음 (가짜 로봇은 받으면 처리한다) |
| FAILED 작업 | 실패로 남긴다. 재할당 정책은 `ErrorInfo.code`별로 정한다 |
| 로봇 끊김 | 작업 중인 로봇의 `RobotState`가 끊겨도 작업을 회수하지 않는다 |
| 작업표 크기 | 끝난 작업도 계속 싣는다. 장시간 운용(4시간) 전에 정리 규칙을 정한다 |
| 로봇 → 서버 토픽 장애 주입 | [api.md 4-4절](../../../docs/api.md#4-4-쟁점-정리) 미결 |
