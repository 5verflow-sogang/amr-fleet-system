# amr_fleet_manager

- 담당: 하연
- 빌드 타입: ament_python
- 책임: 중앙집중 Fleet 서버: 작업 할당, Traffic Management, 교착 해소

> 상태: 최소판(3주차). 작업 접수 → 할당 → Ack·재전송, heartbeat·작업표 방송, `assignments.csv` 기록, 구역 허가(정원·lease·작업 우선순위 대기열, 교착 시 허가 회수)까지 구현했다. 작업 취소, 실패 작업 재할당은 아직 없다.

노드 세 개와 지표 변환 도구를 제공한다.

| 실행 파일 | 역할 |
| --- | --- |
| `fleet_manager` | Fleet 서버 |
| `fake_robot` | 가짜 로봇. Nav2·BT 없이 Fleet 메시지만 주고받는다. `amr_bt`가 완성되기 전에 서버·장애 주입기·하네스를 끝까지 돌려볼 때 쓴다 |
| `fleet_metrics` | rosbag → `fleet_latency.csv`, `response_time.csv` (하네스가 실행마다 부른다) |
| `task_generator` | 작업 생성기. 시나리오(seed로 재현) 또는 JSON Task 파일의 일정대로 작업을 요청한다 |

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

### `task_generator`

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Publish | `/fleet/task_requests` | `amr_msgs/msg/Task` | 일정의 `at_s`마다 1건. 자세는 map 좌표, `created_time`은 발행 시각 |

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
| 작업을 맡은 로봇의 `RobotState`가 `assignment.reclaim_timeout_s`(5초) 동안 없음 (로봇 장애) | 작업을 회수해 다른 로봇에 다시 할당 (새 `assignment_id`). 죽은 로봇의 늦은 보고는 무시. FAILED 재시도 횟수와 따로 센다 |

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

서버가 뜰 때 `epoch_file`의 값에 1을 더해 쓴다. 서버를 강제종료하고 다시 띄우면 epoch가 바뀌어 로봇이 명령 번호 기준을 초기화한다. 경로를 비우면 항상 1이다.

### 서버 강제종료 (연구 1 장애)

`fleet_manager.launch.py`가 정해진 시각에 서버를 SIGKILL로 죽이고, 원하면 다시 띄운다. launch는 자기가 띄운 프로세스에만 신호를 보낼 수 있어 장애 주입기 노드가 아니라 서버 launch에 두었다.

| launch 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `kill_at_s` | `-1` | launch 시작 후 이 시각(실제 시간, s)에 SIGKILL. 음수면 끄기 |
| `restart_after_s` | `-1` | 죽인 뒤 이 시간(s)이 지나면 다시 띄운다. 음수면 죽은 채로 둔다 |
| `fault_events_file` | `''` | 강제종료·재시작 시각 기록 (`fault_events.csv`). 비우면 기록 안 함 |

- 정리 과정 없이 죽이므로 마지막 heartbeat 이후 아무 메시지도 나가지 않는다. 로봇은 heartbeat 타임아웃으로 알아챈다.
- 다시 띄운 서버는 epoch가 1 오르고, 작업·구역 장부가 비어 있다. 서버가 죽기 전에 맡긴 작업을 이어받는 일(작업표 복구)은 아직 없다.
- `fault_events.csv` 컬럼: `time, event`. `time`은 실제 시간(UNIX 초)이라 rosbag2의 수신 시각과 맞춰 볼 수 있다. `event`는 `kill`·`restart`(서버), `robot_kill:<로봇>`·`robot_restart:<로봇>`(로봇).

### 로봇 강제종료 (연구 1 장애)

`fake_robots.launch.py`가 정해진 시각에 가짜 로봇 하나를 SIGKILL로 죽이고, 원하면 다시 띄운다. 서버는 그 로봇의 `RobotState`가 `assignment.reclaim_timeout_s` 동안 끊기면 작업을 회수해 살아 있는 로봇에 다시 할당한다.

| launch 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `kill_robot` | `''` | 죽일 로봇 (예: `amr_03`). 비우거나 `none`이면 끄기 |
| `kill_robot_at_s` | `-1` | launch 시작 후 이 시각(실제 시간, s)에 SIGKILL |
| `restart_robot_after_s` | `-1` | 죽인 뒤 이 시간(s)이 지나면 다시 띄운다. 다시 띄운 로봇은 작업 없이 새로 시작 |
| `fault_events_file` | `''` | 시각 기록 (`fault_events.csv`, 서버 강제종료와 같은 파일을 써도 된다) |

하네스: `run.py fleet_fake --set kill_robot=amr_03 --set kill_robot_at_s=120`. GCP: `fleet-test` 입력 `kill_robot`, `kill_robot_at_s`.

### 작업 생성기

명세의 "JSON 형식 Task Description"을 받는 곳이자, 하네스가 같은 작업 일정을 반복해 넣는 도구다.

- **일정 만들기**: 시나리오([config/scenarios/basic.yaml](config/scenarios/basic.yaml))의 작업 수·간격(`fixed`·`poisson`)·흐름(픽업 → 목적지 스테이션, `*` 패턴)·물품 비율·우선순위·마감으로 일정을 만든다. 같은 시나리오·`seed`면 같은 일정이다. `task_id`는 `task_s<seed>_<번호>`.
- **JSON Task 파일**: `tasks_file`을 주면 시나리오 대신 그 일정을 그대로 발행한다. `schedule_out`으로 만든 일정을 같은 형식으로 남길 수 있어, 실행 폴더의 `tasks.json`을 다시 넣으면 그 실행을 재현한다.

```json
{"tasks": [
  {"at_s": 5.0, "task_id": "task_s3_0001", "type": "delivery", "item": "small",
   "pickup": "dock_1", "dropoff": "rack_a4", "priority": 0, "deadline_s": 0.0},
  {"at_s": 15.0, "task_id": "task_s3_0002", "type": "charge", "item": "none",
   "pickup": "", "dropoff": "charger_c1", "priority": 0, "deadline_s": 0.0}
]}
```

| 필드 | 값 |
| --- | --- |
| `at_s` | 일정 시작 후 발행 시각 (노드 시계, s) |
| `type` | `delivery` / `charge` / `return`. `delivery`만 `pickup`이 있다 |
| `item` | `none` / `small` / `medium` / `large` (원 명세서 물품 유형) |
| `pickup`, `dropoff` | [config/stations.yaml](config/stations.yaml)의 스테이션 이름 |
| `priority` | 0~255 |
| `deadline_s` | 요청 후 마감까지 (s). 0이면 마감 없음 |

- 스테이션 자세는 world 좌표로 적고 발행할 때 `map_origin_in_world`(기본 `[-0.0562, 1.5317, 0.00009]`, 지도 v1 0.025 m 정렬값, 위치추정과 같은 값)로 map 좌표로 바꾼다. 스테이션 목록은 **제안**이다. 도킹 자세는 지수가 ArUco 마커에 맞춰 확정한다.
- 시각은 노드 시계(`use_sim_time`이면 시뮬레이션 시간)다. 시뮬레이터가 느려져도 로봇 속도에 비해 작업이 몰리지 않는다.
- Fleet 서버(`server_node`, 기본 `fleet_manager`)가 `/fleet/task_requests`를 구독해야 일정을 시작한다. rosbag 녹화 같은 다른 구독자가 먼저 떠도 시작하지 않는다.
- **seed 0도 고정 seed다.** 0을 넣으면 매번 같은 일정이 나온다(무작위가 아님). 장애 주입기의 `seed`도 같다. 지수 쪽 `encoder_seed`·`obstacle_seed`는 0이 "매번 무작위"라 의미가 다르므로, 하네스는 반복 번호를 그대로 넘기면 된다.
- 서버가 꺼져 있는 동안(강제종료 실험) 발행한 작업은 받을 노드가 없어 사라진다. 생성기는 다시 보내지 않는다. 하네스가 `tasks.json`의 `at_s`와 `fault_events.csv`로 다운 중 투입 건수를 따로 센다.

## 파라미터

`fleet_manager` ([config/fleet_manager.yaml](config/fleet_manager.yaml))

| 이름 | 기본값 | 설명 |
| --- | --- | --- |
| `server_id` | `fleet_manager` | heartbeat·작업표의 서버 이름 |
| `assignment.algorithm` | `nearest` | `nearest` / `load_balance` / `hungarian`. launch 인자 `algorithm` |
| `assignment.min_battery_percent` | `20.0` | 할당 후보 배터리 하한 (%) |
| `assignment.failed_retries` | `1` | FAILED 보고된 작업을 큐로 되돌려 다시 할당하는 횟수. 넘으면 실패로 남긴다. 분산 재경매(최대 1회)와 같게 둔다 |
| `assignment.reclaim_timeout_s` | `5.0` | 작업을 맡은 로봇의 `RobotState`가 이 시간 동안 끊기면 로봇이 죽은 것으로 보고 작업을 회수해 다시 할당한다 (실제 시간, s). 0이면 끄기. `robot_timeout_s` 이상 |
| `robot_timeout_s` | `3.0` | 이 시간 동안 `RobotState`가 없으면 후보에서 뺀다 (s) |
| `resend.interval_s` | `1.0` | Ack 없을 때 재전송 간격 (s) |
| `resend.max_resends` | `5` | 재전송 한도. 첫 발송은 세지 않는다 |
| `heartbeat_period_s` | `0.2` | heartbeat 주기 (s). 연구 1 실험 변수 |
| `task_table_period_s` | `1.0` | 작업표 방송 주기 (s) |
| `traffic.lease_s` | `5.0` | 구역 허가 유효 시간 (노드 시계, s) |
| `traffic.release_timeout_s` | `30.0` | 점유 로봇이 이 시간 동안 요청·반납이 없으면 구역을 푼다 (노드 시계, s) |
| `traffic.deadlock_strategy` | `revoke` | 교착 해소 전략. `revoke`(순환이 생기면 우선순위 낮은 로봇 허가 회수) / `avoid`(순환을 닫는 대기 요청 거부). [amr_fleet_core README](../amr_fleet_core/README.md) '교착 탐지'. launch 인자 `deadlock_strategy` |
| `traffic.zones_file` | `''` | 구역 목록 파일. 비우면 모든 구역 요청을 거부. launch 인자 `zones_file` (기본 `config/zones.yaml`) |
| `tick_s` | `0.05` | 재전송·구역 만료 확인 주기 (s) |
| `epoch_file` | `''` | 서버 재시작 번호 파일. launch 인자 `epoch_file` (기본 `/tmp/amr_fleet_manager/epoch`) |
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
| `claim_timeout_s` | `5.0` | 다른 로봇의 작업 주장을 세는 시간. 그 로봇의 RobotState가 이만큼 끊기면 뺀다 (서버 `reclaim_timeout_s`와 같게, 실제 시간) |

`task_generator` ([launch/task_generator.launch.py](launch/task_generator.launch.py) 인자)

| 이름 | 기본값 | 설명 |
| --- | --- | --- |
| `scenario_file` | `config/scenarios/basic.yaml` | 작업 시나리오. `tasks_file`을 주면 무시 |
| `tasks_file` | `''` | JSON Task 파일 |
| `seed` | `0` | 일정 난수 seed. 하네스 반복 번호. 0도 고정 seed |
| `stations_file` | `config/stations.yaml` | 스테이션 자세 (world) |
| `schedule_out` | `''` | 만든 일정을 JSON Task 파일로 남길 경로 |
| `map_origin_in_world` | `[-0.0562, 1.5317, 0.00009]` | world에서 본 map 원점 `[x, y, yaw]`. 실수로 적는다. 위치추정(`amr_localization`)과 같은 값이어야 한다 |
| `server_node` | `fleet_manager` | 이 노드가 구독해야 일정을 시작한다. 비우면 아무 구독자나 |

## 실행

```bash
# 터미널 1: 서버
ros2 launch amr_fleet_manager fleet_manager.launch.py algorithm:=hungarian
# 터미널 2: 가짜 로봇 3대 (amr_01~03)
ros2 launch amr_fleet_manager fake_robots.launch.py count:=3
# 터미널 3: 작업 요청 (기본 시나리오: 10초마다 60건)
ros2 launch amr_fleet_manager task_generator.launch.py seed:=3 schedule_out:=/tmp/run1/tasks.json
ros2 topic echo /fleet/task_status

# 작업 하나만 손으로 넣을 때
ros2 topic pub --once /fleet/task_requests amr_msgs/msg/Task \
  "{task_id: task_0001, task_type: 0, item_type: 1, pickup_pose: {position: {x: 5.0, y: 5.0}}, dropoff_pose: {position: {x: 20.0, y: 5.0}}}"
```

### 장애 주입기를 거쳐 돌리기

```bash
ros2 launch amr_fleet_manager fleet_manager.launch.py assignments_csv:=/tmp/run1/assignments.csv
ros2 launch amr_fault_injector fault_injector.launch.py delay_ms:=200.0 loss_pct:=5.0 seed:=1
ros2 launch amr_fleet_manager fake_robots.launch.py count:=5 relayed:=true
```

가짜 로봇은 10대까지 띄울 수 있다(`config/fake_robots.yaml`에 시작 위치가 있는 수). 6대 이상이면 장애 주입기도 같은 수로 띄워야 6번 이후 로봇에 중계본이 간다.

```bash
ros2 launch amr_fault_injector fault_injector.launch.py num_robots:=10
ros2 launch amr_fleet_manager fake_robots.launch.py count:=10 relayed:=true
```

하네스는 이 상태에서 `/fleet/*`, `/<robot>/fleet/*`를 rosbag2로 녹화하면 `response_time.csv`(Ack 컬럼)·`fleet_latency.csv`의 원자료가 된다. 가짜 로봇은 `/<robot>/odom`을 내지 않으므로 응답 시간(`response_time`) 자체는 실제 로봇 스택에서만 잰다.

### Fleet 지표 변환 (`fleet_metrics`)

```bash
# 녹화: 서버 원본, 로봇별 중계본, 로봇 → 서버, odom (응답 시간용)
ros2 bag record -o /tmp/run1/rosbag -e '/fleet/.*|/amr_0[1-5]/fleet/.*|/amr_0[1-5]/odom'
# 변환: --delay-ms는 그 실행의 주입 지연 (관측 마감 계산)
ros2 run amr_fleet_manager fleet_metrics /tmp/run1/rosbag --out /tmp/run1 --delay-ms 200
```

| 결과 | 내용 |
| --- | --- |
| `fleet_latency.csv` | 원본 메시지 1건 × 로봇 1대마다 1행 (`topic, robot, key, send_time, recv_time, latency_ms, delivered`) |
| `response_time.csv` | 할당 명령 1건마다 1행 (명세서 컬럼 + `robot, task_id, latency_sim_ms, assignment_id, ack_time, ack_rtt_ms, resends`) |
| `task_outcomes.csv` | 작업 1건마다 1행 (`task_id, request_time, first_assign_time, robots, final_state, end_time, task_time_s, distance_m, error_code, runners`). `robots`는 서버가 맡긴 로봇 순서(공백 구분), `final_state`는 어느 로봇이든 `COMPLETED`를 보고했으면 `COMPLETED`(하이브리드에서는 경매로 맡은 로봇이 끝낼 수 있다), 아니면 마지막 할당 뒤 그 로봇의 `FAILED`, 없으면 빈칸. 먼저 맡은 로봇에게 밀려난 `FAILED`(`ASSIGN_TASK_TAKEN`)는 끝으로 보지 않는다. `error_code`는 `FAILED`일 때 그 보고의 `ErrorInfo.code`(예: 203 위치, 501 작업 시간 초과). `runners`는 `IN_PROGRESS`를 보고한 로봇(둘 이상이면 중복 실행) |
| `fleet_modes.csv` | 하이브리드 실행만. 로봇별 모드 구간 1개마다 1행 (`robot, mode, start_time, end_time, duration_s`), `mode`는 `central`·`decentral`. `/fleet/robot_modes`(`RobotFleetMode`)에서 만든다 |
| `recovery.csv` | `fault_events.csv`에 `robot_kill:<로봇>`이 있을 때만. 강제종료 1건 × 그 로봇이 하던 작업 1건마다 1행 (`robot, kill_time, task_id, reassign_time, reassigned_robot, recovery_s, completed`). `recovery_s` = 다른 로봇에 다시 할당된 시각 − 죽은 시각. 놀던 로봇이면 `task_id`가 빈 행 1개 |
| `robot_states.csv` | `/fleet/robot_states` 1건마다 1행 (`time, robot, x, y, mode, task_id`). 아래 `fleet_plot`의 원자료 |
| `meta.json` | `observation_cutoff` (`end, delay_ms, grace_s, resend_grace_s, cutoff, ack_cutoff, start`). 다른 키는 그대로 둔다 |

형식과 정의는 [하네스 README](../../../tools/harness/README.md#지표-csv)를 따르고, 아래는 구현에서 정한 것이다.

- 원본과 중계본은 **직렬화 바이트**로 짝짓는다. 장애 주입기가 바이트를 그대로 넘기므로 정확하고, `zone_grants`의 `request_id`처럼 로봇마다 겹치는 번호에 헷갈리지 않는다. `key` 컬럼은 읽기용이다.
- 할당 명령은 (`robot_id`, `assignment_id`, `header.stamp`)로 구분한다. 서버가 재시작하면 번호가 1부터 다시 시작하지만 stamp가 달라 섞이지 않는다. Ack는 `assignment_stamp`로 짝짓는다. 취소(`ACTION_CANCEL`)는 응답 시간 대상이 아니다.
- 관측 시작: `start` = 주기 토픽(heartbeat, task_table)의 모든 로봇 중계본이 한 번씩 녹화된 시각 − `delay_ms`. 그 전에 보낸 메시지는 `delivered`를 비운다. rosbag이 원본을 중계본보다 먼저 구독해 녹화 초반 약 1초가 손실로 잡히던 문제 (10/10 GCP 첫 하네스 실행: 손실 0% 주입에 0.8%).
- 관측 마감: `cutoff` = 끝 − `delay_ms` − `grace_s`(1초) 뒤에 보낸 메시지는 `delivered`를 비운다. `ack_cutoff` = `cutoff` − `resend_grace_s`(5초 = 서버 재전송 1초 × 5회) 뒤에 보낸 명령은 `response_time.csv`에 행을 만들지 않는다.
- `response_time`은 `/<robot>/odom`이 bag에 있을 때만 채운다. 임계값: 선속도 0.01 m/s 또는 각속도 0.05 rad/s.
- 로봇 → 서버 방향은 장애를 걸지 않는다(D8 제안). 걸게 되면 Ack 원본·중계본 짝짓기를 추가한다.

### 작업 타임라인 그림 (`fleet_plot`)

GUI(rviz·Gazebo) 없이 실행 하나를 눈으로 보는 그림. 하네스 `fleet_fake`가 `fleet_metrics` 다음에 부른다. 보기용이라 판정에는 쓰지 않는다.

```bash
ros2 run amr_fleet_manager fleet_plot /tmp/run1            # → /tmp/run1/plots/timeline.png
# ROS 없이 (matplotlib만). GCP 결과 폴더를 받아 맥에서 다시 그릴 때
PYTHONPATH=src/fleet/amr_fleet_manager python3 -m amr_fleet_manager.fleet_plot <실행 폴더>
```

`timeline.png`는 로봇마다 한 줄이다.

- 작업 구간: `current_task_id`가 같은 동안. 작업마다 색이 다르고, 막대가 넓으면 작업 이름을 쓴다
- ERROR·E-Stop: 막대 아래 빨간 띠
- RobotState가 3초 넘게 끊긴 구간: 빗금 (죽고 다시 뜨지 않은 로봇은 실행 끝까지)
- 로봇 강제종료: 그 로봇 줄의 빨간 세로선. 서버 강제종료·재시작: 전체 점선
- 회수된 작업의 재할당: 죽은 로봇 → 넘겨받은 로봇 화살표 (`recovery.csv`, 작업 이름 + 회수 시간)

- 읽는 파일이 없으면 그 부분만 빼고 그린다. `robot_states.csv`가 없거나 비면 그림을 쓰지 않는다.
- 그림이 실패해도(matplotlib 없음, 읽기 오류 등) traceback만 찍고 늘 0으로 끝난다. 그림 때문에 하네스 실행이 실패(`outcome: error`)가 되어 측정 결과까지 버려지지 않게 하기 위해서다 (하네스 `plot_run.py`와 같은 정책). 잘린 줄(쓰는 도중 강제종료 등)은 건너뛰고 나머지로 그린다.
- 그림 글자는 한글 글꼴 문제를 피하려고 영어로 쓴다.

## 테스트

```bash
colcon test --packages-select amr_fleet_manager
```

- `test/test_dispatcher.py`, `test/test_robot_sim.py`, `test/test_fleet_metrics.py`, `test/test_task_gen.py`: rclpy 없이 할당·재전송·수신 규칙·이동 모델·지표 변환·작업 일정을 확인한다.
- `test/test_fleet_metrics_bag.py`: 실제 rosbag2(sqlite3)을 써서 `fleet_metrics`가 읽고 변환하는지 확인한다 (ROS 환경에서만).
- `test/test_fake_fleet_launch.py`: 서버와 가짜 로봇 1대를 띄워 요청 → 할당 → Ack → 완료, 구역 허가 → 대기 → 반납 후 넘김을 확인한다.
- `test/test_server_kill_launch.py`: 서버를 죽였다 다시 띄워 heartbeat가 끊겼다가 epoch 2로 돌아오는지, `fault_events.csv`가 남는지 확인한다.
- `test/test_task_generator_launch.py`: JSON Task 일정대로, map 좌표로 바꿔 발행하는지 확인한다.

## 중복 실행 정리 (하이브리드)

같은 작업을 두 로봇이 맡으면 `TaskStatus.start_time`(수락 시각)이 가장 이른 로봇이 계속한다. COMPLETED를 보고한 로봇이 있으면 그 로봇이 주인이다 ([architecture.md 5-2절](../../../docs/architecture.md#5-2-하이브리드-폴백-연구-1-비교군-1016-하연-설계), `amr_fleet_core.claims`).

- 서버(`dispatcher`): 맡기지 않은 로봇의 보고라도 그 로봇이 주인이면 장부의 맡은 로봇을 바꾸고, 원래 로봇에게 보내던 재전송을 그만둔다. `ErrorInfo.ASSIGN_TASK_TAKEN`(104)으로 온 FAILED·거부 Ack는 주인에게 넘기고 재시도로 세지 않는다.
- 가짜 로봇(`fake_robot`): `/fleet/task_status`·`/fleet/robot_states`를 듣고, 다른 로봇이 먼저 맡은 작업은 거부 Ack(`ASSIGN_TASK_TAKEN`)로 돌려보낸다. 하던 작업에 더 이른 주인이 보이면 멈추고 FAILED + `ASSIGN_TASK_TAKEN`을 보낸다. 같은 작업을 다른 출처(서버·경매)에서 또 받으면 다시 시작하지 않고 수락만 한다.
- 주장은 그 로봇이 살아서 그 작업을 하는 동안만 센다. RobotState가 `reclaim_timeout_s`(가짜 로봇은 `claim_timeout_s`, 둘 다 5초) 넘게 끊기거나, 그 뒤 RobotState의 `current_task_id`가 다른 작업이면 뺀다.

## 미결

| 쟁점 | 초안 |
| --- | --- |
| 구역 목록 | 좁은 통로 2곳 확정. 교차로는 이진이 월드 좌표로 추가, 지도 v1(10/9) 뒤 map 좌표로 확정 |
| 서버 재시작 직후 구역 | 새 서버는 구역 장부가 비어 있어, 통로 안에서 빠져나오는 로봇이 있어도 다른 로봇을 허가할 수 있다. 재시작 뒤 한동안 허가를 보류하거나 `ZoneRequest`에 "구역 안" 표시가 필요 |
| 경로 충돌 예측 | 이진: Nav2 전역 경로 `/<ns>/plan` (`nav_msgs/Path`, map 프레임)을 그대로 구독 |
| 교착 해소 | 2종(`traffic.deadlock_strategy`). 로봇 쪽 처리(REVOKED 후진, REJECTED 재계획)는 `amr_zone_client` 몫. `avoid`에서도 REVOKED가 올 수 있다(amr_fleet_core README). 구역 안에서 REVOKED를 받을 때 BT와 맞추는 방법, 회수 대상을 구역 밖 로봇으로 고를 수 있게 `ZoneRequest`에 "접근 중/구역 안" 상태를 넣을지는 10/12 회의에서 정한다 |
| 가짜 로봇의 구역 요청 | `fake_robot`은 구역을 요청하지 않는다 |
| 작업 취소 | `ACTION_CANCEL` 발행 경로 없음 (가짜 로봇은 받으면 처리한다). 취소로 멈춘 작업은 FAILED + `ErrorInfo.ASSIGN_CANCELED`로 보고받고 다시 할당하지 않는다 |
| FAILED 작업 | `assignment.failed_retries`(1)번까지 다시 할당한다(10/8, 분산 재경매와 같게). 취소(`ASSIGN_CANCELED`)는 빼고 센다. 다시 맡을 로봇은 고르지 않는다(같은 로봇이 다시 받을 수 있음). "다른 로봇 우선"은 분산과 같이 정한다(10/12 회의) |
| 로봇 끊김 | `reclaim_timeout_s` 뒤 작업을 회수해 다시 할당한다. 로봇이 살아 돌아와 옛 작업을 계속하면 "먼저 수락한 쪽이 주인" 규칙으로 정리한다(아래 "중복 실행 정리"). 분산 경매에는 같은 회수가 없다(비교 실험 전에 이진과 맞춤) |
| 작업표 크기 | 끝난 작업도 계속 싣는다. 장시간 운용(4시간) 전에 정리 규칙을 정한다 |
| 로봇 → 서버 토픽 장애 주입 | [api.md 4-4절](../../../docs/api.md#4-4-쟁점-정리) 미결 |
