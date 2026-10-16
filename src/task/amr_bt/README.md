# amr_bt

- 담당: 지수
- 빌드 타입: ament_cmake (C++17)
- 책임: BehaviorTree.CPP 작업 수행 노드와 서브트리, 작업 관리

> 상태: 작업 9. Fleet 서버와 작업을 주고받고, 이동은 Nav2 `navigate_to_pose`, 도킹은 `docking/dock`(Dock.action, 3회 재시도)을 부른다.
> `use_nav2`·`use_docking`이 기본 false라 그대로 띄우면 이전처럼 시간만 흘리는 자리채움으로 돈다(1단계 하네스).
> 도킹 서버는 [`amr_docking`](../amr_docking/README.md) (ArUco). Nav2와 붙인 Gazebo 한 사이클은 bringup 2단계 뒤.

## 제공 인터페이스

`<prefix>` = `server_topic_prefix`: `/fleet`(서버 원본, 기본), 장애 주입기 중계본이면 `fleet`(→ `/<ns>/fleet/...`).

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Subscribe | `<prefix>/task_assignments` | `amr_msgs/msg/TaskAssignment` | 자기 `robot_id`만 받는다 |
| Subscribe | `<prefix>/heartbeat` | `amr_msgs/msg/FleetHeartbeat` | `epoch`가 바뀌면 명령 번호 기준 초기화 |
| Publish | `/fleet/task_assignment_acks` | `amr_msgs/msg/TaskAssignmentAck` | 할당·취소마다. 재전송이면 같은 Ack를 다시 |
| Publish | `/fleet/task_status` | `amr_msgs/msg/TaskStatus` | 상태·단계가 바뀔 때 |
| Publish | `/fleet/robot_states` | `amr_msgs/msg/RobotState` | `state_rate_hz`(2 Hz) |
| Subscribe | `zone_client/state` | `std_msgs/msg/String` | 구역 상태 (#137). `waiting`·`blocked`·`backing`이면 이동을 멈춘다 |
| Subscribe | `kidnap_recovery/state` | `std_msgs/msg/String` | 위치 복구 상태 (#79). `tracking`이 아니면 이동을 멈춘다 |
| Action client | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `MoveTo` (`use_nav2`). `map` 기준 pickup·dropoff 자세 |
| Action client | `docking/dock` | `amr_msgs/action/Dock` | `Dock` (`use_docking`). 한 번 호출에 한 번 시도 |

`/fleet/...` QoS는 Fleet 서버와 같은 RELIABLE depth 10 (api.md 6절이 정해지면 맞춘다). 로봇 안 상태 토픽과 액션은 네임스페이스 안 상대 이름이다.
전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

## 동작

### 수신 규칙 (`include/amr_bt/assignment_filter.hpp`)

`TaskAssignment.msg` 주석과 가짜 로봇의 `robot_sim.AssignmentFilter`(amr_fleet_manager)와 같다. 다른 도메인 패키지는 import하지 않으므로 C++로 다시 썼다.

| 받은 명령 | 처리 |
| --- | --- |
| 처음 보는 `assignment_id` (마지막보다 큼) | 처리하고 Ack. 작업 중에 새 할당이 오면 `RESULT_REJECTED` + `ASSIGN_ALREADY_BUSY` |
| 마지막으로 처리한 번호 (재전송) | 다시 처리하지 않고 같은 Ack만 |
| 마지막보다 작은 번호 (뒤늦게 옴) | 무시 (Ack 없음) |
| `ACTION_CANCEL` | 하던 작업이면 **Ack보다 먼저** BT를 멈추고(halt) FAILED(`canceled`)를 보내고 작업을 비운다 → 바로 뒤 새 할당을 거부하지 않는다. 아니면 수락만. 오류 코드는 `ErrorInfo.ASSIGN_CANCELED`라 서버가 다시 할당하지 않는다 (#107, 가짜 로봇과 같음) |
| heartbeat `epoch` 변경 (서버 재시작) | 번호 기준 초기화 |

수락하면 바로 `TaskStatus(PENDING)`를 보내고 `RobotState.current_task_id`를 채운다. 서버는 Ack가 유실돼도 이 값으로 받은 것을 확인한다.

### 작업 사이클 (`behavior_trees/task_cycle.xml`)

```
TaskCycle (Sequence)
├── WaitForTask                         수락한 작업이 생길 때까지 RUNNING
└── Fallback
    ├── Sequence
    │   ├── IfThenElse(IsDelivery) → VisitStation(pickup: move_to_pickup → MoveTo → DockWithRetry → loading → Dwell)
    │   ├── IfThenElse(IsDelivery)
    │   │   ├── VisitStation(dropoff: move_to_dropoff → MoveTo → DockWithRetry → unloading → Dwell)
    │   │   └── move_to_dropoff → MoveTo → IfThenElse(IsCharge) → DockWithRetry(dropoff) → charging → Charge
    │   └── FinishTask                  COMPLETED
    └── FailTask                        FAILED (실패, 그때 단계와 원인 코드 ctx.fail(code, message))

DockWithRetry = IfThenElse(HasStation) → docking → RetryUntilSuccessful(3) → Dock
```

- 배송(`TYPE_DELIVERY`): `move_to_pickup` → `docking` → `loading` → `move_to_dropoff` → `docking` → `unloading` → COMPLETED
- 충전(`TYPE_CHARGE`): `move_to_dropoff` → `docking` → `charging`(배터리가 `charge_target_percent`까지) → COMPLETED
- 복귀(`TYPE_RETURN`): `move_to_dropoff` → COMPLETED
- 스테이션 이름(`pickup_station_id`·`dropoff_station_id`)이 비면 도킹을 건너뛴다
- 단계 이름은 [architecture.md 5-1절](../../../docs/architecture.md)의 `TaskStatus.phase`
- 끝나면 처음으로 돌아가 다음 작업을 기다린다
- 노드: 펼친 트리 기준 56개, 직접 만든 종류 11개 (`WaitForTask`·`IsDelivery`·`IsCharge`·`HasStation`·`ReportPhase`·`MoveTo`·`Dock`·`Dwell`·`Charge`·`FinishTask`·`FailTask`). 명세 KPI 15개 이상은 테스트(`TreeHasAtLeast15Nodes`)로 지킨다

### 이동 (`MoveTo`)

| 상황 | 동작 | 실패 코드 |
| --- | --- | --- |
| Nav2 성공 | SUCCESS | — |
| Nav2 실패(abort) | 같은 목표를 `nav_retries`(2)번 다시 보낸다 | 다 실패하면 `NAV_NO_PATH`(201) |
| 멈춘 시간을 뺀 이동 시간 > `nav_timeout_s`(300 s) | 목표 취소 | `NAV_TIMEOUT`(202) |
| `zone_client/state` = `waiting`·`blocked`·`backing` | 목표 취소, RUNNING으로 기다림 | 한 번에 `pause_timeout_s`(60 s) 넘게 멈추면 `TRAFFIC_LEASE_TIMEOUT`(301) |
| `kidnap_recovery/state` = `settling`·`rotating` | 같음 | 한 번에 `kidnap_pause_timeout_s`(120 s) 넘게 멈추면 `LOCALIZATION_LOST`(203). 복구 시도 3번이 약 84 s라 구역보다 길게 둔다 (#178) |
| `kidnap_recovery/state` = `failed` (복구 노드가 포기) | 바로 실패 | `LOCALIZATION_LOST`(203) |
| `zone_client/state` = `failed` (후진 안전 한도로 정지) | 바로 실패 | `TRAFFIC_REVOKED`(302) |
| 진행 상태(`idle`·`approaching`·`inside`, `tracking`)로 돌아옴 | `resume_hold_s`(0.3 s) 이어지면 같은 목표를 다시 보낸다. Nav2가 지금 위치에서 경로를 다시 짠다 | — |

- 같은 진행 상태가 1초마다 다시 와도, 목표가 나가 있으면 다시 보내지 않는다. 멈춤 → 진행으로 바뀔 때만 보낸다
- 상태 토픽을 내는 노드가 없으면 기본값(`idle`, `tracking`) = 진행이다
- 후진(`backing`)은 zone_client가 `cmd_vel_zone_limited`로 직접 낸다. BT는 Nav2 목표만 멈춘다 (#137)
- `zone_client/state` = `failed`인 동안은 `RobotState.mode`를 `MODE_ERROR`로 내고 새 할당을 `RESULT_REJECTED` + `TRAFFIC_REVOKED`로 거부한다. 서버가 멈춘 로봇에 작업을 계속 줘서 바로 실패하는 일을 되풀이하지 않게 한다
- `kidnap_recovery/state` = `failed`인 동안도 같다 (`LOCALIZATION_LOST`, "localization lost"). 사람이 위치를 다시 잡거나 복구 노드가 늦게라도 수렴해 `tracking`으로 돌아오면 다시 `MODE_IDLE`로 할당을 받는다 (#148)
- `kidnap_recovery/state`가 `kidnap_state_stale_s`(5 s) 넘게 안 오면(복구 노드가 죽거나 scan·odom이 끊김) 마지막 값을 버리고 기본값 `tracking`으로 돌린다. 안 그러면 `failed`가 남아 `MODE_ERROR`가 풀리지 않는다. `zone_client/state` = `failed`는 사람이 확인해야 하는 상태라 그대로 둔다
  - 복구 노드는 scan·odom·지도가 다 있을 때만 상태를 낸다. 그래서 scan이 끊겨도 5 s 뒤 `failed`가 `tracking`으로 풀려 할당을 받는다. 상태를 모르는 채 멈춰 있기보다 움직이게 둔 것이고(위치가 정말 틀리면 Nav2·구역이 막는다), 멈춰 있는 쪽이 낫다고 정하면 `failed`만 남기게 바꾼다
- `MODE_ERROR`에 들어갈 때 이유를 WARN으로, 나올 때 빠져 있던 시간을 INFO로 남긴다 (할당에서 빠진 로봇·시간을 로그로 본다)

### 가상 배터리 (`include/amr_bt/battery.hpp`)

명세에 수치가 없어 시뮬레이션 운용에 맞춘 값이다 (`battery.*` 파라미터).

| 항목 | 기본값 | 설명 |
| --- | --- | --- |
| 이동 소모 | 0.05 %/m | 배송 1건(약 50 m) ≈ 2.5 % |
| 시간 소모 | 0.002 %/s | 약 7 %/h |
| 충전 | 0.5 %/s | 0 → 100 %가 200 s |

- `RobotState.battery_percent`로 낸다. 이동 거리는 `odom` 기준 TF 위치 변화(2 Hz)
- 충전 작업은 충전소 도킹 뒤 `Charge` 노드가 `charge_target_percent`(100)까지 기다린다. 그동안 `RobotState.mode` = `MODE_CHARGING`
- `reject_below_percent`(10 %)보다 낮으면 충전이 아닌 할당을 `ASSIGN_BATTERY_LOW`로 거부한다. 서버는 20 % 미만 로봇을 후보에서 빼므로(`min_battery_percent`) 보통은 그 전에 걸러진다
- 아직 없는 것: 0 %에서 멈추기, 적재 질량에 따른 소모 차이

### Groot·작업 로그

- `groot_port`를 주면 BehaviorTree.CPP ZMQ 퍼블리셔를 띄운다 (Groot 1 "Monitor" → 포트 `groot_port`/`groot_port+1`). 로봇마다 다른 포트
- `task_log_file`을 주면 작업이 끝날 때마다 CSV 한 줄: `robot_id, task_id, task_type, item_type, state, phase, error_code, error_message, start_s, end_s, duration_s, distance_m, battery_percent` (명세 8장 작업 로그 필드)

### 도킹 (`Dock`, `DockWithRetry`)

- `Dock`은 Dock.action을 한 번 부른다. 재시도는 트리의 `RetryUntilSuccessful(num_attempts=3)`가 한다 (Dock.action 주석: "3회 재시도는 BT가 한다")
- 3번 다 실패하면 마지막 결과의 `error_code`(900번대, ErrorInfo와 같은 값)와 `"docking at <station> failed 3 times: ..."`로 FAILED를 보낸다. 서버는 FAILED 작업을 다른 로봇에 다시 할당한다(대체 작업, `amr_fleet_manager` `failed_retries`)
- `docking` 단계 보고는 재시도해도 한 번이다

### 아직 없는 것

- 도킹 서버 (`amr_docking`, ArUco 마커 정밀 도킹 2 cm·1°). 지금은 `use_docking:=false` 자리채움이나 가짜 서버로만 돈다
- Nav2를 로봇마다 띄우는 bringup 2단계 (Gazebo에서 한 사이클 확인은 그 뒤)
- 적재 질량을 동역학에 반영 (명세 8장, Gazebo 질량을 실행 중 바꾸는 방법이 필요)
- 구역 허가(`ZoneRequest`/`ZoneGrant`), 분산 모드 전환 (heartbeat 끊김)
- `RobotState.pose`는 `map` → `<robot_id>/base_footprint` TF. 위치 추정(map → odom)이 없으면 원점을 보내고 10초마다 경고
- `TaskStatus.distance_traveled`는 작업 중 `state_rate_hz`(2 Hz)마다 `odom` → `base_footprint` TF 위치 변화를 더한 값이다. map 기준이면 AMCL 보정·kidnap 순간 이동만큼 부풀어서 odom으로 잰다 (배터리 이동 소모도 같다)

## 파라미터

`config/amr_bt.yaml`:

| 이름 | 기본값 | 설명 |
| --- | --- | --- |
| `robot_id` | `""` | 비우면 네임스페이스(`/amr_01` → `amr_01`) |
| `server_topic_prefix` | `/fleet` | launch `relayed:=true`면 `fleet` |
| `tick_hz` | `10.0` | BT tick 주기 |
| `state_rate_hz` | `2.0` | `RobotState` 주기 (1~5 Hz) |
| `global_frame` / `base_frame` | `map` / `""` | `RobotState.pose` TF. `base_frame`을 비우면 `<robot_id>/base_footprint` |
| `odom_frame` | `""` | 이동 거리·배터리용. 비우면 `<robot_id>/odom` |
| `use_nav2` / `use_docking` | `false` / `false` | true면 액션 클라이언트, false면 자리채움. launch 인자로도 켠다 |
| `nav_action` / `dock_action` | `navigate_to_pose` / `docking/dock` | 액션 이름 (네임스페이스 안 상대 이름) |
| `server_wait_s` | `10.0` | 액션 서버가 없을 때 기다리는 시간 [s]. 넘으면 그 시도는 실패 |
| `stub_move_s` / `stub_dock_s` | `5.0` / `1.0` | 자리채움 이동·도킹 시간 [s] |
| `dwell_s` | `2.0` | 싣기·내리기 시간 [s] (가상 적재·하역 이벤트) |
| `nav_timeout_s` | `300.0` | 멈춘 시간을 뺀 이동 시간 상한 [s] |
| `nav_retries` | `2` | Nav2 실패 때 다시 보내는 횟수 |
| `pause_timeout_s` | `60.0` | 구역(`waiting`·`blocked`·`backing`)으로 한 번 멈춘 채 기다리는 상한 [s] |
| `resume_hold_s` | `0.3` | 진행 상태가 이만큼 이어져야 다시 보낸다 [s] |
| `kidnap_pause_timeout_s` | `120.0` | 위치 복구(`settling`·`rotating`)로 한 번 멈춘 채 기다리는 상한 [s] (구역은 `pause_timeout_s`) |
| `kidnap_state_stale_s` | `5.0` | `kidnap_recovery/state`가 이만큼 안 오면 기본값(`tracking`)으로 돌린다 [s] |
| `battery.*` | 위 표 | 가상 배터리 (`initial_percent`, `move_per_m`, `idle_per_s`, `charge_per_s`, `charge_target_percent`, `charge_timeout_s`, `reject_below_percent`) |
| `groot_port` | `0` | Groot 모니터링 포트 (0이면 끔) |
| `task_log_file` | `""` | 작업 로그 CSV (비우면 안 씀) |
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

# 실제 이동·도킹 (Nav2와 도킹 서버가 같은 네임스페이스에 떠 있어야 한다)
ros2 launch amr_bt amr_bt.launch.py namespace:=amr_01 use_nav2:=true use_docking:=true

# 장애 주입기를 거칠 때
ros2 launch amr_fault_injector fault_injector.launch.py delay_ms:=200.0 loss_pct:=5.0
ros2 launch amr_bt amr_bt.launch.py relayed:=true use_sim_time:=false
```

Fleet 서버와 확인한 결과 (작업 5 때, 자리채움 이동 3 s·싣기 1 s, 작업 2건: 배송 → 충전):

| 경로 | 할당 → Ack | 재전송 | 결과 |
| --- | --- | --- | --- |
| 직접 (`/fleet`) | 10 ms 안 | 없음 | 두 작업 모두 PENDING → 단계별 IN_PROGRESS → COMPLETED |
| 장애 주입기 (지연 200 ms, 손실 40 %) | 2.2 s, 1.2 s | 2회, 1회 (같은 `assignment_id`) | 같음. 재전송은 한 번만 수락 |

## 테스트

```bash
colcon test --packages-select amr_bt
```

- `test_assignment_filter`: 새 명령·재전송·뒤늦은 명령, epoch 초기화
- `test_task_cycle` (30개): ROS 없이 가짜 시계와 가짜 이동·도킹으로 트리를 돌린다
  - 단계 순서: 배송(도킹 포함·스테이션 없음), 충전(도킹·충전), 복귀, 연속 작업, 노드 15개 이상
  - 충전: 배터리가 찰 때까지 대기, 충전 시간 초과, 충전 중 취소. 배터리 모델 소모·충전·상하한
  - 도킹: 2번 실패 뒤 성공, 3번 실패 → FAILED(900번대·"failed 3 times"), 작업마다 시도 횟수 새로
  - 이동: Nav2 실패 재시도·한도 초과, 타임아웃, 멈춘 시간은 타임아웃에서 뺌
  - 멈춤·재개: 구역 waiting → 취소 → approaching → 재전송, 한 틱 idle 무시, 같은 상태 반복에 재전송 없음, 멈춤 시간 초과(301), zone failed(302), kidnap 멈춤, kidnap failed 바로 실패(203), kidnap 멈춤 상한 120 s, 시작부터 멈춤
  - 취소: 즉시 FAILED(Nav2·도킹 목표도 취소), 취소 직후 새 작업, 도킹·싣기 중 취소, 다른 작업 취소 무시, 실패 코드 전달
- `test_bt_actions.launch.py` (launch_testing): 실제 `amr_bt_node`를 가짜 Nav2·도킹 액션 서버와 띄운다
  - 배송 한 사이클: 목표 자세 2개·도킹 스테이션 2개가 순서대로 가고 COMPLETED
  - 도킹이 늘 실패하는 스테이션: Dock.action 3번 호출 뒤 FAILED(`FAILED_TO_DETECT_DOCK` 904)
  - 이동 중 `zone_client/state` = waiting → Nav2 목표 취소, approaching → 다시 보내고 COMPLETED
  - `zone_client/state` = failed → `MODE_ERROR`, 새 할당 거부(`TRAFFIC_REVOKED`)
  - `kidnap_recovery/state` = failed → `MODE_ERROR`, 새 할당 거부(`LOCALIZATION_LOST`), tracking으로 돌아오면 다시 받아 COMPLETED
  - 작업 중 `kidnap_recovery/state` = failed → 바로 FAILED(`LOCALIZATION_LOST`)
  - `kidnap_recovery/state` = failed 뒤 끊김 → `kidnap_state_stale_s` 뒤 `MODE_IDLE`
