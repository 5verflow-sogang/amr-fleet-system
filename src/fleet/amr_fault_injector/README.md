# amr_fault_injector

- 담당: 하연
- 빌드 타입: ament_python
- 책임: 통신 지연·패킷 손실·중앙 서버 강제종료 장애 주입기

> 상태: 고정 지연·손실 중계 구현(2주차). 서버 강제종료는 `amr_fleet_manager` launch에 구현(3주차, 아래 미결 표). 분산 경매 토픽(로봇 → 로봇) 중계는 `relay_auction`으로 켠다(10/7). 지터, 로봇 → 서버 방향은 미구현(D8).

## 방식 결정: ROS 레이어 중계

서버 → 로봇 토픽을 이 노드가 받아, 로봇별 토픽으로 늦춰서 다시 발행한다. `tc netem`은 실물 네트워크 고도화(명세 고도화안 1, 이진 주도)에서 쓴다.

| 기준 | ROS 레이어 중계 (채택) | `tc netem` |
| --- | --- | --- |
| 대상 선택 | 토픽 단위로 건다. 하네스 기록·Gazebo·`/clock`은 영향 없음 | 인터페이스 단위. 컨테이너가 모두 `network_mode: host`라 `lo`에 걸면 전부 늦어진다 |
| 로봇별 독립 손실 | 링크(토픽×로봇)마다 따로 뽑는다 | 한 호스트에서는 구분 어려움 |
| 재현성 | `seed`로 같은 메시지가 손실된다 | 커널 난수, 재현 불가 |
| 실행 환경 | macOS Docker Desktop·CI 그대로 | `NET_ADMIN` 권한, Docker Desktop은 VM 안이라 제약 |
| 한계 | DDS 재전송·단편화는 흉내 내지 않는다. 손실은 "메시지가 안 옴"이다. 중계 1홉의 지연이 더해진다 | 실제 패킷 수준이라 현실적 |

- 중계 1홉의 추가 지연은 ros2_tracing으로 따로 재서 연구 1 결과에 적는다 (주입 0ms 조건이 baseline).
- 메시지를 직렬화 바이트 그대로 넘기므로 `header.stamp`와 번호 필드가 바뀌지 않는다 ([api.md 4-3절](../../../docs/api.md#4-3-공통-규칙)).
- 지연은 `use_sim_time`과 상관없이 실제 시간(단조 시계)으로 잰다. 하네스 응답 시간도 실제 시간이다.

## 제공 인터페이스

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Subscribe | `/fleet/task_assignments` | `amr_msgs/msg/TaskAssignment` | 서버 원본 |
| Subscribe | `/fleet/heartbeat` | `amr_msgs/msg/FleetHeartbeat` | 서버 원본 |
| Subscribe | `/fleet/zone_grants` | `amr_msgs/msg/ZoneGrant` | 서버 원본 |
| Subscribe | `/fleet/task_table` | `amr_msgs/msg/FleetTaskTable` | 서버 원본 |
| Subscribe | `/fleet/task_auctions` | `amr_msgs/msg/TaskAuction` | `relay_auction`일 때만. 경매인 로봇 원본 |
| Subscribe | `/fleet/task_bids` | `amr_msgs/msg/TaskBid` | `relay_auction`일 때만. 입찰·낙찰·수락·포기 원본 |
| Publish | `/<robot_id>/fleet/<위 토픽>` | 위와 같음 | 로봇별 중계본. 지연·손실이 걸린 뒤. 경매 토픽은 보낸 로봇에게도 가므로 받는 쪽이 자기 메시지를 거른다 |

**로봇별 출력 (제안, 이진·지수와 합의 필요)**: 로봇 노드는 네임스페이스(`amr_0N`) 안에서 상대 이름 `fleet/task_assignments`를 구독하면 된다. 손실이 로봇마다 독립이라 "일부 로봇만 heartbeat를 놓치는" 상황을 만들 수 있다. 하네스는 지금처럼 `/fleet/*` 원본을 기록한다.

## 파라미터

| 이름 | 기본값 | 위치 | 설명 |
| --- | --- | --- | --- |
| `robot_ids` | `[amr_01, ..., amr_05]` | `config/fault_injector.yaml` | 중계할 로봇 |
| `topics` | 위 4개 | `config/fault_injector.yaml` | 중계할 토픽 (`/fleet/` 뒤 이름) |
| `tick_ms` | `1.0` | `config/fault_injector.yaml` | 지연 큐 확인 주기. 주입 지연 오차가 최대 이만큼 |
| `delay_ms` | `100.0` | launch 인자 | 고정 지연 (명세 범위 0~500) |
| `loss_pct` | `0.0` | launch 인자 | 손실률 (명세 범위 0~10) |
| `seed` | `0` | launch 인자 | 손실 난수 seed. 하네스 반복 번호 |
| `relay_auction` | `false` | launch 인자 | 분산 경매 토픽도 중계한다. 분산·하이브리드 비교군 실험에서 켠다. 중앙집중 실험은 끈다 |

QoS는 4개 토픽 모두 Reliable, depth 10이다. [api.md 6절](../../../docs/api.md#6-qos-기본값)이 정해지면 토픽별로 맞춘다.

## 실행

```bash
ros2 launch amr_fault_injector fault_injector.launch.py delay_ms:=200.0 loss_pct:=5.0 seed:=3
```

### 고정 지연 동작 확인

서버 없이 heartbeat를 손으로 발행하고, 중계본이 지연만큼 늦게 오는지 본다.

```bash
# 터미널 1
ros2 launch amr_fault_injector fault_injector.launch.py delay_ms:=200.0
# 터미널 2: 원본 10Hz 발행 (header: auto는 발행 시각을 stamp에 넣는다)
ros2 topic pub -r 10 /fleet/heartbeat amr_msgs/msg/FleetHeartbeat "{header: auto, server_id: test}"
# 터미널 3: stamp 대비 수신 지연. 원본은 수 ms, 중계본은 약 0.2s
ros2 topic delay /fleet/heartbeat
ros2 topic delay /amr_01/fleet/heartbeat
```

## 테스트

```bash
colcon test --packages-select amr_fault_injector
```

`test/test_fault_model.py`는 rclpy 없이 지연 큐·손실률·seed 재현성을 확인한다.

## 미결

| 쟁점 | 초안 |
| --- | --- |
| 로봇별 출력 토픽 | 위 제안. 이진(`ZoneRequest`·heartbeat 폴백), 지수(`amr_bt`)와 합의 |
| 서버 강제종료 | 구현. launch는 자기가 띄운 프로세스에만 신호를 보낼 수 있어 `fleet_manager.launch.py`의 `kill_at_s`·`restart_after_s` 인자로 건다 ([amr_fleet_manager README](../amr_fleet_manager/README.md) "서버 강제종료") |
| 로봇 → 서버 방향 | D8 결정: 걸지 않는다. 연구 1 After 때 필요하면 역방향 링크 추가 |
| 로봇 → 로봇 (분산 경매) | `relay_auction`으로 건다(10/7). 분산 모드에는 서버가 없어, 걸지 않으면 같은 지연 조건에서 분산만 지연을 겪지 않아 중앙집중과의 비교가 불공정해진다 |
| 지터 | 고정 지연만. 지터를 넣으면 FIFO가 깨지므로 큐를 도착 시각 순 힙으로 바꾼다 |
