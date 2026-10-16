# amr_fleet_decentralized

- 담당: 하연, 이진
- 빌드 타입: ament_python
- 책임: 분산 협상 기반 작업 할당과 중앙 서버 장애 시 폴백 (연구 1 비교군)

## 제공 인터페이스

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Publish | `/fleet/task_auctions` | `amr_msgs/msg/TaskAuction` | 경매 공고. 재경매 때도 여기로 보낸다 |
| Publish | `/fleet/task_bids` | `amr_msgs/msg/TaskBid` | 입찰(BID)·낙찰 통보(AWARD)·수락(ACCEPT)·포기(WITHDRAW) |
| Subscribe | `/fleet/task_auctions` (파라미터 `auction_topic_prefix`, 기본 `/fleet`) | `amr_msgs/msg/TaskAuction` | 기본은 절대 경로 원본. `relayed:=true`(launch) 또는 `auction_topic_prefix:=fleet`면 상대 이름 → 장애 주입기 `relay_auction` 중계본을 구독 (`amr_bt`·가짜 로봇의 `relayed` 인자와 같은 방식) |
| Subscribe | `/fleet/task_bids` (파라미터 `auction_topic_prefix`) | `amr_msgs/msg/TaskBid` | 위와 동일 |
| Subscribe | `/fleet/task_status` | `amr_msgs/msg/TaskStatus` | ACCEPT 뒤 낙찰 로봇이 보낸 FAILED를 보고 재경매 여부 판단 (relay 목록에 없어 항상 절대 경로) |
| Subscribe | `/fleet/robot_states` (파라미터 `robot_states_topic`) | `amr_msgs/msg/RobotState` | 내 로봇 것만 걸러서 입찰 판단(위치·배터리·IDLE 여부)에 쓴다 |

전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

### 노드

| 실행 파일 | 노드 이름 | 역할 |
| --- | --- | --- |
| `task_auction_node` | `task_auction` | 공고 수신 → 입찰 판단·발행, 입찰 수집(경매인일 때) → 낙찰·재전송·재경매, 수락/포기 |

## 경매 흐름

`amr_fleet_decentralized/auction/`에 서버 장애 폴백 시 로봇끼리 작업을 나누는 경매
로직을 뒀다. **ROS 비의존**(amr_msgs를 import하지 않는다) - `task_auction_node`가
메시지 ↔ 이 타입 변환과 경매별 상태(진행 중/낙찰/완료) 관리만 맡고, 판단은 전부
여기에 위임한다(`amr_nav2_plugins::DwaController`가 `amr_planning_core`에 위임하는
것과 같은 패턴). 메시지 스키마·흐름·동점 규칙은 `amr_msgs/msg/TaskAuction.msg`·
`TaskBid.msg`와 `docs/api.md`(§4-4, "분산 경매 확정 절차")에 정해져 있고, 이 모듈은
그대로 따른다:

```
공고(TaskAuction)
  -> 참여 로봇이 decide_bid()로 입찰할지 정함 (IDLE 아니면·배터리 부족·마감 불가면 입찰 안 함)
  -> BID는 마감까지 bid_resend_period_s 간격으로 재전송(1회성 발행의 디스커버리 레이스
     유실 대응, #155·#171) - AWARD를 받으면 멈춘다. 경매인은 같은 로봇의 재전송을 중복으로
     세지 않는다(register_bid)
  -> 경매인이 마감(bid_window_s, "경매인이 받은 시각" 기준) 전 입찰만 모아
     decide_winner()로 낙찰자 결정 (최저가, 동점이면 robot_id 사전순 — TaskBid.msg 주석과 동일)
  -> 낙찰 통보(AWARD, 1초 간격 x 5회 재전송)
  -> 낙찰 로봇 수락(ACCEPT) 또는 그사이 사정이 바뀌면 포기(WITHDRAW, ACCEPT 전까지만).
     이미 다른 작업을 ACCEPT한 로봇은 새 AWARD를 WITHDRAW한다(동시 낙찰 경합)
  -> 낙찰자가 WITHDRAW하면 같은 경매의 다음 순위 입찰자에게 AWARD, 남은 입찰자가 없으면 재경매
  -> ACCEPT 없이 재전송을 다 쓰면 재경매(auction_id를 올려 다시 공고)
  -> ACCEPT 뒤 수행 중 TaskStatus FAILED를 받으면 최대 failed_retries번(기본 1) 재경매,
     그래도 실패하면 실패로 남긴다
```

10/7 하연·이진 결정(`docs/api.md` 4-4절 "쟁점 정리" 표, #91 리뷰 제안을 그대로 채택):

- **입찰 자격**: IDLE 로봇만(명세 3장 "가장 가까운 로봇 선점"). `RobotInfo.mode`가
  `MODE_IDLE`(`amr_msgs/msg/RobotState.MODE_*`와 같은 값)이 아니면 입찰하지 않는다.
- **입찰가(`compute_bid_cost`)**: 현재 위치 → 작업 시작점(픽업, 없으면 목적지) 직선
  거리. 중앙 할당(`amr_fleet_core.assignment.cost.euclidean_cost`, `nearest` 규칙)과
  같은 식이다 - 분산·중앙 비교군의 비용 계산이 같아야 공정하다(연구 1). IDLE 로봇만
  입찰해서 "진행 중인 작업을 마치는 지점"(큐) 보정은 필요 없어졌다.
- **배터리 하한**: 중앙집중(`fleet_manager.yaml` `min_battery_percent`)과 같은 20%.
- **입찰 마감 판정**: 경매인이 "받은" 시각 기준(보낸 로봇의 자기 신고 시각이 아니다) -
  시계 오차·네트워크 지연이 있는 로봇의 타임스탬프를 믿으면 마감 판정이 공정하지 않다.
  `task_auction_node`는 `TaskBid`를 받는 즉시 자기 시계로 `Bid.bid_time`을 채운다.
- **AWARD 재전송**: 중앙 Ack 재전송과 같게 1초 간격 x 5회(재전송 정책이 같아야 지연
  내성 비교가 공정하다) - 그래도 ACCEPT가 없으면 재경매.
- **WITHDRAW 범위**: ACCEPT 전까지만. 수락 뒤 못 하게 되면 `TaskStatus` FAILED로
  보고하고(`amr_bt`가 발행) 경매인이 재경매한다 - 아래 "ACCEPT 뒤 실패 재시도" 참고.

10/16 동시 낙찰 경합:

- **문제**: 경매가 동시에 여러 개 열리면 IDLE 로봇 하나가 여러 경매에 입찰해 둘 다 최저가로
  낙찰될 수 있다. 수락한 뒤에도 그 로봇의 `RobotState`는 `amr_bt`가 작업을 시작하기 전까지
  IDLE이라, AWARD 때의 재확인(`decide_bid`)만으로는 못 거른다.
- **로봇 쪽**: 이미 ACCEPT한 작업이 있는 로봇은 새 AWARD에 ACCEPT 대신 WITHDRAW를 보낸다
  (`should_withdraw_award`). 노드는 단일 스레드라 ACCEPT를 보내는 순간 "맡은 작업"으로 기록해서
  ACCEPT 처리 중인 경우도 같다. 한 번 WITHDRAW한 경매는 재전송된 AWARD가 와도 다시 WITHDRAW한다 -
  경매인이 이미 다음 순위에게 줬을 수 있어 뒤늦게 수락하면 같은 작업을 두 로봇이 맡는다. 맡은
  작업이 있는 동안은 새 공고에 입찰하지 않고(IDLE 로봇만 입찰), 자기 `TaskStatus` COMPLETED/FAILED로
  그 작업이 끝나면 다시 입찰한다.
- **경매인 쪽**: 낙찰자의 WITHDRAW를 받으면 같은 경매의 입찰 목록에서 WITHDRAW한 로봇들을 빼고
  `decide_winner(..., excluded=...)`로 다음 순위에게 AWARD한다(재전송 횟수는 새로 센다). 남은
  입찰자가 없으면 새 `auction_id`로 재경매한다(기존 재경매 규칙과 같음). WITHDRAW한 로봇의
  재전송 BID는 그 경매에서 다시 세지 않는다.

10/9 하연·이진 결정(#107·#108):

- **ACCEPT 뒤 실패 재시도**: 수락한 작업을 `amr_bt`가 수행하다 `TaskStatus` FAILED를
  보내면, 경매인이 새 `auction_id`로 최대 `failed_retries`번(기본 1) 재경매한다.
  그래도 또 FAILED면 더 재경매하지 않고 실패로 남긴다 - 중앙집중
  (`fleet_manager.yaml` `assignment.failed_retries`)과 같은 규칙·기본값이어야
  두 비교군의 재시도 조건이 같다. 재시도 횟수는 `task_id`로 세어(`auction_id`가
  재경매로 바뀌어도 이어서 센다) 중앙의 `TaskEntry.failures`와 같은 역할을 한다.
  단 취소(`ErrorInfo.ASSIGN_CANCELED`)로 온 FAILED는 재경매하지 않고 실패로 남기며 횟수에도
  세지 않는다 - 중앙 #107(취소된 작업은 다시 할당하지 않음)과 같은 규칙.
- **낙찰 로봇 장애 회수**: ACCEPT한 낙찰 로봇의 `RobotState`가 `reclaim_timeout_s`(5초, 실제 시간) 동안
  끊기면 경매인이 새 `auction_id`로 재경매한다 - 중앙 #132(작업 회수)와 같은 규칙으로, FAILED 재시도와 따로
  세고 상한이 없다(`max_reclaims`). 옛 낙찰 로봇이 늦게 돌아와 보낸 보고는 무시된다. COMPLETED를 받은 작업은
  감시하지 않는다. 경매인 자신이 죽으면 그 경매를 대신 볼 로봇이 없다(서버 장애 폴백 설계와 같이 정한다).
- **구독 토픽 접두어**: 기본은 절대 경로 원본(`/fleet/task_auctions`,
  `/fleet/task_bids`)을 직접 구독한다. `relayed:=true`(launch 인자, 또는 파라미터
  `auction_topic_prefix:=fleet`)면 상대 이름(장애 주입기 `relay_auction` 중계본)을
  구독한다 - `amr_bt`·가짜 로봇의 `relayed` 인자와 이름·동작을 맞췄다. 발행은
  항상 절대 경로 원본(장애 주입기가 읽어 로봇별로 다시 보낸다). `TaskStatus`는
  장애 주입기 relay 목록에 없어 `relayed`와 무관하게 항상 절대 경로다.

## 확인 필요 (하연과 맞출 부분)

- **경매를 언제 새로 여는지**: 이 노드는 아직 "서버 장애 감지 + 미할당 작업 발견 →
  내가 경매를 연다" 트리거가 없다 - `TaskAuction`을 받았을 때(자기 이름으로 온 것
  포함) 반응하는 것과, 재전송을 다 쓰고도 ACCEPT가 없을 때 재경매하는 것만 한다.
  `FleetTaskTable` 미할당 작업 감지, `FleetHeartbeat` 끊김 판단, 여러 로봇이 같은
  작업을 동시에 경매하지 않게 하는 조정(리더 선출?)은 다음 단계.
- **여러 경매 동시 진행 시 자원 경합**: 한 로봇이 동시에 여러 경매에 입찰해 둘 다
  낙찰되면(각 경매는 독립적으로 판단한다) 실제로 둘 다 못 할 수 있다 - 아직 처리
  안 함.

## 파라미터

`config/` 의 YAML로 관리한다. 파라미터 변경에 재빌드가 필요 없어야 한다.

`config/task_auction.yaml`은 `auction/params.py`의 `AuctionParams`와 1:1로 맞춘다.

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `robot_id` | 노드 네임스페이스 | `TaskBid.robot_id` 등에 쓴다 |
| `bid_window_s` | `2.0` | 공고~마감(`TaskAuction.bid_window`) |
| `min_battery_percent` | `20.0` | 이 미만이면 입찰하지 않는다 (10/7 결정) |
| `robot_speed_mps` | `1.0` | `estimated_finish_time` 추정용 단순 등속 |
| `award_resend_interval_s` / `award_max_attempts` | `1.0` / `5` | AWARD 재전송 간격·횟수 (10/7 결정) |
| `bid_resend_period_s` | `0.5` | BID 재전송 간격. 마감(`bid_window_s`)까지, AWARD를 받거나 마감되면 멈춤 (#155·#171) |
| `failed_retries` | `1` | ACCEPT 뒤 FAILED 보고 시 재경매 최대 횟수 (10/9 결정) |
| `reclaim_timeout_s` | `5.0` | ACCEPT한 낙찰 로봇의 `RobotState`가 이 시간(실제 시간, s) 동안 끊기면 재경매. 0이면 끄기 (중앙 #132와 같음) |
| `max_reclaims` | `-1` | 장애 회수 재경매 상한. 음수면 상한 없음 (#132와 같음) |
| `auction_topic_prefix` | `/fleet` | 구독 접두어. `fleet`면 중계본(`relayed:=true`와 동일) |
| `robot_states_topic` | `/fleet/robot_states` | 내 로봇 상태(위치·배터리·모드) |
| `update_rate_hz` | `10.0` | 마감 판정·AWARD 재전송·재경매를 확인하는 주기 |

## 실행

```bash
ros2 launch amr_fleet_decentralized task_auction.launch.py namespace:=amr_01 robot_id:=amr_01
ros2 launch amr_fleet_decentralized task_auction.launch.py relayed:=true   # 장애 주입기 중계본 구독
```

## 테스트

핵심 로직(`auction/`)은 ROS 비의존이라 Docker/colcon 없이 로컬 python으로 바로
돌릴 수 있다:

```bash
pip install -e ".[test]"
pytest test/test_auction.py
```

`task_auction_node`는 `test/test_task_auction_launch.py`에서 로봇 2대를 실제로
띄워 공고 → 입찰 → 낙찰 → 수락 전체 흐름을 확인한다. 동시 낙찰 경합은
`test/test_task_auction_node.py`에서 노드 여러 개를 가짜 버스로 이어 메시지 순서를 정해 놓고
확인한다(둘 다 rclpy가 필요해 로컬 python으로는 못 돌리고 colcon 환경에서만 된다):

```bash
colcon test --packages-select amr_fleet_decentralized
```
