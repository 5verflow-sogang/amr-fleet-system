# amr_fleet_decentralized

- 담당: 하연, 이진
- 빌드 타입: ament_python
- 책임: 분산 협상 기반 작업 할당과 중앙 서버 장애 시 폴백 (연구 1 비교군)

## 제공 인터페이스

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Publish | | | |
| Subscribe | | | |
| Service | | | |
| Action | | | |

전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

## 경매 흐름 (초안)

`amr_fleet_decentralized/auction/`에 서버 장애 폴백 시 로봇끼리 작업을 나누는
경매 로직을 뒀다. **ROS 비의존**(amr_msgs를 import하지 않는다) - 아직 실제
토픽을 구독·발행하는 노드는 없고, 다음에 노드로 감쌀 순수 로직만 초안으로
먼저 짰다. 메시지 스키마·흐름·동점 규칙은 `amr_msgs/msg/TaskAuction.msg`·
`TaskBid.msg`와 `docs/api.md`(§4-4, "분산 경매 확정 절차")에 이미 정해져 있고,
이 모듈은 그대로 따른다:

```
공고(TaskAuction)
  -> 참여 로봇이 decide_bid()로 입찰할지 정함 (배터리 부족·마감 불가면 입찰 안 함)
  -> 경매인이 마감(bid_window_s) 전 입찰만 모아 decide_winner()로 낙찰자 결정
     (최저가, 동점이면 robot_id 사전순 — TaskBid.msg 주석과 동일)
  -> 낙찰 통보(AWARD) -> 낙찰 로봇 수락(ACCEPT)
```

- `decide_bid()`의 입찰가(`compute_bid_cost`)는 TaskBid.cost("한계비용")의 정의
  그대로 "현재 큐 끝(진행 중인 작업을 마치는 지점) → pickup → dropoff" 증가분
  이다 - 픽업까지 거리 추정과 현재 작업 남은 거리가 여기 다 들어간다.
- AWARD(낙찰 통보)·ACCEPT(수락)·WITHDRAW(포기)를 토픽으로 주고받는 실제 노드,
  그리고 결과를 어디(FleetTaskTable? 로그?)에 통지할지는 아직 안 짰다 - 다음
  단계.

### 확인 필요 (하연과 맞출 부분)

- **배터리 임계값(`min_battery_percent`)**: 지금은 중앙 할당(`amr_fleet_core`,
  `fleet_manager.yaml`)과 같은 20.0을 그대로 썼다. 분산 폴백 상황(서버 이미
  장애)에서 같은 값을 쓸지, 더 낮춰야 할지 하연과 합의 필요.
  `docs/project-spec.md:781`의 "분산 협상 로직 최소 스코프 팀 합의" TODO와
  같은 항목.
- **"입찰 안 함"과 WITHDRAW의 구분**: 지금 `decide_bid()`는 배터리 부족·마감
  불가 시 그냥 입찰을 안 보낸다(None). 이미 낙찰된 뒤에 사정이 바뀌어 포기하는
  경우(TYPE_WITHDRAW)는 이 모듈에 아직 없다 - 노드를 짤 때 하연과 범위 확인.
- **AWARD 유실 시 재통보**: TaskBid.msg 주석의 "낙찰 통보가 유실되면 경매인만
  할당됐다고 믿게 되므로 수락까지 받아야 확정한다"는 재시도/타임아웃 정책이
  필요하다는 뜻인데, 구체적인 재시도 횟수·주기는 아직 정해진 바 없음 - 하연과
  확인 필요.

## 파라미터

`config/` 의 YAML로 관리한다. 파라미터 변경에 재빌드가 필요 없어야 한다.

`config/task_auction.yaml`은 `auction/params.py`의 `AuctionParams`와 1:1로
맞춘다 - 아직 이 파라미터를 로드하는 노드가 없어 지금은 실제로 쓰이지 않는
초안이다.

## 실행

```bash
# TODO: 노드 구현 후 작성
```

## 테스트

Docker/colcon 없이 로컬 python으로 바로 돌릴 수 있다(ROS 비의존 순수 로직):

```bash
pip install -e ".[test]"
pytest test/
```

colcon 환경에서는 그대로:

```bash
colcon test --packages-select amr_fleet_decentralized
```
