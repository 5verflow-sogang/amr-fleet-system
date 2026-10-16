"""경매인(auctioneer) 쪽 입찰 수집·낙찰 결정."""
from typing import Collection, List, Sequence

from .types import AuctionResult, Bid


def register_bid(bids: Sequence[Bid], bid: Bid) -> List[Bid]:
    """입찰 목록에 새 입찰을 더한다.

    #155·#171 BID 재전송(should_resend_bid)과 짝이다: 같은 로봇이 같은 경매에
    재전송으로 BID를 또 보내도 중복으로 세지 않는다 - 이미 그 로봇의 입찰이
    있으면 그대로 두고 새로 받은 건 버린다. 재전송은 cost가 같아서 버려도
    낙찰 결과는 안 바뀌고, 먼저 받은 입찰의 bid_time을 그대로 둬야 마감
    판정(decide_winner)이 맞다 - 나중에 도착한(마감을 넘겼을 수도 있는) 재전송으로
    덮어쓰면 원래 마감 전에 들어온 유효한 입찰이 무효로 바뀔 수 있다.
    """
    if any(b.robot_id == bid.robot_id for b in bids):
        return list(bids)
    return [*bids, bid]


def decide_winner(
    task_id: str,
    bids: Sequence[Bid],
    announce_time_s: float,
    bid_window_s: float,
    excluded: Collection[str] = (),
) -> AuctionResult:
    """마감(`announce_time_s + bid_window_s`) 전에 들어온 입찰만으로 최저가 낙찰을 정한다.

    - 마감 후 도착한 입찰(`bid_time`이 마감보다 늦음)은 무효로 치고 뺀다. 10/7
      결정: "마감 후 도착"은 **경매인이 받은 시각** 기준이다 - 호출 쪽(노드)이
      다른 로봇의 TaskBid를 Bid로 옮길 때 `bid_time`에 보낸 쪽 타임스탬프가 아니라
      자기(경매인) 수신 시각을 넣어야 한다(`types.Bid` 참고). 시계 오차·네트워크
      지연이 있는 로봇의 자기 신고 시각을 믿으면 마감 판정이 공정하지 않다.
    - 유효한 입찰이 하나도 없으면(아예 없거나 전부 늦게 왔으면) `winner_id=None`.
    - 최저가가 여럿이면 `robot_id`가 사전순으로 가장 앞선 로봇이 이긴다
      (TaskBid.msg 주석: "동점이면 robot_id가 사전순으로 앞선 로봇이 이긴다").
    - `excluded`(이 경매를 WITHDRAW한 로봇)는 뺀다 - 낙찰자가 포기하면 같은 입찰 목록에서
      다음 순위를 고르는 데 쓴다(동시 낙찰 경합). 남는 입찰이 없으면 `winner_id=None`
      (호출 쪽이 재경매한다).
    """
    deadline = announce_time_s + bid_window_s
    valid_bids = [b for b in bids if b.bid_time <= deadline and b.robot_id not in excluded]

    if not valid_bids:
        return AuctionResult(task_id=task_id, winner_id=None)

    winner = min(valid_bids, key=lambda b: (b.cost, b.robot_id))

    return AuctionResult(
        task_id=task_id,
        winner_id=winner.robot_id,
        winning_cost=winner.cost,
        winning_bid=winner,
    )
