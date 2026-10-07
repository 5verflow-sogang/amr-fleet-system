"""경매인(auctioneer) 쪽 낙찰 결정. 입찰 수집이 끝난 뒤(또는 마감 시각이 지난 뒤) 호출한다."""
from typing import Sequence

from .types import AuctionResult, Bid


def decide_winner(
    task_id: str,
    bids: Sequence[Bid],
    announce_time_s: float,
    bid_window_s: float,
) -> AuctionResult:
    """마감(`announce_time_s + bid_window_s`) 전에 들어온 입찰만으로 최저가 낙찰을 정한다.

    - 마감 후 도착한 입찰(`bid_time`이 마감보다 늦음)은 무효로 치고 뺀다.
    - 유효한 입찰이 하나도 없으면(아예 없거나 전부 늦게 왔으면) `winner_id=None`.
    - 최저가가 여럿이면 `robot_id`가 사전순으로 가장 앞선 로봇이 이긴다
      (TaskBid.msg 주석: "동점이면 robot_id가 사전순으로 앞선 로봇이 이긴다").
    """
    deadline = announce_time_s + bid_window_s
    valid_bids = [b for b in bids if b.bid_time <= deadline]

    if not valid_bids:
        return AuctionResult(task_id=task_id, winner_id=None)

    winner = min(valid_bids, key=lambda b: (b.cost, b.robot_id))

    return AuctionResult(
        task_id=task_id,
        winner_id=winner.robot_id,
        winning_cost=winner.cost,
        winning_bid=winner,
    )
