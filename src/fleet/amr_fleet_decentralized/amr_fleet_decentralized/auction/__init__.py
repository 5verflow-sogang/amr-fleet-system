"""분산 협상(경매) 로직. 설계는 amr_fleet_decentralized README '경매 흐름' 절."""

from .bidding import compute_bid_cost, decide_bid, estimate_finish_time
from .decision import decide_winner
from .lifecycle import award_exhausted, can_retry_failed_task, can_withdraw, should_resend_award
from .params import AuctionParams
from .types import (
    MODE_BUSY, MODE_CHARGING, MODE_ERROR, MODE_ESTOP, MODE_IDLE, AuctionResult, Bid, RobotInfo,
    TaskInfo,
)

__all__ = [
    'MODE_BUSY',
    'MODE_CHARGING',
    'MODE_ERROR',
    'MODE_ESTOP',
    'MODE_IDLE',
    'AuctionParams',
    'AuctionResult',
    'Bid',
    'RobotInfo',
    'TaskInfo',
    'award_exhausted',
    'can_retry_failed_task',
    'can_withdraw',
    'compute_bid_cost',
    'decide_bid',
    'decide_winner',
    'estimate_finish_time',
    'should_resend_award',
]
