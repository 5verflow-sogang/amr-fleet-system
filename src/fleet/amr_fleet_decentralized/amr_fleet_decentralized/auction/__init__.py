"""분산 협상(경매) 로직. 설계는 amr_fleet_decentralized README '경매 흐름' 절."""

from .bidding import compute_bid_cost, decide_bid, estimate_finish_time
from .decision import decide_winner, register_bid
from .lifecycle import (
    ERROR_CANCELED, TaskRetryLedger, award_exhausted, can_reclaim, can_retry_failed_task,
    can_withdraw, commit_timed_out, should_resend_award, should_resend_bid, should_withdraw_award,
    task_finished_by_state, winner_lost,
)
from .params import AuctionParams
from .types import (
    MODE_BUSY, MODE_CHARGING, MODE_ERROR, MODE_ESTOP, MODE_IDLE, AuctionResult, Bid, RobotInfo,
    TaskInfo,
)

__all__ = [
    'ERROR_CANCELED',
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
    'TaskRetryLedger',
    'award_exhausted',
    'can_reclaim',
    'can_retry_failed_task',
    'can_withdraw',
    'commit_timed_out',
    'compute_bid_cost',
    'decide_bid',
    'decide_winner',
    'estimate_finish_time',
    'register_bid',
    'should_resend_award',
    'should_resend_bid',
    'should_withdraw_award',
    'task_finished_by_state',
    'winner_lost',
]
