"""분산 협상(경매) 로직. 설계는 amr_fleet_decentralized README '경매 흐름' 절."""

from .bidding import compute_bid_cost, decide_bid, estimate_finish_time
from .decision import decide_winner, register_bid
from .lifecycle import (
    ERROR_CANCELED, ERROR_TASK_TAKEN, TaskRetryLedger, award_exhausted, can_reclaim, can_retry_failed_task,
    can_withdraw, commit_timed_out, should_resend_award, should_resend_bid, should_withdraw_award,
    task_finished_by_state, winner_lost,
)
from .hybrid import MODE_CENTRALIZED, MODE_DECENTRALIZED, FallbackMode, FallbackParams, backlog_task_ids
from .params import AuctionParams
from .trigger import (
    ASSIGNMENT_ID_BASE, ASSIGNMENT_ID_ROBOT_STRIDE, assignment_id_for, assignment_topic, auction_opener,
    live_robots, robot_index, task_hash,
)
from .types import (
    MODE_BUSY, MODE_CHARGING, MODE_ERROR, MODE_ESTOP, MODE_IDLE, AuctionResult, Bid, RobotInfo,
    TaskInfo,
)

__all__ = [
    'ASSIGNMENT_ID_BASE',
    'ASSIGNMENT_ID_ROBOT_STRIDE',
    'ERROR_CANCELED',
    'ERROR_TASK_TAKEN',
    'MODE_CENTRALIZED',
    'MODE_DECENTRALIZED',
    'FallbackMode',
    'FallbackParams',
    'backlog_task_ids',
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
    'assignment_id_for',
    'assignment_topic',
    'auction_opener',
    'live_robots',
    'robot_index',
    'task_hash',
]
