"""작업 할당. 설계는 amr_fleet_core README '작업 할당' 절."""

from .common import check_unique_ids, filter_candidates, sort_by_priority
from .cost import CongestedArea, euclidean_cost, make_congestion_cost, segment_length_in_rect
from .hungarian import assign_hungarian
from .load_balance import assign_load_balance
from .nearest import assign_nearest
from .types import PICKUP_TYPES, AssignmentResult, CostFn, FeasibleFn, RobotInfo, TaskInfo, has_pickup

# amr_fleet_manager 파라미터 assignment.algorithm 값 -> 함수
ALGORITHMS = {
    'nearest': assign_nearest,
    'load_balance': assign_load_balance,
    'hungarian': assign_hungarian,
}

# amr_fleet_manager 파라미터 assignment.cost_fn 값
COST_FNS = ('euclidean', 'congestion')

__all__ = [
    'ALGORITHMS',
    'PICKUP_TYPES',
    'AssignmentResult',
    'COST_FNS',
    'CongestedArea',
    'CostFn',
    'FeasibleFn',
    'RobotInfo',
    'TaskInfo',
    'assign_hungarian',
    'assign_load_balance',
    'assign_nearest',
    'check_unique_ids',
    'euclidean_cost',
    'filter_candidates',
    'make_congestion_cost',
    'segment_length_in_rect',
    'sort_by_priority',
]
