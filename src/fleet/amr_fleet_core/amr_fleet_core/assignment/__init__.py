"""작업 할당. 설계는 amr_fleet_core README '작업 할당' 절."""

from .common import check_unique_ids, filter_candidates, sort_by_priority
from .cost import euclidean_cost
from .hungarian import assign_hungarian
from .load_balance import assign_load_balance
from .nearest import assign_nearest
from .types import AssignmentResult, CostFn, RobotInfo, TaskInfo

# amr_fleet_manager 파라미터 assignment.algorithm 값 -> 함수
ALGORITHMS = {
    'nearest': assign_nearest,
    'load_balance': assign_load_balance,
    'hungarian': assign_hungarian,
}

__all__ = [
    'ALGORITHMS',
    'AssignmentResult',
    'CostFn',
    'RobotInfo',
    'TaskInfo',
    'assign_hungarian',
    'assign_load_balance',
    'assign_nearest',
    'check_unique_ids',
    'euclidean_cost',
    'filter_candidates',
    'sort_by_priority',
]
