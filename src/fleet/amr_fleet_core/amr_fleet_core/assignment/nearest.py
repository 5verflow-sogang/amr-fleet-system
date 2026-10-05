"""최소거리 할당 (탐욕)."""

from __future__ import annotations

from typing import Sequence

from .common import assign_greedy
from .cost import euclidean_cost
from .types import AssignmentResult, CostFn, RobotInfo, TaskInfo


def assign_nearest(
    tasks: Sequence[TaskInfo],
    robots: Sequence[RobotInfo],
    cost_fn: CostFn = euclidean_cost,
) -> AssignmentResult:
    """우선순위 순으로 작업을 하나씩 꺼내, 남은 로봇 중 비용이 가장 작은 로봇에 맡긴다.

    비용이 같으면 robot_id 사전순. O(T·R).
    """
    return assign_greedy(tasks, robots, cost_fn, rank=lambda r, cost: (cost, r.robot_id))
