"""부하균형 할당."""

from __future__ import annotations

from typing import Sequence

from .cost import euclidean_cost
from .types import AssignmentResult, CostFn, RobotInfo, TaskInfo


def assign_load_balance(
    tasks: Sequence[TaskInfo],
    robots: Sequence[RobotInfo],
    cost_fn: CostFn = euclidean_cost,
) -> AssignmentResult:
    """우선순위 순으로 작업을 꺼내, 남은 로봇 중 load가 가장 작은 로봇에 맡긴다.

    load가 같으면 비용이 작은 쪽, 그것도 같으면 robot_id 사전순. O(T·R).
    RobotInfo.load는 바꾸지 않는다. 할당 후 증가는 호출 측(amr_fleet_manager)이 한다.
    """
    # TODO(하연, 2주차): 구현
    raise NotImplementedError
