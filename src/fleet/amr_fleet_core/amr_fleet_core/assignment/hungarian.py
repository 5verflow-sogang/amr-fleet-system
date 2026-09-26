"""Hungarian 할당 (비용 합 최소 매칭)."""

from __future__ import annotations

from typing import Sequence

from .cost import euclidean_cost
from .types import AssignmentResult, CostFn, RobotInfo, TaskInfo


def assign_hungarian(
    tasks: Sequence[TaskInfo],
    robots: Sequence[RobotInfo],
    cost_fn: CostFn = euclidean_cost,
) -> AssignmentResult:
    """우선순위 순 상위 min(T, R)개 작업을 고른 뒤, 비용 합이 최소가 되게 매칭한다.

    작업을 먼저 자르는 이유: 비용만 보면 가까운 작업만 뽑혀 급한 작업이 고아가 됨.
    scipy.optimize.linear_sum_assignment를 쓴다. O(R³), R = 후보 로봇 수.
    """
    # TODO(하연, 2주차): 구현. scipy를 package.xml에 추가 (rosdep 키 python3-scipy)
    raise NotImplementedError
