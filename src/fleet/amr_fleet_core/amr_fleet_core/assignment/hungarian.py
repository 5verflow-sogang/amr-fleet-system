"""Hungarian 할당 (비용 합 최소 매칭)."""

from __future__ import annotations

from typing import Sequence

from scipy.optimize import linear_sum_assignment

from .common import check_unique_ids, sort_by_priority
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
    비용 합이 같은 매칭이 여럿이면 scipy가 앞 인덱스끼리 묶으므로,
    로봇을 robot_id 순으로 넣어 결과를 고정한다.
    """
    check_unique_ids(tasks, robots)
    ordered = sort_by_priority(tasks)
    robots = sorted(robots, key=lambda r: r.robot_id)
    k = min(len(ordered), len(robots))
    selected = ordered[:k]
    result = AssignmentResult(unassigned=[t.task_id for t in ordered[k:]])
    if k == 0:
        return result

    cost = [[cost_fn(r, t) for r in robots] for t in selected]
    rows, cols = linear_sum_assignment(cost)
    for i, j in zip(rows, cols):
        result.assignments[selected[i].task_id] = robots[j].robot_id
        result.total_cost += cost[i][j]
    return result
