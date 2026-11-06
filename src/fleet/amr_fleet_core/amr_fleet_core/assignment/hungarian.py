"""Hungarian 할당 (비용 합 최소 매칭)."""

from __future__ import annotations

from typing import Sequence

from scipy.optimize import linear_sum_assignment

from .common import check_unique_ids, sort_by_priority
from .cost import euclidean_cost
from .types import AssignmentResult, CostFn, FeasibleFn, RobotInfo, TaskInfo

# 맡길 수 없는 짝의 비용. 다른 짝을 모두 합친 것보다 커서 가능한 짝이 있으면 그쪽을 고른다
_INFEASIBLE = 1e12


def assign_hungarian(
    tasks: Sequence[TaskInfo],
    robots: Sequence[RobotInfo],
    cost_fn: CostFn = euclidean_cost,
    feasible: FeasibleFn = None,
) -> AssignmentResult:
    """우선순위 순 상위 min(T, R)개 작업을 고른 뒤, 비용 합이 최소가 되게 매칭한다.

    작업을 먼저 자르는 이유: 비용만 보면 가까운 작업만 뽑혀 급한 작업이 고아가 됨.
    scipy.optimize.linear_sum_assignment를 쓴다. O(R³), R = 후보 로봇 수.
    비용 합이 같은 매칭이 여럿이면 scipy가 앞 인덱스끼리 묶으므로,
    로봇을 robot_id 순으로 넣어 결과를 고정한다.
    feasible이 있으면 어느 로봇도 맡을 수 없는 작업은 먼저 빼고(unassigned), 맡을 수 없는 짝은 아주 큰 비용으로 넣어
    매칭에 들어오면 버린다(그 작업은 unassigned).
    """
    check_unique_ids(tasks, robots)
    ordered = sort_by_priority(tasks)
    robots = sorted(robots, key=lambda r: r.robot_id)
    infeasible = []
    if feasible is not None:
        able = {t.task_id: [feasible(r, t) for r in robots] for t in ordered}
        infeasible = [t.task_id for t in ordered if not any(able[t.task_id])]
        ordered = [t for t in ordered if any(able[t.task_id])]
    k = min(len(ordered), len(robots))
    selected = ordered[:k]
    result = AssignmentResult(unassigned=infeasible + [t.task_id for t in ordered[k:]])
    if k == 0:
        return result

    cost = [[cost_fn(r, t) if feasible is None or able[t.task_id][j] else _INFEASIBLE
             for j, r in enumerate(robots)] for t in selected]
    rows, cols = linear_sum_assignment(cost)
    for i, j in zip(rows, cols):
        if cost[i][j] >= _INFEASIBLE:
            result.unassigned.append(selected[i].task_id)
            continue
        result.assignments[selected[i].task_id] = robots[j].robot_id
        result.total_cost += cost[i][j]
    return result
