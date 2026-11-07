"""알고리즘 3종이 함께 쓰는 전처리: 후보 거르기, 우선순위 정렬, 입력 검사. 탐욕 할당 루프."""

from __future__ import annotations

import math
from typing import Callable, Optional, Sequence

from .types import MODE_IDLE, Aging, AssignmentResult, CostFn, FeasibleFn, RobotInfo, TaskInfo


def filter_candidates(robots: Sequence[RobotInfo], min_battery_percent: float) -> list[RobotInfo]:
    """IDLE이고 배터리가 하한 이상인 로봇만 남긴다. robot_id 순으로 정렬해 돌려준다."""
    candidates = [
        r for r in robots
        if r.mode == MODE_IDLE and r.battery_percent >= min_battery_percent
    ]
    return sorted(candidates, key=lambda r: r.robot_id)


def sort_by_priority(tasks: Sequence[TaskInfo], aging: Optional[Aging] = None) -> list[TaskInfo]:
    """priority 내림차순 → deadline 오름차순 (None은 맨 뒤) → created 오름차순 → task_id.

    aging을 주면 priority 대신 유효 우선순위(Aging.effective = priority + rate × 대기 시간)로 줄 세운다. 유효 우선순위가
    같으면 뒤 키는 그대로라, 같은 값이 된 순간에는 먼저 들어온(created가 작은) 작업이 앞선다. aging이 없거나 rate가
    0이면 지금 순서와 똑같다.
    """
    if aging is None or aging.rate == 0:
        def first(t: TaskInfo) -> float:
            return -t.priority
    else:
        def first(t: TaskInfo) -> float:
            return -aging.effective(t)
    return sorted(
        tasks,
        key=lambda t: (
            first(t),
            t.deadline if t.deadline is not None else math.inf,
            t.created,
            t.task_id,
        ),
    )


def check_unique_ids(tasks: Sequence[TaskInfo], robots: Sequence[RobotInfo]) -> None:
    """task_id나 robot_id가 중복되면 ValueError."""
    task_ids = [t.task_id for t in tasks]
    if len(task_ids) != len(set(task_ids)):
        raise ValueError(f'중복된 task_id가 있다: {task_ids}')
    robot_ids = [r.robot_id for r in robots]
    if len(robot_ids) != len(set(robot_ids)):
        raise ValueError(f'중복된 robot_id가 있다: {robot_ids}')


def assign_greedy(
    tasks: Sequence[TaskInfo],
    robots: Sequence[RobotInfo],
    cost_fn: CostFn,
    rank: Callable[[RobotInfo, float], tuple],
    feasible: FeasibleFn = None,
    aging: Optional[Aging] = None,
) -> AssignmentResult:
    """우선순위 순으로 작업을 꺼내, 남은 로봇 중 rank(로봇, 비용)이 가장 작은 로봇에 맡긴다.

    rank 끝에 robot_id를 넣어 동률을 사전순으로 깬다. 로봇이 바닥나면 나머지는 unassigned.
    feasible이 있으면 그 작업을 맡을 수 있는 로봇 중에서만 고르고, 없으면 그 작업만 unassigned (로봇은 다음 작업에 남는다).
    """
    check_unique_ids(tasks, robots)
    result = AssignmentResult()
    remaining = list(robots)
    for task in sort_by_priority(tasks, aging):
        able = [r for r in remaining if feasible is None or feasible(r, task)]
        if not able:
            result.unassigned.append(task.task_id)
            continue
        costs = {r.robot_id: cost_fn(r, task) for r in able}
        best = min(able, key=lambda r: rank(r, costs[r.robot_id]))
        remaining.remove(best)
        result.assignments[task.task_id] = best.robot_id
        result.total_cost += costs[best.robot_id]
    return result
