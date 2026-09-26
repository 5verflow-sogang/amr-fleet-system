"""알고리즘 3종이 함께 쓰는 전처리: 후보 거르기, 우선순위 정렬, 입력 검사."""

from __future__ import annotations

import math
from typing import Sequence

from .types import MODE_IDLE, RobotInfo, TaskInfo


def filter_candidates(robots: Sequence[RobotInfo], min_battery_percent: float) -> list[RobotInfo]:
    """IDLE이고 배터리가 하한 이상인 로봇만 남긴다. robot_id 순으로 정렬해 돌려준다."""
    candidates = [
        r for r in robots
        if r.mode == MODE_IDLE and r.battery_percent >= min_battery_percent
    ]
    return sorted(candidates, key=lambda r: r.robot_id)


def sort_by_priority(tasks: Sequence[TaskInfo]) -> list[TaskInfo]:
    """priority 내림차순 → deadline 오름차순 (None은 맨 뒤) → created 오름차순 → task_id."""
    return sorted(
        tasks,
        key=lambda t: (
            -t.priority,
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
