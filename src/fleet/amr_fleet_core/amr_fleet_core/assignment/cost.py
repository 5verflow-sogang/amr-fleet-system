"""할당 비용 함수."""

import math

from .types import RobotInfo, TaskInfo


def euclidean_cost(robot: RobotInfo, task: TaskInfo) -> float:
    """로봇 → 작업 시작 지점 직선 거리 (m).

    픽업 → 목적지 구간은 어느 로봇이 맡아도 같으므로 넣지 않는다.
    """
    return math.dist(robot.xy, task.start_xy)
