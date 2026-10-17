"""먼저 수락한 쪽이 주인: 같은 작업을 두 로봇이 맡았을 때 누가 계속할지 (architecture.md 5-2절).

하이브리드에서는 서버와 분산 경매가 같은 작업을 서로 다른 로봇에게 맡길 수 있다. 서버 장부(amr_fleet_manager
dispatcher)·가짜 로봇·경매 노드(amr_fleet_decentralized)가 모두 /fleet/task_status를 보고 같은 규칙으로 정한다.

- COMPLETED를 보고한 로봇이 있으면 그 로봇이 주인이다 (끝난 작업을 다시 할 이유가 없다)
- 아니면 PENDING·IN_PROGRESS를 보고한 로봇 중 수락 시각(TaskStatus.start_time)이 가장 이른 로봇. 같으면 robot_id 순
- FAILED를 보고한 로봇은 주인 후보에서 빠진다
- 하는 중인 주장은 그 로봇이 살아서 그 작업을 하고 있을 때만 센다: RobotState가 timeout_s 동안 끊겼거나 (서버 회수와 같은 경계),
  주장 뒤에 온 RobotState의 current_task_id가 다른 작업이면 뺀다 (죽었거나 재시작한 로봇이 작업을 붙잡지 않게)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

# amr_msgs/msg/TaskStatus.STATE_* 와 같은 값
STATE_PENDING = 0
STATE_IN_PROGRESS = 1
STATE_COMPLETED = 2
STATE_FAILED = 3


@dataclass
class _Claim:
    # 수락 시각 (시뮬레이션 시간 초)
    start: float
    # 이 주장을 마지막으로 본 시각 (now와 같은 시계)
    seen: float
    completed: bool = False


@dataclass
class _Robot:
    last_seen: float
    current_task_id: str


class TaskClaims:
    """작업별로 그 작업을 맡았다고 보고한 로봇들. now는 호출 측 시계(초)로, timeout_s와 단위가 같아야 한다."""

    def __init__(self, timeout_s: float = math.inf):
        self.timeout_s = timeout_s
        self._claims: dict[str, dict[str, _Claim]] = {}
        self._robots: dict[str, _Robot] = {}

    def observe(self, task_id: str, robot_id: str, state: int, start: float, now: float) -> None:
        """TaskStatus 1건. start는 start_time(0이면 header.stamp)을 초로 바꾼 값."""
        claims = self._claims.setdefault(task_id, {})
        if state == STATE_FAILED:
            claims.pop(robot_id, None)
            return
        claim = claims.get(robot_id)
        if claim is None:
            claims[robot_id] = _Claim(start, now, state == STATE_COMPLETED)
        else:
            claim.seen = now
            claim.completed = claim.completed or state == STATE_COMPLETED

    def robot_state(self, robot_id: str, current_task_id: str, now: float) -> None:
        """RobotState 1건. 로봇이 살아 있는지, 지금 무슨 작업을 하는지."""
        self._robots[robot_id] = _Robot(now, current_task_id)

    def owner(self, task_id: str, now: float, exclude: str = '') -> Optional[str]:
        """주인 robot_id. exclude를 빼고 고른다 (자기 말고 먼저 맡은 로봇이 있는지 볼 때). 없으면 None."""
        best = None
        for robot_id, claim in self._claims.get(task_id, {}).items():
            if robot_id == exclude or not self._valid(task_id, robot_id, claim, now):
                continue
            key = (not claim.completed, claim.start, robot_id)
            if best is None or key < best:
                best = key
        return None if best is None else best[2]

    def rival(self, task_id: str, robot_id: str, start: Optional[float], now: float) -> Optional[str]:
        """robot_id보다 먼저 맡은 다른 로봇. start는 robot_id의 수락 시각이고, 아직 수락 전이면 None
        (그러면 맡았다고 보고한 다른 로봇이 하나라도 있으면 그 로봇). 없으면 None."""
        other = self.owner(task_id, now, exclude=robot_id)
        if other is None or start is None:
            return other
        claim = self._claims[task_id][other]
        return other if claim.completed or (claim.start, other) < (start, robot_id) else None

    def completed(self, task_id: str, robot_id: str) -> bool:
        claim = self._claims.get(task_id, {}).get(robot_id)
        return claim is not None and claim.completed

    def _valid(self, task_id: str, robot_id: str, claim: _Claim, now: float) -> bool:
        if claim.completed:
            return True
        robot = self._robots.get(robot_id)
        if robot is None:
            # RobotState를 아직 못 봤으면 주장을 본 시각부터 잰다
            return now - claim.seen < self.timeout_s
        if now - max(robot.last_seen, claim.seen) >= self.timeout_s:
            return False
        # 주장 뒤에 온 RobotState가 다른 작업(또는 IDLE)이면 그 작업을 놓은 것이다
        return robot.last_seen <= claim.seen or robot.current_task_id == task_id
