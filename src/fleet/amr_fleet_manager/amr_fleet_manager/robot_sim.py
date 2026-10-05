"""가짜 로봇(fake_robot)의 명령 수신 규칙과 이동 모델. rclpy를 import하지 않는다.

AssignmentFilter는 amr_bt가 지켜야 할 수신 규칙(TaskAssignment.msg 주석)을 그대로 구현한 참고본이다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional, Tuple

Point = Tuple[float, float]

NEW = 'new'              # 처음 받은 명령: 처리하고 Ack
DUPLICATE = 'duplicate'  # 마지막으로 처리한 명령의 재전송: 다시 처리하지 않고 같은 Ack만
STALE = 'stale'          # 마지막보다 작은 번호 (지연으로 순서가 뒤바뀜): 무시


class AssignmentFilter:
    """assignment_id로 새 명령·재전송·뒤늦은 명령을 가른다."""

    def __init__(self):
        self.epoch: Optional[int] = None
        self.last_id = 0
        # 마지막 명령에 보낸 Ack 내용. 재전송이 오면 그대로 다시 보낸다
        self.last_reply: Any = None

    def on_heartbeat(self, epoch: int) -> bool:
        """서버 epoch가 바뀌면 번호 기준을 초기화하고 True. 처음 받은 epoch는 재시작으로 보지 않는다."""
        restarted = self.epoch is not None and epoch != self.epoch
        if restarted:
            self.last_id = 0
            self.last_reply = None
        self.epoch = epoch
        return restarted

    def classify(self, assignment_id: int) -> str:
        if assignment_id == self.last_id and self.last_reply is not None:
            return DUPLICATE
        if assignment_id <= self.last_id:
            return STALE
        return NEW

    def record(self, assignment_id: int, reply: Any) -> None:
        self.last_id = assignment_id
        self.last_reply = reply


@dataclass
class _Leg:
    phase: str
    # 갈 곳. None이면 제자리에서 duration초 기다린다 (싣기·내리기)
    target: Optional[Point]
    duration: float = 0.0


class TaskRun:
    """작업 1건을 직선 이동과 고정 대기로 흉내 낸다. Nav2·도킹 없이 Fleet 메시지 흐름만 확인하는 용도.

    단계 이름은 architecture.md 5-1절의 TaskStatus.phase를 쓴다.
    """

    def __init__(self, xy: Point, pickup: Optional[Point], dropoff: Point, speed: float, dwell_s: float):
        if speed <= 0:
            raise ValueError(f'speed는 0보다 커야 한다: {speed}')
        legs = []
        if pickup is not None:
            legs += [_Leg('move_to_pickup', pickup), _Leg('loading', None, dwell_s)]
        legs.append(_Leg('move_to_dropoff', dropoff))
        if pickup is not None:
            legs.append(_Leg('unloading', None, dwell_s))
        self._legs = legs
        self._index = 0
        self._waited = 0.0
        self._speed = speed
        self.xy: Point = xy
        self.velocity: Point = (0.0, 0.0)
        self.distance = 0.0

    @property
    def done(self) -> bool:
        return self._index >= len(self._legs)

    @property
    def phase(self) -> str:
        return '' if self.done else self._legs[self._index].phase

    def step(self, dt: float) -> bool:
        """dt초만큼 진행한다. 단계가 바뀌었으면 True."""
        start = self._index
        remaining = dt
        self.velocity = (0.0, 0.0)
        while remaining > 0 and not self.done:
            leg = self._legs[self._index]
            if leg.target is None:
                need = leg.duration - self._waited
                if remaining < need:
                    self._waited += remaining
                    remaining = 0.0
                else:
                    remaining -= need
                    self._next()
                continue
            dx, dy = leg.target[0] - self.xy[0], leg.target[1] - self.xy[1]
            dist = math.hypot(dx, dy)
            travel = self._speed * remaining
            if travel < dist:
                self.xy = (self.xy[0] + dx * travel / dist, self.xy[1] + dy * travel / dist)
                self.velocity = (self._speed * dx / dist, self._speed * dy / dist)
                self.distance += travel
                remaining = 0.0
            else:
                self.xy = leg.target
                self.distance += dist
                remaining -= dist / self._speed
                self._next()
        # dwell이 0이면 dt가 0이어도 대기 단계를 넘긴다
        while not self.done and self._legs[self._index].target is None \
                and self._legs[self._index].duration <= self._waited:
            self._next()
        return self._index != start

    def _next(self) -> None:
        self._index += 1
        self._waited = 0.0
