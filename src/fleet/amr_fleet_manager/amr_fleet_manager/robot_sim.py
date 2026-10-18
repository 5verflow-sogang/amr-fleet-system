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

# 명령 번호대 (TaskAssignment.msg, api.md 4-3절). 서버는 1부터, 분산 경매로 작업을 수락한 로봇은
# 1,000,000 + 로봇 번호 × 100,000 + 로봇별 순번 (amr_fleet_decentralized, #211)
SERVER_SOURCE = 'server'
AUCTION_ID_BASE = 1_000_000
AUCTION_ID_STRIDE = 100_000


def assignment_source(assignment_id: int) -> str:
    """명령을 낸 쪽. 번호는 출처마다 따로 늘어나므로 출처끼리만 크기를 비교한다."""
    if assignment_id < AUCTION_ID_BASE:
        return SERVER_SOURCE
    return f'auction:{(assignment_id - AUCTION_ID_BASE) // AUCTION_ID_STRIDE}'


class AssignmentFilter:
    """assignment_id로 새 명령·재전송·뒤늦은 명령을 가른다. 출처(서버, 경매 로봇 k)마다 따로 본다.

    하이브리드(평시 중앙, 끊기면 분산)에서는 서버 명령과 경매 명령이 섞여 온다. 번호 하나로 비교하면
    경매 번호(1,000,000+) 뒤에 온 서버 명령이 "작은 번호"로 버려진다. 서버가 큰 번호대를 써도
    중앙 → 분산 → 중앙을 되풀이하면 어느 한쪽이 다시 작아지므로 출처별 비교만 안전하다 (#211 결정).
    """

    def __init__(self):
        self.epoch: Optional[int] = None
        # 출처 → (마지막으로 처리한 번호, 그 명령에 보낸 Ack 내용). 재전송이 오면 Ack를 그대로 다시 보낸다
        self._last: dict[str, tuple[int, Any]] = {}
        # 마지막으로 처리한 명령의 Ack 내용 (DUPLICATE일 때 다시 보낼 것). classify가 고른 출처의 값
        self.last_reply: Any = None

    @property
    def last_id(self) -> int:
        """서버 명령의 마지막 번호 (RobotState.last_assignment_id). 서버는 자기 번호만 안다."""
        return self._last.get(SERVER_SOURCE, (0, None))[0]

    def on_heartbeat(self, epoch: int) -> bool:
        """서버 epoch가 바뀌면 서버 번호 기준만 초기화하고 True. 처음 받은 epoch는 재시작으로 보지 않는다.

        경매 번호는 서버 재시작과 무관하게 이어지므로 그대로 둔다.
        """
        restarted = self.epoch is not None and epoch != self.epoch
        if restarted:
            self._last.pop(SERVER_SOURCE, None)
        self.epoch = epoch
        return restarted

    def classify(self, assignment_id: int) -> str:
        last_id, last_reply = self._last.get(assignment_source(assignment_id), (0, None))
        if assignment_id == last_id and last_reply is not None:
            self.last_reply = last_reply
            return DUPLICATE
        if assignment_id <= last_id:
            return STALE
        return NEW

    def record(self, assignment_id: int, reply: Any) -> None:
        self._last[assignment_source(assignment_id)] = (assignment_id, reply)
        self.last_reply = reply


class LinkWatch:
    """서버 heartbeat로 서버와의 링크가 끊겼는지 본다 (끊기면 멈춤 정책, 연구 1 끊김 정책 비교).

    상용 AMR은 관제와 연결이 끊기면 안전하게 멈추고 기다리는 경우가 많다. pause_after_s 동안 heartbeat가 안 오면
    끊긴 것으로 본다. 0 이하면 끈다(하던 작업은 끊겨도 계속 - 서버가 작업 전체를 미리 넘긴 경우).
    heartbeat를 한 번도 못 봤으면 끊긴 것으로 보지 않는다 (기동 직후). 시각은 호출 쪽의 한 시계(실제 시간)다.
    """

    def __init__(self, pause_after_s: float = 0.0):
        self.pause_after_s = pause_after_s
        self._last: Optional[float] = None

    def on_heartbeat(self, now: float) -> None:
        self._last = now

    def lost(self, now: float) -> bool:
        return self.pause_after_s > 0 and self._last is not None and now - self._last >= self.pause_after_s


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
        # 지금 구간을 시작했는지(조금이라도 움직이거나 기다렸는지). 허가 지점 대기에 쓴다
        self._progress = False
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

    def step(self, dt: float, hold_at_boundary: bool = False) -> bool:
        """dt초만큼 진행한다. 단계가 바뀌었으면 True.

        hold_at_boundary면 이미 시작한 구간은 마저 가고, 새 구간은 시작하지 않는다 (서버와 끊긴 동안 다음 허가
        지점 앞에서 기다리는 정책, VDA 5050의 확정 구간처럼).
        """
        start = self._index
        remaining = dt
        self.velocity = (0.0, 0.0)
        while remaining > 0 and not self.done:
            if hold_at_boundary and not self._progress:
                break
            self._progress = True
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
        while not hold_at_boundary and not self.done and self._legs[self._index].target is None \
                and self._legs[self._index].duration <= self._waited:
            self._next()
        return self._index != start

    def _next(self) -> None:
        self._index += 1
        self._waited = 0.0
        self._progress = False
