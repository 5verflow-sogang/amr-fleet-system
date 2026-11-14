"""구역 밖 대치(standoff) 탐지와 양보 지시.

구역(좁은 통로)의 교착은 wait-for 그래프(graph.py)로 찾지만, 넓은 통로·스테이션 앞에서 두 로봇이 서로를
보고 멈춰 선 경우는 쥐고 있는 자원이 없어 그래프에 안 잡힌다. Nav2 복구(제자리 회전·잠깐 후진)는 두 로봇이
똑같이 해서 풀리지 않는다(11/14 fleet_stack peer_resp: 0.6 m 간격으로 400~500 s 정지, Failed to make progress
239회). 그래서 위치만 본다: 작업 중인 로봇을 포함해 radius_m 안에 모인 로봇들이 모두 still_s 동안 still_m
넘게 움직이지 않았으면 대치로 보고, 그중 한 대(choose_victim)만 상대에게서 멀어지는 쪽으로 비키게 한다.
비킬 쪽에 다른 로봇이 있으면(LiDAR는 로봇을 못 본다, FleetPeerLayer 참고) 반대쪽, 둘 다 막혔으면 다음 로봇을 고른다.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Callable, Hashable, Optional

from .graph import choose_victim

# 양보 방향: 상대가 앞이면 뒤로, 뒤면 앞으로 (로봇 진행 방향 기준)
BACK = -1
FORWARD = 1


@dataclass(frozen=True)
class StandoffConfig:
    radius_m: float = 1.2      # 이 거리 안이면 서로 막고 있다고 본다 (차체 길이 0.6 m의 두 배)
    # 이만큼 멈춰 있어야 대치. Nav2 progress_checker movement_time_allowance(10 s)와 같다 - 지역 복구에 한 번은 기회를
    # 준다. 11/14 peer_resp·peer_basic 기록 재생: 5 s면 20 s 안에 저절로 풀릴 자리에 양보 12회, 10 s면 3회, 12~20 s는 2회로
    # 평평해서 더 기다려도 얻는 게 없다(첫 양보까지는 still_s만큼 늦어진다)
    still_s: float = 10.0
    still_m: float = 0.2       # 그동안 마지막 위치에서 이 거리 넘게 벗어난 적이 없어야 멈춤 (0.3도 결과 같음)
    # 양보를 지시한 묶음은 이 시간 동안 다시 보지 않는다. 비키기(약 5 s) + 서 있기(8 s) 뒤 다시 still_s를 채워야
    # 하므로 실제 재지시는 지시 뒤 약 23 s부터다
    cooldown_s: float = 20.0
    # 비킬 방향으로 이 거리(중심 간) 안, 옆으로 corridor_m 안에 다른 로봇이 있으면 그쪽으로는 안 비킨다:
    # 비키는 거리 0.6 + 두 차체 반길이 0.3 + 0.3 + 여유 0.1. LiDAR는 다른 로봇을 못 보므로 서버가 걸러 준다
    clear_m: float = 1.3
    corridor_m: float = 0.6    # 차체 반폭 0.2 + 상대 반길이 0.3 + 여유


@dataclass(frozen=True)
class YieldOrder:
    robot_id: str            # 비킬 로봇
    yield_to: str            # 가장 가까운 상대
    direction: int           # BACK 또는 FORWARD
    group: tuple[str, ...]   # 대치 묶음 (robot_id 순)


@dataclass
class _Robot:
    active: bool = False
    busy: bool = False
    yaw: float = 0.0
    history: deque = None    # (t, x, y)


class StandoffDetector:

    def __init__(self, config: StandoffConfig = StandoffConfig()):
        self._config = config
        self._robots: dict[str, _Robot] = {}
        self._cooldown_until: dict[str, float] = {}

    def update(self, robot_id: str, t: float, x: float, y: float, yaw: float, busy: bool, active: bool = True):
        """로봇 상태 하나. active=False(충전·오류·E-Stop, 도킹·적재 중, 구역 대기열)면 대치 묶음에 넣지 않는다
        (비킬 쪽 장애물로는 본다). busy는 작업 중."""
        robot = self._robots.setdefault(robot_id, _Robot(history=deque()))
        robot.active = active
        robot.busy = busy
        robot.yaw = yaw
        if robot.history and t <= robot.history[-1][0]:
            return
        robot.history.append((t, x, y))
        # still_s보다 오래된 점은 하나만 남긴다 (창이 still_s를 덮는지 확인용)
        while len(robot.history) >= 2 and robot.history[1][0] <= t - self._config.still_s:
            robot.history.popleft()

    def _still(self, robot: _Robot, now: float) -> bool:
        h = robot.history
        if not h or h[0][0] > now - self._config.still_s:
            return False
        _, lx, ly = h[-1]
        return all(math.hypot(x - lx, y - ly) <= self._config.still_m for _, x, y in h)

    def check(self, now: float, priority: Callable[[str], Hashable]) -> list[YieldOrder]:
        """지금 양보시킬 로봇들. 지시한 묶음은 cooldown_s 동안 다시 지시하지 않는다.

        priority: 작업 우선순위 (choose_victim). 작업이 없는 로봇이 묶음에 있으면 그 로봇이 먼저 비킨다.
        """
        still = {r: rb for r, rb in sorted(self._robots.items())
                 if rb.active and self._cooldown_until.get(r, -math.inf) <= now and self._still(rb, now)}
        pos = {r: rb.history[-1][1:] for r, rb in still.items()}
        all_pos = {r: rb.history[-1][1:] for r, rb in self._robots.items() if rb.history}
        orders = []
        seen: set[str] = set()
        for start in still:
            if start in seen:
                continue
            group, todo = [], [start]
            seen.add(start)
            while todo:
                r = todo.pop()
                group.append(r)
                for o in still:
                    if o not in seen and math.dist(pos[r], pos[o]) <= self._config.radius_m:
                        seen.add(o)
                        todo.append(o)
            if len(group) < 2 or not any(still[r].busy for r in group):
                continue
            group.sort()
            order = self._order(group, still, pos, all_pos, priority)
            if order is not None:
                orders.append(order)
            for r in group:
                self._cooldown_until[r] = now + self._config.cooldown_s
        return orders

    def _order(self, group, still, pos, all_pos, priority) -> Optional[YieldOrder]:
        """작업 없는 로봇 → 우선순위 낮은 로봇 순으로, 비킬 자리가 있는 첫 로봇. 없으면 None."""
        remaining = list(group)
        while remaining:
            idle = [r for r in remaining if not still[r].busy]
            victim = choose_victim(idle or remaining, priority)
            remaining.remove(victim)
            others = [r for r in group if r != victim]
            yield_to = min(others, key=lambda o: (math.dist(pos[victim], pos[o]), o))
            away = self.direction(victim, yield_to, pos)
            for direction in (away, -away):
                if self._clear(victim, direction, all_pos):
                    return YieldOrder(victim, yield_to, direction, tuple(group))
        return None

    def _clear(self, victim: str, direction: int, all_pos: dict) -> bool:
        """victim이 direction으로 비키는 길(앞 clear_m, 옆 corridor_m)에 다른 로봇이 없나."""
        yaw = self._robots[victim].yaw
        vx, vy = all_pos[victim]
        for other, (ox, oy) in all_pos.items():
            if other == victim:
                continue
            dx, dy = ox - vx, oy - vy
            along = direction * (math.cos(yaw) * dx + math.sin(yaw) * dy)
            lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
            if 0.0 < along <= self._config.clear_m and abs(lateral) <= self._config.corridor_m:
                return False
        return True

    def direction(self, victim: str, other: str, pos: dict) -> int:
        """상대가 진행 방향 앞쪽이면 BACK, 뒤쪽이면 FORWARD."""
        yaw = self._robots[victim].yaw
        dx = pos[other][0] - pos[victim][0]
        dy = pos[other][1] - pos[victim][1]
        return BACK if math.cos(yaw) * dx + math.sin(yaw) * dy >= 0.0 else FORWARD
