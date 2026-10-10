"""구역 허가 (ZoneRequest/ZoneGrant 서버 측). 정원·lease·우선순위 대기열·교착 해소.

규칙은 amr_msgs의 ZoneRequest.msg·ZoneGrant.msg 주석을 따른다.
- 허가는 lease_s 동안 유효하다. 로봇이 같은 구역을 다시 요청하면(연장) 그때부터 다시 lease_s.
- lease가 지나도 구역은 반납이 올 때까지 그 로봇 것으로 둔다. 좁은 통로 안에서 만료되면 로봇은 멈추지 않고
  빠져나간 뒤 반납하므로, 그 전에 다른 로봇을 들이면 통로 안에서 마주친다 (이진 합의, 10/5).
  로봇이 죽어 반납이 영영 안 오는 경우를 위해 마지막 요청 후 release_timeout_s가 지나면 푼다.
- 정원이 차면 QUEUED로 답하고 대기열에 넣는다. 자리가 나면 작업 우선순위가 높은 로봇 → 먼저 기다린 로봇
  → robot_id 순으로 GRANTED를 보낸다 (교차로 우선순위).
- 대기 중인 로봇도 주기적으로 다시 요청한다. lease_s 동안 요청이 없으면 대기열에서 뺀다.
- 서로 상대가 점유한 구역을 기다리면(교착) 우선순위가 가장 낮은 로봇의 허가를 회수(REVOKED)한다.
  회수당한 로봇은 아직 구역 안에 있을 수 있으므로, 물러나 반납할 때까지 구역을 다른 로봇에 주지 않는다.

시각(now)은 초 단위이며 로봇이 lease_expiry와 비교하는 시계(노드 시계)와 같아야 한다.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from typing import Iterable, Optional

from ..deadlock import choose_victim, find_deadlocks

# amr_msgs/msg/ZoneGrant.GRANT_* 와 같은 값
GRANT_GRANTED = 0
GRANT_QUEUED = 1
GRANT_REJECTED = 2
GRANT_REVOKED = 3

# amr_msgs/msg/ZoneGrant.REASON_* 와 같은 값
REASON_NONE = 0
REASON_OCCUPIED = 1
REASON_PRIORITY_YIELD = 2
REASON_DEADLOCK_RESOLUTION = 3


@dataclass(frozen=True)
class ZoneSpec:
    zone_id: str
    # 동시에 들어갈 수 있는 로봇 수
    capacity: int = 1

    def __post_init__(self):
        if self.capacity < 1:
            raise ValueError(f'{self.zone_id}: capacity는 1 이상이어야 한다: {self.capacity}')


@dataclass(frozen=True)
class Decision:
    """ZoneGrant 1건."""

    robot_id: str
    zone_id: str
    # 응답할 요청 번호. 대기열에서 허가로 바뀔 때는 그 로봇의 마지막 요청 번호
    request_id: int
    grant: int
    reason: int = REASON_NONE
    # GRANTED일 때만 의미가 있다
    lease_expiry: Optional[float] = None
    # 구역을 점유 중인 로봇. 없으면 빈 문자열
    holder_robot_id: str = ''


@dataclass
class _Slot:
    request_id: int
    # 마지막 요청 시각 + lease_s. 점유 중이면 lease_expiry로 보낸다
    expiry: float
    # 마지막 요청 시각
    last_seen: float
    # 로봇이 맡은 작업의 우선순위 (Task.priority, 클수록 급함). 요청마다 갱신한다
    priority: int = 0
    # 대기열에 처음 들어온 시각 / 허가받은 시각
    since: float = 0.0
    # 교착 해소로 회수됨. 반납할 때까지 점유로 남는다
    revoked: bool = False
    # 회수할 때 양보받은 로봇
    yield_to: str = ''

    def order(self, robot_id: str) -> tuple:
        """허가 순서. 작으면 먼저."""
        return (-self.priority, self.since, robot_id)


class _Zone:

    def __init__(self, spec: ZoneSpec):
        self.spec = spec
        # 넣은 순서 = 들어온 순서 / 기다린 순서
        self.holders: OrderedDict[str, _Slot] = OrderedDict()
        self.queue: OrderedDict[str, _Slot] = OrderedDict()

    def first_holder(self) -> str:
        return next(iter(self.holders), '')

    def next_waiter(self, exclude: str = '') -> str:
        waiting = [(s.order(r), r) for r, s in self.queue.items() if r != exclude]
        return min(waiting)[1] if waiting else ''


class ZoneManager:

    def __init__(self, zones: Iterable[ZoneSpec], lease_s: float, release_timeout_s: float):
        if lease_s <= 0:
            raise ValueError(f'lease_s는 0보다 커야 한다: {lease_s}')
        if release_timeout_s <= lease_s:
            raise ValueError(f'release_timeout_s({release_timeout_s})는 lease_s({lease_s})보다 커야 한다')
        self._lease_s = lease_s
        self._release_timeout_s = release_timeout_s
        self._zones: dict[str, _Zone] = {}
        for spec in zones:
            if spec.zone_id in self._zones:
                raise ValueError(f'중복된 zone_id: {spec.zone_id}')
            self._zones[spec.zone_id] = _Zone(spec)

    @property
    def zone_ids(self) -> list[str]:
        return list(self._zones)

    def holders(self, zone_id: str) -> list[str]:
        return list(self._zones[zone_id].holders)

    def queue(self, zone_id: str) -> list[str]:
        return list(self._zones[zone_id].queue)

    def request(self, robot_id: str, zone_id: str, request_id: int, now: float,
                priority: int = 0) -> list[Decision]:
        """진입 요청 (연장 포함). 이 로봇에 대한 답 1건을 돌려준다.

        priority는 로봇이 맡은 작업의 우선순위다 (ZoneRequest에는 없어 호출 측이 작업 장부에서 찾아 넘긴다).
        """
        zone = self._zones.get(zone_id)
        if zone is None:
            return [Decision(robot_id, zone_id, request_id, GRANT_REJECTED)]
        expiry = now + self._lease_s

        holding = zone.holders.get(robot_id)
        if holding is not None:
            holding.request_id = request_id
            holding.last_seen = now
            holding.priority = priority
            if holding.revoked:
                # 회수 통보가 유실됐을 수 있으니 같은 답을 다시 준다. 연장은 하지 않는다
                return [self._revoked(zone, robot_id, holding)]
            holding.expiry = expiry
            return [self._granted(zone, robot_id, holding)]

        # 자리가 나면 _promote가 바로 대기열을 비우므로, 대기열이 있으면 정원은 항상 차 있다.
        # 그래서 빈자리가 있으면 새치기 걱정 없이 바로 허가한다
        waiting = zone.queue.get(robot_id)
        if len(zone.holders) < zone.spec.capacity:
            slot = _Slot(request_id, expiry, now, priority, since=now)
            zone.holders[robot_id] = slot
            return [self._granted(zone, robot_id, slot)]

        if waiting is None:
            waiting = zone.queue[robot_id] = _Slot(request_id, expiry, now, priority, since=now)
        else:
            waiting.request_id = request_id
            waiting.expiry = expiry
            waiting.last_seen = now
            waiting.priority = priority
        # 앞에 더 급한 로봇이 기다리고 있으면 그 로봇에게 양보하는 중이라고 알린다
        ahead = zone.next_waiter(exclude=robot_id)
        if ahead and zone.queue[ahead].order(ahead) < waiting.order(robot_id) \
                and zone.queue[ahead].priority > priority:
            return [Decision(robot_id, zone_id, request_id, GRANT_QUEUED, REASON_PRIORITY_YIELD,
                             holder_robot_id=ahead)]
        return [Decision(robot_id, zone_id, request_id, GRANT_QUEUED, REASON_OCCUPIED,
                         holder_robot_id=zone.first_holder())]

    def release(self, robot_id: str, zone_id: str, now: float) -> list[Decision]:
        """반납. 반납한 로봇에게는 답하지 않고, 자리가 나서 허가받은 대기 로봇의 답을 돌려준다."""
        zone = self._zones.get(zone_id)
        if zone is None:
            return []
        zone.holders.pop(robot_id, None)
        zone.queue.pop(robot_id, None)
        return self._promote(zone, now)

    def expire(self, now: float) -> list[Decision]:
        """오래 소식이 없는 점유·대기를 지우고, 자리가 난 구역의 대기 로봇을 허가한다.

        점유는 lease가 지나도 지우지 않는다 (모듈 설명). 마지막 요청 후 release_timeout_s가 지나야 지운다.
        """
        decisions = []
        for zone in self._zones.values():
            for robot_id in [r for r, s in zone.holders.items() if now - s.last_seen >= self._release_timeout_s]:
                del zone.holders[robot_id]
            for robot_id in [r for r, s in zone.queue.items() if s.expiry <= now]:
                del zone.queue[robot_id]
            decisions += self._promote(zone, now)
        return decisions

    def wait_for(self) -> dict[str, dict[str, str]]:
        """wait-for 그래프. wait_for()[A][B] = Z 는 A가 기다리는 구역 Z를 B가 점유 중이라는 뜻.

        회수된 점유는 넣지 않는다. 이미 해소 중인 교착을 다시 잡지 않기 위해서다.
        """
        edges: dict[str, dict[str, str]] = {}
        for zone in self._zones.values():
            holders = [r for r, s in zone.holders.items() if not s.revoked]
            for waiter in zone.queue:
                for holder in holders:
                    edges.setdefault(waiter, {}).setdefault(holder, zone.spec.zone_id)
        return edges

    def resolve_deadlocks(self, now: float) -> list[Decision]:
        """교착마다 우선순위가 가장 낮은 로봇의 허가 하나를 회수한다. 회수 통보(REVOKED)를 돌려준다.

        회수한 구역은 그 로봇이 반납해야 다음 로봇에게 넘어간다 (release()).
        """
        edges = self.wait_for()
        decisions = []
        for group in find_deadlocks(edges):
            victim = choose_victim(group, self._robot_priority)
            # 묶음 안의 다른 로봇이 기다리는, 피해 로봇이 점유한 구역
            zone_id = min(edges[r][victim] for r in group if victim in edges.get(r, {}))
            zone = self._zones[zone_id]
            slot = zone.holders[victim]
            slot.revoked = True
            waiters = [r for r in group if r in zone.queue]
            slot.yield_to = min(waiters, key=lambda r: zone.queue[r].order(r))
            decisions.append(self._revoked(zone, victim, slot))
        return decisions

    def _robot_priority(self, robot_id: str) -> int:
        slots = [z.holders.get(robot_id) or z.queue.get(robot_id) for z in self._zones.values()]
        return max((s.priority for s in slots if s is not None), default=0)

    def _promote(self, zone: _Zone, now: float) -> list[Decision]:
        decisions = []
        while zone.queue and len(zone.holders) < zone.spec.capacity:
            robot_id = zone.next_waiter()
            waiting = zone.queue.pop(robot_id)
            slot = _Slot(waiting.request_id, now + self._lease_s, waiting.last_seen, waiting.priority, since=now)
            zone.holders[robot_id] = slot
            decisions.append(self._granted(zone, robot_id, slot))
        return decisions

    def _revoked(self, zone: _Zone, robot_id: str, slot: _Slot) -> Decision:
        return Decision(robot_id, zone.spec.zone_id, slot.request_id, GRANT_REVOKED,
                        REASON_DEADLOCK_RESOLUTION, holder_robot_id=slot.yield_to)

    def _granted(self, zone: _Zone, robot_id: str, slot: _Slot) -> Decision:
        return Decision(robot_id, zone.spec.zone_id, slot.request_id, GRANT_GRANTED,
                        lease_expiry=slot.expiry, holder_robot_id=robot_id)
