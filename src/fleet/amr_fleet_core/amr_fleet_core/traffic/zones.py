"""구역 허가 (ZoneRequest/ZoneGrant 서버 측). 정원·lease·대기열만 다룬다.

규칙은 amr_msgs의 ZoneRequest.msg·ZoneGrant.msg 주석을 따른다.
- 허가는 lease_s 동안 유효하다. 로봇이 같은 구역을 다시 요청하면(연장) 그때부터 다시 lease_s.
- 정원이 차면 QUEUED로 답하고 대기열에 넣는다. 자리가 나면 먼저 기다린 로봇부터 GRANTED를 보낸다.
- 대기 중인 로봇도 주기적으로 다시 요청한다. lease_s 동안 요청이 없으면 대기열에서 뺀다.

시각(now)은 초 단위이며 로봇이 lease_expiry와 비교하는 시계(노드 시계)와 같아야 한다.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from typing import Iterable, Optional

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
    # 점유 중이면 허가 만료 시각, 대기 중이면 마지막 요청 시각 + lease_s
    expiry: float


class _Zone:

    def __init__(self, spec: ZoneSpec):
        self.spec = spec
        # 넣은 순서 = 들어온 순서 / 기다린 순서
        self.holders: OrderedDict[str, _Slot] = OrderedDict()
        self.queue: OrderedDict[str, _Slot] = OrderedDict()

    def first_holder(self) -> str:
        return next(iter(self.holders), '')


class ZoneManager:

    def __init__(self, zones: Iterable[ZoneSpec], lease_s: float):
        if lease_s <= 0:
            raise ValueError(f'lease_s는 0보다 커야 한다: {lease_s}')
        self._lease_s = lease_s
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

    def request(self, robot_id: str, zone_id: str, request_id: int, now: float) -> list[Decision]:
        """진입 요청 (연장 포함). 이 로봇에 대한 답 1건을 돌려준다."""
        zone = self._zones.get(zone_id)
        if zone is None:
            return [Decision(robot_id, zone_id, request_id, GRANT_REJECTED)]
        expiry = now + self._lease_s

        holding = zone.holders.get(robot_id)
        if holding is not None:
            holding.request_id = request_id
            holding.expiry = expiry
            return [self._granted(zone, robot_id, holding)]

        # 자리가 나면 _promote가 바로 대기열을 비우므로, 대기열이 있으면 정원은 항상 차 있다.
        # 그래서 빈자리가 있으면 새치기 걱정 없이 바로 허가한다
        waiting = zone.queue.get(robot_id)
        if len(zone.holders) < zone.spec.capacity:
            slot = _Slot(request_id, expiry)
            zone.holders[robot_id] = slot
            return [self._granted(zone, robot_id, slot)]

        if waiting is None:
            zone.queue[robot_id] = _Slot(request_id, expiry)
        else:
            waiting.request_id = request_id
            waiting.expiry = expiry
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
        """lease가 지난 허가와 오래 요청이 없는 대기를 지우고, 자리가 난 구역의 대기 로봇을 허가한다."""
        decisions = []
        for zone in self._zones.values():
            for slots in (zone.holders, zone.queue):
                for robot_id in [r for r, s in slots.items() if s.expiry <= now]:
                    del slots[robot_id]
            decisions += self._promote(zone, now)
        return decisions

    def _promote(self, zone: _Zone, now: float) -> list[Decision]:
        decisions = []
        while zone.queue and len(zone.holders) < zone.spec.capacity:
            robot_id, waiting = zone.queue.popitem(last=False)
            slot = _Slot(waiting.request_id, now + self._lease_s)
            zone.holders[robot_id] = slot
            decisions.append(self._granted(zone, robot_id, slot))
        return decisions

    def _granted(self, zone: _Zone, robot_id: str, slot: _Slot) -> Decision:
        return Decision(robot_id, zone.spec.zone_id, slot.request_id, GRANT_GRANTED,
                        lease_expiry=slot.expiry, holder_robot_id=robot_id)
