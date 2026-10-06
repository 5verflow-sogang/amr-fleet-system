"""로봇 측 구역 진입 처리 (ZoneRequest/ZoneGrant). ROS 비의존. rclpy·amr_msgs를 import하지 않는다.

규칙 (이진, 10/5, zones.yaml 주석과 같다):
- 경로 기준 구역 경계 3 m 앞에서 TYPE_ACCESS 요청. 허가(GRANTED) 전에는 경계 0.5 m 앞에 정지하도록
  기준 감속 1.0 m/s²로 미리 감속한다 (speed_cap_for_stop).
- 새 진입마다 request_id를 1 늘린다. 같은 진입의 재전송·연장(대기 중·구역 안)은 같은 request_id를 쓴다.
- 대기(QUEUED) 중에는 계속 기다린다. 서버가 자리가 나면 이 로봇의 마지막 request_id로 GRANTED를 보낸다.
- REJECTED면 진입하지 않고 경고를 남긴다 (서버의 ZoneManager는 지금 구역 자체가 없을 때만 REJECTED를 준다).
- 구역을 나오면 TYPE_RELEASE를 보낸다. 구역 안에서는 lease_expiry와 상관없이 멈추지 않고 빠져나간다
  (서버도 반납이 올 때까지 구역을 쥐고 있는다, amr_fleet_core.traffic.zones).
- ZoneGrant는 모든 로봇에 방송되므로 robot_id로 거른다 (on_grant).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Sequence

from .geometry import Point, Rect, point_in_rect, remaining_distance_to_rect, speed_cap_for_stop

# amr_msgs/msg/ZoneRequest.TYPE_* 와 같은 값
TYPE_ACCESS = 0
TYPE_RELEASE = 1

# amr_msgs/msg/ZoneGrant.GRANT_* 와 같은 값
GRANT_GRANTED = 0
GRANT_QUEUED = 1
GRANT_REJECTED = 2
GRANT_REVOKED = 3


class Phase(Enum):
    IDLE = 'idle'                  # 아직 트리거 거리 밖
    APPROACHING = 'approaching'    # 요청을 보냈고 아직 구역 밖 (대기 또는 통과 허가)
    INSIDE = 'inside'              # 구역 안, 반납 전
    BLOCKED = 'blocked'            # REJECTED. 재시도하지 않는다


@dataclass(frozen=True)
class ZoneConfig:
    zone_id: str
    area: Rect


@dataclass(frozen=True)
class OutgoingRequest:
    zone_id: str
    request_id: int
    type: int


@dataclass
class _ZoneState:
    phase: Phase = Phase.IDLE
    request_id: Optional[int] = None
    grant: Optional[int] = None
    lease_expiry: Optional[float] = None
    last_sent: Optional[float] = None


@dataclass(frozen=True)
class ZoneClientConfig:
    robot_id: str
    zones: Sequence[ZoneConfig] = field(default_factory=list)
    trigger_distance_m: float = 3.0
    stop_margin_m: float = 0.5
    decel_mps2: float = 1.0
    resend_period_s: float = 1.0


class ZoneClientCore:

    def __init__(self, config: ZoneClientConfig):
        self._config = config
        self._zones: Dict[str, ZoneConfig] = {z.zone_id: z for z in config.zones}
        self._state: Dict[str, _ZoneState] = {z.zone_id: _ZoneState() for z in config.zones}
        self._next_request_id = 1

    @property
    def zone_ids(self) -> List[str]:
        return list(self._zones)

    def phase(self, zone_id: str) -> Phase:
        return self._state[zone_id].phase

    def grant(self, zone_id: str) -> Optional[int]:
        return self._state[zone_id].grant

    def request_id(self, zone_id: str) -> Optional[int]:
        return self._state[zone_id].request_id

    def update(self, now: float, path_xy: Sequence[Point], robot_xy: Point):
        """한 틱 처리.

        반환: (보낼 ZoneRequest 목록, cmd_vel 선속도 상한 m/s (제한 없으면 None)).
        """
        outgoing: List[OutgoingRequest] = []
        speed_cap: Optional[float] = None
        for zone_id, spec in self._zones.items():
            state = self._state[zone_id]
            remaining = remaining_distance_to_rect(path_xy, robot_xy, spec.area)
            zone_outgoing, zone_cap = self._update_zone(zone_id, spec, state, now, remaining, robot_xy)
            outgoing.extend(zone_outgoing)
            if zone_cap is not None:
                speed_cap = zone_cap if speed_cap is None else min(speed_cap, zone_cap)
        return outgoing, speed_cap

    def _update_zone(self, zone_id: str, spec: ZoneConfig, state: _ZoneState, now: float,
                      remaining: Optional[float], robot_xy: Point):
        outgoing: List[OutgoingRequest] = []

        if state.phase is Phase.IDLE:
            if remaining is not None and remaining <= self._config.trigger_distance_m:
                state.phase = Phase.APPROACHING
                state.request_id = self._next_request_id
                self._next_request_id += 1
                state.grant = None
                state.lease_expiry = None
                state.last_sent = now
                outgoing.append(OutgoingRequest(zone_id, state.request_id, TYPE_ACCESS))

        elif state.phase in (Phase.APPROACHING, Phase.BLOCKED):
            if point_in_rect(robot_xy, spec.area):
                state.phase = Phase.INSIDE
            elif state.phase is Phase.APPROACHING and now - state.last_sent >= self._config.resend_period_s:
                outgoing.append(OutgoingRequest(zone_id, state.request_id, TYPE_ACCESS))
                state.last_sent = now

        elif state.phase is Phase.INSIDE:
            if not point_in_rect(robot_xy, spec.area):
                outgoing.append(OutgoingRequest(zone_id, state.request_id, TYPE_RELEASE))
                self._state[zone_id] = _ZoneState()
                return outgoing, None
            if now - state.last_sent >= self._config.resend_period_s:
                outgoing.append(OutgoingRequest(zone_id, state.request_id, TYPE_ACCESS))
                state.last_sent = now

        speed_cap = None
        if state.phase in (Phase.APPROACHING, Phase.BLOCKED) and state.grant != GRANT_GRANTED \
                and remaining is not None:
            speed_cap = speed_cap_for_stop(remaining, self._config.stop_margin_m, self._config.decel_mps2)
        return outgoing, speed_cap

    def on_grant(self, robot_id: str, zone_id: str, request_id: int, grant: int,
                 lease_expiry: Optional[float], reason: int, holder_robot_id: str) -> Optional[str]:
        """ZoneGrant 수신. 우리 로봇 것이 아니거나 지금 기다리는 request_id가 아니면 무시한다.

        REJECTED로 막 전환됐을 때만 경고 메시지를 돌려준다 (노드가 로그로 남기도록). 그 외는 None.
        """
        if robot_id != self._config.robot_id:
            return None
        state = self._state.get(zone_id)
        if state is None or state.phase not in (Phase.APPROACHING, Phase.INSIDE):
            return None
        if state.request_id != request_id:
            return None
        state.grant = grant
        state.lease_expiry = lease_expiry
        if grant == GRANT_REJECTED and state.phase is Phase.APPROACHING:
            state.phase = Phase.BLOCKED
            return (f'구역 {zone_id} 진입 거부 (reason={reason}, '
                    f'holder={holder_robot_id or "-"}) → 진입하지 않음')
        return None
