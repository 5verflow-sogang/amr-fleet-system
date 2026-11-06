"""자동 충전 배정 (배터리 인지 운영 1단계). ROS 비의존.

놀고 있는 로봇의 배터리가 low_percent 밑이면 빈 충전기 중 가장 가까운 곳으로 보낸다. 운영 중인 로봇이 최소
min_in_service대 아래로 떨어지지 않게 동시 충전 대수를 (살아 있는 로봇 수 − min_in_service)로 묶는다. 자리가 없으면
기다리고(대기열), 기다리는 동안에도 할당 하한(min_battery_percent) 위면 작업을 계속 받는다. 배터리가 가장 낮은
로봇부터 보낸다. 일하는 중인 로봇은 작업을 끝낸 뒤 본다 (작업 중 배터리 부족 대응은 3단계).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping, Optional, Sequence

from .assignment.types import Point, RobotInfo


@dataclass(frozen=True)
class ChargeConfig:
    # 이보다 낮으면 충전하러 보낸다 (%). 할당 하한(min_battery_percent 20)보다 높게 둬 일을 못 받기 전에 보낸다
    low_percent: float = 30.0
    # 운영 중(충전하러 가거나 충전 중이 아닌) 로봇을 최소 이만큼 남긴다. 0이면 충전기 수로만 묶는다
    min_in_service: int = 0
    # 충전기에서 이 거리 [m] 안에 예약하지 않은 로봇이 서 있으면 그 충전기는 쓰지 않는다
    occupied_radius_m: float = 0.5

    def __post_init__(self):
        if not 0.0 <= self.low_percent <= 100.0:
            raise ValueError(f'low_percent는 0~100: {self.low_percent}')
        if self.min_in_service < 0:
            raise ValueError(f'min_in_service는 0 이상: {self.min_in_service}')
        if not self.occupied_radius_m >= 0.0:
            raise ValueError(f'occupied_radius_m는 0 이상: {self.occupied_radius_m}')


@dataclass(frozen=True)
class Station:
    """충전기·대기 위치 1곳 (map 좌표)."""

    name: str
    xy: Point


@dataclass(frozen=True)
class ChargeOrder:
    robot_id: str
    station: str


@dataclass
class ChargePlan:
    # 이번에 충전하러 보낼 로봇
    orders: list[ChargeOrder] = field(default_factory=list)
    # 충전이 필요하지만 자리(최소 운영 대수·빈 충전기)가 없어 기다리는 로봇 (배터리 낮은 순)
    waiting: list[str] = field(default_factory=list)


def max_charging(alive: int, config: ChargeConfig) -> int:
    """동시에 충전(하러 가는 것 포함)할 수 있는 대수. min_in_service가 0이면 제한 없음(충전기 수로만 묶인다)."""
    if config.min_in_service <= 0:
        return alive
    return max(0, alive - config.min_in_service)


def nearest_free(xy: Point, stations: Sequence[Station], taken: set[str]) -> Optional[Station]:
    """taken에 없는 곳 중 가장 가까운 곳. 거리가 같으면 이름 순. 없으면 None."""
    free = [s for s in stations if s.name not in taken]
    if not free:
        return None
    return min(free, key=lambda s: (math.dist(xy, s.xy), s.name))


def plan_charging(
    alive: Sequence[RobotInfo],
    free_ids: set[str],
    charging: Mapping[str, str],
    chargers: Sequence[Station],
    config: ChargeConfig,
) -> ChargePlan:
    """누구를 어느 충전기로 보낼지.

    alive: 살아 있는 로봇 전부 (충전 중 포함). free_ids: 그중 맡은 작업이 없어 지금 보낼 수 있는 로봇.
    charging: 이미 충전 작업을 맡은 로봇 → 충전기 이름 (이동 중·충전 중 모두).
    """
    plan = ChargePlan()
    slots = max_charging(len(alive), config) - len(charging)
    taken = set(charging.values())
    # 예약 없이 충전기 위에 서 있는 로봇 (충전을 마치고 일을 기다리는 로봇 등). 그 로봇 자신은 그 자리를 쓸 수 있다
    standing = {r.robot_id: {s.name for s in chargers if math.dist(r.xy, s.xy) <= config.occupied_radius_m}
                for r in alive if r.robot_id not in charging}
    need = sorted((r for r in alive if r.robot_id not in charging and r.battery_percent < config.low_percent),
                  key=lambda r: (r.battery_percent, r.robot_id))
    for r in need:
        station = None
        if slots > 0 and r.robot_id in free_ids:
            blocked = {name for other, names in standing.items() if other != r.robot_id for name in names}
            station = nearest_free(r.xy, chargers, taken | blocked)
        if station is None:
            plan.waiting.append(r.robot_id)
            continue
        plan.orders.append(ChargeOrder(r.robot_id, station.name))
        taken.add(station.name)
        slots -= 1
    return plan
