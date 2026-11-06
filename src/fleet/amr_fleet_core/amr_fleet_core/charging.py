"""자동 충전 배정 (배터리 인지 운영 1단계). ROS 비의존.

놀고 있는 로봇의 배터리가 low_percent 밑이면 빈 충전기 중 가장 가까운 곳으로 보낸다. 운영 중인 로봇이 최소
min_in_service대 아래로 떨어지지 않게 동시 충전 대수를 (살아 있는 로봇 수 − min_in_service)로 묶는다. 자리가 없으면
기다리고(대기열), 기다리는 동안에도 할당 하한(min_battery_percent) 위면 작업을 계속 받는다. 배터리가 가장 낮은
로봇부터 보낸다. 일하는 중인 로봇은 작업을 끝낸 뒤 본다 (작업 중 배터리 부족 대응은 3단계).

피크 대비 (4단계, peaks를 줄 때): 시각 t는 호출 측이 정한 일정 시계(서버 시작부터 s)다.
- 피크 pre_peak_s 전부터 피크 시작까지: pre_peak_percent 밑인 놀고 있는 로봇도 충전하러 보낸다 (미리 채움)
- 피크 중: 동시 충전을 peak_max_charging대로 더 묶고(0이면 그대로), 충전은 peak_target_percent까지만 (호출 측이
  그 배터리에 닿은 충전 작업을 끝낸다)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping, Optional, Sequence, Tuple

from .assignment.types import Point, RobotInfo


@dataclass(frozen=True)
class ChargeConfig:
    # 이보다 낮으면 충전하러 보낸다 (%). 할당 하한(min_battery_percent 20)보다 높게 둬 일을 못 받기 전에 보낸다
    low_percent: float = 30.0
    # 운영 중(충전하러 가거나 충전 중이 아닌) 로봇을 최소 이만큼 남긴다. 0이면 충전기 수로만 묶는다
    min_in_service: int = 0
    # 충전기에서 이 거리 [m] 안에 예약하지 않은 로봇이 서 있으면 그 충전기는 쓰지 않는다
    occupied_radius_m: float = 0.5
    # 피크 시간대 [(시작 s, 끝 s), ...] (일정 시계). 비면 피크 대비를 끈다
    peaks: Tuple[Tuple[float, float], ...] = ()
    # 피크 시작 이만큼 전부터 pre_peak_percent 밑이면 미리 충전하러 보낸다 (s, 0이면 끔)
    pre_peak_s: float = 0.0
    pre_peak_percent: float = 80.0
    # 피크 중 동시 충전 상한 (최소 운영 대수 규칙과 둘 중 작은 쪽). 0이면 더 묶지 않는다
    peak_max_charging: int = 0
    # 피크 중 충전 목표 (%). 100이면 로봇 기본(끝까지)
    peak_target_percent: float = 100.0

    def __post_init__(self):
        if not 0.0 <= self.low_percent <= 100.0:
            raise ValueError(f'low_percent는 0~100: {self.low_percent}')
        if self.min_in_service < 0:
            raise ValueError(f'min_in_service는 0 이상: {self.min_in_service}')
        if not self.occupied_radius_m >= 0.0:
            raise ValueError(f'occupied_radius_m는 0 이상: {self.occupied_radius_m}')
        for start, end in self.peaks:
            if not start < end:
                raise ValueError(f'피크는 (시작, 끝)이고 시작 < 끝: {self.peaks}')
        if self.pre_peak_s < 0 or self.peak_max_charging < 0:
            raise ValueError(f'pre_peak_s·peak_max_charging은 0 이상: {self.pre_peak_s}, {self.peak_max_charging}')
        if self.pre_peak_s > 0 and not self.low_percent <= self.pre_peak_percent <= 100.0:
            raise ValueError(f'pre_peak_percent는 low_percent~100: {self.pre_peak_percent}')
        if self.peaks and not self.low_percent < self.peak_target_percent <= 100.0:
            raise ValueError(f'peak_target_percent는 low_percent 초과 100 이하: {self.peak_target_percent}')


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


PEAK = 'peak'
PRE_PEAK = 'pre_peak'


def peak_phase(t: Optional[float], config: ChargeConfig) -> str:
    """일정 시각 t가 피크 중(PEAK), 피크 직전 미리 충전 구간(PRE_PEAK), 아니면 ''."""
    if t is None:
        return ''
    if any(start <= t < end for start, end in config.peaks):
        return PEAK
    if config.pre_peak_s > 0 and any(start - config.pre_peak_s <= t < start for start, _ in config.peaks):
        return PRE_PEAK
    return ''


def charge_threshold(t: Optional[float], config: ChargeConfig) -> float:
    """이보다 낮은 놀고 있는 로봇을 충전하러 보낸다. 피크 직전이면 pre_peak_percent."""
    return config.pre_peak_percent if peak_phase(t, config) == PRE_PEAK else config.low_percent


def charge_target(t: Optional[float], config: ChargeConfig) -> Optional[float]:
    """지금 충전을 끝낼 배터리 (%). None이면 로봇 기본(끝까지). 피크 중에만 peak_target_percent."""
    if peak_phase(t, config) == PEAK and config.peak_target_percent < 100.0:
        return config.peak_target_percent
    return None


def max_charging(alive: int, config: ChargeConfig, t: Optional[float] = None) -> int:
    """동시에 충전(하러 가는 것 포함)할 수 있는 대수. min_in_service가 0이면 제한 없음(충전기 수로만 묶인다).

    피크 중이면 peak_max_charging(0이 아니면)으로 더 묶는다.
    """
    limit = alive if config.min_in_service <= 0 else max(0, alive - config.min_in_service)
    if peak_phase(t, config) == PEAK and config.peak_max_charging > 0:
        limit = min(limit, config.peak_max_charging)
    return limit


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
    t: Optional[float] = None,
) -> ChargePlan:
    """누구를 어느 충전기로 보낼지.

    alive: 살아 있는 로봇 전부 (충전 중 포함). free_ids: 그중 맡은 작업이 없어 지금 보낼 수 있는 로봇.
    charging: 이미 충전 작업을 맡은 로봇 → 충전기 이름 (이동 중·충전 중 모두). t: 일정 시각 (피크 대비, 없으면 끔).
    """
    plan = ChargePlan()
    slots = max_charging(len(alive), config, t) - len(charging)
    threshold = charge_threshold(t, config)
    taken = set(charging.values())
    # 예약 없이 충전기 위에 서 있는 로봇 (충전을 마치고 일을 기다리는 로봇 등). 그 로봇 자신은 그 자리를 쓸 수 있다
    standing = {r.robot_id: {s.name for s in chargers if math.dist(r.xy, s.xy) <= config.occupied_radius_m}
                for r in alive if r.robot_id not in charging}
    need = sorted((r for r in alive if r.robot_id not in charging and r.battery_percent < threshold),
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
