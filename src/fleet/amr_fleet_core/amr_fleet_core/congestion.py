"""구역별 혼잡도와 Nav2 costmap 마스크. 정의는 docs/api.md 4-5절 (D1).

- 원시 값: min(1, 구역 안 사람 수 / capacity). 사람 중심점이 구역 사각형 안(경계 포함)이면 센다. 구역이 겹치면 양쪽에 다 센다.
- 혼잡도: 지수 평활 value = α × raw + (1 − α) × 직전 value. 첫 갱신은 평활 없이 raw다(0에서 천천히 올라오지 않게).
- 마스크: 모든 구역을 덮는 사각형에 둘레 padding을 더해 resolution 격자로 나누고, 칸 중심이 구역 안이면
  round(value × max_value). 구역 밖은 0, 겹치면 큰 값. 행은 아래(y 작은 쪽)부터다 (nav_msgs/OccupancyGrid와 같은 순서).
  padding: Nav2 Humble 속도 필터는 로봇이 마스크 밖에 있으면 마지막 제한을 그대로 둔다. 혼잡 구역에서 곧바로 마스크
  밖으로 나가면 감속이 안 풀리므로, 구역 둘레를 값 0(제한 없음) 칸으로 감싼다.

좌표는 모두 map 기준 m다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

Point = tuple[float, float]


@dataclass(frozen=True)
class CongestionZone:
    zone_id: str
    # [x_min, x_max, y_min, y_max]
    area: tuple[float, float, float, float]
    # 이 인원이면 혼잡도 1.0
    capacity: int

    def __post_init__(self):
        x0, x1, y0, y1 = self.area
        if not (x0 < x1 and y0 < y1):
            raise ValueError(f'{self.zone_id}: area는 [x_min, x_max, y_min, y_max] 순서여야 한다: {self.area}')
        if self.capacity < 1:
            raise ValueError(f'{self.zone_id}: capacity는 1 이상이어야 한다: {self.capacity}')

    def contains(self, p: Point) -> bool:
        x0, x1, y0, y1 = self.area
        return x0 <= p[0] <= x1 and y0 <= p[1] <= y1


@dataclass(frozen=True)
class ZoneValue:
    """ZoneCongestion 1건."""

    zone_id: str
    people_count: int
    value: float


@dataclass(frozen=True)
class Mask:
    """OccupancyGrid 한 장. data는 width × height, 아래 행부터."""

    origin: Point
    resolution: float
    width: int
    height: int
    data: list[int]


def zones_from_dict(data: dict) -> list[CongestionZone]:
    """혼잡도 구역 파일(yaml을 읽은 dict) → 구역 목록. id가 겹치면 ValueError."""
    zones = [CongestionZone(str(z['id']), tuple(float(v) for v in z['area']), int(z['capacity']))
             for z in (data or {}).get('zones') or []]
    ids = [z.zone_id for z in zones]
    dup = sorted({i for i in ids if ids.count(i) > 1})
    if dup:
        raise ValueError(f'구역 id가 겹친다: {dup}')
    return zones


def count_people(zones: Sequence[CongestionZone], people: Iterable[Point]) -> list[int]:
    people = list(people)
    return [sum(z.contains(p) for p in people) for z in zones]


class CongestionTracker:
    """구역별 혼잡도를 평활하며 갱신한다."""

    def __init__(self, zones: Sequence[CongestionZone], alpha: float = 0.5):
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f'alpha는 (0, 1] 범위여야 한다: {alpha}')
        self.zones = list(zones)
        self.alpha = alpha
        self._values: Optional[list[float]] = None

    def update(self, people: Iterable[Point]) -> list[ZoneValue]:
        counts = count_people(self.zones, people)
        raw = [min(1.0, n / z.capacity) for n, z in zip(counts, self.zones)]
        if self._values is None:
            self._values = raw
        else:
            a = self.alpha
            self._values = [a * r + (1.0 - a) * v for r, v in zip(raw, self._values)]
        return [ZoneValue(z.zone_id, n, v) for z, n, v in zip(self.zones, counts, self._values)]


def make_mask(zones: Sequence[CongestionZone], values: Sequence[float], resolution: float = 0.1,
              max_value: int = 90, padding: float = 0.0) -> Optional[Mask]:
    """구역별 혼잡도 → 마스크. 구역이 없으면 None."""
    if resolution <= 0.0:
        raise ValueError(f'resolution은 0보다 커야 한다: {resolution}')
    if not 0 <= max_value <= 100:
        raise ValueError(f'max_value는 0~100이어야 한다: {max_value}')
    if padding < 0.0:
        raise ValueError(f'padding은 0 이상이어야 한다: {padding}')
    if not zones:
        return None
    x0 = min(z.area[0] for z in zones) - padding
    x1 = max(z.area[1] for z in zones) + padding
    y0 = min(z.area[2] for z in zones) - padding
    y1 = max(z.area[3] for z in zones) + padding
    # 부동소수 오차로 칸 하나가 더 붙지 않게 조금 깎고 올린다
    width = max(1, math.ceil((x1 - x0) / resolution - 1e-9))
    height = max(1, math.ceil((y1 - y0) / resolution - 1e-9))
    data = [0] * (width * height)
    for z, v in zip(zones, values):
        cell = round(min(1.0, max(0.0, v)) * max_value)
        if cell == 0:
            continue
        zx0, zx1, zy0, zy1 = z.area
        # 칸 중심이 구역 안인 칸 범위
        i0 = max(0, math.ceil((zx0 - x0) / resolution - 0.5))
        i1 = min(width - 1, math.floor((zx1 - x0) / resolution - 0.5))
        j0 = max(0, math.ceil((zy0 - y0) / resolution - 0.5))
        j1 = min(height - 1, math.floor((zy1 - y0) / resolution - 0.5))
        for j in range(j0, j1 + 1):
            row = j * width
            for i in range(i0, i1 + 1):
                if data[row + i] < cell:
                    data[row + i] = cell
    return Mask((x0, y0), resolution, width, height, data)
