"""마커 두 장의 중심 → 도킹 자세 오차 (ROS 비의존, 2D).

좌표는 모두 로봇 base_footprint 기준 (x 앞, y 왼쪽). 판 위 왼쪽·오른쪽 마커 중심을 이은 선 u(왼→오)가
판 면의 방향이고, 판 앞면의 바깥 방향(로봇 쪽) n은 u를 -90° 돌린 것이다. 도킹 자세는 판 중심에서
n 방향으로 standoff만큼 나온 점이고, 그 자세의 진행 방향은 -n(판을 바라봄)이다.

오차(DockError)는 "목표 자세 기준 로봇 위치·방향"이다.
- along: 목표까지 진행 방향으로 남은 거리. 양수면 아직 못 갔고, 음수면 지나침
- lateral: 목표 진행선에서 로봇이 옆으로 벗어난 거리 (목표 기준 왼쪽 +)
- heading: 로봇 방향 − 목표 방향 (rad, 왼쪽으로 돌아 있으면 +)
"""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple

Point = Tuple[float, float]


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class DockError:
    along: float
    lateral: float
    heading: float

    @property
    def position(self) -> float:
        """목표 자세와의 위치 차 [m]."""
        return math.hypot(self.along, self.lateral)


def dock_error(left: Point, right: Point, standoff: float) -> DockError:
    """왼쪽·오른쪽 마커 중심(로봇 기준 xy) → 도킹 자세 오차."""
    ux, uy = right[0] - left[0], right[1] - left[1]
    norm = math.hypot(ux, uy)
    if norm < 1e-6:
        raise ValueError('마커 두 장의 중심이 겹친다')
    ux, uy = ux / norm, uy / norm
    nx, ny = uy, -ux                      # u를 -90° 돌림: 판 바깥(로봇 쪽)
    cx, cy = (left[0] + right[0]) / 2, (left[1] + right[1]) / 2
    tx, ty = cx + standoff * nx, cy + standoff * ny   # 목표 위치 (로봇 기준)
    theta = math.atan2(-ny, -nx)          # 목표 방향 (로봇 기준) = 판을 바라봄
    # 로봇(원점, 방향 0)을 목표 좌표계로: p = R(-theta)·(0 - t)
    c, s = math.cos(theta), math.sin(theta)
    px = c * (-tx) + s * (-ty)
    py = -s * (-tx) + c * (-ty)
    return DockError(along=-px, lateral=py, heading=wrap(-theta))


def average(errors: Sequence[DockError]) -> DockError:
    n = len(errors)
    if n == 0:
        raise ValueError('빈 목록')
    h = math.atan2(sum(math.sin(e.heading) for e in errors), sum(math.cos(e.heading) for e in errors))
    return DockError(sum(e.along for e in errors) / n, sum(e.lateral for e in errors) / n, h)
