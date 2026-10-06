"""경로 위 구역 경계까지 남은 거리, 정지 속도 프로파일. ROS 비의존, 순수 함수만 있다.

Rect는 (x_min, x_max, y_min, y_max) (zones.yaml의 area_world 순서, m).
"""

from __future__ import annotations

from typing import Optional, Sequence, Tuple

Point = Tuple[float, float]
Rect = Tuple[float, float, float, float]


def point_in_rect(xy: Point, rect: Rect) -> bool:
    x, y = xy
    x_min, x_max, y_min, y_max = rect
    return x_min <= x <= x_max and y_min <= y <= y_max


def _dist(a: Point, b: Point) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def _segment_rect_entry(a: Point, b: Point, rect: Rect) -> Optional[Point]:
    """a→b 선분이 rect 밖에서 안으로 들어가는 첫 지점. a가 이미 안이거나 안 만나면 None (Liang-Barsky)."""
    x_min, x_max, y_min, y_max = rect
    dx, dy = b[0] - a[0], b[1] - a[1]
    t_near, t_far = 0.0, 1.0
    for origin, delta, lo, hi in ((a[0], dx, x_min, x_max), (a[1], dy, y_min, y_max)):
        if delta == 0.0:
            if origin < lo or origin > hi:
                return None
            continue
        t0 = (lo - origin) / delta
        t1 = (hi - origin) / delta
        if t0 > t1:
            t0, t1 = t1, t0
        t_near = max(t_near, t0)
        t_far = min(t_far, t1)
        if t_near > t_far:
            return None
    if t_near <= 0.0:
        return None  # a가 이미 rect 안(또는 경계)이면 point_in_rect 쪽에서 처리한다
    return (a[0] + dx * t_near, a[1] + dy * t_near)


def _closest_index(path_xy: Sequence[Point], robot_xy: Point) -> int:
    """robot_xy와 가장 가까운 path_xy의 인덱스. 경로가 로봇 뒤쪽 점도 담고 있을 때,
    그 지난 점들부터 거리를 쌓지 않으려고 쓴다(하연 리뷰, #66)."""
    best_index = 0
    best_distance = _dist(robot_xy, path_xy[0])
    for i in range(1, len(path_xy)):
        d = _dist(robot_xy, path_xy[i])
        if d < best_distance:
            best_distance = d
            best_index = i
    return best_index


def remaining_distance_to_rect(
    path_xy: Sequence[Point], robot_xy: Point, rect: Rect, max_distance: Optional[float] = None,
) -> Optional[float]:
    """robot_xy에서, path_xy 중 robot_xy와 가장 가까운 점부터 따라갈 때 rect에 처음
    들어가는 지점까지 남은 거리(그 앞쪽 점은 이미 지난 것으로 보고 무시한다).

    이미 rect 안이면 0.0. path가 rect에 닿지 않거나, max_distance(주면)보다 멀면 None
    (지금은 그 구역을 신경 쓸 때가 아니라는 뜻 - 긴 경로를 끝까지 훑지 않게 한다).
    """
    if point_in_rect(robot_xy, rect):
        return 0.0
    if not path_xy:
        return None

    start = _closest_index(path_xy, robot_xy)
    total = 0.0
    prev = robot_xy
    for point in path_xy[start:]:
        if max_distance is not None and total > max_distance:
            return None
        hit = _segment_rect_entry(prev, point, rect)
        if hit is not None:
            distance = total + _dist(prev, hit)
            return None if (max_distance is not None and distance > max_distance) else distance
        total += _dist(prev, point)
        prev = point
    return None


def speed_cap_for_stop(remaining_to_boundary: float, stop_margin: float, decel: float) -> float:
    """경계에서 stop_margin만큼 앞에 정지하기 위한 선속도 상한 (등가감속 decel 기준)."""
    remaining_to_stopline = remaining_to_boundary - stop_margin
    if remaining_to_stopline <= 0.0:
        return 0.0
    return (2.0 * decel * remaining_to_stopline) ** 0.5
