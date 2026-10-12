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


def distance(a: Point, b: Point) -> float:
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
    best_distance = distance(robot_xy, path_xy[0])
    for i in range(1, len(path_xy)):
        d = distance(robot_xy, path_xy[i])
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
            hit_distance = total + distance(prev, hit)
            return None if (max_distance is not None and hit_distance > max_distance) else hit_distance
        total += distance(prev, point)
        prev = point
    return None


def speed_cap_for_stop(remaining_to_boundary: float, stop_margin: float, decel: float) -> float:
    """경계에서 stop_margin만큼 앞에 정지하기 위한 선속도 상한 (등가감속 decel 기준)."""
    remaining_to_stopline = remaining_to_boundary - stop_margin
    if remaining_to_stopline <= 0.0:
        return 0.0
    return (2.0 * decel * remaining_to_stopline) ** 0.5


def ramp_speed(current: float, target: float, dt: float, rate: float) -> float:
    """current에서 target으로 이번 틱에 rate(단위/s) 이내로만 움직인다 (양방향).

    후진 속도(크기, 항상 0 이상)의 시작·정지를 decel_mps2 이내로 부드럽게 하는 데 쓴다
    (geometry.speed_cap_for_stop로 정한 목표를 향해, ramp_cap_down과 달리 올라갈 때도 똑같이
    rate로 묶는다 - 후진 시작도 "느닷없이"가 아니라 가속해야 하므로).
    """
    if dt <= 0.0:
        return current
    step = rate * dt
    diff = target - current
    if abs(diff) <= step:
        return target
    return current + (step if diff > 0.0 else -step)


def combine_caps(*caps: Optional[float]) -> Optional[float]:
    """여러 선속도 상한 중 가장 작은 값(가장 빡빡한 제약). None(제한 없음)은 무시하고,
    전부 None이면 None."""
    finite = [c for c in caps if c is not None]
    return min(finite) if finite else None


def ramp_cap_down(
    target: Optional[float], prev: Optional[float], dt: float, decel: float, no_limit_value: float,
    current_speed_mps: Optional[float] = None,
) -> Optional[float]:
    """선속도 상한이 낮아질 때만 decel(m/s^2) 이내로 따라가게 한다 - 느닷없이 낮아지는
    상한(예: SafetyStatus.speed_limit)을 구역 진입 감속(speed_cap_for_stop)과 같은 방식으로
    부드럽게 반영하는 데 쓴다.

    높아지거나 그대로면(완화) 바로 target을 쓴다 - 완화는 안전 쪽이라 부드럽게 할 필요가
    없다. dt<=0이면(아직 시간이 안 지났으면) 낮아지는 쪽으로는 움직이지 않는다(prev 유지).

    None(제한 없음)은 no_limit_value(로봇이 낼 수 있는 가장 빠른 속도)로 다룬다 - "제한
    없음"과 "그 속도로 제한"은 로봇 입장에서 같은 뜻이고, 뺄셈이 가능한 값으로 바꿔야
    램프가 된다(무한대는 아무리 빼도 무한대다). 결과가 no_limit_value 이상이면 다시
    None으로 돌려준다.

    current_speed_mps(로봇이 지금 실제로 받고 있는 명령 선속도)를 주면 램프 시작점을
    min(prev, current_speed_mps)로 낮춘다(#93 연우 리뷰) - prev가 None이거나 아직 높은
    값일 때, 로봇이 실제로는 더 느리게 가고 있으면(예: 제한이 없어서 prev가 최고속으로
    잡혀 있지만 Nav2가 어차피 느리게 명령하는 중) 그 가짜로 높은 지점에서부터 내려가느라
    감속이 실제 속도를 따라잡을 때까지 미뤄진다 - cmd_vel = min(명령, 상한)이라 상한이
    실제 속도보다 높은 동안은 상한이 전혀 효과가 없기 때문이다. 안 주면(None) 기존과
    같다(현재 속도를 모른다고 보고 안 낮춤).
    """
    target_v = no_limit_value if target is None else target
    prev_v = no_limit_value if prev is None else prev
    if current_speed_mps is not None:
        prev_v = min(prev_v, current_speed_mps)
    if target_v >= prev_v:
        result = target_v
    elif dt <= 0.0:
        result = prev_v
    else:
        result = max(target_v, prev_v - decel * dt)
    return None if result >= no_limit_value else result
