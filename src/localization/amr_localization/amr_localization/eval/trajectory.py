"""궤적 표현과 TUM 형식 입출력, 정답·추정 궤적의 시각 짝짓기. ROS에 의존하지 않는다.

TUM 형식은 한 줄에 `timestamp x y z qx qy qz qw` (공백 구분, 초·m·쿼터니언)다.
하네스 로그 포맷(tools/harness/README.md)의 원자료 형식이며 evo가 그대로 읽는다.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Iterable, Sequence


@dataclass(frozen=True)
class Pose2D:
    """한 시점의 평면 위치·방향. 시각은 초, 위치는 m, 방향(yaw)은 rad."""

    t: float
    x: float
    y: float
    yaw: float


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """쿼터니언에서 z축 회전(yaw)만 뽑는다."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """z축 회전만 있는 쿼터니언 (qx, qy, qz, qw)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def read_tum(path: str) -> list[Pose2D]:
    """TUM 파일을 읽는다. 빈 줄과 `#` 주석은 건너뛴다. 시각 순으로 정렬해 돌려준다."""
    poses = []
    with open(path, encoding='utf-8') as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            values = line.split()
            if len(values) != 8:
                raise ValueError(f'{path}:{line_no}: TUM 형식은 값 8개여야 한다 (지금 {len(values)}개)')
            t, x, y, _z, qx, qy, qz, qw = (float(v) for v in values)
            poses.append(Pose2D(t, x, y, yaw_from_quaternion(qx, qy, qz, qw)))
    return sorted(poses, key=lambda p: p.t)


def tum_line(p: Pose2D) -> str:
    """TUM 한 줄 (줄바꿈 포함). 평면 로봇이라 z는 0이다."""
    qx, qy, qz, qw = quaternion_from_yaw(p.yaw)
    return f'{p.t:.9f} {p.x:.6f} {p.y:.6f} 0.0 {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n'


def write_tum(path: str, poses: Iterable[Pose2D]) -> None:
    """TUM 파일로 쓴다."""
    with open(path, 'w', encoding='utf-8') as f:
        for p in poses:
            f.write(tum_line(p))


def transform_to_map(poses: Iterable[Pose2D], map_origin_in_world: tuple[float, float, float]) -> list[Pose2D]:
    """world 기준 자세를 map 기준으로 바꾼다.

    map_origin_in_world는 world에서 본 map 원점의 (x, y, yaw)다. slam_toolbox는 매핑을 시작한
    자세를 map 원점으로 잡으므로, 지도 인계 때 받은 매핑 시작 자세를 그대로 넣는다.
    """
    ox, oy, oyaw = map_origin_in_world
    c, s = math.cos(oyaw), math.sin(oyaw)
    result = []
    for p in poses:
        dx, dy = p.x - ox, p.y - oy
        result.append(Pose2D(p.t, c * dx + s * dy, -s * dx + c * dy,
                             math.atan2(math.sin(p.yaw - oyaw), math.cos(p.yaw - oyaw))))
    return result


def interpolate(
    gt: Sequence[Pose2D],
    t: float,
    max_gap: float,
    times: Sequence[float] | None = None,
) -> Pose2D | None:
    """시각 t를 사이에 둔 두 정답 시점을 선형 보간한다. 방향은 짧은 쪽으로 돈다.

    t가 정답 범위 밖이거나 두 시점 간격이 max_gap(초)보다 크면 None.
    gt는 시각 순으로 정렬돼 있어야 한다. 여러 번 부를 때는 times(gt의 시각 목록)를
    미리 만들어 넘긴다. 매번 만들면 전체 계산이 기록 길이의 제곱으로 느려진다.
    """
    if not gt:
        return None
    if times is None:
        times = [p.t for p in gt]
    i = bisect.bisect_left(times, t)
    if i < len(gt) and times[i] == t:
        return gt[i]
    if i == 0 or i == len(gt):
        return None
    a, b = gt[i - 1], gt[i]
    if b.t - a.t > max_gap:
        return None
    r = (t - a.t) / (b.t - a.t)
    dyaw = math.atan2(math.sin(b.yaw - a.yaw), math.cos(b.yaw - a.yaw))
    yaw = a.yaw + r * dyaw
    return Pose2D(t, a.x + r * (b.x - a.x), a.y + r * (b.y - a.y), math.atan2(math.sin(yaw), math.cos(yaw)))


def associate_interpolated(
    gt: Sequence[Pose2D],
    est: Sequence[Pose2D],
    max_gap: float = 0.05,
) -> list[tuple[Pose2D, Pose2D]]:
    """추정 시점마다 정답을 같은 시각으로 보간해 짝짓는다.

    associate()의 가장 가까운 시점 방식은 두 주기가 어긋나면 시각 차이만큼 오차가 생긴다
    (2 m/s에서 10 ms면 2 cm). 보간하면 이 오차가 사라진다. 기본 max_gap 0.05초는
    100 Hz 정답에서 샘플 몇 개가 빠져도 보간하고, 그보다 긴 공백은 버린다.
    """
    times = [p.t for p in gt]
    pairs = []
    for e in est:
        g = interpolate(gt, e.t, max_gap, times)
        if g is not None:
            pairs.append((g, e))
    return pairs


def associate(
    gt: Sequence[Pose2D],
    est: Sequence[Pose2D],
    max_dt: float = 0.02,
) -> list[tuple[Pose2D, Pose2D]]:
    """추정 시점마다 시각이 가장 가까운 정답 시점을 짝짓는다.

    두 시각 차이가 max_dt(초)를 넘으면 버린다. 기본 0.02초는 50Hz 한 주기다.
    gt는 시각 순으로 정렬돼 있어야 한다.
    """
    if not gt:
        return []
    gt_times = [p.t for p in gt]
    pairs = []
    for e in est:
        i = bisect.bisect_left(gt_times, e.t)
        candidates = [j for j in (i - 1, i) if 0 <= j < len(gt)]
        j = min(candidates, key=lambda k: abs(gt_times[k] - e.t))
        if abs(gt_times[j] - e.t) <= max_dt:
            pairs.append((gt[j], e))
    return pairs
