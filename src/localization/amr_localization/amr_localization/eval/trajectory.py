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


def write_tum(path: str, poses: Iterable[Pose2D]) -> None:
    """TUM 파일로 쓴다. 평면 로봇이라 z는 0이다."""
    with open(path, 'w', encoding='utf-8') as f:
        for p in poses:
            qx, qy, qz, qw = quaternion_from_yaw(p.yaw)
            f.write(f'{p.t:.9f} {p.x:.6f} {p.y:.6f} 0.0 {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n')


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
