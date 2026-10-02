"""정답 대비 위치 오차, 정지·직선·회전 구간 분류, RMSE·Max, KPI 판정. ROS에 의존하지 않는다.

로그 형식은 하네스 로그 포맷(tools/harness/README.md)의 `localization.csv`를 따른다.
"""

from __future__ import annotations

import bisect
import csv
import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

from .trajectory import Pose2D

STATIONARY = 'stationary'
STRAIGHT = 'straight'
ROTATION = 'rotation'
SEGMENTS = (STATIONARY, STRAIGHT, ROTATION)

# 원 명세서 위치 오차 KPI [m]
KPI_THRESHOLDS = {STATIONARY: 0.03, STRAIGHT: 0.05, ROTATION: 0.08}
# 원 명세서: 최소 100개 시점 샘플링
MIN_SAMPLES = 100

# localization.csv 컬럼. 명세서 컬럼 6개 뒤에 부가 컬럼을 붙인다.
CSV_COLUMNS = ('timestamp', 'gt_x', 'gt_y', 'est_x', 'est_y', 'error', 'segment', 'yaw_error', 'robot')


@dataclass(frozen=True)
class ErrorSample:
    """짝지어진 한 시점의 오차."""

    gt: Pose2D
    est: Pose2D
    error: float
    yaw_error: float
    segment: str


@dataclass(frozen=True)
class Summary:
    """한 구간의 오차 요약 [m]."""

    n: int
    rmse: float
    mean: float
    max: float


def normalize_angle(a: float) -> float:
    """각도를 -π~π로 맞춘다."""
    return math.atan2(math.sin(a), math.cos(a))


def gt_velocity(gt: Sequence[Pose2D], t: float) -> tuple[float, float]:
    """정답 궤적에서 시각 t 근처의 선속도 [m/s]·각속도 [rad/s]를 앞뒤 차분으로 구한다."""
    if len(gt) < 2:
        return 0.0, 0.0
    times = [p.t for p in gt]
    i = min(max(bisect.bisect_left(times, t), 1), len(gt) - 1)
    a, b = gt[i - 1], gt[i]
    dt = b.t - a.t
    if dt <= 0.0:
        return 0.0, 0.0
    v = math.hypot(b.x - a.x, b.y - a.y) / dt
    w = normalize_angle(b.yaw - a.yaw) / dt
    return v, w


def classify_segment(v: float, w: float, moving_v: float = 0.01, moving_w: float = 0.05) -> str:
    """속도로 구간을 나눈다.

    각속도가 moving_w 이상이면 회전, 아니고 선속도가 moving_v 이상이면 직선, 둘 다 아니면 정지다.
    곡선 주행(선속도와 각속도가 함께 있음)은 회전으로 센다. 기준값은 초안이며 하네스 문서에서 확정한다.
    """
    if abs(w) >= moving_w:
        return ROTATION
    if abs(v) >= moving_v:
        return STRAIGHT
    return STATIONARY


def compute_errors(
    pairs: Iterable[tuple[Pose2D, Pose2D]],
    gt: Sequence[Pose2D],
    moving_v: float = 0.01,
    moving_w: float = 0.05,
) -> list[ErrorSample]:
    """짝지어진 (정답, 추정)마다 위치 오차·방향 오차·구간을 계산한다. 구간은 정답 궤적의 속도로 정한다."""
    samples = []
    for g, e in pairs:
        v, w = gt_velocity(gt, g.t)
        samples.append(ErrorSample(
            gt=g,
            est=e,
            error=math.hypot(g.x - e.x, g.y - e.y),
            yaw_error=normalize_angle(e.yaw - g.yaw),
            segment=classify_segment(v, w, moving_v, moving_w),
        ))
    return samples


def summarize(errors: Sequence[float]) -> Optional[Summary]:
    """오차 목록의 RMSE·평균·최대. 비어 있으면 None."""
    if not errors:
        return None
    n = len(errors)
    return Summary(
        n=n,
        rmse=math.sqrt(sum(x * x for x in errors) / n),
        mean=sum(errors) / n,
        max=max(errors),
    )


def summarize_by_segment(samples: Sequence[ErrorSample]) -> dict[str, Optional[Summary]]:
    """구간별 요약. 샘플이 없는 구간은 None."""
    return {seg: summarize([s.error for s in samples if s.segment == seg]) for seg in SEGMENTS}


def kpi_check(summaries: dict[str, Optional[Summary]]) -> dict[str, Optional[bool]]:
    """구간별 KPI 통과 여부. RMSE가 기준 이하이고 샘플이 MIN_SAMPLES 이상이면 통과.

    샘플이 없는 구간은 None (판정 불가). Max는 판정에 쓰지 않고 보고만 한다.
    """
    result = {}
    for seg, threshold in KPI_THRESHOLDS.items():
        s = summaries.get(seg)
        result[seg] = None if s is None else (s.n >= MIN_SAMPLES and s.rmse <= threshold)
    return result


def write_localization_csv(path: str, samples: Iterable[ErrorSample], robot: str = '') -> None:
    """하네스 `localization.csv`로 쓴다. 시각은 정답 시점 기준이다."""
    with open(path, 'w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(CSV_COLUMNS)
        for s in samples:
            writer.writerow([
                f'{s.gt.t:.9f}', f'{s.gt.x:.6f}', f'{s.gt.y:.6f}', f'{s.est.x:.6f}', f'{s.est.y:.6f}',
                f'{s.error:.6f}', s.segment, f'{s.yaw_error:.6f}', robot,
            ])
