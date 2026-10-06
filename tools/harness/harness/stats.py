"""반복 실행 간 통계. 표준 라이브러리만 쓴다 (ROS·numpy 없이 테스트).

신뢰구간은 실행 단위 부트스트랩이다(README 합의 기록 "지연 신뢰구간", #33). 실행마다 지표 하나를 먼저 낸 뒤
실행을 복원 추출한다. 메시지·시점 단위로 섞으면 같은 실행 안의 값들이 독립이 아니라서 구간이 실제보다 좁아진다.
"""

from __future__ import annotations

import math
import random
from dataclasses import asdict, dataclass
from typing import Optional, Sequence

RESAMPLES = 10_000
CONFIDENCE = 0.95


@dataclass(frozen=True)
class Interval:
    """실행별 값의 요약. ci_low·ci_high는 실행이 2개 미만이면 None."""

    n: int
    mean: float
    std: float
    min: float
    max: float
    ci_low: Optional[float]
    ci_high: Optional[float]

    def as_dict(self) -> dict:
        return asdict(self)


def percentile(sorted_values: Sequence[float], q: float) -> float:
    """선형 보간 백분위수 (q는 0~1). 값은 정렬돼 있어야 한다."""
    if not sorted_values:
        raise ValueError('빈 목록')
    pos = q * (len(sorted_values) - 1)
    lo = math.floor(pos)
    hi = min(lo + 1, len(sorted_values) - 1)
    return sorted_values[lo] + (pos - lo) * (sorted_values[hi] - sorted_values[lo])


def bootstrap_mean_ci(values: Sequence[float], resamples: int = RESAMPLES,
                      confidence: float = CONFIDENCE, seed: int = 0) -> tuple[Optional[float], Optional[float]]:
    """평균의 부트스트랩 백분위 신뢰구간. 값이 2개 미만이면 (None, None).

    seed를 고정해 같은 결과 폴더로 다시 요약하면 같은 구간이 나온다.
    """
    n = len(values)
    if n < 2:
        return None, None
    rng = random.Random(seed)
    means = sorted(sum(rng.choice(values) for _ in range(n)) / n for _ in range(resamples))
    alpha = (1.0 - confidence) / 2.0
    return percentile(means, alpha), percentile(means, 1.0 - alpha)


def summarize_runs(values: Sequence[float], seed: int = 0) -> Optional[Interval]:
    """실행별 값 목록 → 평균·표준편차·범위·부트스트랩 CI. 비어 있으면 None."""
    vals = [float(v) for v in values if v is not None and not math.isnan(v)]
    if not vals:
        return None
    n = len(vals)
    mean = sum(vals) / n
    std = math.sqrt(sum((v - mean) ** 2 for v in vals) / (n - 1)) if n > 1 else 0.0
    low, high = bootstrap_mean_ci(vals, seed=seed)
    return Interval(n, mean, std, min(vals), max(vals), low, high)
