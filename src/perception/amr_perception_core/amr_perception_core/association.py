"""LiDAR 트랙 ↔ 카메라 검출 연관과 트랙 클래스 누적 (ROS 비의존).

원 명세 6장 인식 + 7장 추적: 카메라(YOLO) 검출을 LiDAR 트랙에 짝지어 트랙에 클래스를 붙인다.
LiDAR는 위치·속도가 정확하지만 무엇인지 모르고, 카메라는 무엇인지 알지만 시야(87°)가 좁고 프레임이 적다.

- associate: 트랙을 검출 촬영 시각으로 등속 예측하고, 마할라노비스 게이트와 최대 거리 안에서 가까운 쌍부터
  1:1로 짝짓는다 (추적기 연관과 같은 방식)
- ClassLabeler: 트랙마다 클래스별 근거(confidence 합)를 시간에 따라 줄이며 쌓고, 근거가 충분한 클래스를 붙인다.
  한 번 오검출로 클래스가 붙지 않고(min_evidence), 붙은 클래스는 근거가 release_evidence 아래로 줄 때까지
  유지한다(히스테리시스). 카메라가 몇 초에 한 번만 검출해도(Gazebo CPU 0.3 FPS) 클래스가 깜빡이지 않는다
"""

from dataclasses import dataclass, field
import math
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np

UNKNOWN_CLASS = 'unknown'


@dataclass(frozen=True)
class AssociationConfig:
    # 게이트: 마할라노비스 거리² 상한. 2자유도 χ² 99%
    gate_mahalanobis_sq: float = 9.21
    # 공분산이 커도 이보다 멀면 짝짓지 않는다 [m]
    max_distance: float = 1.0
    # 이보다 낮은 검출은 쓰지 않는다 (amr_detection confidence_threshold와 같게)
    min_confidence: float = 0.5
    # 클래스 근거가 줄어드는 시정수 [s]: 근거 *= exp(−Δt / τ)
    evidence_tau: float = 5.0
    # 근거 합이 이 이상이어야 클래스를 붙인다 (confidence 0.5 검출 두 번)
    min_evidence: float = 1.0
    # 붙은 클래스는 근거가 이 아래로 줄어야 뗀다 (τ 5 s면 1.0 → 0.3까지 약 6 s)
    release_evidence: float = 0.3
    # 이 시간 [s] 안에 짝지어진 검출이 있으면 지금 카메라로도 보인다고 본다 (source = LIDAR_CAMERA)
    camera_recent_s: float = 1.0

    def __post_init__(self):
        for name in ('gate_mahalanobis_sq', 'max_distance', 'evidence_tau', 'min_evidence',
                     'release_evidence', 'camera_recent_s'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name}는 양수여야 한다: {value}')
        if self.release_evidence > self.min_evidence:
            raise ValueError('release_evidence는 min_evidence 이하여야 한다')
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError(f'min_confidence는 [0, 1] 범위여야 한다: {self.min_confidence}')


@dataclass(frozen=True)
class FixedDetection:
    """고정 좌표계(odom)로 옮긴 카메라 검출 하나."""
    class_name: str
    confidence: float
    xy: np.ndarray          # (2,) [m]
    cov: np.ndarray         # (2, 2) 위치 공분산 [m²]


@dataclass(frozen=True)
class TrackSnapshot:
    """연관에 쓰는 트랙 상태 (stamp 시각의 KF 추정, 고정 좌표계)."""
    track_id: int
    xy: np.ndarray
    velocity: np.ndarray
    cov: np.ndarray
    velocity_cov: np.ndarray
    stamp: float


def associate(tracks: Sequence[TrackSnapshot], detections: Sequence[FixedDetection],
              t_detection: float, config: AssociationConfig = AssociationConfig()
              ) -> List[Tuple[int, int]]:
    """(트랙 번호, 검출 번호) 쌍 목록.

    카메라는 LiDAR와 촬영 시각이 달라(1 m/s 작업자면 0.2 s에 0.2 m) 트랙을 t_detection으로 등속 예측해
    비교한다. 예측 공분산은 P + Δt²·P_v (위치·속도 상관은 무시한다).
    """
    candidates = []
    for ti, track in enumerate(tracks):
        dt = t_detection - track.stamp
        velocity = np.asarray(track.velocity, dtype=float)
        predicted = np.asarray(track.xy, dtype=float) + velocity * dt
        cov = (np.asarray(track.cov, dtype=float)
               + dt * dt * np.asarray(track.velocity_cov, dtype=float))
        for di, det in enumerate(detections):
            if det.confidence < config.min_confidence:
                continue
            diff = np.asarray(det.xy, dtype=float) - predicted
            if math.hypot(diff[0], diff[1]) > config.max_distance:
                continue
            d2 = float(diff @ np.linalg.solve(cov + np.asarray(det.cov, dtype=float), diff))
            if d2 <= config.gate_mahalanobis_sq:
                candidates.append((d2, ti, di))
    candidates.sort()
    used_tracks, used_detections, pairs = set(), set(), []
    for _, ti, di in candidates:
        if ti in used_tracks or di in used_detections:
            continue
        used_tracks.add(ti)
        used_detections.add(di)
        pairs.append((ti, di))
    return pairs


@dataclass
class _Evidence:
    scores: Dict[str, float] = field(default_factory=dict)      # 시간에 따라 줄어든 근거
    conf_sum: Dict[str, float] = field(default_factory=dict)    # confidence 평균용 (줄이지 않음)
    count: Dict[str, int] = field(default_factory=dict)
    updated: float = -math.inf      # scores를 마지막으로 줄인 시각
    last_match: float = -math.inf
    current: str = 'unknown'        # 지금 붙어 있는 클래스 (히스테리시스)


@dataclass(frozen=True)
class TrackLabel:
    class_name: str        # 근거가 모자라면 "unknown"
    confidence: float      # 그 클래스로 짝지어진 검출 confidence의 평균, unknown이면 0
    camera_recent: bool    # 최근 camera_recent_s 안에 카메라 검출과 짝지어졌는지


class ClassLabeler:

    def __init__(self, config: AssociationConfig = AssociationConfig()):
        self.config = config
        self._tracks: Dict[int, _Evidence] = {}

    def _decay(self, ev: _Evidence, t: float) -> None:
        if math.isfinite(ev.updated) and t > ev.updated:
            factor = math.exp(-(t - ev.updated) / self.config.evidence_tau)
            for name in ev.scores:
                ev.scores[name] *= factor
        if t > ev.updated:
            ev.updated = t

    def update(self, track_id: int, class_name: str, confidence: float, t: float) -> None:
        """트랙에 짝지어진 검출 하나를 근거로 더한다."""
        ev = self._tracks.setdefault(track_id, _Evidence())
        self._decay(ev, t)
        ev.scores[class_name] = ev.scores.get(class_name, 0.0) + confidence
        ev.conf_sum[class_name] = ev.conf_sum.get(class_name, 0.0) + confidence
        ev.count[class_name] = ev.count.get(class_name, 0) + 1
        ev.last_match = max(ev.last_match, t)

    def label(self, track_id: int, t: float) -> TrackLabel:
        """t 시각의 트랙 클래스. 근거가 가장 큰 클래스가 min_evidence 이상이면 그 클래스."""
        ev = self._tracks.get(track_id)
        if ev is None:
            return TrackLabel(UNKNOWN_CLASS, 0.0, False)
        self._decay(ev, t)
        recent = t - ev.last_match <= self.config.camera_recent_s
        best = max(ev.scores, key=ev.scores.get)
        current_score = ev.scores.get(ev.current, 0.0)
        if ev.current != UNKNOWN_CLASS and current_score < self.config.release_evidence:
            ev.current = UNKNOWN_CLASS
        if ev.scores[best] >= self.config.min_evidence and (
                ev.current == UNKNOWN_CLASS or ev.scores[best] > ev.scores.get(ev.current, 0.0)):
            ev.current = best
        if ev.current == UNKNOWN_CLASS:
            return TrackLabel(UNKNOWN_CLASS, 0.0, recent)
        name = ev.current
        return TrackLabel(name, ev.conf_sum[name] / ev.count[name], recent)

    def prune(self, alive_track_ids: Iterable[int]) -> None:
        """사라진 트랙의 근거를 지운다 (트랙 ID는 다시 쓰지 않는다)."""
        alive = set(alive_track_ids)
        for track_id in list(self._tracks):
            if track_id not in alive:
                del self._tracks[track_id]
