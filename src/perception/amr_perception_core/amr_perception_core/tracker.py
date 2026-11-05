"""여러 장애물을 KF로 동시에 추적한다 (ROS 비의존).

매 프레임: 모든 트랙 예측 → 측정과 짝짓기(연관) → 짝지은 트랙 갱신 → 남은 측정으로 새 트랙
→ 동적 여부 갱신 → 오래 관측되지 않은 트랙 삭제.

연관은 마할라노비스 거리가 가까운 쌍부터 고르는 탐욕(greedy) 방식이고, 확정 트랙이 먼저 측정을 고른다.
게이트 밖으로 한 번 벗어난 측정이 만든 미확정 트랙이 다음 프레임에 기존 트랙의 측정을 가로채
ID가 바뀌는 것을 막기 위해서다. 붐비는 상황에서 최적 할당(Hungarian)이 필요해지면 _associate만 바꾼다.
"""

from dataclasses import dataclass, field
import math
from typing import List, Optional, Sequence, Set, Tuple

import numpy as np

from amr_perception_core.kalman import ConstantVelocityKF, KalmanConfig


@dataclass(frozen=True)
class TrackerConfig:
    kalman: KalmanConfig = field(default_factory=KalmanConfig)
    # 연관 게이트: 마할라노비스 거리² 상한. 2자유도 χ² 99.9% 값.
    # 99%(9.21)는 정상 측정도 프레임당 1%가 게이트 밖으로 나가 ID가 바뀌는 일이 잦았다
    gate_mahalanobis_sq: float = 13.82
    # 누적 관측이 이 횟수에 도달하면 확정 트랙으로 내보낸다 (한두 프레임짜리 잡음 군집 배제)
    confirm_hits: int = 3
    # 마지막 관측 후 이 시간 [s]이 지나면 삭제한다. LiDAR 10 Hz 기준 5프레임
    max_coast_time: float = 0.5
    # 추정 속력이 이 값 [m/s]을 넘는 프레임이 dynamic_confirm_frames번 연속되면 동적으로 판정하고
    # 트랙이 사라질 때까지 유지한다 (멈춰 선 사람도 동적 장애물). 정지 물체도 속도 추정 잡음으로
    # 순간 속력이 0.2 m/s를 넘곤 해서(σa 1.0일 때 프레임의 약 8%) 연속 조건을 둔다
    dynamic_speed_threshold: float = 0.25
    dynamic_confirm_frames: int = 5
    # 빈 공간 침입 증거(free_space.py)가 붙은 측정을 이 프레임 수 이상 받아야 동적으로 판정한다.
    # step()에 free_evidence를 주지 않으면(위치 측정 입력 등) 증거 조건 없이 속력만 본다
    dynamic_evidence_frames: int = 3
    # 측정 반경을 트랙 반경에 반영하는 비율 (지수 평활). 프레임마다 보이는 면이 달라 군집 반경이 흔들린다
    radius_smoothing: float = 0.3
    # 보조 연관 (급한 기동): 1차 연관에서 짝을 못 찾은 동적 트랙과 측정을, 트랙을 마지막으로 본 자리에서 물리적으로
    # 갈 수 있는 거리(maneuver_max_speed × 지난 시간 + maneuver_margin) 안이면 짝짓는다. 등속 예측은 반전 순간 앞으로
    # 가 버려 게이트(약 0.25 m)를 벗어나지만 실제 물체는 마지막으로 본 자리 근처에 있다 (1.0 m/s 반전이면 2프레임,
    # 1.5 m/s면 1프레임에 놓쳐 새 트랙이 생기고 옛 트랙은 헛트랙으로 미끄러졌다. 근거·측정은 perception.md 1-3-3).
    # 짝지으면 마지막으로 본 자리와 지금 자리, 두 관측으로 위치·속도를 다시 잡는다 (ID·확정·동적 판정은 그대로).
    # 동적으로 판정된 트랙만 한다: 정적 물체는 가림으로 군집 중심이 튀어도 새 트랙으로 두어야 속도가 붙지 않는다
    # (붙이면 튄 거리/프레임 간격만큼 속도가 생겨 가짜 동적이 될 수 있다).
    # maneuver_max_age [s]보다 오래 못 본 트랙(가려짐)은 하지 않는다: 그때는 예측 위치 기준 게이트가 맞다.
    # maneuver_max_speed는 명세 장애물 최대 속력(1.5 m/s)으로 둔다. 넉넉히 2.0으로 두면 지게차처럼 큰 물체의 군집 중심이
    # 보이는 면에 따라 0.3~0.7 m 튀는 것(2~3.5 m/s 꼴)까지 기동으로 받아 속도가 엉터리로 다시 잡혔다(개발용 B).
    # maneuver_margin은 두 관측 차이의 잡음 몫(measurement_noise_std 0.05면 약 2σ). 0.1이면 1.5 m/s 반전을 놓친다.
    # maneuver_max_radius: 지금까지 본 가장 큰 군집 반경이 이보다 큰 트랙(지게차)은 하지 않는다. 큰 물체는 가려지거나
    # 보이는 면이 바뀔 때 군집 중심이 0.4~0.6 m 튀어, 명세 최대 속력 안에서도 기동으로 받아 12 m 안 지게차 속도
    # RMSE가 0.236 → 0.281 m/s로 나빠졌다(개발용 B). 사람 트랙은 가장 큰 반경이 0.24 m 이하였다(개발용 녹화 87개).
    # 다른 로봇(0.6 × 0.4 m)도 대개 넘어 빠진다(개발용 B robot_1, 짝은 모두 12 m 밖). 반경 없이 측정하면 제한하지 않는다
    # maneuver_max_speed 0(기본)이면 끈다. 노드는 tracking.yaml 값을 쓴다
    maneuver_max_speed: float = 0.0
    maneuver_margin: float = 0.15
    maneuver_max_age: float = 0.5
    maneuver_max_radius: float = 0.4

    def __post_init__(self):
        if not (math.isfinite(self.gate_mahalanobis_sq) and self.gate_mahalanobis_sq > 0.0):
            raise ValueError(f'gate_mahalanobis_sq는 양수여야 한다: {self.gate_mahalanobis_sq}')
        for name in ('confirm_hits', 'dynamic_confirm_frames', 'dynamic_evidence_frames'):
            value = getattr(self, name)
            if int(value) != value or value < 1:
                raise ValueError(f'{name}는 1 이상의 정수여야 한다: {value}')
        if not 0.0 < self.radius_smoothing <= 1.0:
            raise ValueError(f'radius_smoothing은 (0, 1] 범위여야 한다: {self.radius_smoothing}')
        for name in ('maneuver_max_speed', 'maneuver_margin'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f'{name}는 0 이상이어야 한다: {value}')
        for name in ('max_coast_time', 'dynamic_speed_threshold', 'maneuver_max_age', 'maneuver_max_radius'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name}는 양수여야 한다: {value}')


@dataclass
class Track:
    track_id: int
    kf: ConstantVelocityKF
    hits: int = 1               # 누적 관측 횟수
    last_update: float = 0.0    # 마지막 관측 시각 [s]
    confirmed: bool = False
    is_dynamic: bool = False
    fast_frames: int = 0        # 속력이 임계값을 넘은 연속 프레임 수
    evidence_frames: int = 0    # 빈 공간 침입 증거가 붙은 측정을 받은 프레임 수 (누적)
    radius: Optional[float] = None  # 장애물 반경 [m] (측정 반경의 지수 평활). 반경 없이 측정하면 None
    last_z: Optional[np.ndarray] = None   # 마지막 관측 위치 (보조 연관 기준점)
    max_radius: Optional[float] = None    # 지금까지 본 가장 큰 측정 반경 [m] (보조 연관에서 큰 물체를 뺀다)


def _as_measurement(value: Sequence[float]) -> np.ndarray:
    z = np.asarray(value, dtype=float)
    if z.shape != (2,) or not np.all(np.isfinite(z)):
        raise ValueError(f'측정은 유한한 (x, y)여야 한다: {value}')
    return z


class MultiTargetTracker:

    def __init__(self, config: TrackerConfig = TrackerConfig()):
        self.config = config
        self._tracks: List[Track] = []
        self._next_id = 0
        self._last_time: Optional[float] = None

    @property
    def tracks(self) -> List[Track]:
        """확정 전 트랙을 포함한 전체 트랙."""
        return list(self._tracks)

    @property
    def confirmed_tracks(self) -> List[Track]:
        return [track for track in self._tracks if track.confirmed]

    @property
    def last_time(self) -> Optional[float]:
        return self._last_time

    def reset(self) -> None:
        """모든 트랙을 지운다. ID는 이어서 매긴다 (사라진 트랙의 ID는 다시 쓰지 않는다)."""
        self._tracks = []
        self._last_time = None

    def step(self, timestamp: float, measurements: Sequence[Sequence[float]],
             radii: Optional[Sequence[float]] = None,
             free_evidence: Optional[Sequence[bool]] = None) -> List[Track]:
        """timestamp [s]의 측정 위치 목록을 반영하고 확정 트랙을 돌려준다. 측정이 없어도 호출한다.

        radii: 측정마다 장애물 반경 [m] (LiDAR 군집 반경). 주면 트랙 반경을 지수 평활로 갱신한다.
        free_evidence: 측정마다 빈 공간 침입 증거 (free_space.FreeSpaceGrid). 주면 동적 판정에
        증거 조건(dynamic_evidence_frames)을 더한다. 안 주면 속력만 본다.

        측정은 고정 좌표계(odom 또는 map) 기준이어야 한다. 로봇 기준 좌표면 로봇 자신의 움직임이
        장애물 속도로 잡힌다. 시각이 거꾸로 가면 ValueError를 내고, 호출자가 reset()으로 처리한다.
        """
        t = float(timestamp)
        if not math.isfinite(t):
            raise ValueError(f'시각은 유한해야 한다: {timestamp}')
        if self._last_time is not None and t < self._last_time:
            raise ValueError(f'시각이 거꾸로 갔다: {t} < {self._last_time}')
        zs = [_as_measurement(m) for m in measurements]
        rs = self._as_radii(radii, len(zs))
        use_evidence = free_evidence is not None
        evidence = [bool(e) for e in free_evidence] if use_evidence else [False] * len(zs)
        if len(evidence) != len(zs):
            raise ValueError(f'free_evidence 개수({len(evidence)})가 측정 개수({len(zs)})와 다르다')
        self._last_time = t

        for track in self._tracks:
            track.kf.predict(t)

        matches, maneuver, unmatched = self._associate(zs, t)
        for ti, mi in matches + maneuver:
            track = self._tracks[ti]
            if (ti, mi) in maneuver:
                track.kf.restart(zs[mi], track.last_z, t - track.last_update)
            else:
                track.kf.update(zs[mi])
            track.last_z = zs[mi]
            self._update_radius(track, rs[mi])
            track.evidence_frames += evidence[mi]
            track.hits += 1
            track.last_update = t
            if track.hits >= self.config.confirm_hits:
                track.confirmed = True
        for mi in unmatched:
            track = self._new_track(zs[mi], t, rs[mi])
            track.evidence_frames = int(evidence[mi])
            self._tracks.append(track)

        self._tracks = [
            track for track in self._tracks
            if t - track.last_update <= self.config.max_coast_time
        ]
        for track in self._tracks:
            self._update_dynamic(track, use_evidence)
        return self.confirmed_tracks

    def _update_dynamic(self, track: Track, use_evidence: bool) -> None:
        if track.kf.speed > self.config.dynamic_speed_threshold:
            track.fast_frames += 1
        else:
            track.fast_frames = 0
        evidence_ok = (not use_evidence
                       or track.evidence_frames >= self.config.dynamic_evidence_frames)
        if track.fast_frames >= self.config.dynamic_confirm_frames and evidence_ok:
            track.is_dynamic = True

    @staticmethod
    def _as_radii(radii: Optional[Sequence[float]], count: int) -> List[Optional[float]]:
        if radii is None:
            return [None] * count
        rs = [float(r) for r in radii]
        if len(rs) != count:
            raise ValueError(f'radii 개수({len(rs)})가 측정 개수({count})와 다르다')
        if not all(math.isfinite(r) and r > 0.0 for r in rs):
            raise ValueError(f'반경은 양수여야 한다: {rs}')
        return rs

    def _update_radius(self, track: Track, r: Optional[float]) -> None:
        if r is None:
            return
        track.max_radius = r if track.max_radius is None else max(track.max_radius, r)
        if track.radius is None:
            track.radius = r
        else:
            track.radius += self.config.radius_smoothing * (r - track.radius)

    def _new_track(self, z: np.ndarray, t: float, radius: Optional[float] = None) -> Track:
        track = Track(
            track_id=self._next_id,
            kf=ConstantVelocityKF(z, t, self.config.kalman),
            last_update=t,
            last_z=z,
            confirmed=self.config.confirm_hits <= 1,
            radius=radius,
            max_radius=radius,
        )
        self._next_id += 1  # 사라진 트랙의 ID는 다시 쓰지 않는다
        return track

    def _associate_maneuver(self, zs: List[np.ndarray], matches: List[Tuple[int, int]], used_measurements: Set[int],
                            t: float) -> List[Tuple[int, int]]:
        """보조 연관: 게이트에서 짝을 못 찾은 동적 트랙 ↔ 남은 측정을 마지막 관측 위치 기준으로 (TrackerConfig 주석).

        이미 짝지어진 측정은 건드리지 않으므로 다른 물체의 측정을 빼앗지 않는다. 가까운 쌍부터 1:1로 짝짓는다.
        (트랙 index, 측정 index) 목록을 돌려준다.
        """
        cfg = self.config
        if cfg.maneuver_max_speed <= 0.0:
            return []
        matched_tracks = {ti for ti, _ in matches}
        free = [mi for mi in range(len(zs)) if mi not in used_measurements]
        candidates = []
        for ti, track in enumerate(self._tracks):
            age = t - track.last_update
            if (ti in matched_tracks or not (track.confirmed and track.is_dynamic) or track.last_z is None
                    or age <= 0.0 or age > cfg.maneuver_max_age
                    or (track.max_radius is not None and track.max_radius > cfg.maneuver_max_radius)):
                continue
            reach = cfg.maneuver_max_speed * age + cfg.maneuver_margin
            for mi in free:
                d = float(np.hypot(*(zs[mi] - track.last_z)))
                if d <= reach:
                    candidates.append((d, ti, mi))
        candidates.sort()
        used_tracks, taken, extra = set(), set(), []
        for _, ti, mi in candidates:
            if ti in used_tracks or mi in taken:
                continue
            used_tracks.add(ti)
            taken.add(mi)
            extra.append((ti, mi))
        return extra

    def _associate(self, zs: List[np.ndarray], t: float
                   ) -> Tuple[List[Tuple[int, int]], List[Tuple[int, int]], List[int]]:
        """확정 트랙 → 보조 연관(동적 트랙) → 미확정 트랙 순서로, 게이트 안에서 가까운 쌍부터 짝짓는다.

        보조 연관을 미확정 트랙보다 먼저 한다: 잡음으로 생겨 오래 예측만 한 미확정 트랙은 게이트가 넓어서,
        돌아선 사람의 측정을 확정 트랙보다 먼저 가져가 새 ID를 만들 수 있다.
        게이트 짝 목록, 보조 연관 짝 목록 (각각 (트랙 index, 측정 index)), 짝이 없는 측정 index를 돌려준다.
        """
        used_measurements, matches, maneuver = set(), [], []
        z_all = np.array(zs).reshape(-1, 2)
        for confirmed in (True, False):
            candidates = []
            for ti, track in enumerate(self._tracks):
                if track.confirmed != confirmed or not zs:
                    continue
                d2_all = track.kf.mahalanobis_sq_many(z_all)   # 트랙마다 S⁻¹ 한 번
                for mi in np.flatnonzero(d2_all <= self.config.gate_mahalanobis_sq):
                    mi = int(mi)
                    if mi not in used_measurements:
                        candidates.append((float(d2_all[mi]), ti, mi))
            candidates.sort()
            used_tracks = set()
            for _, ti, mi in candidates:
                if ti in used_tracks or mi in used_measurements:
                    continue
                used_tracks.add(ti)
                used_measurements.add(mi)
                matches.append((ti, mi))
            if confirmed:
                maneuver = self._associate_maneuver(zs, matches, used_measurements, t)
                used_measurements.update(mi for _, mi in maneuver)
        unmatched = [mi for mi in range(len(zs)) if mi not in used_measurements]
        return matches, maneuver, unmatched
