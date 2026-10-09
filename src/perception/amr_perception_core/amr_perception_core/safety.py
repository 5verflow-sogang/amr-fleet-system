"""안전 단계 판정 (ROS 비의존). amr_safety 노드가 매 주기 update()를 부른다.

원 명세 7장 안전 시스템: 긴급정지 0.3 m, Safety Zone(Warning/Critical), E-Stop, 센서 고장 → 정지·저속.
단계와 speed_limit 값은 amr_msgs/SafetyStatus 규칙(#70)을 따른다: NORMAL inf, WARNING 0.6, 정지 0.0.

단계와 조건 (하나라도 맞으면):
- ESTOP: E-Stop 요청이 걸려 있음 (해제 요청 전까지 유지)
- CRITICAL: 지금 명령의 진행 경로에 LiDAR 점이 걸림 (collision.path_hazard)
  / 움직이는 트랙의 TTC ≤ ttc_critical / 스캔 끊김
- WARNING: 진행 경로 밖이지만 LiDAR 점이 차체에서 critical_distance 이내
  / 움직이는 트랙의 TTC ≤ ttc_warning / 움직이는 트랙이 dynamic_warning_distance 이내 / 추적·odom 끊김
- NORMAL: 그 밖

- 긴급정지는 추적이 아니라 LiDAR 점으로 본다. 트랙은 확정까지 몇 프레임 걸리고, 동적 판정이 틀릴 수 있다.
- 긴급정지는 진행 방향만 본다 (Nav2 collision_monitor approach, 속도별 안전 스캐너 영역과 같은 방식).
  사방 0.3 m로 막으면 벽 앞에 선 뒤 후진도 막혀 갇히고(Gazebo 실검증), 0.6 m 좁은 통로(옆 여유 0.1 m)에
  들어갈 수 없다. 가까운데 진행 방향이 아니면 WARNING(저속)으로 둔다.
- TTC와 "가까우면 저속"은 움직이는 트랙에만 건다. 정적 장애물에 TTC를 걸면 벽·랙을 향해 가기만 해도
  1 s 앞(0.5 m/s면 0.5 m)에서 서서 작업 위치에 다가갈 수 없다. 정적 장애물은 진행 경로 판정이 지킨다.
- 위험해질 때는 바로 올라가고, 내려갈 때는 조건이 release_hold_s 동안 계속 풀려 있어야 내려온다 (깜빡임 방지).
  예외: 진행 경로 정지(CRITICAL·장애물 근접)는 바로 내린다. 경로 판정은 명령에 따라 바뀌므로 멀어지는 명령
  (벽 앞에서 후진)이 오면 위험이 없고, 다시 다가가는 명령이 오면 그 즉시 다시 정지하기 때문이다.
  기다리게 하면 정지 뒤 후진이 0.5 s 늦게 풀렸다 (Gazebo 실검증).
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence

_TIME_EPS = 1e-9   # 시각 뺄셈의 부동소수점 오차 (0.7 − 0.2 = 0.4999…)

LEVEL_NORMAL, LEVEL_WARNING, LEVEL_CRITICAL, LEVEL_ESTOP = 0, 1, 2, 3
REASON_NONE, REASON_OBSTACLE_PROXIMITY, REASON_COLLISION_RISK = 0, 1, 2
REASON_ESTOP_REQUEST, REASON_SENSOR_FAULT = 3, 4
LEVEL_NAMES = {LEVEL_NORMAL: 'NORMAL', LEVEL_WARNING: 'WARNING', LEVEL_CRITICAL: 'CRITICAL',
               LEVEL_ESTOP: 'ESTOP'}
# 같은 단계에서 원인이 여럿이면 앞쪽을 보고한다 (센서 고장은 detail에 함께 적는다)
_REASON_ORDER = [REASON_ESTOP_REQUEST, REASON_OBSTACLE_PROXIMITY, REASON_COLLISION_RISK,
                 REASON_SENSOR_FAULT]


@dataclass(frozen=True)
class SafetyConfig:
    # 원 명세 긴급정지 거리 [m]. 진행 방향 차체 앞 띠 (collision.path_hazard front_margin)이자,
    # 진행 방향이 아닌 점의 WARNING 거리
    critical_distance: float = 0.3
    # TTC 임계 [s]. CRITICAL: 0.6 m/s에서 반응 0.2 s + 1.0 m/s² 정지 0.6 s ≈ 0.8 s에 여유.
    # WARNING: 최고속 2.0 m/s에서 0.6까지 줄이는 반응 0.2 s + 1.4 s ≈ 1.6 s에 여유
    ttc_critical: float = 1.0
    ttc_warning: float = 3.0
    # 움직이는 트랙(사람·지게차)이 차체에서 이 거리 [m] 안이면 미리 저속
    dynamic_warning_distance: float = 1.5
    # WARNING의 speed_limit [m/s] (#70: 반응 0.2 s + 1.0 m/s²로 0.3 m 안에 서는 최대 속도)
    warning_speed_limit: float = 0.6
    # 단계를 내릴 때 조건이 풀린 채 유지돼야 하는 시간 [s]
    release_hold_s: float = 0.5
    # 센서 끊김 판정 [s]. 스캔(10 Hz)이 끊기면 정지, 추적·odom이 끊기면 저속
    scan_timeout_s: float = 0.5
    tracks_timeout_s: float = 1.0
    odom_timeout_s: float = 1.0

    def __post_init__(self):
        for name in ('critical_distance', 'ttc_critical', 'ttc_warning',
                     'dynamic_warning_distance', 'warning_speed_limit',
                     'scan_timeout_s', 'tracks_timeout_s', 'odom_timeout_s'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name}는 양수여야 한다: {value}')
        if not (math.isfinite(self.release_hold_s) and self.release_hold_s >= 0.0):
            raise ValueError(f'release_hold_s는 0 이상이어야 한다: {self.release_hold_s}')
        if self.ttc_critical > self.ttc_warning:
            raise ValueError('ttc_critical은 ttc_warning 이하여야 한다')


@dataclass(frozen=True)
class ObstacleInfo:
    """추적 장애물 하나의 안전 판정 입력."""
    track_id: int
    clearance: float      # 차체에서 장애물 표면까지 [m] (장애물 반경을 뺀 값)
    ttc: float            # [s], 충돌 경로가 아니면 inf
    is_dynamic: bool


@dataclass(frozen=True)
class SafetyState:
    level: int
    reason: int
    speed_limit: float
    detail: str
    related_track_id: int          # 장애물과 무관하면 -1
    min_obstacle_distance: float   # 차체에서 가장 가까운 장애물까지 [m], 없으면 inf

    @property
    def level_name(self) -> str:
        return LEVEL_NAMES[self.level]


@dataclass(frozen=True)
class _Cause:
    level: int
    reason: int
    detail: str
    track_id: int = -1


def _stale(age: Optional[float], timeout: float) -> bool:
    """한 번도 못 받았거나(None) timeout보다 오래됐는지."""
    return age is None or age > timeout


class SafetyMonitor:

    def __init__(self, config: SafetyConfig = SafetyConfig()):
        self.config = config
        self._estop = False
        self._state: Optional[SafetyState] = None
        self._below_since: Optional[float] = None

    @property
    def estop(self) -> bool:
        return self._estop

    def set_estop(self, active: bool) -> None:
        """E-Stop 걸기·해제. 해제 요청이 올 때까지 ESTOP을 유지한다."""
        self._estop = bool(active)

    def speed_limit_for(self, level: int) -> float:
        if level == LEVEL_NORMAL:
            return math.inf
        if level == LEVEL_WARNING:
            return self.config.warning_speed_limit
        return 0.0

    def update(self, now: float, scan_min_clearance: Optional[float],
               obstacles: Sequence[ObstacleInfo], scan_age: Optional[float],
               tracks_age: Optional[float], odom_age: Optional[float],
               path_clearance: Optional[float] = None) -> SafetyState:
        """now [s]의 입력으로 안전 단계를 정한다.

        scan_min_clearance: 차체에서 가장 가까운 LiDAR 점까지 [m] (점이 없으면 None 또는 inf).
        path_clearance: 지금 명령의 진행 경로에 걸린 점까지 거리 [m] (collision.path_hazard), 없으면 None.
        *_age: 마지막 메시지 뒤 지난 시간 [s], 한 번도 못 받았으면 None.
        """
        c = self.config
        causes: List[_Cause] = []
        faults = []
        if self._estop:
            causes.append(_Cause(LEVEL_ESTOP, REASON_ESTOP_REQUEST, 'E-Stop 요청'))
        if _stale(scan_age, c.scan_timeout_s):
            faults.append('scan 끊김')
            causes.append(_Cause(LEVEL_CRITICAL, REASON_SENSOR_FAULT, 'scan 끊김'))
        elif path_clearance is not None:
            causes.append(_Cause(LEVEL_CRITICAL, REASON_OBSTACLE_PROXIMITY,
                                 f'진행 경로에 장애물 {path_clearance:.2f} m'))
        elif scan_min_clearance is not None and scan_min_clearance <= c.critical_distance:
            causes.append(_Cause(LEVEL_WARNING, REASON_OBSTACLE_PROXIMITY,
                                 f'근접 {scan_min_clearance:.2f} m (진행 경로 밖)'))
        for name, age, timeout in (('추적', tracks_age, c.tracks_timeout_s),
                                   ('odom', odom_age, c.odom_timeout_s)):
            if _stale(age, timeout):
                faults.append(f'{name} 끊김')
                causes.append(_Cause(LEVEL_WARNING, REASON_SENSOR_FAULT, f'{name} 끊김'))
        for obs in obstacles:
            if not obs.is_dynamic:
                continue
            if obs.ttc <= c.ttc_critical:
                causes.append(_Cause(LEVEL_CRITICAL, REASON_COLLISION_RISK,
                                     f'TTC {obs.ttc:.1f} s (track {obs.track_id})', obs.track_id))
            elif obs.ttc <= c.ttc_warning:
                causes.append(_Cause(LEVEL_WARNING, REASON_COLLISION_RISK,
                                     f'TTC {obs.ttc:.1f} s (track {obs.track_id})', obs.track_id))
            if obs.clearance <= c.dynamic_warning_distance:
                causes.append(_Cause(LEVEL_WARNING, REASON_OBSTACLE_PROXIMITY,
                                     f'움직이는 장애물 {obs.clearance:.2f} m (track {obs.track_id})',
                                     obs.track_id))

        distances = [d for d in [scan_min_clearance] + [o.clearance for o in obstacles]
                     if d is not None and math.isfinite(d)]
        min_distance = max(0.0, min(distances)) if distances else math.inf
        raw = self._pick(causes, faults, min_distance)
        return self._apply_hysteresis(now, raw)

    def _pick(self, causes: List[_Cause], faults: List[str], min_distance: float) -> SafetyState:
        if not causes:
            return SafetyState(LEVEL_NORMAL, REASON_NONE, math.inf, '', -1, min_distance)
        top = max(cause.level for cause in causes)
        best = min((cause for cause in causes if cause.level == top),
                   key=lambda cause: _REASON_ORDER.index(cause.reason))
        detail = best.detail
        if faults and best.reason != REASON_SENSOR_FAULT:
            detail += ' / 센서: ' + ', '.join(faults)   # SafetyStatus.msg: 센서 고장은 detail에 함께
        return SafetyState(top, best.reason, self.speed_limit_for(top), detail, best.track_id,
                           min_distance)

    def _apply_hysteresis(self, now: float, raw: SafetyState) -> SafetyState:
        current = self._state
        path_stop = current is not None and (current.level, current.reason) == (
            LEVEL_CRITICAL, REASON_OBSTACLE_PROXIMITY)
        if (current is None or raw.level >= current.level or raw.level == LEVEL_ESTOP
                or path_stop):
            self._state, self._below_since = raw, None
            return raw
        # 내려가는 중: 조건이 release_hold_s 동안 계속 풀려 있어야 내린다
        if self._below_since is None:
            self._below_since = now
        if now - self._below_since >= self.config.release_hold_s - _TIME_EPS:
            self._state, self._below_since = raw, None
            return raw
        held = SafetyState(current.level, current.reason, current.speed_limit, current.detail,
                           current.related_track_id, raw.min_obstacle_distance)
        self._state = held
        return held
