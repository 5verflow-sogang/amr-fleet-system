"""Kidnapped Robot 감지·복구 상태 머신. ROS에 의존하지 않는다 (노드는 kidnap_recovery_node.py).

    TRACKING ─(일치율 < lost_ratio가 lost_checks번 연속)→ 재초기화 → SETTLING
      재초기화는 첫 시도만 국소(마지막으로 확실했던 자세 주변에 파티클), 다음부터 전역(지도 전체)이다.
      반복 구조 창고에서 전역 재초기화는 비슷한 다른 통로로 수렴하기 쉽다(10/6 실험)
    SETTLING: 움직이지 않고 AMCL 갱신을 요청한다 (request_nomotion_update, 시도마다 nomotion_max번까지)
      ─(수렴)→ TRACKING
      ─(settle_s 지나도 미수렴)→ ROTATING (allow_rotate일 때) 또는 다음 시도
    ROTATING: 제자리 회전으로 스캔을 모은다 ─(수렴)→ TRACKING
      ─(rotate_s 지남)→ 멈추고 confirm_s 동안 한 번 더 판정(SETTLING) → 그래도 미수렴이면 다음 시도
      회전 중에는 스캔과 자세의 시각 차이로 일치율이 낮게 나와, 맞게 수렴해도 판정을 놓친다 (10/12 진단)
    시도가 max_attempts를 넘으면 FAILED (사람·상위 BT가 처리). FAILED에서도 계속 지켜보다가
    수렴 조건을 다시 만족하면(사람이 위치를 다시 잡아 준 경우 등) TRACKING으로 돌아온다
    마커판 자세(aruco_pose)가 들어오면 SETTLING·ROTATING 중 어디서든 그 자세로 다시 뿌리고(MARKER_REINIT)
      멈춰서 marker_settle_s 동안 판정한다 → 미수렴이면 다음 시도. 판 ID가 다 달라 통로·앞뒤를 헷갈리지 않아
      스캔만으로 못 찾는 다른 통로 이동을 푼다 (4-7절).
      마커를 한 번이라도 받았으면 다음 시도도 전역 대신 마지막 마커 자세 주변으로 뿌리고(전역은 맞는 통로 후보를
      지운다), 제자리 갱신을 marker_nomotion_max번까지 부른다 (좁게 뿌려 한 점에 몰려도 맞는 곳이다.
      far 10회(10/25) seed 10: 4번 갱신에 3.8 → 1.1 m로 줄다가 갱신이 끊겨 멈췄다).
      마커가 틀려도(뒤집힌 PnP 해, 다른 판) 빠져나오게, 마커로 뿌린 뒤의 마지막 시도는 직전 일치율이 ok_ratio 아래면
      전역이고 그 시도에서는 마커를 받지 않는다 (#240 민지 리뷰). 일치율이 ok_ratio 이상이면 자세는 맞고 σ만 덜 줄어든
      것이라 마커 주변으로 한 번 더 한다 (seed 19: 일치율 1.00·σ 0.33 m인 맞는 자세를 전역이 지웠다).
      멈춰서 수렴 조건을 통과하는 중(_ok > 0)에도 마커가 끊지 않는다 (같은 통로에서 국소로 맞춰 가던 시도를 틀린
      마커가 버리지 않게)
    잃은 뒤 max_recovery_s가 지나면 시도 중이어도 FAILED. 마커가 시도 끝 무렵에 오면 그 시도가 marker_settle_s만큼
      늘어 상한이 약 84초(3 × 28초)에서 113.5초까지 늘어나서다. BT 복구 대기 상한 120초(#166) 안에 끝나게 둔다

수렴: 멈춘 상태(SETTLING)에서 일치율 ≥ ok_ratio이고 AMCL 위치 표준편차 ≤ ok_std_m가 ok_checks번 연속.
회전 중에는 판정하지 않는다 (스캔·자세 시각 차이로 일치율이 흔들린다).
기준값 근거 (localization.md 4-6절, 10/14 튜닝 seed 1001~1010): 멈춘 상태에서 GT로 맞게 복구한 8회의 일치율은
0.96~1.00 (1회 0.79), 옆 통로(7.5 m)로 틀리게 받아들인 1회는 0.87 → 0.9. 반복 구조라 틀린 곳도 87%까지 겹친다.
맞게 수렴해도 AMCL σ가 0.27 m인 경우가 있어 σ 기준은 0.3 m.
제자리 갱신을 계속 부르면 리샘플링만 반복돼 파티클이 한 점(틀린 곳 포함)에 몰리므로 시도마다 nomotion_max번만 부른다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

TRACKING, SETTLING, ROTATING, FAILED = 'tracking', 'settling', 'rotating', 'failed'

# 노드가 수행할 동작
REINIT = 'reinit'          # AMCL reinitialize_global_localization (지도 전체)
LOCAL_REINIT = 'local_reinit'  # 마지막으로 확실했던 자세 주변에 initialpose (넓은 공분산)
MARKER_REINIT = 'marker_reinit'  # 마커판으로 잰 자세에 initialpose (마커 공분산)
NOMOTION = 'nomotion'      # AMCL request_nomotion_update
ROTATE = 'rotate'          # 제자리 회전 명령
STOP = 'stop'              # 회전 멈춤


@dataclass
class RecoveryConfig:
    lost_ratio: float = 0.5
    lost_checks: int = 3
    ok_ratio: float = 0.9
    ok_std_m: float = 0.3
    ok_checks: int = 3
    nomotion_max: int = 4
    confirm_s: float = 3.0
    settle_s: float = 10.0
    allow_rotate: bool = True
    rotate_s: float = 15.0
    max_attempts: int = 3
    local_first: bool = True
    # 마커 재초기화 뒤 판정 시간. 이 동안은 다음 마커 자세를 받지 않는다. 한 번 잃은 동안 marker_max번까지
    marker_settle_s: float = 10.0
    marker_max: int = 3
    marker_nomotion_max: int = 12
    # 잃은 뒤 이 시간이 지나면 FAILED (0이면 끔). 감지 1.5초를 더해도 BT 대기 상한 120초(#166)에 18초 여유
    max_recovery_s: float = 100.0


@dataclass
class Event:
    t: float
    kind: str        # lost / reinit / rotate / recovered / failed
    detail: str = ''


@dataclass
class KidnapRecovery:
    config: RecoveryConfig = field(default_factory=RecoveryConfig)
    state: str = TRACKING
    attempts: int = 0
    events: list = field(default_factory=list)
    _low: int = 0
    _ok: int = 0
    _since: float = 0.0
    _nomotion: int = 0
    _confirming: bool = False
    _marker: bool = False
    _markers: int = 0
    _ratio: float = math.nan       # 마지막 일치율 (마지막 시도를 전역으로 할지 고를 때)
    _global_last: bool = False     # 마커 뒤 마지막 시도를 전역으로 했다 (마커를 받지 않는다)
    lost_at: Optional[float] = None

    def _converged(self, ratio: float, std_m: float) -> bool:
        good = not math.isnan(ratio) and ratio >= self.config.ok_ratio and std_m <= self.config.ok_std_m
        self._ok = self._ok + 1 if good else 0
        return self._ok >= self.config.ok_checks

    def _start_attempt(self, now: float) -> list[str]:
        self.attempts += 1
        if self.attempts > self.config.max_attempts:
            self.state = FAILED
            self.events.append(Event(now, 'failed', f'{self.config.max_attempts}번 시도'))
            return [STOP]
        self.state, self._since, self._ok = SETTLING, now, 0
        self._nomotion, self._confirming, self._marker = 1, False, False
        last = self.attempts >= self.config.max_attempts
        fits = not math.isnan(self._ratio) and self._ratio >= self.config.ok_ratio
        if self._markers and (not last or fits):
            self.events.append(Event(now, 'marker_retry', f'{self.attempts}번째'))
            return [STOP, MARKER_REINIT, NOMOTION]
        # 마커로 뿌렸는데 스캔이 지도와 안 맞으면 마지막 시도는 전역 (마커가 틀렸으면 마커 주변만 돌다 끝나지 않게)
        self._global_last = bool(self._markers)
        local = self.config.local_first and self.attempts == 1
        self.events.append(Event(now, 'local_reinit' if local else 'reinit', f'{self.attempts}번째'))
        return [STOP, LOCAL_REINIT if local else REINIT, NOMOTION]

    def step(self, now: float, ratio: float, std_m: float) -> list[str]:
        """일치율(ratio)과 AMCL 위치 표준편차(std_m, x·y 중 큰 값)로 한 번 판단하고 동작 목록을 돌려준다."""
        c = self.config
        self._ratio = ratio
        if self.state == TRACKING:
            self._low = self._low + 1 if (not math.isnan(ratio) and ratio < c.lost_ratio) else 0
            if self._low >= c.lost_checks:
                self._low, self.attempts, self._markers, self.lost_at = 0, 0, 0, now
                self._global_last = False
                self.events.append(Event(now, 'lost', f'일치율 {ratio:.2f}'))
                return self._start_attempt(now)
            return []
        if self.state in (SETTLING, ROTATING):
            if self.state == SETTLING and self._converged(ratio, std_m):
                self.state = TRACKING
                self.events.append(Event(now, 'recovered', f'{now - self.lost_at:.1f}s, 일치율 {ratio:.2f}'))
                return [STOP]
            if c.max_recovery_s > 0 and now - self.lost_at >= c.max_recovery_s:
                self.state = FAILED
                self.events.append(Event(now, 'failed', f'복구 시간 {c.max_recovery_s:.0f}s'))
                return [STOP]
            elapsed = now - self._since
            if self.state == SETTLING:
                window = c.marker_settle_s if self._marker else c.confirm_s if self._confirming else c.settle_s
                if elapsed < window:
                    if self._nomotion < (c.marker_nomotion_max if self._markers else c.nomotion_max):
                        self._nomotion += 1
                        return [NOMOTION]
                    return []
                if self._confirming:
                    return self._start_attempt(now)
                if c.allow_rotate:
                    self.state, self._since = ROTATING, now
                    self.events.append(Event(now, 'rotate'))
                    return [ROTATE]
                return self._start_attempt(now)
            if elapsed < c.rotate_s:
                return [ROTATE]
            if c.confirm_s > 0:
                # 멈춘 상태에서 한 번 더 판정한다 (회전 중 놓친 수렴을 다음 시도로 버리지 않게)
                self.state, self._since, self._ok, self._confirming = SETTLING, now, 0, True
                self._nomotion = 1
                self.events.append(Event(now, 'confirm'))
                return [STOP, NOMOTION]
            return self._start_attempt(now)
        if self.state == FAILED and self._converged(ratio, std_m):
            self.state, self.attempts = TRACKING, 0
            self.events.append(Event(now, 'restored', f'일치율 {ratio:.2f}'))
        return []

    def marker_hint(self, now: float) -> list[str]:
        """마커판으로 잰 로봇 자세가 들어왔다. 복구 중이면 그 자세로 다시 뿌리고 멈춰서 판정한다.

        지금 시도의 남은 단계를 마커 판정(marker_settle_s, 정지 확인과 같이 미수렴이면 다음 시도)으로 바꾼다.
        tracking이면 필요 없고, failed는 포기한 상태라 사람·BT에 맡긴다 (그래도 수렴하면 restored로 돌아온다).
        받지 않는 경우: 수렴 조건을 통과하는 중(멈춰서 _ok > 0), 마커로 뿌린 뒤 일치율이 낮아 전역으로 간 마지막 시도
        """
        c = self.config
        if self.state not in (SETTLING, ROTATING) or self._markers >= c.marker_max:
            return []
        if self.state == SETTLING and self._ok > 0:
            return []
        if self._global_last:
            return []
        if self._marker and now - self._since < c.marker_settle_s:
            return []
        self._markers += 1
        self.state, self._since, self._ok = SETTLING, now, 0
        self._nomotion, self._confirming, self._marker = 1, True, True
        self.events.append(Event(now, 'marker_reinit', f'{self._markers}번째'))
        return [STOP, MARKER_REINIT, NOMOTION]


def carry_forward(pose, odom_then, odom_now):
    """그때(odom_then) 자세 pose(x, y, yaw)에 그 뒤 odom으로 움직인 만큼(odom_then → odom_now)을 더한다.

    마커 자세는 영상 시각 기준이라, 회전 중에 본 판이면 처리하는 동안 돈 각도만큼 틀린다 (0.5 rad/s × 0.2 s = 6°)
    """
    x0, y0, a0 = odom_then
    x1, y1, a1 = odom_now
    dx, dy = x1 - x0, y1 - y0
    fx, fy = math.cos(a0) * dx + math.sin(a0) * dy, -math.sin(a0) * dx + math.cos(a0) * dy   # 그때 로봇 기준
    x, y, a = pose
    yaw = a + a1 - a0
    return (x + math.cos(a) * fx - math.sin(a) * fy, y + math.sin(a) * fx + math.cos(a) * fy,
            math.atan2(math.sin(yaw), math.cos(yaw)))
