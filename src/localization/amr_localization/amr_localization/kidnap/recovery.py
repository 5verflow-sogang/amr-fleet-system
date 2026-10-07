"""Kidnapped Robot 감지·복구 상태 머신. ROS에 의존하지 않는다 (노드는 kidnap_recovery_node.py).

    TRACKING ─(일치율 < lost_ratio가 lost_checks번 연속)→ 재초기화 → SETTLING
      재초기화는 첫 시도만 국소(마지막으로 확실했던 자세 주변에 파티클), 다음부터 전역(지도 전체)이다.
      반복 구조 창고에서 전역 재초기화는 비슷한 다른 통로로 수렴하기 쉽다(10/6 실험)
    SETTLING: 움직이지 않고 AMCL 갱신을 요청한다 (request_nomotion_update)
      ─(수렴)→ TRACKING
      ─(settle_s 지나도 미수렴)→ ROTATING (allow_rotate일 때) 또는 다음 시도
    ROTATING: 제자리 회전으로 스캔을 모은다 ─(수렴)→ TRACKING / ─(rotate_s 지남)→ 다음 시도
    시도가 max_attempts를 넘으면 FAILED (사람·상위 BT가 처리). FAILED에서도 계속 지켜보다가
    수렴 조건을 다시 만족하면(사람이 위치를 다시 잡아 준 경우 등) TRACKING으로 돌아온다

수렴: 일치율 ≥ ok_ratio이고 AMCL 위치 표준편차 ≤ ok_std_m가 ok_checks번 연속.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

TRACKING, SETTLING, ROTATING, FAILED = 'tracking', 'settling', 'rotating', 'failed'

# 노드가 수행할 동작
REINIT = 'reinit'          # AMCL reinitialize_global_localization (지도 전체)
LOCAL_REINIT = 'local_reinit'  # 마지막으로 확실했던 자세 주변에 initialpose (넓은 공분산)
NOMOTION = 'nomotion'      # AMCL request_nomotion_update
ROTATE = 'rotate'          # 제자리 회전 명령
STOP = 'stop'              # 회전 멈춤


@dataclass
class RecoveryConfig:
    lost_ratio: float = 0.5
    lost_checks: int = 3
    ok_ratio: float = 0.8
    ok_std_m: float = 0.15
    ok_checks: int = 3
    settle_s: float = 10.0
    allow_rotate: bool = True
    rotate_s: float = 15.0
    max_attempts: int = 3
    local_first: bool = True


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
        local = self.config.local_first and self.attempts == 1
        self.events.append(Event(now, 'local_reinit' if local else 'reinit', f'{self.attempts}번째'))
        return [STOP, LOCAL_REINIT if local else REINIT, NOMOTION]

    def step(self, now: float, ratio: float, std_m: float) -> list[str]:
        """일치율(ratio)과 AMCL 위치 표준편차(std_m, x·y 중 큰 값)로 한 번 판단하고 동작 목록을 돌려준다."""
        c = self.config
        if self.state == TRACKING:
            self._low = self._low + 1 if (not math.isnan(ratio) and ratio < c.lost_ratio) else 0
            if self._low >= c.lost_checks:
                self._low, self.attempts, self.lost_at = 0, 0, now
                self.events.append(Event(now, 'lost', f'일치율 {ratio:.2f}'))
                return self._start_attempt(now)
            return []
        if self.state in (SETTLING, ROTATING):
            if self._converged(ratio, std_m):
                self.state = TRACKING
                self.events.append(Event(now, 'recovered', f'{now - self.lost_at:.1f}s, 일치율 {ratio:.2f}'))
                return [STOP]
            elapsed = now - self._since
            if self.state == SETTLING:
                if elapsed < c.settle_s:
                    return [NOMOTION]
                if c.allow_rotate:
                    self.state, self._since = ROTATING, now
                    self.events.append(Event(now, 'rotate'))
                    return [ROTATE]
                return self._start_attempt(now)
            if elapsed < c.rotate_s:
                return [ROTATE]
            return self._start_attempt(now)
        if self.state == FAILED and self._converged(ratio, std_m):
            self.state, self.attempts = TRACKING, 0
            self.events.append(Event(now, 'restored', f'일치율 {ratio:.2f}'))
        return []
