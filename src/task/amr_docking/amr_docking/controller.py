"""도킹 접근 제어 (ROS 비의존). 차동 구동이라 옆으로 못 움직이므로 단계를 나눈다.

    SEARCH   마커판이 보일 때까지 기다리고, 안 보이면 제자리에서 좌우로 돌며 찾는다
    BACKOFF  목표에 너무 가깝거나 지나쳤는데 옆 오차가 크면, 뒤로 빠져 접근 거리를 번다
    APPROACH 목표 진행선을 따라 들어간다 (옆 오차를 진행 방향으로 바로잡음). 남은 거리 ≈ 0이면 멈춤
    ROTATE   제자리에서 목표 방향으로 돈다
    VERIFY   멈춘 채 여러 번 재서 평균 오차를 보고, 허용치 안이면 성공. 아니면 BACKOFF부터 다시

step(error, dt)는 매 주기 부른다. error는 이번 주기에 잰 도킹 오차(geometry.DockError)이고 마커판이
안 보이면 None이다. 반환은 (v, w) 명령과 끝났으면 결과(Outcome).
"""

from dataclasses import dataclass, field
import math
from typing import List, Optional, Tuple

from .geometry import DockError, average

# Dock.action Result.error_code와 같은 값
NONE = 0
FAILED_TO_DETECT_DOCK = 904
FAILED_TO_CONTROL = 905


@dataclass
class ControllerConfig:
    v_max: float = 0.12             # m/s
    v_min: float = 0.015            # m/s. 접근 끝에서 너무 느려 멈추지 않게
    w_max: float = 0.4              # rad/s
    w_min: float = 0.03             # rad/s. 제자리 회전 끝에서
    k_along: float = 0.6            # 남은 거리 → 속도
    k_lateral: float = 6.0          # 옆 오차 → 목표 방향 보정 (atan)
    max_correction: float = 0.3     # rad. 옆 오차 보정 방향 상한
    k_heading: float = 2.0          # 방향 오차 → 각속도
    k_rotate: float = 1.2
    along_tol: float = 0.003        # m. 접근 멈춤
    heading_tol: float = math.radians(0.25)   # 제자리 회전 멈춤
    min_run: float = 0.25           # m. 옆 오차를 잡으려면 이만큼 앞에서 접근을 시작해야 한다
    backoff_to: float = 0.4         # m. BACKOFF는 남은 거리가 이만큼 될 때까지 뒤로
    lateral_ok: float = 0.008       # m. 이보다 작으면 BACKOFF 없이 바로 접근
    lateral_finish: float = 0.005   # m. 접근 끝에서 옆 오차가 이보다 크면 다시 뒤로 빠져 접근한다
    max_approaches: int = 3         # 한 번 시도 안에서 접근을 다시 하는 횟수
    position_tol: float = 0.02      # m. 성공 판정 (명세 KPI 2 cm)
    yaw_tol: float = math.radians(1.0)        # 성공 판정 (명세 KPI 1도)
    verify_samples: int = 15
    verify_heading: float = math.radians(0.5)   # VERIFY: 위치는 됐는데 방향이 이보다 크면 ROTATE부터 다시
    max_rotations: int = 3
    settle_s: float = 0.5           # VERIFY: 멈춘 뒤 이만큼 기다렸다 잰다 (영상 지연 ≈ 50 ms라 돌던 중 영상이 섞인다)
    search_wait_s: float = 2.0      # 처음 안 보이면 이만큼 기다린 뒤 돌며 찾는다
    search_sweep: float = 0.6       # rad. 좌우로 찾는 각도
    search_w: float = 0.25          # rad/s
    lost_timeout_s: float = 3.0     # 접근 중 이만큼 안 보이면 실패
    timeout_s: float = 60.0         # 한 번 시도의 전체 시간
    max_retries: int = 2            # VERIFY에서 허용치 밖이면 BACKOFF부터 다시 하는 횟수


@dataclass
class Outcome:
    success: bool
    error_code: int
    message: str
    error: Optional[DockError] = None


@dataclass
class DockingController:
    config: ControllerConfig = field(default_factory=ControllerConfig)
    stage: str = 'SEARCH'
    elapsed: float = 0.0
    _stage_time: float = 0.0
    _since_seen: float = 0.0
    _seen_once: bool = False
    _sweep_angle: float = 0.0       # SEARCH 중 제자리에서 돈 누적 각도 (명령 적분)
    _sweep_index: int = 0
    _samples: List[DockError] = field(default_factory=list)
    _retries: int = 0
    _approaches: int = 0
    _rotations: int = 0
    last_error: Optional[DockError] = None

    def _go(self, stage: str):
        self.stage = stage
        self._stage_time = 0.0
        self._samples = []

    def _clamp(self, value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def step(self, error: Optional[DockError], dt: float) -> Tuple[float, float, Optional[Outcome]]:
        cfg = self.config
        self.elapsed += dt
        self._stage_time += dt
        if self.elapsed > cfg.timeout_s:
            return 0.0, 0.0, Outcome(False, FAILED_TO_CONTROL,
                                     f'timeout {cfg.timeout_s:.0f} s in {self.stage}', self.last_error)
        if error is not None:
            self._since_seen = 0.0
            self._seen_once = True
            self.last_error = error
        else:
            self._since_seen += dt

        if self.stage == 'SEARCH':
            if error is not None:
                self._go(self._next_after_seen(error))
                return 0.0, 0.0, None
            if self._stage_time < cfg.search_wait_s:
                return 0.0, 0.0, None
            # 좌우로 search_sweep만큼 한 번씩 돌아본다: 0 → +sweep → -sweep → 0. 다 돌아도 없으면 실패
            targets = (cfg.search_sweep, -cfg.search_sweep, 0.0)
            while self._sweep_index < len(targets):
                remaining = targets[self._sweep_index] - self._sweep_angle
                if abs(remaining) > 1e-6:
                    break
                self._sweep_index += 1
            if self._sweep_index >= len(targets):
                return 0.0, 0.0, Outcome(False, FAILED_TO_DETECT_DOCK, 'marker board not found')
            step = math.copysign(min(cfg.search_w * dt, abs(remaining)), remaining)
            self._sweep_angle += step
            return 0.0, step / dt if dt > 0 else 0.0, None

        if error is None:
            if self._since_seen > cfg.lost_timeout_s:
                return 0.0, 0.0, Outcome(False, FAILED_TO_DETECT_DOCK,
                                         f'marker board lost for {cfg.lost_timeout_s:.0f} s in {self.stage}',
                                         self.last_error)
            return 0.0, 0.0, None   # 잠깐 안 보이면 멈춰 기다린다

        if self.stage == 'BACKOFF':
            if error.along >= cfg.backoff_to:
                self._go('APPROACH')
                return 0.0, 0.0, None
            return -cfg.v_max * 0.7, self._clamp(-cfg.k_heading * error.heading, cfg.w_max), None

        if self.stage == 'APPROACH':
            if abs(error.along) <= cfg.along_tol:
                # 옆 오차가 남았으면 차동 구동은 옆으로 못 가니 뒤로 빠져 다시 들어간다
                if abs(error.lateral) > cfg.lateral_finish and self._approaches < cfg.max_approaches:
                    self._approaches += 1
                    self._go('BACKOFF')
                else:
                    self._go('ROTATE')
                return 0.0, 0.0, None
            if error.along < -cfg.along_tol and abs(error.lateral) > cfg.lateral_ok:
                self._go('BACKOFF')   # 옆 오차가 남은 채 지나쳤다
                return 0.0, 0.0, None
            # 진행선 쪽으로 기운 방향을 목표로 (Stanley 방식). 뒤로 갈 때는 보정 방향이 반대
            direction = 1.0 if error.along > 0 else -1.0
            correction = self._clamp(math.atan(cfg.k_lateral * error.lateral), cfg.max_correction)
            heading_target = -direction * correction
            w = self._clamp(cfg.k_heading * (heading_target - error.heading), cfg.w_max)
            speed = max(cfg.v_min, min(cfg.v_max, cfg.k_along * abs(error.along)))
            return direction * speed, w, None

        if self.stage == 'ROTATE':
            if abs(error.heading) <= cfg.heading_tol:
                self._go('VERIFY')
                return 0.0, 0.0, None
            w = -cfg.k_rotate * error.heading
            w = math.copysign(max(cfg.w_min, min(cfg.w_max, abs(w))), w)
            return 0.0, w, None

        if self.stage == 'VERIFY':
            if self._stage_time < cfg.settle_s:
                return 0.0, 0.0, None
            self._samples.append(error)
            if len(self._samples) < cfg.verify_samples:
                return 0.0, 0.0, None
            mean = average(self._samples)
            self.last_error = mean
            # 돌다 멈출 때 관성·영상 지연으로 방향이 조금 더 가 있으면 제자리 회전만 다시 한다
            if (mean.position <= cfg.position_tol and abs(mean.heading) > cfg.verify_heading
                    and self._rotations < cfg.max_rotations):
                self._rotations += 1
                self._go('ROTATE')
                return 0.0, 0.0, None
            if mean.position <= cfg.position_tol and abs(mean.heading) <= cfg.yaw_tol:
                return 0.0, 0.0, Outcome(True, NONE, '', mean)
            if self._retries < cfg.max_retries:
                self._retries += 1
                self._go(self._next_after_seen(mean))
                return 0.0, 0.0, None
            return 0.0, 0.0, Outcome(
                False, FAILED_TO_CONTROL,
                f'error {mean.position * 100:.1f} cm / {math.degrees(abs(mean.heading)):.2f} deg '
                f'after {cfg.max_retries + 1} tries', mean)

        raise RuntimeError(f'unknown stage {self.stage}')

    def _next_after_seen(self, error: DockError) -> str:
        """마커판이 보였을 때 어디서 시작할지: 접근 거리가 모자라고 옆 오차가 크면 먼저 뒤로 뺀다."""
        cfg = self.config
        if abs(error.lateral) > cfg.lateral_ok and error.along < cfg.min_run:
            return 'BACKOFF'
        if abs(error.along) <= cfg.along_tol:
            return 'ROTATE'
        return 'APPROACH'
