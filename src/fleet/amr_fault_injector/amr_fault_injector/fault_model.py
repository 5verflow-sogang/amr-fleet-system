"""지연·손실 모델. rclpy를 import하지 않아 단위 테스트를 ROS 없이 돌린다."""

from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass
from typing import Any, Optional

# 명세 범위 (project-spec 3장 장애 주입기: 지연 0~500ms, 손실 0~10%). 밖의 값은 실험 조건 실수로 보고 거부한다
MAX_DELAY_MS = 500.0
MAX_LOSS_PCT = 10.0
# 연구 1 확장 조건 (10/16 결정): 손실 0~10%로는 중앙집중 재전송(1 s × 5회)이 다 막혀 역전 지점이 안 나온다.
# 분산 할당 선행연구는 5~70%를 쓴다. allow_out_of_spec을 켤 때만 손실 50%까지와 연속 끊김을 허용한다
EXTENDED_MAX_LOSS_PCT = 50.0


@dataclass(frozen=True)
class Blackout:
    """연속 끊김 (Wi-Fi 핸드오프·음영 지역 모사). 이 구간에 들어온 메시지는 모두 버린다.

    시각은 주입기 시작부터 잰 초. period_s가 0이면 한 번, 아니면 at_s부터 period_s마다 되풀이한다.
    """

    at_s: float
    duration_s: float
    period_s: float = 0.0

    def __post_init__(self):
        if not (self.at_s >= 0 and self.duration_s > 0 and self.period_s >= 0):
            raise ValueError(f'blackout은 at_s ≥ 0, duration_s > 0, period_s ≥ 0: {self}')
        if self.period_s and self.period_s <= self.duration_s:
            raise ValueError(f'blackout period_s는 duration_s보다 길어야 한다 (끊김 사이에 연결 구간): {self}')

    def active(self, elapsed_s: float) -> bool:
        if elapsed_s < self.at_s:
            return False
        offset = elapsed_s - self.at_s
        if self.period_s:
            offset %= self.period_s
        return offset < self.duration_s


@dataclass(frozen=True)
class FaultConfig:
    """링크 하나에 거는 장애 조건. 범위 밖이면 ValueError.

    allow_out_of_spec이 거짓(기본)이면 명세 범위만, 참이면 손실 50%까지와 blackout을 허용한다. 지연 상한은 같다.
    """

    delay_ms: float = 0.0
    loss_pct: float = 0.0
    allow_out_of_spec: bool = False
    blackout: Optional[Blackout] = None

    def __post_init__(self):
        max_loss = EXTENDED_MAX_LOSS_PCT if self.allow_out_of_spec else MAX_LOSS_PCT
        # NaN은 비교가 모두 False라 범위 검사를 통과하지 않는다
        if not 0 <= self.delay_ms <= MAX_DELAY_MS:
            raise ValueError(f'delay_ms는 0~{MAX_DELAY_MS:g}이어야 한다: {self.delay_ms}')
        if not 0 <= self.loss_pct <= max_loss:
            hint = '' if self.allow_out_of_spec else f' (연구용 {EXTENDED_MAX_LOSS_PCT:g}%까지는 allow_out_of_spec)'
            raise ValueError(f'loss_pct는 0~{max_loss:g}이어야 한다: {self.loss_pct}{hint}')
        if self.blackout is not None and not self.allow_out_of_spec:
            raise ValueError('blackout은 명세 밖 조건이라 allow_out_of_spec을 켜야 한다')


class DelayLine:
    """링크 하나(토픽 1개 × 로봇 1대)의 지연·손실 큐.

    시각은 초 단위 단조 시계(time.monotonic) 값을 호출 측이 넘긴다.
    지연이 고정이라 도착 순서가 보낸 순서와 같으므로 FIFO 큐로 충분하다.
    """

    def __init__(self, config: FaultConfig, rng: random.Random, start: float = 0.0,
                 blackout: bool = True):
        """start: blackout 시각의 기준(주입기 시작 시각). blackout: 이 링크에 config.blackout을 걸지 (대상 로봇만)."""
        self._config = config
        self._rng = rng
        self._start = start
        self._blackout = config.blackout if blackout else None
        self._queue: deque[tuple[float, Any]] = deque()
        self.received = 0
        self.dropped = 0
        self.blacked_out = 0

    def push(self, item: Any, now: float) -> bool:
        """메시지를 넣는다. 끊김 구간이거나 손실로 버리면 False."""
        self.received += 1
        # 손실 난수는 끊김 구간에서도 똑같이 뽑는다. 그래야 같은 seed면 끊김 밖의 손실 위치가 끊김이 없는 실행과
        # 같아서, "끊김 있음 vs 없음"을 짝지어 비교할 때 차이가 끊김에서만 나온다
        lost = self._config.loss_pct > 0 and self._rng.random() * 100 < self._config.loss_pct
        if self._blackout is not None and self._blackout.active(now - self._start):
            self.dropped += 1
            self.blacked_out += 1
            return False
        if lost:
            self.dropped += 1
            return False
        self._queue.append((now + self._config.delay_ms / 1000.0, item))
        return True

    def pop_due(self, now: float) -> list[Any]:
        """now까지 지연이 끝난 메시지를 보낸 순서대로 꺼낸다."""
        due = []
        while self._queue and self._queue[0][0] <= now:
            due.append(self._queue.popleft()[1])
        return due

    def next_due(self) -> Optional[float]:
        """다음 메시지가 나갈 시각. 큐가 비면 None."""
        return self._queue[0][0] if self._queue else None

    def __len__(self) -> int:
        return len(self._queue)


def link_rng(seed: int, robot_id: str, topic: str) -> random.Random:
    """링크마다 독립인 난수열. 같은 seed면 실행마다 같은 메시지가 손실된다 (하네스 재현성)."""
    return random.Random(f'{seed}:{robot_id}:{topic}')
