"""지연·손실 모델. rclpy를 import하지 않아 단위 테스트를 ROS 없이 돌린다."""

from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass
from typing import Any, Optional

# 명세 범위 (project-spec 3장 장애 주입기: 지연 0~500ms, 손실 0~10%). 밖의 값은 실험 조건 실수로 보고 거부한다
MAX_DELAY_MS = 500.0
MAX_LOSS_PCT = 10.0


@dataclass(frozen=True)
class FaultConfig:
    """링크 하나에 거는 장애 조건. 명세 범위 밖이면 ValueError."""

    delay_ms: float = 0.0
    loss_pct: float = 0.0

    def __post_init__(self):
        # NaN은 비교가 모두 False라 범위 검사를 통과하지 않는다
        if not 0 <= self.delay_ms <= MAX_DELAY_MS:
            raise ValueError(f'delay_ms는 0~{MAX_DELAY_MS:g}이어야 한다: {self.delay_ms}')
        if not 0 <= self.loss_pct <= MAX_LOSS_PCT:
            raise ValueError(f'loss_pct는 0~{MAX_LOSS_PCT:g}이어야 한다: {self.loss_pct}')


class DelayLine:
    """링크 하나(토픽 1개 × 로봇 1대)의 지연·손실 큐.

    시각은 초 단위 단조 시계(time.monotonic) 값을 호출 측이 넘긴다.
    지연이 고정이라 도착 순서가 보낸 순서와 같으므로 FIFO 큐로 충분하다.
    """

    def __init__(self, config: FaultConfig, rng: random.Random):
        self._config = config
        self._rng = rng
        self._queue: deque[tuple[float, Any]] = deque()
        self.received = 0
        self.dropped = 0

    def push(self, item: Any, now: float) -> bool:
        """메시지를 넣는다. 손실로 버리면 False."""
        self.received += 1
        if self._config.loss_pct > 0 and self._rng.random() * 100 < self._config.loss_pct:
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
