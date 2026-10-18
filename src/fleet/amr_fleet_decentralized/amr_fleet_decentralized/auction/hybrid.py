"""하이브리드 폴백: 로봇마다 서버 heartbeat로 중앙·분산 모드를 정한다 (architecture.md 5-2절, B안). ROS 비의존.

- 중앙 → 분산: 자기 heartbeat 중계본이 fallback_timeout_s(2초) 동안 안 오면
- 분산 → 중앙: heartbeat가 다시 오기 시작한 뒤 return_stable_s(5초) 동안 fallback_timeout_s 넘게 끊기지 않으면
- C안 자리: heartbeat의 mode가 MODE_DECENTRALIZED면 시간을 재지 않고 바로 분산, MODE_CENTRALIZED로 돌아오면
  그때부터 return_stable_s를 잰다
- 서버 heartbeat를 한 번도 못 봤으면 전환하지 않는다 (기동 직후 디스커버리가 늦어도 분산으로 넘어가지 않게)

시각은 호출 쪽의 한 시계(실제 시간)다. heartbeat 주기·타임아웃은 연구 1 변수라 메시지가 아니라 파라미터다.
"""

from dataclasses import dataclass
from typing import Optional

# amr_msgs/msg/FleetHeartbeat.MODE_* 와 같은 값
MODE_CENTRALIZED = 0
MODE_DECENTRALIZED = 1


@dataclass(frozen=True)
class FallbackParams:
    fallback_timeout_s: float = 2.0
    return_stable_s: float = 5.0

    def __post_init__(self):
        if self.fallback_timeout_s <= 0:
            raise ValueError(f'fallback_timeout_s는 0보다 커야 한다: {self.fallback_timeout_s}')
        if self.return_stable_s < 0:
            raise ValueError(f'return_stable_s는 0 이상이어야 한다: {self.return_stable_s}')


class FallbackMode:
    """로봇 한 대의 모드. on_heartbeat·update가 모드가 바뀌었을 때만 새 모드를 돌려준다."""

    def __init__(self, params: FallbackParams = FallbackParams()):
        self.params = params
        self.mode = MODE_CENTRALIZED
        self.transitions = 0
        self._last_heartbeat: Optional[float] = None
        # 분산 모드에서 heartbeat가 끊김 없이 이어지기 시작한 시각. None이면 아직 안정되지 않음
        self._stable_since: Optional[float] = None
        self._server_decentral = False

    def on_heartbeat(self, now: float, server_mode: int = MODE_CENTRALIZED) -> Optional[int]:
        gap = None if self._last_heartbeat is None else now - self._last_heartbeat
        self._last_heartbeat = now
        if server_mode == MODE_DECENTRALIZED:
            self._server_decentral = True
            self._stable_since = None
            return self._set(MODE_DECENTRALIZED)
        if self._server_decentral:
            # 서버가 중앙으로 돌아오라고 한 첫 heartbeat부터 잰다
            self._server_decentral = False
            self._stable_since = now
        elif self.mode == MODE_DECENTRALIZED and (
                self._stable_since is None or (gap is not None and gap >= self.params.fallback_timeout_s)):
            self._stable_since = now
        return self.update(now)

    def update(self, now: float) -> Optional[int]:
        if self._last_heartbeat is None:
            return None
        silent = now - self._last_heartbeat >= self.params.fallback_timeout_s
        if self.mode == MODE_CENTRALIZED:
            if silent:
                self._stable_since = None
                return self._set(MODE_DECENTRALIZED)
            return None
        if self._server_decentral:
            return None
        if silent:
            self._stable_since = None
            return None
        if self._stable_since is not None and now - self._stable_since >= self.params.return_stable_s:
            self._stable_since = None
            return self._set(MODE_CENTRALIZED)
        return None

    def _set(self, mode: int) -> Optional[int]:
        if mode == self.mode:
            return None
        self.mode = mode
        self.transitions += 1
        return mode


def backlog_task_ids(records, robot_id: str) -> list:
    """작업표에서 분산 모드 로봇들이 이어받을 작업: 아직 맡은 로봇이 없거나, 나에게 맡겨졌지만 시작 전(PENDING)인 것.

    records는 FleetTaskTable.records처럼 task.task_id, assignee_id, state를 가진 것. 서버가 다른 로봇에게 맡긴
    작업은 서버에 맡긴다 (그 로봇이 받았을 수 있다). 겹쳐 맡으면 먼저 수락한 쪽이 주인 규칙이 정리한다.
    """
    state_pending = 0  # TaskStatus.STATE_PENDING
    return [r.task.task_id for r in records
            if r.state == state_pending and r.assignee_id in ('', robot_id)]
