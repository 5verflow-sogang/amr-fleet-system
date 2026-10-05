"""Fleet 서버의 작업·로봇 장부와 할당 명령 재전송. rclpy를 import하지 않아 ROS 없이 테스트한다.

시각(now)은 호출 측이 초 단위로 넘긴다. 노드는 재전송·로봇 생존 판정을 실제 시간(단조 시계)으로 잰다.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any, Optional

from amr_fleet_core.assignment import ALGORITHMS, RobotInfo, TaskInfo, euclidean_cost, filter_candidates

# amr_msgs/msg/TaskStatus.STATE_* 와 같은 값
STATE_PENDING = 0
STATE_IN_PROGRESS = 1
STATE_COMPLETED = 2
STATE_FAILED = 3
FINISHED = (STATE_COMPLETED, STATE_FAILED)

COST_FN_NAME = 'euclidean'


@dataclass(frozen=True)
class DispatcherConfig:
    algorithm: str = 'nearest'
    min_battery_percent: float = 20.0
    # 이 시간 동안 RobotState가 안 오면 할당 후보에서 뺀다
    robot_timeout_s: float = 3.0
    # Ack가 없으면 이 간격으로 같은 assignment_id를 다시 보낸다
    resend_interval_s: float = 1.0
    # 재전송 한도. 넘으면 작업을 큐로 되돌린다 (첫 발송은 세지 않는다)
    max_resends: int = 5

    def __post_init__(self):
        if self.algorithm not in ALGORITHMS:
            raise ValueError(f'알 수 없는 할당 알고리즘: {self.algorithm} (가능: {sorted(ALGORITHMS)})')
        if self.resend_interval_s <= 0:
            raise ValueError(f'resend_interval_s는 0보다 커야 한다: {self.resend_interval_s}')
        if self.max_resends < 0:
            raise ValueError(f'max_resends는 0 이상이어야 한다: {self.max_resends}')


@dataclass
class TaskEntry:
    info: TaskInfo
    # 원본 Task 메시지. 이 모듈은 들여다보지 않고 노드가 TaskAssignment에 담는다
    payload: Any = None
    state: int = STATE_PENDING
    # 맡은 로봇. 큐에서 기다리는 중이면 빈 문자열
    assignee: str = ''
    assignment_id: int = 0


@dataclass
class RobotEntry:
    info: RobotInfo
    last_seen: float
    # 부하균형용 누적 할당 수 (amr_fleet_core README '작업 할당')
    load: int = 0
    # 서버가 맡긴 작업. 완료·실패 보고 전까지 다른 작업을 주지 않는다
    task_id: str = ''


@dataclass
class Outgoing:
    """보낼 할당 명령 1건."""

    assignment_id: int
    robot_id: str
    task_id: str
    # 첫 발송이면 True, 재전송이면 False
    first: bool


@dataclass
class AssignmentRow:
    """assignments.csv 한 행 (tools/harness/README.md)."""

    call_id: int
    algorithm: str
    cost_fn: str
    task_id: str
    # 할당 못 한 작업이면 빈 문자열과 None
    robot_id: str
    cost: Optional[float]


@dataclass
class _PendingAck:
    robot_id: str
    task_id: str
    last_sent: float
    sends: int = 1


@dataclass
class Dispatcher:
    config: DispatcherConfig = field(default_factory=DispatcherConfig)

    def __post_init__(self):
        self._assign_fn = ALGORITHMS[self.config.algorithm]
        # 넣은 순서를 유지한다 (작업표 방송 순서)
        self.tasks: dict[str, TaskEntry] = {}
        self.robots: dict[str, RobotEntry] = {}
        self._pending: dict[int, _PendingAck] = {}
        self._next_assignment_id = 1
        self._next_call_id = 1

    # ---- 입력 ----

    def add_task(self, info: TaskInfo, payload: Any = None) -> bool:
        """새 작업을 큐에 넣는다. 같은 task_id가 이미 있으면 무시하고 False."""
        if info.task_id in self.tasks:
            return False
        self.tasks[info.task_id] = TaskEntry(info=info, payload=payload)
        return True

    def update_robot(self, info: RobotInfo, current_task_id: str, now: float) -> None:
        """RobotState 수신. info.load는 무시하고 서버가 센 값을 쓴다."""
        entry = self.robots.get(info.robot_id)
        if entry is None:
            self.robots[info.robot_id] = RobotEntry(info=info, last_seen=now)
        else:
            entry.info = info
            entry.last_seen = now
        # Ack가 유실돼도 로봇이 그 작업을 하고 있으면 받은 것이다 (TaskAssignmentAck 주석).
        # last_assignment_id 대신 작업 ID로 본다: 서버가 재시작하면 번호가 1부터 다시 시작해
        # 로봇이 epoch를 바꾸기 전에 보낸 큰 번호를 확인으로 잘못 볼 수 있다
        if current_task_id:
            self._confirm(info.robot_id, current_task_id)

    def on_ack(self, robot_id: str, assignment_id: int, accepted: bool) -> bool:
        """TaskAssignmentAck 수신. 처음 받은 확인이면 True. 거부면 작업을 큐로 되돌린다."""
        pending = self._pending.get(assignment_id)
        if pending is None or pending.robot_id != robot_id:
            return False
        del self._pending[assignment_id]
        if not accepted:
            self._release(pending.task_id, robot_id)
        return True

    def on_status(self, task_id: str, robot_id: str, state: int) -> bool:
        """TaskStatus 수신. 맡긴 로봇의 보고가 아니거나 이미 끝난 작업이면 무시하고 False."""
        task = self.tasks.get(task_id)
        if task is None or task.assignee != robot_id or task.state in FINISHED:
            return False
        # 상태 보고가 왔으면 명령을 받은 것이다
        self._confirm(robot_id, task_id)
        task.state = state
        if state in FINISHED:
            robot = self.robots.get(robot_id)
            if robot is not None and robot.task_id == task_id:
                robot.task_id = ''
        return True

    # ---- 출력 ----

    def assign(self, now: float) -> tuple[list[Outgoing], list[AssignmentRow]]:
        """큐의 작업을 놀고 있는 로봇에 맡긴다. 작업이나 후보가 없으면 알고리즘을 부르지 않는다."""
        queued = [t.info for t in self.tasks.values() if t.state == STATE_PENDING and not t.assignee]
        free = [
            replace(r.info, load=r.load) for r in self.robots.values()
            if not r.task_id and now - r.last_seen <= self.config.robot_timeout_s
        ]
        candidates = filter_candidates(free, self.config.min_battery_percent)
        if not queued or not candidates:
            return [], []

        result = self._assign_fn(queued, candidates, euclidean_cost)
        call_id = self._next_call_id
        self._next_call_id += 1
        by_id = {r.robot_id: r for r in candidates}

        commands, rows = [], []
        for info in queued:
            robot_id = result.assignments.get(info.task_id)
            if robot_id is None:
                continue
            assignment_id = self._next_assignment_id
            self._next_assignment_id += 1
            task = self.tasks[info.task_id]
            task.assignee = robot_id
            task.assignment_id = assignment_id
            robot = self.robots[robot_id]
            robot.task_id = info.task_id
            robot.load += 1
            self._pending[assignment_id] = _PendingAck(robot_id, info.task_id, now)
            commands.append(Outgoing(assignment_id, robot_id, info.task_id, first=True))
            rows.append(AssignmentRow(
                call_id, self.config.algorithm, COST_FN_NAME, info.task_id, robot_id,
                euclidean_cost(by_id[robot_id], info)))
        for task_id in result.unassigned:
            rows.append(AssignmentRow(
                call_id, self.config.algorithm, COST_FN_NAME, task_id, '', None))
        return commands, rows

    def due_resends(self, now: float) -> tuple[list[Outgoing], list[str]]:
        """Ack가 아직 없는 명령 중 재전송할 것과, 한도를 넘어 포기한 task_id를 돌려준다.

        포기한 작업은 큐로 돌아가 다음 assign()에서 다시 할당된다.
        """
        resend, given_up = [], []
        for assignment_id in sorted(self._pending):
            pending = self._pending[assignment_id]
            if now - pending.last_sent < self.config.resend_interval_s:
                continue
            if pending.sends - 1 >= self.config.max_resends:
                del self._pending[assignment_id]
                self._release(pending.task_id, pending.robot_id)
                given_up.append(pending.task_id)
                continue
            pending.sends += 1
            pending.last_sent = now
            resend.append(Outgoing(assignment_id, pending.robot_id, pending.task_id, first=False))
        return resend, given_up

    def pending_ids(self) -> set[int]:
        """Ack를 기다리는 assignment_id."""
        return set(self._pending)

    # ---- 내부 ----

    def _confirm(self, robot_id: str, task_id: str) -> None:
        for assignment_id, pending in list(self._pending.items()):
            if pending.robot_id == robot_id and pending.task_id == task_id:
                del self._pending[assignment_id]

    def _release(self, task_id: str, robot_id: str) -> None:
        """작업을 큐로 되돌리고 로봇을 비운다. 전달되지 않은 할당이므로 load에서도 뺀다."""
        task = self.tasks.get(task_id)
        if task is not None and task.assignee == robot_id and task.state not in FINISHED:
            task.state = STATE_PENDING
            task.assignee = ''
            task.assignment_id = 0
        robot = self.robots.get(robot_id)
        if robot is not None and robot.task_id == task_id:
            robot.task_id = ''
            robot.load = max(0, robot.load - 1)
