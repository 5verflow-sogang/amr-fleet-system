"""Fleet 서버의 작업·로봇 장부와 할당 명령 재전송. rclpy를 import하지 않아 ROS 없이 테스트한다.

시각(now)은 호출 측이 초 단위로 넘긴다. 노드는 재전송·로봇 생존 판정을 실제 시간(단조 시계)으로 잰다.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any, Optional

from amr_fleet_core.assignment import ALGORITHMS, RobotInfo, TaskInfo, euclidean_cost, filter_candidates

from .claims import TaskClaims

# amr_msgs/msg/TaskStatus.STATE_* 와 같은 값
STATE_PENDING = 0
STATE_IN_PROGRESS = 1
STATE_COMPLETED = 2
STATE_FAILED = 3
FINISHED = (STATE_COMPLETED, STATE_FAILED)
# amr_msgs/msg/ErrorInfo.ASSIGN_CANCELED 와 같은 값. 서버가 취소한 작업의 FAILED는 다시 할당하지 않는다
ERROR_CANCELED = 103
# amr_msgs/msg/ErrorInfo.ASSIGN_TASK_TAKEN 와 같은 값. 다른 로봇이 먼저 맡아 물러난 것이라 재시도로 세지 않는다
ERROR_TASK_TAKEN = 104

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
    # FAILED로 보고된 작업을 큐로 되돌려 다시 할당하는 횟수. 넘으면 실패로 남긴다.
    # 분산 경매의 재경매(최대 1회, api.md 4-4절)와 같게 둬야 중앙·분산 비교가 공정하다
    failed_retries: int = 1
    # 작업을 맡은 로봇의 RobotState가 이 시간 동안 안 오면 로봇이 죽은 것으로 보고 작업을 회수해 다시 할당한다.
    # 0 이하면 회수하지 않는다. robot_timeout_s(후보 제외)보다 길게 둬 잠깐 끊긴 로봇의 작업을 뺏지 않는다
    reclaim_timeout_s: float = 5.0

    def __post_init__(self):
        if self.algorithm not in ALGORITHMS:
            raise ValueError(f'알 수 없는 할당 알고리즘: {self.algorithm} (가능: {sorted(ALGORITHMS)})')
        if self.resend_interval_s <= 0:
            raise ValueError(f'resend_interval_s는 0보다 커야 한다: {self.resend_interval_s}')
        if self.max_resends < 0:
            raise ValueError(f'max_resends는 0 이상이어야 한다: {self.max_resends}')
        if self.failed_retries < 0:
            raise ValueError(f'failed_retries는 0 이상이어야 한다: {self.failed_retries}')
        if 0 < self.reclaim_timeout_s < self.robot_timeout_s:
            raise ValueError(f'reclaim_timeout_s({self.reclaim_timeout_s})는 robot_timeout_s'
                             f'({self.robot_timeout_s}) 이상이어야 한다 (0 이하면 끈다)')


@dataclass
class TaskEntry:
    info: TaskInfo
    # 원본 Task 메시지. 이 모듈은 들여다보지 않고 노드가 TaskAssignment에 담는다
    payload: Any = None
    state: int = STATE_PENDING
    # 맡은 로봇. 큐에서 기다리는 중이면 빈 문자열
    assignee: str = ''
    assignment_id: int = 0
    # FAILED 보고로 다시 할당한 횟수
    failures: int = 0
    # 맡은 로봇이 죽어 회수한 횟수 (failures와 따로 센다. 작업이 아니라 로봇의 문제다)
    reclaims: int = 0


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
        # 먼저 수락한 쪽이 주인 (architecture.md 5-2절). 죽은 로봇의 주장은 회수와 같은 시간 뒤에 뺀다
        timeout = self.config.reclaim_timeout_s if self.config.reclaim_timeout_s > 0 else float('inf')
        self.claims = TaskClaims(timeout)

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
        self.claims.robot_state(info.robot_id, current_task_id, now)

    def on_ack(self, robot_id: str, assignment_id: int, accepted: bool, error_code: int = 0,
               now: float = 0.0) -> bool:
        """TaskAssignmentAck 수신. 처음 받은 확인이면 True. 거부면 작업을 큐로 되돌린다.

        다른 로봇이 먼저 맡았다는 거부(ERROR_TASK_TAKEN)이고 서버도 그 주인을 알면 주인에게 넘긴다.
        """
        pending = self._pending.get(assignment_id)
        if pending is None or pending.robot_id != robot_id:
            return False
        del self._pending[assignment_id]
        if not accepted:
            owner = self.claims.owner(pending.task_id, now) if error_code == ERROR_TASK_TAKEN else None
            task = self.tasks.get(pending.task_id)
            if owner is not None and task is not None and task.assignee == robot_id:
                self._hand_over(task, owner, STATE_COMPLETED if self.claims.completed(pending.task_id, owner)
                                else STATE_IN_PROGRESS)
            else:
                self._release(pending.task_id, robot_id)
        return True

    def on_status(self, task_id: str, robot_id: str, state: int, error_code: int = 0,
                  start: float = 0.0, now: float = 0.0) -> bool:
        """TaskStatus 수신. 장부가 바뀌지 않았으면(모르는 작업, 이미 끝난 작업, 주인이 아닌 로봇) False.

        start는 그 로봇의 수락 시각(TaskStatus.start_time), now는 update_robot과 같은 시계다.
        맡기지 않은 로봇의 보고라도 그 로봇이 주인(먼저 수락한 쪽)이면 맡은 로봇을 그 로봇으로 바꾼다.
        분산 경매로 맡았거나, 서버가 다른 로봇에게 다시 맡긴 뒤 옛 명령이 늦게 도착한 경우다 (architecture.md 5-2절).

        FAILED는 failed_retries번까지 큐로 되돌려(state PENDING) 다시 할당받게 한다. 그 뒤에는 실패로 남긴다.
        취소(error_code ERROR_CANCELED)로 온 FAILED는 서버가 스스로 멈춘 것이므로 다시 할당하지 않는다.
        다른 로봇이 먼저 맡아 물러난 FAILED(ERROR_TASK_TAKEN)는 주인에게 넘기고 재시도로 세지 않는다.
        """
        # 작업 요청보다 먼저 온 보고도 남긴다 (분산 경매가 서버보다 먼저 맡을 수 있다). assign()이 본다
        self.claims.observe(task_id, robot_id, state, start, now)
        task = self.tasks.get(task_id)
        if task is None or task.state in FINISHED:
            return False
        if task.assignee != robot_id:
            if self.claims.owner(task_id, now) != robot_id:
                return False
            self._hand_over(task, robot_id, state)
            return True
        # 상태 보고가 왔으면 명령을 받은 것이다
        self._confirm(robot_id, task_id)
        if state == STATE_FAILED and error_code == ERROR_TASK_TAKEN:
            owner = self.claims.owner(task_id, now)
            if owner is not None:
                self._hand_over(task, owner, STATE_COMPLETED if self.claims.completed(task_id, owner)
                                else STATE_IN_PROGRESS)
            else:
                # 주인이 안 보이면(그 사이 놓았거나 죽음) 큐로 되돌려 다시 맡긴다
                self._release(task_id, robot_id)
            return True
        if state == STATE_FAILED and error_code != ERROR_CANCELED and task.failures < self.config.failed_retries:
            task.failures += 1
            task.state = STATE_PENDING
            task.assignee = ''
            task.assignment_id = 0
        else:
            task.state = state
        if state in FINISHED:
            robot = self.robots.get(robot_id)
            if robot is not None and robot.task_id == task_id:
                robot.task_id = ''
        return True

    # ---- 출력 ----

    def assign(self, now: float) -> tuple[list[Outgoing], list[AssignmentRow]]:
        """큐의 작업을 놀고 있는 로봇에 맡긴다. 작업이나 후보가 없으면 알고리즘을 부르지 않는다.

        이미 다른 로봇이 맡았다고 보고한 작업은 맡기지 않고 그 로봇(주인)에게 넘긴다.
        """
        for t in self.tasks.values():
            if t.state == STATE_PENDING and not t.assignee:
                owner = self.claims.owner(t.info.task_id, now)
                if owner is not None:
                    self._hand_over(t, owner, STATE_COMPLETED if self.claims.completed(t.info.task_id, owner)
                                    else STATE_IN_PROGRESS)
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

    def robot_priority(self, robot_id: str) -> int:
        """로봇이 맡은 작업의 우선순위. 맡은 작업이 없으면 0 (구역 허가 순서에 쓴다)."""
        robot = self.robots.get(robot_id)
        task = self.tasks.get(robot.task_id) if robot is not None and robot.task_id else None
        return task.info.priority if task is not None else 0

    def reclaim_lost(self, now: float) -> list[tuple[str, str]]:
        """맡은 로봇의 RobotState가 reclaim_timeout_s 동안 없으면 작업을 큐로 되돌린다. (task_id, 죽은 robot_id) 목록.

        되돌린 작업은 다음 assign()에서 살아 있는 로봇에게 새 assignment_id로 간다. 죽은 로봇의 주장도 같은 시간 뒤
        빠진다(claims). 로봇이 살아 돌아와 옛 작업을 계속 보고하면 먼저 수락한 쪽이 주인이라 그 로봇이 다시 맡고,
        새로 맡은 로봇은 그 보고를 보고 스스로 물러난다(ERROR_TASK_TAKEN, architecture.md 5-2절).
        """
        if self.config.reclaim_timeout_s <= 0:
            return []
        reclaimed = []
        for robot_id, robot in self.robots.items():
            if not robot.task_id or now - robot.last_seen < self.config.reclaim_timeout_s:
                continue
            task_id = robot.task_id
            task = self.tasks.get(task_id)
            for assignment_id, pending in list(self._pending.items()):
                if pending.robot_id == robot_id:
                    del self._pending[assignment_id]
            self._release(task_id, robot_id)
            if task is not None:
                task.reclaims += 1
            reclaimed.append((task_id, robot_id))
        return reclaimed

    def pending_ids(self) -> set[int]:
        """Ack를 기다리는 assignment_id."""
        return set(self._pending)

    # ---- 내부 ----

    def _confirm(self, robot_id: str, task_id: str) -> None:
        for assignment_id, pending in list(self._pending.items()):
            if pending.robot_id == robot_id and pending.task_id == task_id:
                del self._pending[assignment_id]

    def _hand_over(self, task: TaskEntry, robot_id: str, state: int) -> None:
        """작업의 맡은 로봇을 robot_id(주인)로 바꾼다. 원래 로봇에게 보내던 명령의 재전송은 그만둔다."""
        task_id = task.info.task_id
        old = task.assignee
        if old:
            self._confirm(old, task_id)
            self._release(task_id, old)
        task.assignee = robot_id
        task.assignment_id = 0
        task.state = state
        robot = self.robots.get(robot_id)
        if robot is not None and state not in FINISHED:
            robot.task_id = task_id
            robot.load += 1

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
