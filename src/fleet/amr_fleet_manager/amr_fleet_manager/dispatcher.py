"""Fleet 서버의 작업·로봇 장부와 할당 명령 재전송. rclpy를 import하지 않아 ROS 없이 테스트한다.

시각(now)은 호출 측이 초 단위로 넘긴다. 노드는 재전송·로봇 생존 판정을 실제 시간(단조 시계)으로 잰다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Any, Optional

from amr_fleet_core.assignment import (ALGORITHMS, COST_FNS, CongestedArea, CostFn, RobotInfo, TaskInfo,
                                       euclidean_cost, filter_candidates, make_congestion_cost, sort_by_priority)

from amr_fleet_core.charging import (ChargeConfig, Station, charge_target, max_charging, nearest_free,
                                     plan_charging)
from amr_fleet_core.claims import TaskClaims
from amr_fleet_core.energy import (ChargeQueue, EnergyModel, charge_remaining_s, make_battery_cost,
                                   make_energy_feasible)

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
# amr_msgs/msg/Task.TYPE_* 와 같은 값
TYPE_CHARGE = 1
TYPE_RETURN = 2
# 서버가 만드는 충전·복귀 작업의 우선순위 (구역 허가 순서에서 배송보다 먼저)
CHARGE_PRIORITY = 200


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
    # 스테이션 예약: 맡겨진 작업의 픽업·하역 위치에서 이 거리 [m] 안을 쓰는 작업은 그 작업이 끝날 때까지 맡기지 않는다.
    # 같은 도크로 가는 두 작업을 동시에 내보내 두 로봇이 한 정차 자세로 들어가 막혔다 (11/2 fleet_stack-release_build,
    # dock_a). 0이면 끈다 (기본. 노드 설정은 1.0, 연구 1 가짜 로봇 실험은 지난 조건대로 끈다)
    station_clearance_m: float = 0.0
    # 비용 함수 (COST_FNS). congestion: 직선 거리 + congestion_weight × Σ(혼잡도 × 혼잡 구역 통과 길이).
    # 혼잡도는 set_congestion으로 넣는다(노드가 /fleet/congestion을 구독). 식당 시연 1-1 켬/끔 비교
    cost_fn: str = 'euclidean'
    congestion_weight: float = 3.0
    # 혼잡도를 이 시간 [s] 동안 못 받으면 직선 거리로 돌아간다. 혼잡도 노드가 죽었을 때 마지막 값으로 계속 벌점을 주지 않게
    congestion_timeout_s: float = 3.0
    # 자동 충전 배정 (배터리 인지 운영 1단계, amr_fleet_core charging). 끄면(기본) 지금처럼 하한 밑 로봇을 빼기만 한다
    charging_enabled: bool = False
    charge: ChargeConfig = ChargeConfig()
    # 충전기 위치 (map 좌표). 비면 충전하러 보내지 않는다
    chargers: tuple[Station, ...] = ()
    # 충전을 마친 로봇을 보낼 대기 위치. 비면 충전기 위에 그대로 둔다 (다음 작업을 받으면 비켜 준다)
    wait_stations: tuple[Station, ...] = ()
    # 에너지 예측 배정 (2단계, amr_fleet_core energy): 작업을 끝내고 가장 가까운 충전기까지 갈 배터리가 없는 로봇은
    # 그 작업에서만 뺀다. 충전기는 chargers를 쓴다 (비면 배달지까지만 본다). 끄면(기본) min_battery_percent만 본다
    energy_check: bool = False
    energy: EnergyModel = EnergyModel()
    # 비용 + battery_weight × (필요 배터리 / 남은 배터리) [m]. 배터리가 많은 로봇이 먼 작업을 맡게 한다. 0이면 끈다
    battery_weight: float = 0.0

    def __post_init__(self):
        if self.algorithm not in ALGORITHMS:
            raise ValueError(f'알 수 없는 할당 알고리즘: {self.algorithm} (가능: {sorted(ALGORITHMS)})')
        if self.cost_fn not in COST_FNS:
            raise ValueError(f'알 수 없는 비용 함수: {self.cost_fn} (가능: {list(COST_FNS)})')
        if not self.congestion_weight >= 0:
            raise ValueError(f'congestion_weight는 0 이상이어야 한다: {self.congestion_weight}')
        if not self.battery_weight >= 0:
            raise ValueError(f'battery_weight는 0 이상이어야 한다: {self.battery_weight}')
        if not self.congestion_timeout_s > 0:
            raise ValueError(f'congestion_timeout_s는 0보다 커야 한다: {self.congestion_timeout_s}')
        if self.resend_interval_s <= 0:
            raise ValueError(f'resend_interval_s는 0보다 커야 한다: {self.resend_interval_s}')
        if self.max_resends < 0:
            raise ValueError(f'max_resends는 0 이상이어야 한다: {self.max_resends}')
        if self.failed_retries < 0:
            raise ValueError(f'failed_retries는 0 이상이어야 한다: {self.failed_retries}')
        if not self.station_clearance_m >= 0:
            raise ValueError(f'station_clearance_m는 0 이상이어야 한다: {self.station_clearance_m}')
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
    # 서버가 이 로봇에게 직접 낸 작업 (자동 충전·충전 뒤 복귀). 할당 알고리즘을 거치지 않는다
    for_robot: str = ''
    # 목적지 스테이션 이름 (충전기·대기 위치 예약에 쓴다)
    station: str = ''


@dataclass
class RobotEntry:
    info: RobotInfo
    last_seen: float
    # 부하균형용 누적 할당 수 (amr_fleet_core README '작업 할당')
    load: int = 0
    # 서버가 맡긴 작업. 완료·실패 보고 전까지 다른 작업을 주지 않는다
    task_id: str = ''
    # 하이브리드: 서버와 끊겨 분산 모드로 도는 로봇 (RobotFleetMode). 명령이 닿지 않으므로 맡기지 않는다
    decentral: bool = False


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
class ChargeTask:
    """서버가 만든 충전·복귀 작업 1건. 노드가 payload(Task 메시지)를 붙인다."""

    task_id: str
    robot_id: str
    task_type: int
    station: str


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
        # 혼잡도 반영 비용과 그 혼잡도를 받은 시각 (cost_fn이 congestion일 때만)
        self._congestion_cost: Optional[CostFn] = None
        self._congestion_at = -math.inf
        # 넣은 순서를 유지한다 (작업표 방송 순서)
        self.tasks: dict[str, TaskEntry] = {}
        self.robots: dict[str, RobotEntry] = {}
        self._pending: dict[int, _PendingAck] = {}
        self._next_assignment_id = 1
        self._next_call_id = 1
        # 먼저 수락한 쪽이 주인 (architecture.md 5-2절). 죽은 로봇의 주장은 회수와 같은 시간 뒤에 뺀다
        timeout = self.config.reclaim_timeout_s if self.config.reclaim_timeout_s > 0 else float('inf')
        self.claims = TaskClaims(timeout)
        # 자동 충전: 만든 작업 번호, 충전을 마치고 대기 위치로 보낼 로봇, 마지막 계획에서 충전 차례를 기다리는 로봇
        self._next_charge = 1
        self._return_after: set[str] = set()
        # 피크 대비: 마지막 plan_charging의 일정 시각, 일찍 끝내려고 취소를 보낸 충전 작업 → (명령 번호, 마지막 발송)
        self.sched_t: Optional[float] = None
        self._stopping: dict[str, tuple[int, float]] = {}
        self.charge_waiting: list[str] = []

    # ---- 입력 ----

    def set_congestion(self, areas: list[CongestedArea], now: float) -> None:
        """구역별 혼잡도 (/fleet/congestion 한 장). cost_fn이 congestion일 때만 할당에 쓴다."""
        if self.config.cost_fn == 'congestion':
            self._congestion_cost = make_congestion_cost(areas, self.config.congestion_weight)
            self._congestion_at = now

    def _current_cost_fn(self, now: float) -> tuple[str, CostFn]:
        """이번 할당에 쓸 (이름, 비용 함수). 혼잡도를 아직 못 받았거나 오래됐으면 euclidean."""
        if self._congestion_cost is not None and now - self._congestion_at <= self.config.congestion_timeout_s:
            return 'congestion', self._congestion_cost
        return 'euclidean', euclidean_cost

    def add_task(self, info: TaskInfo, payload: Any = None, for_robot: str = '', station: str = '') -> bool:
        """새 작업을 큐에 넣는다. 같은 task_id가 이미 있으면 무시하고 False.

        for_robot을 주면 할당 알고리즘을 거치지 않고 그 로봇이 비는 대로 맡긴다 (서버가 낸 충전·복귀).
        """
        if info.task_id in self.tasks:
            return False
        self.tasks[info.task_id] = TaskEntry(info=info, payload=payload, for_robot=for_robot, station=station)
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
            # 충전을 마쳤거나 피크 목표에 닿아 서버가 끝낸 충전 → 대기 위치로
            ended = state == STATE_COMPLETED or (error_code == ERROR_CANCELED and task_id in self._stopping)
            if ended and task.for_robot and task.info.task_type == TYPE_CHARGE and self.config.wait_stations:
                self._return_after.add(robot_id)
            self._stopping.pop(task_id, None)
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
        # 서버가 낸 충전을 먼저 보낸다 (그 로봇에게 다른 작업을 주지 않게). 복귀는 할당 알고리즘 뒤
        commands = self._assign_directed(TYPE_CHARGE, now)
        queued = self._station_free(
            [t.info for t in self.tasks.values() if t.state == STATE_PENDING and not t.assignee and not t.for_robot])
        reserved = self._reserved()
        free = [
            replace(r.info, load=r.load) for r in self.robots.values()
            if self._free(r, now) and r.info.robot_id not in reserved
        ]
        candidates = filter_candidates(free, self.config.min_battery_percent)
        if not queued or not candidates:
            return commands + self._assign_directed(TYPE_RETURN, now), []

        cost_name, cost_fn = self._current_cost_fn(now)
        cost_fn = make_battery_cost(cost_fn, self.config.chargers, self.config.energy, self.config.battery_weight)
        feasible = None
        if self.config.energy_check:
            queue = self.charge_queue(now) if self.config.charging_enabled else None
            feasible = make_energy_feasible(self.config.chargers, self.config.energy, queue,
                                            self.config.charge.low_percent if queue is not None else None)
        result = self._assign_fn(queued, candidates, cost_fn, feasible)
        call_id = self._next_call_id
        self._next_call_id += 1
        by_id = {r.robot_id: r for r in candidates}

        rows = []
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
                call_id, self.config.algorithm, cost_name, info.task_id, robot_id,
                cost_fn(by_id[robot_id], info)))
        for task_id in result.unassigned:
            rows.append(AssignmentRow(
                call_id, self.config.algorithm, cost_name, task_id, '', None))
        return commands + self._assign_directed(TYPE_RETURN, now), rows

    def plan_charging(self, now: float, t: Optional[float] = None) -> list[ChargeTask]:
        """자동 충전 (charging_enabled): 배터리 낮은 로봇에게 충전, 충전을 마친 로봇에게 대기 위치 복귀 작업을 만든다.

        만든 작업은 장부에 넣고(for_robot) 돌려준다. 노드가 payload를 붙인 뒤 assign()이 보낸다.
        보내기 전에 로봇이 죽으면(robot_timeout_s) 그 작업은 지운다. 충전 차례를 기다리는 로봇은 charge_waiting.
        t는 피크 대비용 일정 시각이다 (ChargeConfig.peaks와 같은 시계, 없으면 피크 대비를 끈다).
        """
        self.sched_t = t
        if not self.config.charging_enabled or not self.config.chargers:
            return []
        alive = {rid: r for rid, r in self.robots.items()
                 if not r.decentral and now - r.last_seen <= self.config.robot_timeout_s}
        for task_id, entry in list(self.tasks.items()):
            if entry.for_robot and not entry.assignee and entry.state == STATE_PENDING and entry.for_robot not in alive:
                del self.tasks[task_id]
        reserved = self._reserved()
        plan = plan_charging([r.info for r in alive.values()],
                             {rid for rid, r in alive.items() if self._free(r, now) and rid not in reserved},
                             self._charging(), self.config.chargers, self.config.charge, t)
        self.charge_waiting = plan.waiting
        by_name = {s.name: s for s in self.config.chargers}
        created = []
        for order in plan.orders:
            self._return_after.discard(order.robot_id)
            created.append(self._add_directed(order.robot_id, TYPE_CHARGE, by_name[order.station], now))
        # 충전을 마친 로봇: 일을 받아 이미 떠났으면 그만, 아니면 빈 대기 위치로
        reserved = self._reserved()
        for rid in sorted(self._return_after):
            robot = alive.get(rid)
            if robot is None or robot.task_id:
                self._return_after.discard(rid)
                continue
            if rid in reserved:
                continue
            taken = {t.station for t in self.tasks.values()
                     if t.for_robot and t.info.task_type == TYPE_RETURN and t.state not in FINISHED}
            taken |= {s.name for s in self.config.wait_stations for other, r in alive.items()
                      if other != rid and math.dist(r.info.xy, s.xy) <= self.config.charge.occupied_radius_m}
            station = nearest_free(robot.info.xy, self.config.wait_stations, taken)
            if station is None:
                continue
            self._return_after.discard(rid)
            created.append(self._add_directed(rid, TYPE_RETURN, station, now))
        return created

    def charge_queue(self, now: float) -> ChargeQueue:
        """지금 충전 대기열 (에너지 예측 배정이 충전 차례 대기 시간을 추정하는 데 쓴다).

        대기 로봇 수는 마지막 plan_charging의 charge_waiting이다 (노드는 할당 직전에 부른다).
        """
        model = self.config.energy
        alive = [r for r in self.robots.values()
                 if not r.decentral and now - r.last_seen <= self.config.robot_timeout_s]
        charging = self._charging()
        by_name = {s.name: s.xy for s in self.config.chargers}
        remaining = tuple(charge_remaining_s(self.robots[rid].info, by_name.get(name, self.robots[rid].info.xy), model)
                          for rid, name in charging.items() if rid in self.robots)
        slots = max_charging(len(alive), self.config.charge, self.sched_t) - len(charging)
        free_chargers = len(self.config.chargers) - len(set(charging.values()))
        target = charge_target(self.sched_t, self.config.charge) or model.charge_target_percent
        cycle = max(0.0, target - self.config.charge.low_percent) / model.charge_per_s
        return ChargeQueue(remaining, len(self.charge_waiting), max(0, min(slots, free_chargers)), cycle)

    def stop_charging(self, now: float) -> list[Outgoing]:
        """피크 중 충전 목표(peak_target_percent)에 닿은 서버 충전 작업을 끝낼 취소 명령 (마지막 plan_charging 시각 기준).

        같은 작업에는 같은 명령 번호로 resend_interval_s마다 다시 보낸다 (로봇은 재전송으로 보고 같은 Ack만 낸다).
        로봇이 취소 완료(FAILED + ASSIGN_CANCELED)를 보고하면 대기 위치로 보낸다.
        """
        target = charge_target(self.sched_t, self.config.charge)
        out = []
        for task_id, entry in self.tasks.items():
            if not entry.for_robot or entry.info.task_type != TYPE_CHARGE or entry.state != STATE_IN_PROGRESS:
                continue
            robot = self.robots.get(entry.assignee)
            if robot is None:
                continue
            sent = self._stopping.get(task_id)
            if sent is None:
                if target is None or robot.info.battery_percent < target:
                    continue
                assignment_id = self._next_assignment_id
                self._next_assignment_id += 1
                self._stopping[task_id] = (assignment_id, now)
                out.append(Outgoing(assignment_id, entry.assignee, task_id, first=True))
            elif now - sent[1] >= self.config.resend_interval_s:
                self._stopping[task_id] = (sent[0], now)
                out.append(Outgoing(sent[0], entry.assignee, task_id, first=False))
        return out

    def charging_robots(self) -> dict[str, str]:
        """충전 작업을 맡았거나 맡을 로봇 → 충전기 (이동 중·충전 중 포함, 작업이 끝나면 빠진다)."""
        return self._charging()

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

    def set_robot_mode(self, robot_id: str, decentral: bool) -> list[str]:
        """하이브리드 로봇의 모드 (RobotFleetMode, architecture.md 5-2절). 분산 모드 로봇에게는 맡기지 않고,
        아직 확인되지 않은 명령은 거둬 큐로 되돌린다(task_id 목록). 이미 시작한 작업은 그 로봇이 끝까지 한다."""
        robot = self.robots.get(robot_id)
        if robot is None or robot.decentral == decentral:
            return []
        robot.decentral = decentral
        if not decentral:
            return []
        released = []
        for assignment_id, pending in list(self._pending.items()):
            if pending.robot_id == robot_id:
                del self._pending[assignment_id]
                self._release(pending.task_id, robot_id)
                released.append(pending.task_id)
        return released

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

    def _free(self, robot: RobotEntry, now: float) -> bool:
        """맡은 작업이 없고 서버 명령이 닿는 살아 있는 로봇."""
        return not robot.task_id and not robot.decentral and now - robot.last_seen <= self.config.robot_timeout_s

    def _reserved(self) -> set[str]:
        """아직 보내지 않은 충전 작업이 기다리는 로봇 (다른 작업을 주지 않는다)."""
        return {t.for_robot for t in self.tasks.values()
                if t.for_robot and not t.assignee and t.state == STATE_PENDING and t.info.task_type == TYPE_CHARGE}

    def _charging(self) -> dict[str, str]:
        """끝나지 않은 충전 작업의 로봇 → 충전기 이름. 작업 생성기가 낸 충전 작업도 센다 (충전기는 목적지에서 가장 가까운 곳)."""
        out = {}
        for t in self.tasks.values():
            robot_id = t.assignee or t.for_robot
            if t.info.task_type != TYPE_CHARGE or t.state in FINISHED or not robot_id:
                continue
            station = t.station
            if not station:
                near = nearest_free(t.info.dropoff_xy, self.config.chargers, set())
                station = near.name if near is not None else t.info.task_id
            out[robot_id] = station
        return out

    def _add_directed(self, robot_id: str, task_type: int, station: Station, now: float) -> ChargeTask:
        kind = 'charge' if task_type == TYPE_CHARGE else 'return'
        task_id = f'{kind}_{robot_id}_{self._next_charge:04d}'
        self._next_charge += 1
        info = TaskInfo(task_id=task_id, task_type=task_type, dropoff_xy=station.xy, priority=CHARGE_PRIORITY,
                        created=now)
        self.add_task(info, for_robot=robot_id, station=station.name)
        return ChargeTask(task_id, robot_id, task_type, station.name)

    def _assign_directed(self, task_type: int, now: float) -> list[Outgoing]:
        """for_robot 작업 중 task_type인 것을 그 로봇이 비어 있으면 맡긴다.

        복귀는 할당 알고리즘 뒤에 본다. 그 사이 로봇이 일을 받았으면 보낸 적 없는 복귀 작업은 지운다.
        """
        commands = []
        for task_id, t in list(self.tasks.items()):
            if not t.for_robot or t.assignee or t.state != STATE_PENDING or t.info.task_type != task_type:
                continue
            robot = self.robots.get(t.for_robot)
            if robot is None:
                continue
            if robot.task_id:
                if task_type == TYPE_RETURN and t.failures == 0:
                    del self.tasks[task_id]
                continue
            if not self._free(robot, now):
                continue
            assignment_id = self._next_assignment_id
            self._next_assignment_id += 1
            t.assignee = t.for_robot
            t.assignment_id = assignment_id
            robot.task_id = task_id
            self._pending[assignment_id] = _PendingAck(t.for_robot, task_id, now)
            commands.append(Outgoing(assignment_id, t.for_robot, task_id, first=True))
        return commands

    def _station_free(self, queued: list[TaskInfo]) -> list[TaskInfo]:
        """스테이션 예약(DispatcherConfig.station_clearance_m)에 걸리지 않는 작업만 남긴다 (순서는 그대로).

        맡겨진 작업의 스테이션은 끝날 때까지 잡혀 있다. 큐 안에서 같은 스테이션을 쓰는 작업끼리는 우선순위가 높은
        하나만 이번에 남기고, 나머지는 큐에서 기다렸다가 다음 assign()에서 다시 본다.
        """
        clearance = self.config.station_clearance_m
        if clearance <= 0:
            return queued
        taken = [xy for t in self.tasks.values() if t.assignee and t.state not in FINISHED
                 for xy in _stations(t.info)]
        allowed = set()
        for info in sort_by_priority(queued):
            mine = _stations(info)
            if any(math.dist(a, b) <= clearance for a in mine for b in taken):
                continue
            allowed.add(info.task_id)
            taken += mine
        return [info for info in queued if info.task_id in allowed]

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
            # 서버가 낸 충전·복귀는 load에 넣지 않았다
            if task is None or not task.for_robot:
                robot.load = max(0, robot.load - 1)


def _stations(info: TaskInfo) -> list:
    """작업이 쓰는 스테이션 위치: 픽업(배송만)과 하역."""
    return [xy for xy in (info.pickup_xy, info.dropoff_xy) if xy is not None]
