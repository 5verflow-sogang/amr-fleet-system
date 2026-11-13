"""중앙집중 Fleet 서버 (최소판): 작업 접수 → 할당 → Ack·재전송, heartbeat·작업표 방송.

구역 허가(ZoneRequest/ZoneGrant)는 정원·lease·작업 우선순위 대기열, 교착이면 우선순위 낮은 로봇의 허가를 회수한다.
흐름은 architecture.md 5-1절.
자동 충전(charging.enabled): 배터리 낮은 로봇에게 충전 작업을 직접 내고, 최소 운영 대수를 지키며 동시 충전을 묶는다.
"""

from __future__ import annotations

import fnmatch
import uuid

import rclpy
from builtin_interfaces.msg import Time
from amr_fleet_core.assignment import RobotInfo, TaskInfo, has_pickup
from amr_fleet_core.charging import ChargeConfig, Station
from amr_fleet_core.energy import EnergyModel
from amr_fleet_core.traffic import Decision, ZoneManager
from amr_msgs.msg import (ErrorInfo, FleetHeartbeat, FleetTaskTable, RobotFleetMode, RobotState, Task,
                          TaskAssignment, TaskAssignmentAck, TaskRecord, TaskStatus, ZoneCongestionArray, ZoneGrant,
                          ZoneRequest)
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .dispatcher import TYPE_CHARGE, ChargeTask, Dispatcher, DispatcherConfig, Outgoing
from .records import AssignmentLog, load_zones, next_epoch
from .task_gen import Pose2D, load_stations, world_to_map, yaw_to_quaternion


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def sec_to_stamp(seconds: float) -> Time:
    sec, nanosec = divmod(int(round(seconds * 1e9)), 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def task_info(msg: Task) -> TaskInfo:
    """Task 메시지 → amr_fleet_core 입력. DELIVERY·COLLECT만 픽업이 있다 (Task.msg 주석)."""
    pickup = None
    if has_pickup(msg.task_type):
        pickup = (msg.pickup_pose.position.x, msg.pickup_pose.position.y)
    deadline = stamp_to_sec(msg.deadline)
    return TaskInfo(
        task_id=msg.task_id,
        task_type=msg.task_type,
        dropoff_xy=(msg.dropoff_pose.position.x, msg.dropoff_pose.position.y),
        priority=msg.priority,
        # 기록용. aging은 이 값이 아니라 dispatcher.add_task의 received_at(서버 시계)으로 잰다 - 보낸 쪽
        # (대시보드 등) 시계가 서버와 다르면 msg.created_time만으로는 대기 시간을 믿을 수 없다
        created=stamp_to_sec(msg.created_time),
        pickup_xy=pickup,
        deadline=deadline if deadline > 0 else None,
    )


def pick_stations(stations: dict[str, Pose2D], pattern: str) -> dict[str, Pose2D]:
    """이름이 pattern(* 패턴)에 맞는 스테이션. 패턴이 비면 없음."""
    return {name: pose for name, pose in sorted(stations.items()) if pattern and fnmatch.fnmatchcase(name, pattern)}


class FleetManager(Node):

    def __init__(self):
        super().__init__('fleet_manager')
        self._server_id = self.declare_parameter('server_id', 'fleet_manager').value
        # 자동 충전: 충전기·대기 위치는 작업 생성기와 같은 스테이션 파일(world 좌표)에서 이름으로 고른다
        charging = self.declare_parameter('charging.enabled', False).value
        stations = {}
        stations_file = self.declare_parameter('charging.stations_file', '').value
        origin = list(self.declare_parameter('charging.map_origin_in_world', [-0.0439, 1.4712, -0.00088]).value)
        if stations_file:
            stations = {name: world_to_map(pose, origin) for name, pose in load_stations(stations_file).items()}
        chargers = pick_stations(stations, self.declare_parameter('charging.charger_pattern', 'charger_*').value)
        waits = pick_stations(stations, self.declare_parameter('charging.wait_pattern', 'wait_*').value)
        energy_check = self.declare_parameter('energy.enabled', False).value
        interrupt = self.declare_parameter('interrupt.enabled', False).value
        if (charging or energy_check or interrupt) and not chargers:
            raise ValueError('charging.enabled·energy.enabled·interrupt.enabled인데 충전기가 없다 '
                             f'(charging.stations_file={stations_file!r})')
        # Task 메시지에 넣을 자세 (map 좌표)
        self._station_poses = {**chargers, **waits}
        # 피크 시간대 [시작, 끝, 시작, 끝, ...] (서버 시작부터 노드 시계 s). 작업 생성기 profile과 같은 시각으로 맞춘다
        flat = [float(v) for v in self.declare_parameter('charging.peaks', [0.0]).value]
        flat = [] if flat == [0.0] else flat
        if len(flat) % 2:
            raise ValueError(f'charging.peaks는 [시작, 끝, ...] 짝수 개: {flat}')
        peaks = tuple(zip(flat[0::2], flat[1::2]))
        self._t0 = self._now_sec()
        self._dispatcher = Dispatcher(DispatcherConfig(
            algorithm=self.declare_parameter('assignment.algorithm', 'nearest').value,
            min_battery_percent=self.declare_parameter('assignment.min_battery_percent', 20.0).value,
            robot_timeout_s=self.declare_parameter('robot_timeout_s', 3.0).value,
            resend_interval_s=self.declare_parameter('resend.interval_s', 1.0).value,
            max_resends=self.declare_parameter('resend.max_resends', 5).value,
            failed_retries=self.declare_parameter('assignment.failed_retries', 1).value,
            reclaim_timeout_s=self.declare_parameter('assignment.reclaim_timeout_s', 5.0).value,
            station_clearance_m=self.declare_parameter('assignment.station_clearance_m', 1.0).value,
            cost_fn=self.declare_parameter('assignment.cost_fn', 'euclidean').value,
            congestion_weight=self.declare_parameter('assignment.congestion_weight', 3.0).value,
            congestion_timeout_s=self.declare_parameter('assignment.congestion_timeout_s', 3.0).value,
            charging_enabled=charging,
            charge=ChargeConfig(
                low_percent=self.declare_parameter('charging.low_percent', 30.0).value,
                min_in_service=self.declare_parameter('charging.min_in_service', 0).value,
                peaks=peaks,
                pre_peak_s=self.declare_parameter('charging.pre_peak_s', 0.0).value,
                pre_peak_percent=self.declare_parameter('charging.pre_peak_percent', 80.0).value,
                peak_max_charging=self.declare_parameter('charging.peak_max_charging', 0).value,
                peak_target_percent=self.declare_parameter('charging.peak_target_percent', 100.0).value,
            ),
            chargers=tuple(Station(name, pose[:2]) for name, pose in chargers.items()),
            wait_stations=tuple(Station(name, pose[:2]) for name, pose in waits.items()),
            energy_check=energy_check,
            energy=EnergyModel(
                move_per_m=self.declare_parameter('energy.move_per_m', 0.05).value,
                idle_per_s=self.declare_parameter('energy.idle_per_s', 0.002).value,
                speed_mps=self.declare_parameter('energy.speed_mps', 0.5).value,
                path_factor=self.declare_parameter('energy.path_factor', 1.3).value,
                reserve_percent=self.declare_parameter('energy.reserve_percent', 5.0).value,
                charge_per_s=self.declare_parameter('energy.charge_per_s', 0.5).value,
                charge_target_percent=self.declare_parameter('energy.charge_target_percent', 100.0).value,
            ),
            battery_weight=self.declare_parameter('assignment.battery_weight', 0.0).value,
            aging_rate=self.declare_parameter('assignment.aging_rate', 0.0).value,
            interrupt_enabled=interrupt,
            interrupt_reserve_percent=self.declare_parameter('interrupt.reserve_percent', 2.0).value,
        ))
        self._charge_waiting: list[str] = []
        heartbeat_period_s = self.declare_parameter('heartbeat_period_s', 0.2).value
        task_table_period_s = self.declare_parameter('task_table_period_s', 1.0).value
        tick_s = self.declare_parameter('tick_s', 0.05).value
        self._epoch = next_epoch(self.declare_parameter('epoch_file', '').value)
        self._zones = ZoneManager(
            load_zones(self.declare_parameter('traffic.zones_file', '').value),
            lease_s=self.declare_parameter('traffic.lease_s', 5.0).value,
            release_timeout_s=self.declare_parameter('traffic.release_timeout_s', 30.0).value,
            deadlock_strategy=self.declare_parameter('traffic.deadlock_strategy', 'revoke').value)
        csv_path = self.declare_parameter('assignments_csv', '').value
        self._log = AssignmentLog(csv_path) if csv_path else None

        self._heartbeat_seq = 0
        self._table_seq = 0
        # 재전송할 때 header.stamp를 바꾸지 않도록 첫 발송 메시지를 그대로 둔다
        self._sent: dict[int, TaskAssignment] = {}

        # TODO(하연): api.md 6절 QoS 기본값이 정해지면 토픽별로 맞춘다
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._assignment_pub = self.create_publisher(TaskAssignment, '/fleet/task_assignments', qos)
        self._heartbeat_pub = self.create_publisher(FleetHeartbeat, '/fleet/heartbeat', qos)
        self._table_pub = self.create_publisher(FleetTaskTable, '/fleet/task_table', qos)
        self._grant_pub = self.create_publisher(ZoneGrant, '/fleet/zone_grants', qos)
        self.create_subscription(Task, '/fleet/task_requests', self._on_task, qos)
        self.create_subscription(RobotState, '/fleet/robot_states', self._on_robot_state, qos)
        self.create_subscription(TaskAssignmentAck, '/fleet/task_assignment_acks', self._on_ack, qos)
        self.create_subscription(TaskStatus, '/fleet/task_status', self._on_status, qos)
        self.create_subscription(ZoneRequest, '/fleet/zone_requests', self._on_zone_request, qos)
        # 하이브리드: 서버와 끊겨 분산 모드로 도는 로봇 (amr_fleet_decentralized, architecture.md 5-2절)
        self.create_subscription(RobotFleetMode, '/fleet/robot_modes', self._on_robot_mode, qos)
        # 혼잡도 반영 할당 (식당 시연 1-1, api.md 4-5절). 끈 쪽은 구독하지 않아 켬/끔 조건이 섞이지 않는다
        if self._dispatcher.config.cost_fn == 'congestion':
            self.create_subscription(ZoneCongestionArray, '/fleet/congestion', self._on_congestion, qos)

        # 재전송·heartbeat 주기는 네트워크 쪽 시간이라 use_sim_time과 상관없이 실제 시간으로 잰다
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(heartbeat_period_s, self._publish_heartbeat, clock=steady)
        self.create_timer(task_table_period_s, self._publish_table, clock=steady)
        self.create_timer(tick_s, self._resend, clock=steady)
        self.create_timer(tick_s, self._expire_zones, clock=steady)
        self.get_logger().info(
            f'Fleet 서버 시작: epoch={self._epoch}, algorithm={self._dispatcher.config.algorithm}, '
            f'resend={self._dispatcher.config.resend_interval_s}s x {self._dispatcher.config.max_resends}, '
            f'aging_rate={self._dispatcher.config.aging_rate}/s, zones={self._zones.zone_ids}')
        if charging:
            c = self._dispatcher.config.charge
            self.get_logger().info(
                f'자동 충전: {c.low_percent:g} % 밑이면 충전, 최소 운영 {c.min_in_service}대, '
                f'충전기 {list(chargers)}, 충전 뒤 대기 위치 {list(waits)}')
            if c.peaks:
                self.get_logger().info(
                    f'피크 대비: 피크 {list(c.peaks)} s, {c.pre_peak_s:g} s 전부터 {c.pre_peak_percent:g} % 밑 미리 충전, '
                    f'피크 중 동시 충전 ≤ {c.peak_max_charging or "제한 없음"}, {c.peak_target_percent:g} %까지만')
        if energy_check:
            e = self._dispatcher.config.energy
            self.get_logger().info(
                f'에너지 예측 배정: 작업 + 가장 가까운 충전기까지 {e.per_m:.3f} %/m × 경로 {e.path_factor:g}배 '
                f'+ 여유 {e.reserve_percent:g} %'
                + (' + 충전 차례 대기 정지 소모' if charging else '')
                + f', 배터리 가중치 {self._dispatcher.config.battery_weight:g}')
        if interrupt:
            d = self._dispatcher.config
            self.get_logger().info(
                f'배터리 부족 인터럽트: 단계 {list(d.interruptible_phases)}에서 남은 경로 + 충전기까지 '
                f'(여유 {d.interrupt_reserve_percent:g} %) 모자라면 작업을 되돌리고 바로 충전')

    # ---- 구독 ----

    def _on_task(self, msg: Task):
        if not msg.task_id:
            msg.task_id = f'task_{uuid.uuid4().hex[:12]}'
        # aging은 받은 시각(서버 시계, _now_sec)으로 잰다 - msg.created_time은 보낸 쪽(대시보드 등) 시계라 다를 수 있다
        if not self._dispatcher.add_task(task_info(msg), msg, received_at=self._now_sec()):
            self.get_logger().warn(f'이미 받은 작업이라 무시: {msg.task_id}')
            return
        self._assign()

    def _on_robot_state(self, msg: RobotState):
        info = RobotInfo(
            robot_id=msg.robot_id,
            xy=(msg.pose.position.x, msg.pose.position.y),
            battery_percent=msg.battery_percent,
            mode=msg.mode,
        )
        self._dispatcher.update_robot(info, msg.current_task_id, self._now_sec())
        self._assign()

    def _on_ack(self, msg: TaskAssignmentAck):
        accepted = msg.result == TaskAssignmentAck.RESULT_ACCEPTED
        if self._dispatcher.on_ack(msg.robot_id, msg.assignment_id, accepted, msg.error.code,
                                   self._now_sec()) and not accepted:
            self.get_logger().warn(
                f'{msg.robot_id}가 할당 {msg.assignment_id}를 거부: '
                f'code={msg.error.code} {msg.error.message}')
            self._assign()

    def _on_status(self, msg: TaskStatus):
        task = self._dispatcher.tasks.get(msg.task_id)
        before = task.assignee if task is not None else ''
        # 수락 시각. 비어 있으면 보고 시각으로 본다 (TaskStatus.msg)
        start = stamp_to_sec(msg.start_time) or stamp_to_sec(msg.header.stamp)
        interrupting = msg.task_id in self._dispatcher.interrupting()
        if not self._dispatcher.on_status(msg.task_id, msg.robot_id, msg.state, msg.error.code,
                                          start, self._now_sec(), phase=msg.phase):
            return
        if interrupting and msg.state == TaskStatus.STATE_FAILED and msg.error.code == ErrorInfo.ASSIGN_CANCELED:
            self.get_logger().warn(
                f'{msg.robot_id}가 {msg.task_id} 인터럽트 취소 완료: 작업은 다시 할당, 로봇은 바로 충전')
            self._assign()
            return
        handed_over = bool(task.assignee) and task.assignee != before
        if handed_over:
            # 먼저 수락한 쪽이 주인 (architecture.md 5-2절)
            self.get_logger().warn(
                f'{msg.task_id}: {task.assignee}가 먼저 맡아 장부를 넘김 (원래 {before or "없음"})')
        elif msg.state == TaskStatus.STATE_FAILED:
            if task.state == TaskStatus.STATE_PENDING:
                self.get_logger().warn(
                    f'{msg.robot_id}가 {msg.task_id} 실패 보고 (code={msg.error.code} {msg.error.message}), '
                    f'다시 할당 ({task.failures}/{self._dispatcher.config.failed_retries})')
            elif msg.error.code == ErrorInfo.ASSIGN_CANCELED:
                self.get_logger().info(f'{msg.robot_id}가 {msg.task_id} 취소 완료 보고, 다시 할당하지 않음')
            else:
                self.get_logger().warn(f'{msg.task_id} 실패로 남김 (재할당 {task.failures}회 뒤)')
        if handed_over or msg.state in (TaskStatus.STATE_COMPLETED, TaskStatus.STATE_FAILED):
            self._assign()

    def _on_congestion(self, msg: ZoneCongestionArray):
        self._dispatcher.set_congestion([(tuple(z.area), z.value) for z in msg.zones], self._now_sec())

    def _on_robot_mode(self, msg: RobotFleetMode):
        decentral = msg.mode == FleetHeartbeat.MODE_DECENTRALIZED
        robot = self._dispatcher.robots.get(msg.robot_id)
        if robot is None or robot.decentral == decentral:
            return
        released = self._dispatcher.set_robot_mode(msg.robot_id, decentral)
        self.get_logger().warn(
            f'{msg.robot_id} {"분산" if decentral else "중앙"} 모드 (전환 {msg.transitions}회). '
            f'{"맡기지 않음" if decentral else "다시 맡김"}' + (f', 확인 전 명령 거둠: {released}' if released else ''))
        self._assign()

    def _on_zone_request(self, msg: ZoneRequest):
        # lease_expiry는 로봇이 자기 노드 시계와 비교하므로 서버도 노드 시계(use_sim_time이면 시뮬레이션 시간)로 잰다
        now = self._now_sec()
        if msg.type == ZoneRequest.TYPE_RELEASE:
            self._publish_grants(self._zones.release(msg.robot_id, msg.zone_id, now))
        else:
            # ZoneRequest에는 우선순위가 없어 그 로봇이 맡은 작업의 priority를 쓴다 (교차로 우선순위)
            priority = self._dispatcher.robot_priority(msg.robot_id)
            self._publish_grants(self._zones.request(msg.robot_id, msg.zone_id, msg.request_id, now, priority))

    # ---- 발행 ----

    def _expire_zones(self):
        now = self._now_sec()
        self._publish_grants(self._zones.expire(now))
        revoked = self._zones.resolve_deadlocks(now)
        for d in revoked:
            self.get_logger().warn(
                f'교착 해소: {d.robot_id}의 {d.zone_id} 허가 회수, {d.holder_robot_id}에게 양보')
        self._publish_grants(revoked)

    def _publish_grants(self, decisions: list[Decision]):
        for d in decisions:
            msg = ZoneGrant()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.request_id = d.request_id
            msg.robot_id = d.robot_id
            msg.zone_id = d.zone_id
            msg.grant = d.grant
            if d.lease_expiry is not None:
                msg.lease_expiry = sec_to_stamp(d.lease_expiry)
            msg.reason = d.reason
            msg.holder_robot_id = d.holder_robot_id
            self._grant_pub.publish(msg)

    def _now_sec(self) -> float:
        """노드 시계(초). use_sim_time:=true면 시뮬레이션 시간이다.

        배정·Ack 재전송·로봇 끊김 회수·혼잡도 만료를 모두 이 시계로 잰다. 예전엔 실제 시간(time.monotonic)이라,
        카메라를 켜 RTF 0.08로 느려진 시연 녹화에서 시뮬레이션 시간 2 Hz RobotState가 실제 6 s마다 와
        5 s 끊김 회수가 계속 일어났다(같은 작업이 여러 로봇에 배정, 혼잡도 3 s 만료로 직선 거리 배정, 11/6).
        use_sim_time:=false(가짜 로봇)면 지금처럼 실제 시간이다.
        """
        return self.get_clock().now().nanoseconds * 1e-9

    def _plan_charging(self):
        now = self._now_sec()
        for order in self._dispatcher.plan_charging(now, now - self._t0):
            self._dispatcher.tasks[order.task_id].payload = self._charge_task(order)
            kind = '충전' if order.task_type == TYPE_CHARGE else '충전 뒤 복귀'
            self.get_logger().info(f'{kind}: {order.robot_id} → {order.station} ({order.task_id})')
        waiting = self._dispatcher.charge_waiting
        if waiting != self._charge_waiting:
            self._charge_waiting = waiting
            if waiting:
                self.get_logger().info(
                    f'충전 차례 기다림: {waiting} (충전 중 {sorted(self._dispatcher.charging_robots())})')
        for command in self._dispatcher.stop_charging(now):
            self._send_cancel(command)
        for command in self._dispatcher.interrupt_low_battery(now):
            self._send_cancel(command, interrupt=True)

    def _send_cancel(self, command: Outgoing, interrupt: bool = False):
        """작업 취소 (TaskAssignment ACTION_CANCEL, 재전송은 같은 번호).

        피크 중 충전 목표에 닿은 충전 작업, 또는 배터리 부족 인터럽트(3단계)로 놓게 할 픽업 전 배송 작업.
        """
        msg = TaskAssignment()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.assignment_id = command.assignment_id
        msg.robot_id = command.robot_id
        msg.action = TaskAssignment.ACTION_CANCEL
        msg.task = self._dispatcher.tasks[command.task_id].payload
        if command.first and interrupt:
            robot = self._dispatcher.robots[command.robot_id]
            need = self._dispatcher.interrupt_need(command.task_id)
            self.get_logger().warn(
                f'배터리 부족 인터럽트: {command.robot_id} {robot.info.battery_percent:.1f} % < 필요 '
                f'{need:.1f} %, 픽업 전 {command.task_id} 취소')
        elif command.first:
            self.get_logger().info(f'피크 중 짧은 충전: {command.robot_id} 충전 끝냄 ({command.task_id})')
        self._assignment_pub.publish(msg)

    def _charge_task(self, order: ChargeTask) -> Task:
        x, y, yaw = self._station_poses[order.station]
        msg = Task()
        msg.task_id = order.task_id
        msg.task_type = order.task_type
        msg.item_type = Task.ITEM_NONE
        msg.dropoff_station_id = order.station
        msg.dropoff_pose.position.x, msg.dropoff_pose.position.y = x, y
        msg.dropoff_pose.orientation.z, msg.dropoff_pose.orientation.w = yaw_to_quaternion(yaw)
        msg.priority = self._dispatcher.tasks[order.task_id].info.priority
        msg.created_time = self.get_clock().now().to_msg()
        return msg

    def _assign(self):
        self._plan_charging()
        # aging의 대기 시간도 같은 노드 시계로 잰다 (#322 리뷰, _now_sec 주석)
        now = self._now_sec()
        commands, rows = self._dispatcher.assign(now, aging_now=now)
        if rows and self._log is not None:
            self._log.write(stamp_to_sec(self.get_clock().now().to_msg()), rows)
        self._send(commands)

    def _resend(self):
        now = self._now_sec()
        reclaimed = self._dispatcher.reclaim_lost(now)
        for task_id, robot_id in reclaimed:
            self.get_logger().warn(
                f'{robot_id}의 RobotState가 {self._dispatcher.config.reclaim_timeout_s:g}초 끊김, '
                f'작업 회수해 다시 할당: {task_id}')
        commands, given_up = self._dispatcher.due_resends(now)
        for task_id in given_up:
            self.get_logger().warn(f'Ack 재전송 한도 초과, 작업을 큐로 되돌림: {task_id}')
        self._send(commands)
        pending = self._dispatcher.pending_ids()
        for assignment_id in list(self._sent):
            if assignment_id not in pending:
                del self._sent[assignment_id]
        if given_up or reclaimed:
            self._assign()

    def _send(self, commands: list[Outgoing]):
        for command in commands:
            msg = self._sent.get(command.assignment_id)
            if msg is None:
                msg = TaskAssignment()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.assignment_id = command.assignment_id
                msg.robot_id = command.robot_id
                msg.action = TaskAssignment.ACTION_ASSIGN
                msg.task = self._dispatcher.tasks[command.task_id].payload
                self._sent[command.assignment_id] = msg
                self.get_logger().info(
                    f'할당 {command.assignment_id}: {command.task_id} → {command.robot_id}')
            self._assignment_pub.publish(msg)

    def _publish_heartbeat(self):
        msg = FleetHeartbeat()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.server_id = self._server_id
        msg.epoch = self._epoch
        msg.seq = self._heartbeat_seq
        msg.mode = FleetHeartbeat.MODE_CENTRALIZED
        self._heartbeat_seq += 1
        self._heartbeat_pub.publish(msg)

    def _publish_table(self):
        msg = FleetTaskTable()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.server_id = self._server_id
        msg.epoch = self._epoch
        msg.seq = self._table_seq
        self._table_seq += 1
        msg.records = [
            TaskRecord(task=t.payload, assignee_id=t.assignee, assignment_id=t.assignment_id, state=t.state)
            for t in self._dispatcher.tasks.values()
        ]
        self._table_pub.publish(msg)

    def destroy_node(self):
        if self._log is not None:
            self._log.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FleetManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C나 launch 종료(SIGINT)는 정상 종료로 본다
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
