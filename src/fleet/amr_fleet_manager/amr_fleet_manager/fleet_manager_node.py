"""중앙집중 Fleet 서버 (최소판): 작업 접수 → 할당 → Ack·재전송, heartbeat·작업표 방송.

구역 허가(ZoneRequest/ZoneGrant)는 정원·lease·작업 우선순위 대기열, 교착이면 우선순위 낮은 로봇의 허가를 회수한다.
흐름은 architecture.md 5-1절.
"""

from __future__ import annotations

import time
import uuid

import rclpy
from builtin_interfaces.msg import Time
from amr_fleet_core.assignment import RobotInfo, TaskInfo
from amr_fleet_core.traffic import Decision, ZoneManager
from amr_msgs.msg import (FleetHeartbeat, FleetTaskTable, RobotState, Task, TaskAssignment,
                          TaskAssignmentAck, TaskRecord, TaskStatus, ZoneGrant, ZoneRequest)
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .dispatcher import Dispatcher, DispatcherConfig, Outgoing
from .records import AssignmentLog, load_zones, next_epoch


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def sec_to_stamp(seconds: float) -> Time:
    sec, nanosec = divmod(int(round(seconds * 1e9)), 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def task_info(msg: Task) -> TaskInfo:
    """Task 메시지 → amr_fleet_core 입력. CHARGE·RETURN은 픽업이 없다 (Task.msg 주석)."""
    pickup = None
    if msg.task_type == Task.TYPE_DELIVERY:
        pickup = (msg.pickup_pose.position.x, msg.pickup_pose.position.y)
    deadline = stamp_to_sec(msg.deadline)
    return TaskInfo(
        task_id=msg.task_id,
        task_type=msg.task_type,
        dropoff_xy=(msg.dropoff_pose.position.x, msg.dropoff_pose.position.y),
        priority=msg.priority,
        created=stamp_to_sec(msg.created_time),
        pickup_xy=pickup,
        deadline=deadline if deadline > 0 else None,
    )


class FleetManager(Node):

    def __init__(self):
        super().__init__('fleet_manager')
        self._server_id = self.declare_parameter('server_id', 'fleet_manager').value
        self._dispatcher = Dispatcher(DispatcherConfig(
            algorithm=self.declare_parameter('assignment.algorithm', 'nearest').value,
            min_battery_percent=self.declare_parameter('assignment.min_battery_percent', 20.0).value,
            robot_timeout_s=self.declare_parameter('robot_timeout_s', 3.0).value,
            resend_interval_s=self.declare_parameter('resend.interval_s', 1.0).value,
            max_resends=self.declare_parameter('resend.max_resends', 5).value,
        ))
        heartbeat_period_s = self.declare_parameter('heartbeat_period_s', 0.2).value
        task_table_period_s = self.declare_parameter('task_table_period_s', 1.0).value
        tick_s = self.declare_parameter('tick_s', 0.05).value
        self._epoch = next_epoch(self.declare_parameter('epoch_file', '').value)
        self._zones = ZoneManager(
            load_zones(self.declare_parameter('traffic.zones_file', '').value),
            lease_s=self.declare_parameter('traffic.lease_s', 5.0).value,
            release_timeout_s=self.declare_parameter('traffic.release_timeout_s', 30.0).value)
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

        # 재전송·heartbeat 주기는 네트워크 쪽 시간이라 use_sim_time과 상관없이 실제 시간으로 잰다
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(heartbeat_period_s, self._publish_heartbeat, clock=steady)
        self.create_timer(task_table_period_s, self._publish_table, clock=steady)
        self.create_timer(tick_s, self._resend, clock=steady)
        self.create_timer(tick_s, self._expire_zones, clock=steady)
        self.get_logger().info(
            f'Fleet 서버 시작: epoch={self._epoch}, algorithm={self._dispatcher.config.algorithm}, '
            f'resend={self._dispatcher.config.resend_interval_s}s x {self._dispatcher.config.max_resends}, '
            f'zones={self._zones.zone_ids}')

    # ---- 구독 ----

    def _on_task(self, msg: Task):
        if not msg.task_id:
            msg.task_id = f'task_{uuid.uuid4().hex[:12]}'
        if not self._dispatcher.add_task(task_info(msg), msg):
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
        self._dispatcher.update_robot(info, msg.current_task_id, time.monotonic())
        self._assign()

    def _on_ack(self, msg: TaskAssignmentAck):
        accepted = msg.result == TaskAssignmentAck.RESULT_ACCEPTED
        if self._dispatcher.on_ack(msg.robot_id, msg.assignment_id, accepted) and not accepted:
            self.get_logger().warn(
                f'{msg.robot_id}가 할당 {msg.assignment_id}를 거부: '
                f'code={msg.error.code} {msg.error.message}')
            self._assign()

    def _on_status(self, msg: TaskStatus):
        if self._dispatcher.on_status(msg.task_id, msg.robot_id, msg.state) and \
                msg.state in (TaskStatus.STATE_COMPLETED, TaskStatus.STATE_FAILED):
            # TODO(하연): FAILED 작업 재할당 정책. 지금은 실패로 남긴다
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
        return self.get_clock().now().nanoseconds * 1e-9


    def _assign(self):
        commands, rows = self._dispatcher.assign(time.monotonic())
        if rows and self._log is not None:
            self._log.write(stamp_to_sec(self.get_clock().now().to_msg()), rows)
        self._send(commands)

    def _resend(self):
        commands, given_up = self._dispatcher.due_resends(time.monotonic())
        for task_id in given_up:
            self.get_logger().warn(f'Ack 재전송 한도 초과, 작업을 큐로 되돌림: {task_id}')
        self._send(commands)
        pending = self._dispatcher.pending_ids()
        for assignment_id in list(self._sent):
            if assignment_id not in pending:
                del self._sent[assignment_id]
        if given_up:
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
