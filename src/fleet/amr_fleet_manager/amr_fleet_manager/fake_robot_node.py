"""가짜 로봇: Nav2·BT 없이 Fleet 메시지만 주고받는다.

amr_bt가 완성되기 전에 Fleet 서버·장애 주입기·하네스 기록을 끝까지 돌려보는 용도다.
- TaskAssignment를 받아 수신 규칙대로 Ack를 보내고 (robot_sim.AssignmentFilter)
- 작업을 직선 이동과 고정 대기로 흉내 내며 TaskStatus를 보내고 (robot_sim.TaskRun)
- RobotState를 주기적으로 보낸다
- 다른 로봇이 먼저 맡은 작업은 거부하거나 멈춘다 (claims.TaskClaims, architecture.md 5-2절)
"""

from __future__ import annotations

import time

import rclpy
from amr_msgs.msg import ErrorInfo, FleetHeartbeat, RobotState, Task, TaskAssignment, TaskAssignmentAck, TaskStatus
from builtin_interfaces.msg import Time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .claims import TaskClaims
from .robot_sim import DUPLICATE, STALE, AssignmentFilter, TaskRun


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class FakeRobot(Node):

    def __init__(self):
        super().__init__('fake_robot')
        self._robot_id = self.declare_parameter('robot_id', 'amr_01').value
        start = self.declare_parameter('start_xy', [0.0, 0.0]).value
        self._speed = self.declare_parameter('speed_mps', 1.0).value
        self._dwell_s = self.declare_parameter('dwell_s', 1.0).value
        self._battery = self.declare_parameter('battery_percent', 100.0).value
        state_rate_hz = self.declare_parameter('state_rate_hz', 2.0).value
        step_hz = self.declare_parameter('step_hz', 10.0).value
        # 서버 → 로봇 토픽 앞부분. 장애 주입기를 거치면 상대 이름 'fleet'(→ /<robot_id>/fleet/...)
        prefix = self.declare_parameter('server_topic_prefix', '/fleet').value
        # 다른 로봇의 작업 주장을 세는 시간 (서버 reclaim_timeout_s와 같게). 실제 시간
        claim_timeout_s = self.declare_parameter('claim_timeout_s', 5.0).value

        self._xy = (float(start[0]), float(start[1]))
        self._step_s = 1.0 / step_hz
        self._filter = AssignmentFilter()
        self._run: TaskRun | None = None
        self._task: Task | None = None
        self._start_time = Time()
        self._claims = TaskClaims(claim_timeout_s)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._ack_pub = self.create_publisher(TaskAssignmentAck, '/fleet/task_assignment_acks', qos)
        self._state_pub = self.create_publisher(RobotState, '/fleet/robot_states', qos)
        self._status_pub = self.create_publisher(TaskStatus, '/fleet/task_status', qos)
        self.create_subscription(TaskAssignment, f'{prefix}/task_assignments', self._on_assignment, qos)
        self.create_subscription(FleetHeartbeat, f'{prefix}/heartbeat', self._on_heartbeat, qos)
        # 다른 로봇의 보고 (로봇 → 서버 방향이라 장애 주입기를 거치지 않는다, D8)
        self.create_subscription(TaskStatus, '/fleet/task_status', self._on_task_status, qos)
        self.create_subscription(RobotState, '/fleet/robot_states', self._on_robot_state, qos)

        # 이동은 시뮬레이션 시간을 따른다 (use_sim_time이면 /clock 기준)
        self.create_timer(self._step_s, self._step)
        self.create_timer(1.0 / state_rate_hz, self._publish_state)
        self.get_logger().info(
            f'가짜 로봇 {self._robot_id} 시작: xy={self._xy}, 구독 {prefix}/task_assignments')

    def _on_heartbeat(self, msg: FleetHeartbeat):
        if self._filter.on_heartbeat(msg.epoch):
            self.get_logger().info(f'서버 재시작 감지 (epoch={msg.epoch}), 명령 번호 기준 초기화')

    def _on_task_status(self, msg: TaskStatus):
        if msg.robot_id == self._robot_id:
            return
        start = stamp_to_sec(msg.start_time) or stamp_to_sec(msg.header.stamp)
        self._claims.observe(msg.task_id, msg.robot_id, msg.state, start, time.monotonic())
        if self._task is None or self._task.task_id != msg.task_id:
            return
        rival = self._claims.rival(msg.task_id, self._robot_id, stamp_to_sec(self._start_time), time.monotonic())
        if rival is not None:
            self.get_logger().warn(f'{msg.task_id}를 {rival}이 먼저 맡아 멈춘다')
            self._finish(TaskStatus.STATE_FAILED, ErrorInfo(
                code=ErrorInfo.ASSIGN_TASK_TAKEN, message=f'taken by {rival}'))

    def _on_robot_state(self, msg: RobotState):
        if msg.robot_id != self._robot_id:
            self._claims.robot_state(msg.robot_id, msg.current_task_id, time.monotonic())

    def _on_assignment(self, msg: TaskAssignment):
        if msg.robot_id != self._robot_id:
            return
        kind = self._filter.classify(msg.assignment_id)
        if kind == STALE:
            return
        if kind == DUPLICATE:
            result, error = self._filter.last_reply
        else:
            result, error = self._handle(msg)
            self._filter.record(msg.assignment_id, (result, error))
        ack = TaskAssignmentAck()
        ack.header.stamp = self.get_clock().now().to_msg()
        ack.robot_id = self._robot_id
        ack.assignment_id = msg.assignment_id
        ack.assignment_stamp = msg.header.stamp
        ack.result = result
        ack.error = error
        self._ack_pub.publish(ack)

    def _handle(self, msg: TaskAssignment) -> tuple[int, ErrorInfo]:
        if msg.action == TaskAssignment.ACTION_CANCEL:
            if self._task is not None and self._task.task_id == msg.task.task_id:
                self._finish(TaskStatus.STATE_FAILED, ErrorInfo(code=ErrorInfo.ASSIGN_CANCELED, message='canceled'))
            return TaskAssignmentAck.RESULT_ACCEPTED, ErrorInfo()
        task_id = msg.task.task_id
        if self._task is not None and self._task.task_id == task_id:
            # 같은 작업을 다른 출처(서버·경매)에서 또 받았다. 다시 시작하지 않는다
            return TaskAssignmentAck.RESULT_ACCEPTED, ErrorInfo()
        rival = self._claims.rival(task_id, self._robot_id, None, time.monotonic())
        if rival is not None or self._claims.completed(task_id, self._robot_id):
            return TaskAssignmentAck.RESULT_REJECTED, ErrorInfo(
                code=ErrorInfo.ASSIGN_TASK_TAKEN, message=f'taken by {rival or self._robot_id}')
        if self._run is not None:
            return TaskAssignmentAck.RESULT_REJECTED, ErrorInfo(
                code=ErrorInfo.ASSIGN_ALREADY_BUSY, message=f'busy with {self._task.task_id}')

        task = msg.task
        pickup = None
        if task.task_type == Task.TYPE_DELIVERY:
            pickup = (task.pickup_pose.position.x, task.pickup_pose.position.y)
        dropoff = (task.dropoff_pose.position.x, task.dropoff_pose.position.y)
        self._task = task
        self._run = TaskRun(self._xy, pickup, dropoff, self._speed, self._dwell_s)
        self._start_time = self.get_clock().now().to_msg()
        self._publish_status(TaskStatus.STATE_PENDING, '')
        self._publish_status(TaskStatus.STATE_IN_PROGRESS, self._run.phase)
        return TaskAssignmentAck.RESULT_ACCEPTED, ErrorInfo()

    def _step(self):
        if self._run is None:
            return
        changed = self._run.step(self._step_s)
        self._xy = self._run.xy
        if self._run.done:
            self._finish(TaskStatus.STATE_COMPLETED, ErrorInfo())
        elif changed:
            self._publish_status(TaskStatus.STATE_IN_PROGRESS, self._run.phase)

    def _finish(self, state: int, error: ErrorInfo):
        self._publish_status(state, '', error, end=True)
        self._run = None
        self._task = None

    def _publish_status(self, state: int, phase: str, error: ErrorInfo | None = None, end: bool = False):
        msg = TaskStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = self._task.task_id
        msg.robot_id = self._robot_id
        msg.state = state
        msg.phase = phase
        msg.error = error or ErrorInfo()
        msg.start_time = self._start_time
        if end:
            msg.end_time = msg.header.stamp
        msg.distance_traveled = float(self._run.distance)
        self._status_pub.publish(msg)
        # 내가 끝낸 작업을 다른 출처에서 또 받으면 거부한다
        self._claims.observe(msg.task_id, self._robot_id, state, stamp_to_sec(msg.start_time), time.monotonic())

    def _publish_state(self):
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.robot_id = self._robot_id
        msg.pose.position.x, msg.pose.position.y = self._xy
        msg.pose.orientation.w = 1.0
        if self._run is not None:
            msg.velocity.linear.x = float(abs(complex(*self._run.velocity)))
            msg.mode = RobotState.MODE_BUSY
            msg.current_task_id = self._task.task_id
            msg.item_type = self._task.item_type
        else:
            msg.mode = RobotState.MODE_IDLE
        msg.battery_percent = float(self._battery)
        msg.last_assignment_id = self._filter.last_id
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
