"""amr_bt_node를 가짜 Nav2(navigate_to_pose)·도킹(docking/dock) 액션 서버와 함께 띄워 한 사이클을 확인한다 (launch_testing).

작업 수신 → 이동 → 도킹 → 싣기 → 이동 → 도킹 → 내리기 → COMPLETED, 도킹 3회 실패 → FAILED(900번대),
zone_client/state가 waiting이면 Nav2 목표 취소 → 진행 상태로 돌아오면 다시 보냄. 서버는 이 테스트 노드가 흉내 낸다.
"""

import itertools
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from amr_msgs.action import Dock
from amr_msgs.msg import ErrorInfo, RobotState, Task, TaskAssignment, TaskAssignmentAck, TaskStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

ROBOT = 'amr_01'
ASSIGNMENT_IDS = itertools.count(1)
BAD_STATION = 'bad_dock'   # 이 스테이션은 도킹이 항상 실패한다 (FAILED_TO_DETECT_DOCK)


@pytest.mark.launch_test
def generate_test_description():
    bt = launch_ros.actions.Node(
        package='amr_bt', executable='amr_bt_node', namespace=ROBOT,
        parameters=[{
            'use_sim_time': False, 'use_nav2': True, 'use_docking': True,
            'dwell_s': 0.3, 'server_wait_s': 5.0, 'resume_hold_s': 0.3,
            'kidnap_state_stale_s': 3.0,
        }])
    return launch.LaunchDescription([bt, launch_testing.actions.ReadyToTest()])


class FakeServers:
    """가짜 Nav2·도킹 액션 서버. 받은 목표와 취소를 기록한다."""

    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.nav_goals = []       # (x, y)
        self.nav_canceled = 0
        self.dock_goals = []      # station_id
        self.nav_duration = 0.5
        group = ReentrantCallbackGroup()
        ActionServer(
            node, NavigateToPose, f'/{ROBOT}/navigate_to_pose', self.execute_nav,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        ActionServer(
            node, Dock, f'/{ROBOT}/docking/dock', self.execute_dock,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)

    def execute_nav(self, goal_handle):
        p = goal_handle.request.pose.pose.position
        with self.lock:
            self.nav_goals.append((round(p.x, 3), round(p.y, 3)))
            duration = self.nav_duration
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if goal_handle.is_cancel_requested:
                with self.lock:
                    self.nav_canceled += 1
                goal_handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.02)
        goal_handle.succeed()
        return NavigateToPose.Result()

    def execute_dock(self, goal_handle):
        station = goal_handle.request.station_id
        with self.lock:
            self.dock_goals.append(station)
            attempt = self.dock_goals.count(station)
        time.sleep(0.2)
        result = Dock.Result()
        result.attempts = attempt
        if station == BAD_STATION:
            result.success = False
            result.error_code = Dock.Result.FAILED_TO_DETECT_DOCK
            result.message = 'no marker'
            goal_handle.abort()
        else:
            result.success = True
            goal_handle.succeed()
        return result


class TestBtActions(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_bt_actions')
        cls.servers = FakeServers(cls.node)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        cls.statuses = []
        cls.node.create_subscription(TaskStatus, '/fleet/task_status', cls.statuses.append, qos)
        cls.acks = []
        cls.node.create_subscription(TaskAssignmentAck, '/fleet/task_assignment_acks', cls.acks.append, qos)
        cls.states = []
        cls.node.create_subscription(RobotState, '/fleet/robot_states', cls.states.append, qos)
        cls.assign_pub = cls.node.create_publisher(TaskAssignment, '/fleet/task_assignments', qos)
        cls.zone_pub = cls.node.create_publisher(String, f'/{ROBOT}/zone_client/state', 10)
        cls.kidnap_pub = cls.node.create_publisher(String, f'/{ROBOT}/kidnap_recovery/state', 10)
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.node)
        cls.spin = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_until(self, condition, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if condition():
                return True
            time.sleep(0.05)
        return condition()

    def assign(self, task_id, task_type, pickup='', dropoff='', pickup_xy=(0.0, 0.0),
               dropoff_xy=(0.0, 0.0)):
        task = Task()
        task.task_id = task_id
        task.task_type = task_type
        task.pickup_station_id = pickup
        task.dropoff_station_id = dropoff
        task.pickup_pose.position.x, task.pickup_pose.position.y = pickup_xy
        task.dropoff_pose.position.x, task.dropoff_pose.position.y = dropoff_xy
        task.pickup_pose.orientation.w = task.dropoff_pose.orientation.w = 1.0
        msg = TaskAssignment()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.assignment_id = next(ASSIGNMENT_IDS)
        self.last_assignment_id = msg.assignment_id
        msg.robot_id = ROBOT
        msg.action = TaskAssignment.ACTION_ASSIGN
        msg.task = task
        # 노드가 구독을 붙일 때까지 기다린다
        self.assertTrue(self.wait_until(lambda: self.assign_pub.get_subscription_count() > 0, 20.0))
        # 구독이 보여도 노드 쪽 연결(discovery)이 덜 끝나면 첫 메시지가 버려진다 (volatile).
        # 서버처럼 Ack가 올 때까지 같은 assignment_id로 다시 보낸다. 노드는 중복이면 같은 Ack만 다시 보낸다
        for _ in range(10):
            self.assign_pub.publish(msg)
            if self.wait_until(
                    lambda: any(a.assignment_id == msg.assignment_id for a in self.acks), 1.0):
                return
        self.fail(f'{task_id} Ack 없음')

    def final(self, task_id, timeout):
        def done():
            return [s for s in self.statuses if s.task_id == task_id
                    and s.state in (TaskStatus.STATE_COMPLETED, TaskStatus.STATE_FAILED)]
        self.assertTrue(self.wait_until(lambda: bool(done()), timeout), f'{task_id} 끝나지 않음')
        return done()[0]

    def phases(self, task_id):
        return [s.phase for s in self.statuses
                if s.task_id == task_id and s.state == TaskStatus.STATE_IN_PROGRESS]

    def test_1_delivery_cycle(self):
        self.assign('t1', Task.TYPE_DELIVERY, 'dock_1', 'rack_a1', (1.0, 2.0), (3.0, 4.0))
        last = self.final('t1', 30.0)
        self.assertEqual(last.state, TaskStatus.STATE_COMPLETED)
        self.assertEqual(
            self.phases('t1'),
            ['move_to_pickup', 'docking', 'loading', 'move_to_dropoff', 'docking', 'unloading'])
        with self.servers.lock:
            self.assertEqual(self.servers.nav_goals, [(1.0, 2.0), (3.0, 4.0)])
            self.assertEqual(self.servers.dock_goals, ['dock_1', 'rack_a1'])

    def test_2_dock_fails_three_times(self):
        self.assign('t2', Task.TYPE_DELIVERY, BAD_STATION, 'rack_a1', (1.0, 0.0), (2.0, 0.0))
        last = self.final('t2', 30.0)
        self.assertEqual(last.state, TaskStatus.STATE_FAILED)
        self.assertEqual(last.phase, 'docking')
        self.assertEqual(last.error.code, Dock.Result.FAILED_TO_DETECT_DOCK)
        self.assertIn('failed 3 times', last.error.message)
        with self.servers.lock:
            self.assertEqual(self.servers.dock_goals.count(BAD_STATION), 3)

    def test_3_zone_waiting_pauses_nav(self):
        with self.servers.lock:
            self.servers.nav_duration = 3.0
            before = len(self.servers.nav_goals)
        self.assign('t3', Task.TYPE_RETURN, '', 'wait_1', dropoff_xy=(5.0, 0.0))
        self.assertTrue(self.wait_until(lambda: len(self.servers.nav_goals) == before + 1, 10.0))
        time.sleep(0.5)
        self.zone_pub.publish(String(data='waiting'))
        self.assertTrue(self.wait_until(lambda: self.servers.nav_canceled == 1, 5.0), 'Nav2 목표 취소 안 됨')
        time.sleep(1.0)
        self.assertEqual(len(self.servers.nav_goals), before + 1)   # 기다리는 동안 다시 보내지 않는다
        with self.servers.lock:
            self.servers.nav_duration = 0.5
        self.zone_pub.publish(String(data='approaching'))
        last = self.final('t3', 15.0)
        self.assertEqual(last.state, TaskStatus.STATE_COMPLETED)
        self.assertEqual(len(self.servers.nav_goals), before + 2)
        self.assertEqual(last.error.code, ErrorInfo.NONE)

    # zone_client가 후진 안전 한도로 멈춘 상태(failed)면 MODE_ERROR를 내고 새 작업을 거부한다
    def test_4_zone_failed_rejects_tasks(self):
        self.zone_pub.publish(String(data='failed'))
        self.assertTrue(self.wait_until(
            lambda: bool(self.states) and self.states[-1].mode == RobotState.MODE_ERROR, 5.0))
        self.assign('t4', Task.TYPE_RETURN, '', 'wait_1', dropoff_xy=(1.0, 0.0))
        ack_id = self.last_assignment_id
        self.assertTrue(self.wait_until(
            lambda: any(a.assignment_id == ack_id for a in self.acks), 5.0))
        ack = [a for a in self.acks if a.assignment_id == ack_id][0]
        self.assertEqual(ack.result, TaskAssignmentAck.RESULT_REJECTED)
        self.assertEqual(ack.error.code, ErrorInfo.TRAFFIC_REVOKED)

    # 위치 복구가 failed인 동안도 MODE_ERROR·할당 거부, tracking으로 돌아오면 다시 받는다 (#148)
    def test_5_kidnap_failed_rejects_until_tracking(self):
        # 앞 테스트의 zone failed를 풀고 MODE_IDLE이 된 걸 본 뒤 kidnap failed를 보낸다.
        # 바로 보내면 zone failed의 MODE_ERROR가 남아 있어 아래 할당이 TRAFFIC_REVOKED로 거부된다
        self.zone_pub.publish(String(data='idle'))
        self.assertTrue(self.wait_until(
            lambda: bool(self.states) and self.states[-1].mode == RobotState.MODE_IDLE, 5.0))
        self.kidnap_pub.publish(String(data='failed'))
        self.assertTrue(self.wait_until(
            lambda: bool(self.states) and self.states[-1].mode == RobotState.MODE_ERROR, 5.0))
        self.assign('t5', Task.TYPE_RETURN, '', 'wait_1', dropoff_xy=(1.0, 0.0))
        ack_id = self.last_assignment_id
        self.assertTrue(self.wait_until(lambda: any(a.assignment_id == ack_id for a in self.acks), 5.0))
        ack = [a for a in self.acks if a.assignment_id == ack_id][0]
        self.assertEqual(ack.result, TaskAssignmentAck.RESULT_REJECTED)
        self.assertEqual(ack.error.code, ErrorInfo.NAV_TIMEOUT)
        self.assertIn('localization', ack.error.message)

        self.kidnap_pub.publish(String(data='tracking'))
        self.assertTrue(self.wait_until(
            lambda: bool(self.states) and self.states[-1].mode == RobotState.MODE_IDLE, 5.0))
        with self.servers.lock:
            self.servers.nav_duration = 0.3
        self.assign('t6', Task.TYPE_RETURN, '', 'wait_1', dropoff_xy=(1.0, 0.0))
        self.assertEqual(self.final('t6', 15.0).state, TaskStatus.STATE_COMPLETED)

    # 복구 노드가 failed를 낸 뒤 죽으면(상태가 끊기면) kidnap_state_stale_s 뒤 MODE_IDLE로 돌아온다
    def test_6_kidnap_failed_expires_when_silent(self):
        self.kidnap_pub.publish(String(data='failed'))
        self.assertTrue(self.wait_until(
            lambda: bool(self.states) and self.states[-1].mode == RobotState.MODE_ERROR, 2.0))
        self.assertTrue(self.wait_until(
            lambda: self.states[-1].mode == RobotState.MODE_IDLE, 6.0))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
