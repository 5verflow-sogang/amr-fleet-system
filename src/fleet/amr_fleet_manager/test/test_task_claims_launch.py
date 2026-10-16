"""먼저 수락한 쪽이 주인 (architecture.md 5-2절)을 서버와 가짜 로봇 1대로 확인한다 (launch_testing).

테스트 노드가 분산 경매로 작업을 맡은 로봇(amr_09)처럼 RobotState·TaskStatus를 보낸다.
- 서버: 그 작업 요청이 와도 amr_01에게 맡기지 않고 작업표의 맡은 로봇을 amr_09로 둔다
- 가짜 로봇: 그 작업을 다른 출처(경매 번호)로 받으면 거부한다 (ASSIGN_TASK_TAKEN)
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from amr_msgs.msg import ErrorInfo, FleetTaskTable, RobotState, Task, TaskAssignment, TaskAssignmentAck, TaskStatus

TASK_ID = 'task_taken_by_amr_09'
# 경매로 로봇 9가 낸 명령 번호 (1,000,000 + 9 × 100,000 + 순번, TaskAssignment.msg)
AUCTION_ASSIGNMENT_ID = 1_900_001


@pytest.mark.launch_test
def generate_test_description():
    server = launch_ros.actions.Node(
        package='amr_fleet_manager',
        executable='fleet_manager',
        parameters=[{'traffic.zones_file': ''}],
    )
    robot = launch_ros.actions.Node(
        package='amr_fleet_manager',
        executable='fake_robot',
        namespace='amr_01',
        parameters=[{'robot_id': 'amr_01', 'start_xy': [0.0, 0.0]}],
    )
    return launch.LaunchDescription([server, robot, launch_testing.actions.ReadyToTest()])


class TestTaskClaims(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_task_claims')

    def tearDown(self):
        self.node.destroy_node()

    def spin_until(self, condition, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not condition():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return condition()

    def test_claimed_task_not_assigned_and_rejected(self):
        assignments, acks, tables = [], [], []
        self.node.create_subscription(TaskAssignment, '/fleet/task_assignments', assignments.append, 10)
        self.node.create_subscription(TaskAssignmentAck, '/fleet/task_assignment_acks', acks.append, 10)
        self.node.create_subscription(FleetTaskTable, '/fleet/task_table', tables.append, 10)
        state_pub = self.node.create_publisher(RobotState, '/fleet/robot_states', 10)
        status_pub = self.node.create_publisher(TaskStatus, '/fleet/task_status', 10)
        request_pub = self.node.create_publisher(Task, '/fleet/task_requests', 10)
        assign_pub = self.node.create_publisher(TaskAssignment, '/fleet/task_assignments', 10)

        # 서버와 가짜 로봇이 둘 다 구독할 때까지 (발견 전에 보낸 보고는 사라진다)
        self.assertTrue(self.spin_until(
            lambda: status_pub.get_subscription_count() >= 2 and request_pub.get_subscription_count() >= 1
            and assign_pub.get_subscription_count() >= 1, 15.0), '서버·가짜 로봇 발견 실패')

        start = self.node.get_clock().now().to_msg()

        def report():
            # amr_09: 살아 있고(RobotState) 그 작업을 하고 있다(TaskStatus). 주장은 RobotState가 이어져야 센다
            state = RobotState(robot_id='amr_09', current_task_id=TASK_ID, mode=RobotState.MODE_BUSY,
                               battery_percent=100.0)
            state.header.frame_id = 'map'
            state.pose.position.x = 50.0
            state_pub.publish(state)
            status = TaskStatus(task_id=TASK_ID, robot_id='amr_09', state=TaskStatus.STATE_IN_PROGRESS,
                                phase='move_to_pickup', start_time=start)
            status.header.stamp = self.node.get_clock().now().to_msg()
            status_pub.publish(status)

        self.node.create_timer(0.2, report)
        self.spin_until(lambda: False, 1.0)

        task = Task(task_id=TASK_ID, task_type=Task.TYPE_DELIVERY, priority=1)
        task.created_time = self.node.get_clock().now().to_msg()
        request_pub.publish(task)

        def record():
            for table in reversed(tables):
                for r in table.records:
                    if r.task.task_id == TASK_ID:
                        return r
            return None

        self.assertTrue(self.spin_until(lambda: record() is not None and record().assignee_id == 'amr_09', 5.0),
                        f'작업표에서 맡은 로봇이 amr_09가 아님: {record()}')
        # 서버는 이 작업을 amr_01에게 맡기지 않는다
        self.spin_until(lambda: False, 2.0)
        self.assertEqual([a for a in assignments if a.task.task_id == TASK_ID and a.robot_id == 'amr_01'], [])

        # 가짜 로봇은 amr_09가 맡은 작업을 다른 출처(경매 번호)로 받아도 거부한다
        cmd = TaskAssignment(assignment_id=AUCTION_ASSIGNMENT_ID, robot_id='amr_01',
                             action=TaskAssignment.ACTION_ASSIGN, task=task)
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        assign_pub.publish(cmd)

        def ack():
            return next((a for a in acks if a.assignment_id == AUCTION_ASSIGNMENT_ID), None)

        self.assertTrue(self.spin_until(lambda: ack() is not None, 5.0), 'Ack 없음')
        self.assertEqual(ack().result, TaskAssignmentAck.RESULT_REJECTED)
        self.assertEqual(ack().error.code, ErrorInfo.ASSIGN_TASK_TAKEN)
