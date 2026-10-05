"""Fleet 서버와 가짜 로봇 1대를 띄워 작업 요청 → 할당 → Ack → 완료까지 확인한다 (launch_testing)."""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from amr_msgs.msg import FleetHeartbeat, Task, TaskAssignment, TaskAssignmentAck, TaskStatus


@pytest.mark.launch_test
def generate_test_description():
    server = launch_ros.actions.Node(
        package='amr_fleet_manager',
        executable='fleet_manager',
        # config yaml을 쓰지 않으므로 epoch_file은 기본값('')이고 epoch는 1이다
        parameters=[{'assignment.algorithm': 'hungarian'}],
    )
    robot = launch_ros.actions.Node(
        package='amr_fleet_manager',
        executable='fake_robot',
        namespace='amr_01',
        parameters=[{'robot_id': 'amr_01', 'start_xy': [0.0, 0.0], 'speed_mps': 5.0, 'dwell_s': 0.1}],
    )
    return launch.LaunchDescription([server, robot, launch_testing.actions.ReadyToTest()])


class TestFakeFleet(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_fake_fleet')

    def tearDown(self):
        self.node.destroy_node()

    def spin_until(self, condition, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not condition():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return condition()

    def test_task_completes(self):
        assignments, acks, statuses, heartbeats = [], [], [], []
        self.node.create_subscription(TaskAssignment, '/fleet/task_assignments', assignments.append, 10)
        self.node.create_subscription(TaskAssignmentAck, '/fleet/task_assignment_acks', acks.append, 10)
        self.node.create_subscription(TaskStatus, '/fleet/task_status', statuses.append, 10)
        # colcon test는 패키지를 병렬로 돌린다. 장애 주입기 테스트도 같은 토픽에 heartbeat를 보내므로 이 서버 것만 센다
        self.node.create_subscription(
            FleetHeartbeat, '/fleet/heartbeat',
            lambda msg: heartbeats.append(msg) if msg.server_id == 'fleet_manager' else None, 10)
        publisher = self.node.create_publisher(Task, '/fleet/task_requests', 10)

        # 서버와 서로 발견될 때까지 기다린다. 발견 전에 보낸 요청은 전달되지 않는다
        self.assertTrue(self.spin_until(
            lambda: publisher.get_subscription_count() >= 1 and heartbeats, 15.0))

        task = Task(task_id='task_test', task_type=Task.TYPE_DELIVERY, item_type=Task.ITEM_SMALL)
        task.pickup_pose.position.x = 1.0
        task.dropoff_pose.position.x = 2.0
        publisher.publish(task)

        done = self.spin_until(
            lambda: any(s.state == TaskStatus.STATE_COMPLETED for s in statuses), 20.0)
        self.assertTrue(done, f'완료 보고 없음: {[(s.state, s.phase) for s in statuses]}')

        self.assertEqual(heartbeats[0].epoch, 1)
        self.assertEqual(assignments[0].robot_id, 'amr_01')
        self.assertEqual(assignments[0].task.task_id, 'task_test')
        ack = acks[0]
        self.assertEqual(ack.assignment_id, assignments[0].assignment_id)
        self.assertEqual(ack.result, TaskAssignmentAck.RESULT_ACCEPTED)
        self.assertEqual(ack.assignment_stamp, assignments[0].header.stamp)

        phases = [s.phase for s in statuses if s.state == TaskStatus.STATE_IN_PROGRESS]
        self.assertEqual(phases, ['move_to_pickup', 'loading', 'move_to_dropoff', 'unloading'])
        final = statuses[-1]
        self.assertEqual(final.state, TaskStatus.STATE_COMPLETED)
        self.assertAlmostEqual(final.distance_traveled, 2.0, places=3)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
