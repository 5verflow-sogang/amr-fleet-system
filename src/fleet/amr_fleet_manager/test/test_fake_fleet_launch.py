"""Fleet 서버와 가짜 로봇 1대를 띄워 확인한다 (launch_testing).

- 작업 요청 → 할당 → Ack → 완료
- 구역 허가: 한 대만 허가, 나머지는 대기, 반납하면 대기 로봇에게 허가
"""

import tempfile
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from amr_msgs.msg import (FleetHeartbeat, RobotState, Task, TaskAssignment, TaskAssignmentAck, TaskStatus,
                          ZoneGrant, ZoneRequest)


ZONES_YAML = 'zones:\n  - id: test_zone\n    capacity: 1\n'


@pytest.mark.launch_test
def generate_test_description():
    zones = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    zones.write(ZONES_YAML)
    zones.close()
    server = launch_ros.actions.Node(
        package='amr_fleet_manager',
        executable='fleet_manager',
        # config yaml을 쓰지 않으므로 epoch_file은 기본값('')이고 epoch는 1이다
        parameters=[{'assignment.algorithm': 'hungarian', 'traffic.zones_file': zones.name}],
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
        assignments, acks, statuses, heartbeats, robot_states = [], [], [], [], []
        self.node.create_subscription(TaskAssignment, '/fleet/task_assignments', assignments.append, 10)
        self.node.create_subscription(TaskAssignmentAck, '/fleet/task_assignment_acks', acks.append, 10)
        self.node.create_subscription(TaskStatus, '/fleet/task_status', statuses.append, 10)
        # colcon test는 패키지를 병렬로 돌린다. 장애 주입기 테스트도 같은 토픽에 heartbeat를 보내므로 이 서버 것만 센다
        self.node.create_subscription(
            FleetHeartbeat, '/fleet/heartbeat',
            lambda msg: heartbeats.append(msg) if msg.server_id == 'fleet_manager' else None, 10)
        self.node.create_subscription(
            RobotState, '/fleet/robot_states',
            lambda msg: robot_states.append(msg) if msg.robot_id == 'amr_01' else None, 10)
        publisher = self.node.create_publisher(Task, '/fleet/task_requests', 10)

        # 서버와 가짜 로봇 둘 다와 연결될 때까지 기다린다. 발견 전에 보낸 요청이나, 발견 전에 로봇이 보낸 상태 보고는
        # 전달되지 않는다. 가짜 로봇은 작업을 1초 안에 끝내므로 로봇과 먼저 연결돼 있어야 보고를 받는다 (CI에서 실패한 경합)
        self.assertTrue(self.spin_until(
            lambda: publisher.get_subscription_count() >= 1 and heartbeats and robot_states, 15.0))

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

    def test_zone_handover(self):
        grants = []
        # 다른 패키지 테스트와 섞이지 않게 이 테스트의 구역만 센다
        self.node.create_subscription(
            ZoneGrant, '/fleet/zone_grants',
            lambda msg: grants.append(msg) if msg.zone_id == 'test_zone' else None, 10)
        publisher = self.node.create_publisher(ZoneRequest, '/fleet/zone_requests', 10)
        self.assertTrue(self.spin_until(
            lambda: publisher.get_subscription_count() >= 1
            and self.node.count_publishers('/fleet/zone_grants') >= 1, 15.0))
        self.spin_until(lambda: False, 1.0)

        def request(robot_id, request_id, kind=ZoneRequest.TYPE_ACCESS):
            publisher.publish(ZoneRequest(
                robot_id=robot_id, zone_id='test_zone', request_id=request_id, type=kind))

        request('amr_08', 1)
        self.assertTrue(self.spin_until(lambda: len(grants) >= 1, 5.0))
        request('amr_09', 7)
        self.assertTrue(self.spin_until(lambda: len(grants) >= 2, 5.0))
        request('amr_08', 1, ZoneRequest.TYPE_RELEASE)
        self.assertTrue(self.spin_until(lambda: len(grants) >= 3, 5.0))

        summary = [(g.robot_id, g.request_id, g.grant, g.holder_robot_id) for g in grants[:3]]
        self.assertEqual(summary, [
            ('amr_08', 1, ZoneGrant.GRANT_GRANTED, 'amr_08'),
            ('amr_09', 7, ZoneGrant.GRANT_QUEUED, 'amr_08'),
            ('amr_09', 7, ZoneGrant.GRANT_GRANTED, 'amr_09'),
        ])
        self.assertGreater(grants[0].lease_expiry.sec, 0)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
