"""장애주입기 노드를 띄워 고정 지연이 실제로 걸리는지 확인한다 (launch_testing)."""

import statistics
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from amr_msgs.msg import FleetHeartbeat

DELAY_MS = 200.0
COUNT = 20


@pytest.mark.launch_test
def generate_test_description():
    injector = launch_ros.actions.Node(
        package='amr_fault_injector',
        executable='fault_injector',
        parameters=[{
            'robot_ids': ['amr_01', 'amr_02'],
            'topics': ['heartbeat'],
            'delay_ms': DELAY_MS,
            'loss_pct': 0.0,
        }],
    )
    return launch.LaunchDescription([injector, launch_testing.actions.ReadyToTest()])


class TestRelayDelay(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_relay_delay')

    def tearDown(self):
        self.node.destroy_node()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def test_fixed_delay(self):
        received = {}
        publisher = self.node.create_publisher(FleetHeartbeat, '/fleet/heartbeat', 10)
        # colcon test는 패키지를 병렬로 돌린다. amr_fleet_manager 테스트의 서버 heartbeat도 이 주입기를 거쳐 오고
        # seq가 겹치므로, 이 테스트가 보낸 것(server_id='test')만 센다
        self.node.create_subscription(
            FleetHeartbeat, '/amr_01/fleet/heartbeat',
            lambda msg: received.setdefault(msg.seq, time.monotonic()) if msg.server_id == 'test' else None, 10)

        # 주입기와 서로 발견될 때까지 기다린다. 발견 전에 보낸 메시지는 전달되지 않는다
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and (
                publisher.get_subscription_count() < 1
                or self.node.count_publishers('/amr_01/fleet/heartbeat') < 1):
            self.spin_for(0.1)
        self.spin_for(1.0)

        sent = {}
        for seq in range(COUNT):
            sent[seq] = time.monotonic()
            publisher.publish(FleetHeartbeat(server_id='test', seq=seq))
            self.spin_for(0.05)

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and len(received) < COUNT:
            self.spin_for(0.05)

        self.assertEqual(sorted(received), list(range(COUNT)))
        delays = [(received[s] - sent[s]) * 1000.0 for s in range(COUNT)]
        # 주입기는 받은 뒤 delay_ms를 기다리므로 그보다 일찍 올 수 없다
        self.assertGreaterEqual(min(delays), DELAY_MS)
        # 중계 1홉 + 타이머 주기 + 이 테스트 노드의 spin 오차
        self.assertLess(statistics.median(delays), DELAY_MS + 50.0)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
