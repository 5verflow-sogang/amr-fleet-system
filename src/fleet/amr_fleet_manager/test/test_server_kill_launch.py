"""fleet_manager.launch.py의 서버 강제종료·재시작을 확인한다 (launch_testing).

heartbeat가 끊겼다가 epoch가 1 오른 채로 다시 오는지, fault_events.csv에 시각이 남는지 본다.
"""

import csv
import os
import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from amr_msgs.msg import FleetHeartbeat
from launch.launch_description_sources import PythonLaunchDescriptionSource

KILL_AT_S = 4.0
RESTART_AFTER_S = 2.0


@pytest.mark.launch_test
def generate_test_description():
    tmp = tempfile.mkdtemp()
    events = os.path.join(tmp, 'fault_events.csv')
    fleet = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('amr_fleet_manager'), 'launch', 'fleet_manager.launch.py')),
        launch_arguments={
            'epoch_file': os.path.join(tmp, 'epoch'),
            'zones_file': '',
            'kill_at_s': str(KILL_AT_S),
            'restart_after_s': str(RESTART_AFTER_S),
            'fault_events_file': events,
        }.items(),
    )
    return launch.LaunchDescription([fleet, launch_testing.actions.ReadyToTest()]), {'events': events}


class TestServerKill(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_server_kill')

    def tearDown(self):
        self.node.destroy_node()

    def test_kill_and_restart(self, events):
        beats = []
        # colcon test는 패키지를 병렬로 돌린다. 장애 주입기 테스트도 같은 토픽에 heartbeat를 보내므로 이 서버 것만 센다
        self.node.create_subscription(
            FleetHeartbeat, '/fleet/heartbeat',
            lambda msg: beats.append((time.monotonic(), msg.epoch)) if msg.server_id == 'fleet_manager' else None,
            10)

        end = time.monotonic() + KILL_AT_S + RESTART_AFTER_S + 15.0
        while time.monotonic() < end and not any(epoch == 2 for _, epoch in beats):
            rclpy.spin_once(self.node, timeout_sec=0.05)

        epochs = [epoch for _, epoch in beats]
        self.assertIn(1, epochs, '처음 서버의 heartbeat를 못 받음')
        self.assertIn(2, epochs, '재시작한 서버의 heartbeat를 못 받음')
        # 1 → 2 순서로만 바뀐다 (죽은 서버가 다시 보내지 않는다)
        self.assertEqual(epochs, sorted(epochs))

        last_old = max(t for t, epoch in beats if epoch == 1)
        first_new = min(t for t, epoch in beats if epoch == 2)
        # 끊긴 시간은 대략 restart_after_s + 노드 기동 시간이다
        self.assertGreater(first_new - last_old, RESTART_AFTER_S * 0.8)

        with open(events, newline='') as f:
            rows = list(csv.reader(f))
        self.assertEqual(rows[0], ['time', 'event'])
        self.assertEqual([r[1] for r in rows[1:]], ['kill', 'restart'])
        kill_time, restart_time = (float(r[0]) for r in rows[1:])
        self.assertAlmostEqual(restart_time - kill_time, RESTART_AFTER_S, delta=0.5)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # 강제종료한 첫 서버는 SIGKILL(-9)로 끝난다
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -9])
