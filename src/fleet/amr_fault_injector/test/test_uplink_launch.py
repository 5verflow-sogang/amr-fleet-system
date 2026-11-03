"""양방향 끊김(relay_uplink): 로봇 → 서버 토픽도 같은 끊김 일정으로 중계되는지 (launch_testing).

amr_01만 주입기 시작 4초 뒤부터 4초 동안 끊는다. 끊긴 동안 amr_01의 상태 보고는 /fleet/robot_states에 나오지 않고
(delay 모드라 끊김이 끝나면 붙잡혔던 것이 나온다), 끊기지 않은 amr_02는 계속 나온다.
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from amr_msgs.msg import RobotState

BLACKOUT_AT_S = 4.0
BLACKOUT_S = 4.0


@pytest.mark.launch_test
def generate_test_description():
    injector = launch_ros.actions.Node(
        package='amr_fault_injector',
        executable='fault_injector',
        parameters=[{
            'robot_ids': ['amr_01', 'amr_02'],
            'topics': ['heartbeat'],
            'relay_uplink': True,
            'delay_ms': 0.0,
            'allow_out_of_spec': True,
            'blackout_s': BLACKOUT_S,
            'blackout_at_s': BLACKOUT_AT_S,
            'blackout_robot': 'amr_01',
        }],
    )
    return launch.LaunchDescription([injector, launch_testing.actions.ReadyToTest()])


class TestUplinkBlackout(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_uplink_blackout')

    def tearDown(self):
        self.node.destroy_node()

    def test_uplink_follows_blackout(self):
        t0 = time.monotonic()   # 주입기 시작과 거의 같다 (몇백 ms 늦을 수 있어 판정에 여유를 둔다)
        got = {'amr_01': [], 'amr_02': []}
        self.node.create_subscription(
            RobotState, '/fleet/robot_states',
            lambda m: got[m.robot_id].append(time.monotonic() - t0) if m.robot_id in got else None, 50)
        pubs = {r: self.node.create_publisher(RobotState, f'/{r}/uplink/robot_states', 10) for r in got}

        end = t0 + BLACKOUT_AT_S + BLACKOUT_S + 3.0
        next_send = time.monotonic()
        while time.monotonic() < end:
            if time.monotonic() >= next_send:
                for robot_id, pub in pubs.items():
                    pub.publish(RobotState(robot_id=robot_id))
                next_send += 0.2
            rclpy.spin_once(self.node, timeout_sec=0.02)

        mid = (BLACKOUT_AT_S + 1.0, BLACKOUT_AT_S + BLACKOUT_S - 1.0)
        in_mid = lambda ts: [t for t in ts if mid[0] <= t <= mid[1]]  # noqa: E731
        self.assertTrue(got['amr_02'], '끊기지 않은 amr_02의 상태 보고가 중계되지 않음')
        self.assertTrue(in_mid(got['amr_02']), 'amr_02는 끊김 시간에도 계속 나와야 한다')
        self.assertTrue(got['amr_01'], 'amr_01 상태 보고가 한 번도 중계되지 않음')
        self.assertEqual(in_mid(got['amr_01']), [], 'amr_01은 끊긴 동안 나오면 안 된다')
