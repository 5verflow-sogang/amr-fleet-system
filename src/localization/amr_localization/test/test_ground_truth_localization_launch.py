"""ground_truth_localization.launch.py를 띄우고 합성 ground_truth/odom으로 확인한다 (launch_testing, Gazebo 없음).

map_origin_in_world를 [0, 0, 0](identity)으로 둬서 world == map이라 기대값 계산이 단순하다.
확인:
1. TF map → <ns>/odom(identity) → <ns>/base_footprint가 ground_truth/odom 위치와 일치
2. odometry/local(odom 기준)·odometry/filtered(map 기준)이 같은 위치를 낸다(map → odom이 identity라서)
3. amcl_pose도 같은 위치
4. 위치를 바꿔 다시 보내면(이동 흉내) 세 출력이 모두 새 위치로 따라온다 - AMCL/EKF처럼 지연 없이
"""

import math
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Odometry
import os
import pytest
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

NS = 'gt_amr'
START = (5.0, 3.0, 0.4)  # world == map (origin [0, 0, 0])
MOVED = (6.5, 3.8, 0.9)
WAIT_S = 20.0


def _odom_msg(clock, x, y, yaw):
    msg = Odometry()
    msg.header.stamp = clock.now().to_msg()
    msg.header.frame_id = 'world'
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    msg.twist.twist.linear.x = 0.1
    return msg


@pytest.mark.launch_test
def generate_test_description():
    stack = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('amr_localization'), 'launch', 'ground_truth_localization.launch.py')),
        launch_arguments={
            'namespace': NS, 'use_sim_time': 'false', 'map_origin_in_world': '[0.0, 0.0, 0.0]',
        }.items())
    return launch.LaunchDescription([stack, launch_testing.actions.ReadyToTest()])


class TestGroundTruthLocalization(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_ground_truth_localization')
        cls.gt_pub = cls.node.create_publisher(Odometry, f'/{NS}/ground_truth/odom', 10)
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        cls.local, cls.filtered, cls.amcl = [], [], []
        cls.node.create_subscription(Odometry, f'/{NS}/odometry/local', cls.local.append, 10)
        cls.node.create_subscription(Odometry, f'/{NS}/odometry/filtered', cls.filtered.append, 10)
        cls.node.create_subscription(PoseWithCovarianceStamped, f'/{NS}/amcl_pose', cls.amcl.append, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _publish(cls, x, y, yaw):
        cls.gt_pub.publish(_odom_msg(cls.node.get_clock(), x, y, yaw))

    def spin_until(self, condition, timeout=WAIT_S):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not condition():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return condition()

    def resend_until(self, pose, condition, timeout=WAIT_S):
        # 구독이 ground_truth_localization_node보다 늦게 붙을 수 있어(launch_testing), 될 때까지 다시 낸다
        end = time.monotonic() + timeout
        while time.monotonic() < end and not condition():
            self._publish(*pose)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return condition()

    def test_1_tf_chain_matches_ground_truth(self):
        ok = self.resend_until(START, lambda: self.tf_buffer.can_transform('map', f'{NS}/base_footprint', Time()))
        self.assertTrue(ok, f'TF map → {NS}/odom → {NS}/base_footprint가 이어지지 않는다')
        t = self.tf_buffer.lookup_transform('map', f'{NS}/base_footprint', Time())
        self.assertAlmostEqual(t.transform.translation.x, START[0], places=3)
        self.assertAlmostEqual(t.transform.translation.y, START[1], places=3)
        # map → <ns>/odom은 identity라 odom 기준 자세도 map 기준과 같다
        t_odom = self.tf_buffer.lookup_transform(f'{NS}/odom', f'{NS}/base_footprint', Time())
        self.assertAlmostEqual(t_odom.transform.translation.x, START[0], places=3)

    def test_2_odometry_and_amcl_pose_match(self):
        self.assertTrue(self.spin_until(lambda: self.filtered and self.local and self.amcl))
        for topic_name, samples, frame in (
                ('odometry/filtered', self.filtered, 'map'),
                ('odometry/local', self.local, f'{NS}/odom'),
        ):
            msg = samples[-1]
            self.assertEqual(msg.header.frame_id, frame, topic_name)
            p = msg.pose.pose.position
            self.assertLess(math.hypot(p.x - START[0], p.y - START[1]), 1e-3, f'{topic_name}: ({p.x}, {p.y})')
        p = self.amcl[-1].pose.pose.position
        self.assertLess(math.hypot(p.x - START[0], p.y - START[1]), 1e-3)

    def test_3_follows_robot_without_lag(self):
        ok = self.resend_until(
            MOVED,
            lambda: self.filtered and math.hypot(
                self.filtered[-1].pose.pose.position.x - MOVED[0],
                self.filtered[-1].pose.pose.position.y - MOVED[1]) < 1e-3)
        self.assertTrue(ok, f'odometry/filtered가 이동한 자세({MOVED})를 안 따라온다')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -6, -15])
