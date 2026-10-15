"""I14 안전 감속: safety_node를 띄우고 가짜 입력을 넣어 SafetyStatus와 최종 cmd_vel을 확인한다 (launch_testing).

입력(scan, odom, perception/obstacles, perception/collision_risks, cmd_vel_zone_limited)은 모두 이 테스트가
10 Hz로 낸다. 센서 끊김으로 떨어지지 않게 전부 내다가, 단계마다 장면만 바꾼다. 로봇은 odom 원점에 정지.
1. 빈 장면, 전진 0.5 m/s → NORMAL, 명령 그대로
2. 차체 앞면 0.2 m 앞(진행 경로)에 LiDAR 점 → CRITICAL(장애물 근접), cmd_vel 0
3. 앞 4 m의 움직이는 트랙, TTC가 ttc_critical(1 s)과 ttc_warning(2 s) 사이 → WARNING(충돌 위험),
   speed_limit 0.6, 명령 (1.0, 0.5)가 곡률을 지킨 채 (0.6, 0.3)으로
4. 스캔 끊김 → CRITICAL(센서 고장), cmd_vel 0
"""

import math
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Twist
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster

from amr_msgs.msg import (CollisionRisk, CollisionRiskArray, SafetyStatus, TrackedObstacle,
                          TrackedObstacleArray)

# 다른 패키지 테스트와 병렬로 돌아도 토픽·TF 프레임이 겹치지 않게 전용 네임스페이스를 쓴다
NS = 'i14_amr'
RATE = 10.0                 # Hz. 스캔·odom·추적·명령 입력 주기
NUM_BEAMS = 720
ANGLE_MIN = -math.pi
ANGLE_INC = 2.0 * math.pi / NUM_BEAMS
RANGE_MAX = 12.0
HALF_LENGTH = 0.3           # 차체 0.6 × 0.4 m (safety.yaml robot_length)
TRACK_ID = 7
WAIT_S = 10.0               # 단계마다 기다리는 최대 시간
DRAIN_S = 0.3               # 단계가 바뀐 뒤 이전 출력 명령을 버리는 시간
SETTLE_S = 0.5              # 그 뒤 출력 명령을 모으는 시간


def scan_ranges(obstacle_x=None):
    """LiDAR(= base_footprint)에서 본 거리. obstacle_x를 주면 정면 ±1° 빔이 그 거리에서 맞는다 (반사 없음 = inf)."""
    ranges = [math.inf] * NUM_BEAMS
    if obstacle_x is not None:
        center = round(-ANGLE_MIN / ANGLE_INC)          # 0° 빔
        for i in range(center - 2, center + 3):
            ranges[i] = obstacle_x / math.cos(ANGLE_MIN + ANGLE_INC * i)
    return ranges


@pytest.mark.launch_test
def generate_test_description():
    safety = launch_ros.actions.Node(
        package='amr_safety',
        executable='safety_node',
        name='safety',
        namespace=NS,
        # 실제 설정 그대로 (cmd_vel_zone_limited → cmd_vel, TTC 1.0/2.0 s, 감속 0.6 m/s)
        parameters=[os.path.join(get_package_share_directory('amr_safety'), 'config', 'safety.yaml'),
                    {'use_sim_time': False, 'publish_markers': False}],
        output='screen',
    )
    return launch.LaunchDescription([safety, launch_testing.actions.ReadyToTest()])


class TestSafetyLimits(unittest.TestCase):
    """단계를 차례로 바꾼다 (unittest는 메서드 이름 순으로 돈다). 노드·입력 타이머는 클래스 전체에서 하나."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_safety_launch')
        node = cls.node
        cls.tf_broadcaster = StaticTransformBroadcaster(node)
        cls.tf_broadcaster.sendTransform([
            cls._transform(f'{NS}/odom', f'{NS}/base_footprint'),
            cls._transform(f'{NS}/base_footprint', f'{NS}/lidar_link'),
        ])
        cls.statuses, cls.cmds = [], []
        node.create_subscription(SafetyStatus, f'/{NS}/safety/status', cls.statuses.append, 10)
        node.create_subscription(Twist, f'/{NS}/cmd_vel', cls.cmds.append, 10)
        cls.scan_pub = node.create_publisher(LaserScan, f'/{NS}/scan', qos_profile_sensor_data)
        cls.odom_pub = node.create_publisher(Odometry, f'/{NS}/odom', 10)
        cls.obstacles_pub = node.create_publisher(TrackedObstacleArray, f'/{NS}/perception/obstacles', 10)
        cls.risks_pub = node.create_publisher(CollisionRiskArray, f'/{NS}/perception/collision_risks', 10)
        cls.cmd_pub = node.create_publisher(Twist, f'/{NS}/cmd_vel_zone_limited', 10)
        # 단계마다 바꾸는 장면
        cls.scene = {'scan': True, 'obstacle_x': None, 'track': None, 'cmd': (0.5, 0.0)}
        node.create_timer(1.0 / RATE, cls._publish_inputs)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _transform(cls, parent, child):
        tf = TransformStamped()
        tf.header.stamp = cls.node.get_clock().now().to_msg()
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.rotation.w = 1.0
        return tf

    @classmethod
    def _publish_inputs(cls):
        scene, stamp = cls.scene, cls.node.get_clock().now().to_msg()
        if scene['scan']:
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, f'{NS}/lidar_link'
            scan.angle_min, scan.angle_increment = ANGLE_MIN, ANGLE_INC
            scan.angle_max = ANGLE_MIN + ANGLE_INC * (NUM_BEAMS - 1)
            scan.range_min, scan.range_max = 0.12, RANGE_MAX
            scan.ranges = scan_ranges(scene['obstacle_x'])
            cls.scan_pub.publish(scan)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, f'{NS}/odom'
        odom.child_frame_id = f'{NS}/base_footprint'
        odom.pose.pose.orientation.w = 1.0
        cls.odom_pub.publish(odom)
        obstacles, risks = TrackedObstacleArray(), CollisionRiskArray()
        obstacles.header.stamp, obstacles.header.frame_id = stamp, f'{NS}/odom'
        risks.header = obstacles.header
        if scene['track'] is not None:
            x, ttc = scene['track']
            obs = TrackedObstacle(track_id=TRACK_ID, is_dynamic=True, radius=0.2)
            obs.pose.pose.position.x = x
            obs.pose.pose.orientation.w = 1.0
            obs.twist.twist.linear.x = -1.0          # 로봇 쪽으로 다가온다
            obstacles.obstacles.append(obs)
            risks.risks.append(CollisionRisk(track_id=TRACK_ID, ttc=ttc))
        cls.obstacles_pub.publish(obstacles)
        cls.risks_pub.publish(risks)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = scene['cmd']
        cls.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------ 도우미
    def spin_until(self, condition, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not condition():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return condition()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_status(self, level, reason):
        """장면을 바꾼 뒤 온 상태 중 (level, reason)이 나올 때까지. 마지막 상태를 돌려준다."""
        self.statuses.clear()
        ok = self.spin_until(
            lambda: bool(self.statuses) and (self.statuses[-1].level, self.statuses[-1].reason)
            == (level, reason), WAIT_S)
        last = self.statuses[-1] if self.statuses else None
        self.assertTrue(ok, f'SafetyStatus ({level}, {reason})가 오지 않았다. 마지막: {last}')
        return last

    def settled_cmds(self):
        """상태가 바뀐 뒤 SETTLE_S 동안 나온 최종 cmd_vel (선속도, 각속도).

        상태와 명령은 다른 토픽이라 바뀌기 전에 낸 명령이 아직 큐에 있을 수 있다 → 먼저 비운다.
        """
        self.spin_for(DRAIN_S)
        self.cmds.clear()
        self.spin_for(SETTLE_S)
        self.assertGreater(len(self.cmds), 0, 'cmd_vel이 나오지 않았다')
        return [(c.linear.x, c.angular.z) for c in self.cmds]

    # ------------------------------------------------------------------ 단계
    def test_1_normal_passes_command(self):
        # 노드 발견 + 끊김 상태(CRITICAL)에서 release_hold 0.5 s 뒤 NORMAL
        self.scene.update(scan=True, obstacle_x=None, track=None, cmd=(0.5, 0.0))
        status = self.wait_status(SafetyStatus.LEVEL_NORMAL, SafetyStatus.REASON_NONE)
        self.assertEqual(status.speed_limit, math.inf)
        for vx, wz in self.settled_cmds():
            self.assertAlmostEqual(vx, 0.5)
            self.assertAlmostEqual(wz, 0.0)

    def test_2_obstacle_on_path_stops(self):
        self.scene.update(obstacle_x=HALF_LENGTH + 0.2, track=None, cmd=(0.5, 0.0))
        status = self.wait_status(SafetyStatus.LEVEL_CRITICAL, SafetyStatus.REASON_OBSTACLE_PROXIMITY)
        self.assertEqual(status.speed_limit, 0.0)
        self.assertAlmostEqual(status.min_obstacle_distance, 0.2, delta=0.01)
        for vx, wz in self.settled_cmds():
            self.assertEqual((vx, wz), (0.0, 0.0))

    def test_3_ttc_warning_limits_speed(self):
        # 앞 4 m 트랙: 차체까지 4 − 0.3 − 0.2 = 3.5 m로 근접 규칙(1.5 m) 밖. TTC 1.5 s만 걸린다
        self.scene.update(obstacle_x=None, track=(4.0, 1.5), cmd=(1.0, 0.5))
        status = self.wait_status(SafetyStatus.LEVEL_WARNING, SafetyStatus.REASON_COLLISION_RISK)
        self.assertAlmostEqual(status.speed_limit, 0.6)
        self.assertEqual(status.related_track_id, TRACK_ID)
        for vx, wz in self.settled_cmds():
            self.assertAlmostEqual(vx, 0.6)
            self.assertAlmostEqual(wz, 0.3)         # 곡률 ω/v = 0.5 유지

    def test_4_scan_loss_stops(self):
        self.scene.update(scan=False, obstacle_x=None, track=None, cmd=(0.5, 0.0))
        status = self.wait_status(SafetyStatus.LEVEL_CRITICAL, SafetyStatus.REASON_SENSOR_FAULT)
        self.assertEqual(status.speed_limit, 0.0)
        for vx, wz in self.settled_cmds():
            self.assertEqual((vx, wz), (0.0, 0.0))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
