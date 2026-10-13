"""I13 장애물 추적: tracking_node를 띄우고 합성 LiDAR 스캔을 넣어 TrackedObstacleArray를 확인한다 (launch_testing).

녹화 rosbag 대신 이 테스트가 정한 장면을 10 Hz 스캔으로 그려 보낸다 (결정적, CI에서 Gazebo 없이 돈다).
- 로봇은 odom 원점에 정지. TF는 odom → base_footprint → lidar_link (LiDAR가 앞으로 0.1 m)
- 사람: 다리 두 개(반경 0.06 m, 0.3 m 간격)가 로봇 앞 2 m를 옆으로 1.0 m/s로 가로지른다 (y −1.5 → 1.5).
  3 s 걷고 2.5 s 사라지기를 되풀이한다 (사라진 동안 이전 트랙이 max_coast_time 2 s 뒤 지워진다)
- 벽: x = 4 m, 길이 16 m. R1 크기 규칙(2 m 초과)으로 빠진다. 사람 그림자가 벽을 잘라도 조각이 2 m를 넘는다
- 기둥: (1.0, 2.5) 반경 0.1 m. 정적 트랙으로 남아야 한다

확인: 사람 트랙 하나만 동적이고 위치(0.3 m)·속력(0.3 m/s)이 참값에 가깝다. 기둥·벽은 동적이 되지 않는다.
"""

import math
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster

from amr_msgs.msg import CollisionRiskArray, TrackedObstacleArray

# 다른 패키지 테스트와 병렬로 돌아도 토픽·TF 프레임이 겹치지 않게 전용 네임스페이스를 쓴다
NS = 'i13_amr'

SCAN_RATE = 10.0                    # Hz. 실제 LiDAR 주기 (amr_description sensors.yaml)
NUM_BEAMS = 720                     # 0.5° 간격 360°
ANGLE_MIN = -math.pi
ANGLE_INC = 2.0 * math.pi / NUM_BEAMS
RANGE_MAX = 12.0
LIDAR_XY = (0.1, 0.0)               # base_footprint(= odom 원점) 기준 LiDAR 위치

PERSON_X = 2.0                      # 사람이 지나가는 줄 (odom x)
PERSON_Y0 = -1.5
PERSON_SPEED = 1.0                  # m/s, +y 방향
WALK_S = 3.0
CYCLE_S = 5.5                       # 걷기 3 s + 사라짐 2.5 s
LEG_RADIUS = 0.06
LEG_SPACING = 0.3                   # 두 다리 중심 간격 (진행 방향)
WALL_X = 4.0
WALL_HALF = 8.0
PILLAR = (1.0, 2.5, 0.1)            # (x, y, 반경)

POS_TOL = 0.3                       # m. 군집 중심은 센서 쪽 면이라 실제 중심보다 앞으로 치우친다
SPEED_TOL = 0.3                     # m/s
DYNAMIC_X_BAND = 0.5                # 동적 트랙은 사람이 다니는 줄 x = 2 ± 0.5 안에만 있어야 한다
GOOD_FRAMES = 5                     # 조건을 만족한 프레임이 이만큼 모이면 통과


def person_truth(t):
    """시각 t [s] (주기 시작 = 0)의 사람 중심 (x, y). 사라진 동안은 None."""
    phase = t % CYCLE_S
    if phase > WALK_S:
        return None
    return (PERSON_X, PERSON_Y0 + PERSON_SPEED * phase)


def _ray_circle(ox, oy, dx, dy, cx, cy, r):
    """원점 (ox, oy), 단위 방향 (dx, dy) 빔이 원에 처음 닿는 거리. 안 닿으면 inf."""
    px, py = ox - cx, oy - cy
    b = px * dx + py * dy
    c = px * px + py * py - r * r
    disc = b * b - c
    if disc < 0.0:
        return math.inf
    s = -b - math.sqrt(disc)
    return s if s > 0.0 else math.inf


def scan_ranges(t):
    """시각 t의 장면을 LiDAR 위치에서 본 거리 배열 (반사 없음 = inf)."""
    circles = [PILLAR]
    truth = person_truth(t)
    if truth is not None:
        circles += [(truth[0], truth[1] - 0.5 * LEG_SPACING, LEG_RADIUS),
                    (truth[0], truth[1] + 0.5 * LEG_SPACING, LEG_RADIUS)]
    ox, oy = LIDAR_XY
    ranges = []
    for i in range(NUM_BEAMS):
        a = ANGLE_MIN + ANGLE_INC * i
        dx, dy = math.cos(a), math.sin(a)
        best = math.inf
        if dx > 1e-9:
            s = (WALL_X - ox) / dx
            if abs(oy + s * dy) <= WALL_HALF:
                best = s
        for cx, cy, r in circles:
            best = min(best, _ray_circle(ox, oy, dx, dy, cx, cy, r))
        ranges.append(best if best <= RANGE_MAX else math.inf)
    return ranges


@pytest.mark.launch_test
def generate_test_description():
    tracking = launch_ros.actions.Node(
        package='amr_tracking',
        executable='tracking_node',
        name='tracking',
        namespace=NS,
        # 실제 설정 그대로 (fixed_frame = i13_amr/odom, base_frame = i13_amr/base_footprint)
        parameters=[os.path.join(get_package_share_directory('amr_tracking'), 'config', 'tracking.yaml'),
                    {'use_sim_time': False, 'publish_markers': False}],
        output='screen',
    )
    return launch.LaunchDescription([tracking, launch_testing.actions.ReadyToTest()])


class TestTrackingSyntheticScan(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_tracking_launch')
        self.tf_broadcaster = StaticTransformBroadcaster(self.node)
        self.tf_broadcaster.sendTransform([
            self._transform(f'{NS}/odom', f'{NS}/base_footprint', 0.0, 0.0),
            self._transform(f'{NS}/base_footprint', f'{NS}/lidar_link', *LIDAR_XY),
        ])

    def tearDown(self):
        self.node.destroy_node()

    def _transform(self, parent, child, x, y):
        tf = TransformStamped()
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
        tf.transform.rotation.w = 1.0
        return tf

    def test_person_tracked_as_single_dynamic_obstacle(self):
        frames, risks = [], []
        self.node.create_subscription(TrackedObstacleArray, f'/{NS}/perception/obstacles',
                                      frames.append, 10)
        self.node.create_subscription(CollisionRiskArray, f'/{NS}/perception/collision_risks',
                                      risks.append, 10)
        scan_pub = self.node.create_publisher(LaserScan, f'/{NS}/scan', qos_profile_sensor_data)

        # 장면 시각은 첫 스캔부터 잰다. 스캔 촬영 시각(header.stamp)으로 참값을 다시 구한다
        t0 = self.node.get_clock().now().nanoseconds * 1e-9

        def publish_scan():
            now = self.node.get_clock().now()
            msg = LaserScan()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = f'{NS}/lidar_link'
            msg.angle_min, msg.angle_increment = ANGLE_MIN, ANGLE_INC
            msg.angle_max = ANGLE_MIN + ANGLE_INC * (NUM_BEAMS - 1)
            msg.scan_time = 1.0 / SCAN_RATE
            msg.range_min, msg.range_max = 0.12, RANGE_MAX
            msg.ranges = scan_ranges(now.nanoseconds * 1e-9 - t0)
            scan_pub.publish(msg)

        self.node.create_timer(1.0 / SCAN_RATE, publish_scan)

        good, false_dynamic, pillar_static = 0, [], 0
        checked = 0
        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline and good < GOOD_FRAMES:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            for msg in frames[checked:]:
                self.assertEqual(msg.header.frame_id, f'{NS}/odom')
                dynamic = [o for o in msg.obstacles if o.is_dynamic]
                # 벽 조각·기둥이 동적이 되면 사람 줄 밖에 동적 트랙이 생긴다
                false_dynamic += [(o.track_id, round(o.pose.pose.position.x, 2),
                                   round(o.pose.pose.position.y, 2)) for o in dynamic
                                  if abs(o.pose.pose.position.x - PERSON_X) > DYNAMIC_X_BAND]
                pillar_static += any(
                    not o.is_dynamic and math.hypot(o.pose.pose.position.x - PILLAR[0],
                                                    o.pose.pose.position.y - PILLAR[1]) < POS_TOL
                    for o in msg.obstacles)
                truth = person_truth(Time.from_msg(msg.header.stamp).nanoseconds * 1e-9 - t0)
                if truth is None or len(dynamic) != 1:
                    continue
                p, v = dynamic[0].pose.pose.position, dynamic[0].twist.twist.linear
                if (math.hypot(p.x - truth[0], p.y - truth[1]) < POS_TOL
                        and abs(math.hypot(v.x, v.y) - PERSON_SPEED) < SPEED_TOL
                        and v.y > 0.0):
                    good += 1
                    person_id = dynamic[0].track_id
            checked = len(frames)

        self.assertGreater(len(frames), 0, 'perception/obstacles가 오지 않았다')
        self.assertEqual(false_dynamic, [], '사람 줄 밖의 동적 트랙 (벽·기둥 오분류)')
        self.assertGreaterEqual(
            good, GOOD_FRAMES,
            f'사람이 동적 트랙 하나로 참값 근처에 잡힌 프레임이 부족하다 ({good}/{len(frames)})')
        self.assertGreater(pillar_static, 0, '기둥이 정적 트랙으로 잡히지 않았다')
        # 충돌 위험(TTC)도 같은 트랙으로 나온다 (로봇 위치 TF가 풀렸다는 뜻)
        self.assertTrue(any(r.track_id == person_id for msg in risks for r in msg.risks),
                        'collision_risks에 사람 트랙이 없다')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
