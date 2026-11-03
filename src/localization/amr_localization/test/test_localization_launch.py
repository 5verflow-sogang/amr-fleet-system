"""I6 위치추정 스택 기동: localization.launch.py를 띄우고 합성 입력으로 확인한다 (launch_testing, Gazebo 없음).

지도는 테스트가 만든다: 10 x 10 m 방(벽 두께 0.1 m) + 대칭을 깨는 기둥 하나, 0.05 m 칸.
로봇은 방 가운데 (5, 5)에 정지해 있다. 그 자리에서 보이는 스캔을 계산해 10 Hz, 정지 odom 50 Hz, 정지 IMU 100 Hz로 낸다.
확인 (docs/test-plan.md I6):
1. map_server·AMCL lifecycle active
2. TF map → <ns>/odom → <ns>/base_footprint 연결
3. odometry/filtered(ekf_global) 주기: ekf.yaml frequency가 50 Hz(KPI)이고, EKF 자체 진단(/diagnostics의 실측 주기)이
   정상(OK, robot_localization 허용 범위)이다. 테스트 노드가 수신 간격을 재면 테스트 쪽 지연이 섞여 EKF 진단을 쓴다
입력은 따로 노드를 만들어 백그라운드 스레드에서 낸다. 테스트 스레드가 확인하며 같이 내면 입력이 몰렸다 끊겨 EKF 실측이
39 Hz까지 떨어졌다 (같은 스택에 입력만 멀티스레드로 내면 두 EKF 모두 50.0 Hz)
4. amcl_pose·odometry/filtered가 실제 위치 (5, 5)에서 0.1 m 안
"""

import math
import os
import tempfile
import threading
import time
import unittest

import yaml

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

# 다른 패키지 테스트와 겹치지 않게 전용 네임스페이스를 쓴다
NS = 'i6_amr'
RESOLUTION = 0.05
SIZE_M = 10.0
WALL_M = 0.1
PILLAR = (2.0, 3.0, 7.0, 8.0)        # 기둥 [x_min, x_max, y_min, y_max] (m). 방이 대칭이면 AMCL 방향이 갈린다
ROBOT = (5.0, 5.0)
NUM_BEAMS = 360
RANGE_MAX = 12.0
EKF_HZ = 50.0
WAIT_S = 30.0


def _occupied(x, y):
    px0, px1, py0, py1 = PILLAR
    wall = x < WALL_M or y < WALL_M or x > SIZE_M - WALL_M or y > SIZE_M - WALL_M
    return wall or (px0 <= x <= px1 and py0 <= y <= py1)


def write_map(folder):
    """P5 PGM + map_server yaml. 원점 (0, 0), 0.05 m 칸."""
    n = round(SIZE_M / RESOLUTION)
    rows = []
    for j in range(n - 1, -1, -1):          # PGM 첫 줄이 지도 위쪽(y 큰 쪽)
        y = (j + 0.5) * RESOLUTION
        rows.append(bytes(0 if _occupied((i + 0.5) * RESOLUTION, y) else 254 for i in range(n)))
    pgm = os.path.join(folder, 'room.pgm')
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{n} {n}\n255\n'.encode() + b''.join(rows))
    yaml_path = os.path.join(folder, 'room.yaml')
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(f'image: {pgm}\nmode: trinary\nresolution: {RESOLUTION}\norigin: [0.0, 0.0, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return yaml_path


def _ray_box(ox, oy, dx, dy, box):
    """반직선이 상자 [x0, x1, y0, y1]에 처음 닿는 거리 (slab). 안 닿으면 inf."""
    x0, x1, y0, y1 = box
    t_near, t_far = -math.inf, math.inf
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            if not lo <= o <= hi:
                return math.inf
            continue
        a, b = (lo - o) / d, (hi - o) / d
        t_near, t_far = max(t_near, min(a, b)), min(t_far, max(a, b))
    return t_near if 0.0 < t_near <= t_far else math.inf


def scan_ranges():
    """ROBOT에서 본 거리 (방향 0 = +x). 방 안쪽 벽면까지와 기둥까지 중 가까운 쪽."""
    lo, hi = WALL_M, SIZE_M - WALL_M
    ranges = []
    for k in range(NUM_BEAMS):
        a = -math.pi + 2.0 * math.pi * k / NUM_BEAMS
        dx, dy = math.cos(a), math.sin(a)
        walls = [t for t in ((lo - ROBOT[0]) / dx if dx < 0 else (hi - ROBOT[0]) / dx if dx > 0 else math.inf,
                             (lo - ROBOT[1]) / dy if dy < 0 else (hi - ROBOT[1]) / dy if dy > 0 else math.inf)]
        ranges.append(min(min(walls), _ray_box(ROBOT[0], ROBOT[1], dx, dy, PILLAR), RANGE_MAX))
    return ranges


@pytest.mark.launch_test
def generate_test_description():
    map_yaml = write_map(tempfile.mkdtemp())
    stack = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('amr_localization'), 'launch', 'localization.launch.py')),
        launch_arguments={
            'namespace': NS, 'use_sim_time': 'false', 'map': map_yaml,
            'initial_x': str(ROBOT[0]), 'initial_y': str(ROBOT[1]), 'initial_yaw': '0.0',
        }.items())
    return launch.LaunchDescription([stack, launch_testing.actions.ReadyToTest()])


class TestLocalizationStack(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_localization_launch')
        node = cls.node
        # 입력을 내는 노드 (백그라운드 스레드). 확인용 구독은 node가 테스트 스레드에서 돈다
        cls.inputs = rclpy.create_node('test_localization_inputs')
        tf = TransformStamped()
        tf.header.stamp = cls.inputs.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = f'{NS}/base_footprint', f'{NS}/lidar_link'
        tf.transform.rotation.w = 1.0
        cls.static_tf = StaticTransformBroadcaster(cls.inputs)
        cls.static_tf.sendTransform(tf)
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, node)
        cls.filtered, cls.amcl = [], []
        node.create_subscription(Odometry, f'/{NS}/odometry/filtered',
                                 lambda m: cls.filtered.append((time.monotonic(), m)), 50)
        node.create_subscription(PoseWithCovarianceStamped, f'/{NS}/amcl_pose', cls.amcl.append, 10)
        # ekf_global이 스스로 잰 출력 주기 (print_diagnostics: true)
        cls.ekf_diag = []
        node.create_subscription(DiagnosticArray, '/diagnostics', lambda m: cls.ekf_diag.extend(
            st for st in m.status if st.name == 'ekf_global: odometry/filtered topic status'), 50)
        inputs = cls.inputs
        cls.scan_pub = inputs.create_publisher(LaserScan, f'/{NS}/scan', qos_profile_sensor_data)
        cls.odom_pub = inputs.create_publisher(Odometry, f'/{NS}/odom', 10)
        cls.imu_pub = inputs.create_publisher(Imu, f'/{NS}/imu/data', 10)
        cls.ranges = scan_ranges()
        inputs.create_timer(0.1, cls._publish_scan)
        inputs.create_timer(0.02, cls._publish_odom)
        inputs.create_timer(0.01, cls._publish_imu)
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(inputs)
        cls.spinner = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spinner.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.spinner.join(timeout=5.0)
        cls.inputs.destroy_node()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _publish_scan(cls):
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = cls.inputs.get_clock().now().to_msg(), f'{NS}/lidar_link'
        scan.angle_min, scan.angle_increment = -math.pi, 2.0 * math.pi / NUM_BEAMS
        scan.angle_max = scan.angle_min + scan.angle_increment * (NUM_BEAMS - 1)
        scan.range_min, scan.range_max = 0.12, RANGE_MAX
        scan.ranges = cls.ranges
        cls.scan_pub.publish(scan)

    @classmethod
    def _publish_odom(cls):
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = cls.inputs.get_clock().now().to_msg(), f'{NS}/odom'
        odom.child_frame_id = f'{NS}/base_footprint'
        odom.pose.pose.orientation.w = 1.0
        odom.twist.covariance[0] = odom.twist.covariance[7] = odom.twist.covariance[35] = 1e-4
        cls.odom_pub.publish(odom)

    @classmethod
    def _publish_imu(cls):
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = cls.inputs.get_clock().now().to_msg(), f'{NS}/imu_link'
        imu.orientation.w = 1.0
        imu.angular_velocity_covariance[8] = 1e-4
        imu.linear_acceleration.z = 9.81
        cls.imu_pub.publish(imu)

    # ------------------------------------------------------------------ 도우미
    def spin_until(self, condition, timeout=WAIT_S):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not condition():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return condition()

    def lifecycle_state(self, service):
        client = self.node.create_client(GetState, service)
        try:
            if not client.wait_for_service(timeout_sec=WAIT_S):
                return None
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            return future.result().current_state.id if future.result() else None
        finally:
            self.node.destroy_client(client)

    # ------------------------------------------------------------------ 확인
    def test_1_lifecycle_active(self):
        for service in ('/map_server/get_state', f'/{NS}/amcl/get_state'):
            self.assertTrue(
                self.spin_until(lambda: self.lifecycle_state(service) == State.PRIMARY_STATE_ACTIVE),
                f'{service}가 active가 아니다')

    def test_2_tf_chain(self):
        ok = self.spin_until(lambda: self.tf_buffer.can_transform('map', f'{NS}/base_footprint', Time()))
        self.assertTrue(ok, f'TF map → {NS}/odom → {NS}/base_footprint가 이어지지 않는다')
        self.assertTrue(self.tf_buffer.can_transform(f'{NS}/odom', f'{NS}/base_footprint', Time()))

    def test_3_ekf_rate(self):
        ekf = os.path.join(get_package_share_directory('amr_localization'), 'config', 'ekf.yaml')
        with open(ekf, encoding='utf-8') as f:
            config = yaml.safe_load(f)['/**']
        self.assertGreaterEqual(config['ekf_global']['ros__parameters']['frequency'], EKF_HZ)
        self.assertTrue(self.spin_until(lambda: len(self.filtered) > 10), 'odometry/filtered가 나오지 않는다')
        # 기동 직후 창이 아니라 그 다음 창(3초)을 본다
        self.ekf_diag.clear()
        self.assertTrue(self.spin_until(lambda: len(self.ekf_diag) >= 2), 'ekf_global 진단이 오지 않는다')
        status = self.ekf_diag[-1]
        values = {kv.key: kv.value for kv in status.values}
        low, high = values.get('Minimum acceptable frequency (Hz)'), values.get('Maximum acceptable frequency (Hz)')
        print(f"ekf_global odometry/filtered 실측 {values.get('Actual frequency (Hz)')} Hz (허용 {low}~{high})")
        self.assertEqual(status.level, DiagnosticStatus.OK, f'EKF 진단 {status.message}: {values}')

    def test_4_pose_at_robot(self):
        self.assertTrue(self.spin_until(lambda: bool(self.amcl)), 'amcl_pose가 나오지 않는다')
        p = self.amcl[-1].pose.pose.position
        self.assertLess(math.hypot(p.x - ROBOT[0], p.y - ROBOT[1]), 0.1, f'amcl_pose ({p.x:.3f}, {p.y:.3f})')
        q = self.filtered[-1][1].pose.pose.position
        self.assertLess(math.hypot(q.x - ROBOT[0], q.y - ROBOT[1]), 0.1,
                        f'odometry/filtered ({q.x:.3f}, {q.y:.3f})')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -6, -15])
