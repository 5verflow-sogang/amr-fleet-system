"""zone_client_node를 실제 amr_fleet_manager 서버·amr_fault_injector 중계와 함께 띄워 확인한다 (launch_testing).

요청(경계 3 m 앞) → 허가(대기 로봇 없음 → 바로 GRANTED) → 진입 → 반납까지.
위치는 TF(map → base_footprint)를, 경로는 nav_msgs/Path를 이 테스트가 직접 발행해서 흉내 낸다.
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
from amr_msgs.msg import ZoneGrant, ZoneRequest
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster

ZONES_YAML = 'zones:\n  - id: test_zone\n    capacity: 1\n    area_world: [5.0, 6.0, -1.0, 1.0]\n'


@pytest.mark.launch_test
def generate_test_description():
    zones = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    zones.write(ZONES_YAML)
    zones.close()

    server = launch_ros.actions.Node(
        package='amr_fleet_manager', executable='fleet_manager',
        parameters=[{'traffic.zones_file': zones.name}])
    relay = launch_ros.actions.Node(
        package='amr_fault_injector', executable='fault_injector',
        parameters=[{'robot_ids': ['amr_01'], 'topics': ['zone_grants'], 'delay_ms': 0.0, 'loss_pct': 0.0}])
    client = launch_ros.actions.Node(
        package='amr_zone_client', executable='zone_client_node', namespace='amr_01',
        parameters=[{
            'robot_id': 'amr_01', 'zones_file': zones.name, 'update_rate_hz': 20.0,
            'trigger_distance_m': 3.0, 'stop_margin_m': 0.5,
        }])
    return launch.LaunchDescription([
        server, relay, client, launch_testing.actions.ReadyToTest(),
    ])


class TestZoneClientLive(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_zone_client_live')
        self.tf = TransformBroadcaster(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def spin_until(self, condition, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not condition():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return condition()

    def publish_robot_x(self, x: float):
        tf = TransformStamped()
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = x
        tf.transform.rotation.w = 1.0
        self.tf.sendTransform(tf)

    def test_request_wait_grant_enter_release(self):
        requests, grants = [], []
        self.node.create_subscription(ZoneRequest, '/fleet/zone_requests', requests.append, 10)
        self.node.create_subscription(ZoneGrant, '/amr_01/fleet/zone_grants', grants.append, 10)
        path_pub = self.node.create_publisher(Path, '/amr_01/plan', 10)

        # Nav2 /plan은 로봇 현재 위치에서 다시 계산되므로 항상 로봇 앞쪽만 담는다. 여기서도 로봇
        # 시작 위치(x=2.0) 바로 앞부터 목표까지로 만든다 (뒤쪽 점이 있으면 거리 계산이 거꾸로 꼬인다)
        path = Path()
        path.header.frame_id = 'map'
        for i in range(1, 81):
            pose = PoseStamped()
            pose.pose.position.x = 2.0 + float(i) * 0.1
            path.poses.append(pose)

        self.assertTrue(self.spin_until(lambda: path_pub.get_subscription_count() >= 1, 15.0))
        path_pub.publish(path)

        # 경계(5.0) 3 m 앞에서 TF를 계속 흘려준다 (노드 타이머가 주기적으로 읽는다)
        end = time.monotonic() + 5.0
        while time.monotonic() < end and not requests:
            self.publish_robot_x(2.0)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(requests, 'ZoneRequest가 발행되지 않음')
        self.assertEqual(requests[0].type, ZoneRequest.TYPE_ACCESS)
        request_id = requests[0].request_id

        def granted():
            self.publish_robot_x(2.0)
            return any(g.grant == ZoneGrant.GRANT_GRANTED for g in grants)
        self.assertTrue(self.spin_until(granted, 10.0), f'GRANTED를 못 받음: {[g.grant for g in grants]}')

        # 구역 안으로 진입
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            self.publish_robot_x(5.5)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # 반대편으로 빠져나온다
        del requests[:]
        end = time.monotonic() + 5.0
        released = False
        while time.monotonic() < end and not released:
            self.publish_robot_x(7.0)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            released = any(r.type == ZoneRequest.TYPE_RELEASE for r in requests)
        self.assertTrue(released, f'RELEASE가 발행되지 않음: {[(r.type, r.request_id) for r in requests]}')
        release = next(r for r in requests if r.type == ZoneRequest.TYPE_RELEASE)
        self.assertEqual(release.request_id, request_id)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
