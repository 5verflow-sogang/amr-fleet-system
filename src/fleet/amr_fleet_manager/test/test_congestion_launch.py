"""혼잡도 노드: 손님 정답 위치(world)를 map으로 옮겨 구역별 혼잡도·마스크·필터 정보를 내는지 (launch_testing)."""

import os
import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from amr_msgs.msg import ZoneCongestionArray
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

ZONES = """zones:
  - {id: line, area: [0.0, 4.0, 0.0, 1.0], capacity: 2}
  - {id: hall, area: [4.0, 6.0, 0.0, 1.0], capacity: 1}
"""
OBSTACLES = """obstacles:
  - {id: guest_a}
  - {id: guest_b}
"""
# world = map + (0, 1.5)
ORIGIN = '[0.0, 1.5, 0.0]'


@pytest.mark.launch_test
def generate_test_description():
    tmp = tempfile.mkdtemp()
    zones, obstacles = os.path.join(tmp, 'zones.yaml'), os.path.join(tmp, 'obstacles.yaml')
    with open(zones, 'w', encoding='utf-8') as f:
        f.write(ZONES)
    with open(obstacles, 'w', encoding='utf-8') as f:
        f.write(OBSTACLES)
    node = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('amr_fleet_manager'), 'launch', 'congestion.launch.py')),
        launch_arguments={'zones_file': zones, 'obstacles_file': obstacles, 'map_origin_in_world': ORIGIN,
                          'use_sim_time': 'false'}.items(),
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestCongestion(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_congestion')

    def tearDown(self):
        self.node.destroy_node()

    def test_guests_in_line_raise_its_congestion_and_mask(self):
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        got = {'congestion': [], 'mask': [], 'keepout': [], 'speed': []}
        self.node.create_subscription(ZoneCongestionArray, '/fleet/congestion',
                                      lambda m: got['congestion'].append(m), 10)
        self.node.create_subscription(OccupancyGrid, '/fleet/congestion_mask', lambda m: got['mask'].append(m), latched)
        self.node.create_subscription(CostmapFilterInfo, '/fleet/congestion_keepout_info',
                                      lambda m: got['keepout'].append(m), latched)
        self.node.create_subscription(CostmapFilterInfo, '/fleet/congestion_speed_info',
                                      lambda m: got['speed'].append(m), latched)
        pubs = {oid: self.node.create_publisher(Odometry, f'/dynamic_obstacles/{oid}/ground_truth/odom', 10)
                for oid in ('guest_a', 'guest_b')}

        def odom(x, y):
            m = Odometry()
            m.header.frame_id = 'world'
            m.pose.pose.position.x, m.pose.pose.position.y = x, y
            return m

        # 두 손님 모두 world (1, 2) / (3, 2) = map (1, 0.5) / (3, 0.5) → line 구역 2명 = 정원
        end = time.time() + 15.0
        while time.time() < end:
            pubs['guest_a'].publish(odom(1.0, 2.0))
            pubs['guest_b'].publish(odom(3.0, 2.0))
            rclpy.spin_once(self.node, timeout_sec=0.1)
            last = got['congestion'][-1] if got['congestion'] else None
            if last and last.zones[0].value > 0.95 and got['mask'] and got['keepout'] and got['speed']:
                break
        self.assertTrue(got['congestion'], '/fleet/congestion을 받지 못했다')
        line, hall = got['congestion'][-1].zones
        self.assertEqual((line.zone_id, line.people_count, hall.people_count), ('line', 2, 0))
        self.assertGreater(line.value, 0.95)
        self.assertEqual(hall.value, 0.0)
        self.assertEqual(list(line.area), [0.0, 4.0, 0.0, 1.0])

        mask = got['mask'][-1]
        # 구역 x 0~6, y 0~1 + 둘레 1 m (mask.padding) → 원점 (-1, -1), 0.1 m 칸 80 × 30
        self.assertEqual((mask.header.frame_id, mask.info.width, mask.info.height), ('map', 80, 30))
        self.assertEqual((mask.info.origin.position.x, mask.info.origin.position.y), (-1.0, -1.0))
        w = mask.info.width
        row = list(mask.data[15 * w:16 * w])          # y 0.5
        self.assertEqual(max(row[:10]), 0)            # 둘레: 제한 없음
        self.assertGreaterEqual(min(row[10:50]), 86)  # line: round(value × 90), value가 1에 가깝다
        self.assertEqual(max(row[50:]), 0)            # hall은 비어 있음 + 둘레
        self.assertEqual(max(mask.data[:10 * w]), 0)  # 아래 둘레

        keepout, speed = got['keepout'][-1], got['speed'][-1]
        self.assertEqual((keepout.type, keepout.base, keepout.multiplier), (0, 0.0, 1.0))
        self.assertEqual((speed.type, speed.base, speed.multiplier), (1, 100.0, -0.5))
        self.assertEqual(speed.filter_mask_topic, '/fleet/congestion_mask')
