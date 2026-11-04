"""혼잡도 노드: 손님(동적 장애물) 정답 위치 → 구역별 혼잡도 + Nav2 costmap 마스크. 정의는 docs/api.md 4-5절.

인식 결과가 아니라 시뮬레이터 정답 위치(/dynamic_obstacles/<id>/ground_truth/odom, frame "world")를 쓴다.
시연이 인식 불안정에 흔들리지 않게 하려는 것이다 (7주차 결정). world → map은 map_origin_in_world로 옮긴다.
"""

from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from amr_fleet_core.congestion import CongestionTracker, Mask, ZoneValue, make_mask, zones_from_dict
from amr_msgs.msg import ZoneCongestion, ZoneCongestionArray
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .task_gen import world_to_map

# nav2_msgs/msg/CostmapFilterInfo.type
FILTER_KEEPOUT = 0
FILTER_SPEED_PERCENT = 1


def load_obstacle_ids(path: str) -> list[str]:
    """amr_gazebo dynamic_obstacles.yaml 형식에서 장애물 id 목록."""
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
    return [str(o['id']) for o in data.get('obstacles') or []]


def to_array_msg(values: list[ZoneValue], areas: list[tuple], frame_id: str, stamp) -> ZoneCongestionArray:
    msg = ZoneCongestionArray()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    for v, area in zip(values, areas):
        msg.zones.append(ZoneCongestion(zone_id=v.zone_id, area=[float(a) for a in area],
                                        people_count=v.people_count, value=float(v.value)))
    return msg


def to_grid_msg(mask: Mask, frame_id: str, stamp) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.info.map_load_time = stamp
    msg.info.resolution = float(mask.resolution)
    msg.info.width = mask.width
    msg.info.height = mask.height
    msg.info.origin.position.x = float(mask.origin[0])
    msg.info.origin.position.y = float(mask.origin[1])
    msg.info.origin.orientation.w = 1.0
    msg.data = mask.data
    return msg


class CongestionNode(Node):

    def __init__(self):
        super().__init__('congestion')
        zones_file = self.declare_parameter('zones_file', '').value
        obstacles_file = self.declare_parameter('obstacles_file', '').value
        # 비어 있지 않으면 obstacles_file 대신 이 id들을 쓴다
        obstacle_ids = list(self.declare_parameter('obstacle_ids', ['']).value)
        self._origin = list(self.declare_parameter('map_origin_in_world', [-0.0439, 1.4712, -0.00088]).value)
        rate_hz = self.declare_parameter('rate_hz', 2.0).value
        alpha = self.declare_parameter('alpha', 0.5).value
        self._frame_id = self.declare_parameter('frame_id', 'map').value
        self._resolution = self.declare_parameter('mask.resolution', 0.1).value
        self._max_value = self.declare_parameter('mask.max_value', 90).value
        # 이보다 오래 위치가 안 온 손님은 세지 않는다 (시뮬레이션 시간)
        self._stale_s = self.declare_parameter('stale_s', 2.0).value
        speed_base = self.declare_parameter('speed_filter.base', 100.0).value
        speed_multiplier = self.declare_parameter('speed_filter.multiplier', -0.5).value

        if not zones_file:
            raise ValueError('zones_file이 필요하다 (혼잡도 구역 파일, api.md 4-5절)')
        zones = zones_from_dict(yaml.safe_load(Path(zones_file).read_text(encoding='utf-8')))
        if not zones:
            raise ValueError(f'구역이 없다: {zones_file}')
        self._areas = [z.area for z in zones]
        self._tracker = CongestionTracker(zones, alpha)
        ids = [i for i in obstacle_ids if i] or (load_obstacle_ids(obstacles_file) if obstacles_file else [])
        if not ids:
            raise ValueError('손님 id가 없다: obstacles_file 또는 obstacle_ids를 준다')

        # 손님 id → (수신 시각 [s], map 좌표)
        self._people: dict[str, tuple[float, tuple[float, float]]] = {}
        for oid in ids:
            self.create_subscription(Odometry, f'/dynamic_obstacles/{oid}/ground_truth/odom',
                                     lambda msg, oid=oid: self._on_odom(oid, msg), 10)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._congestion_pub = self.create_publisher(
            ZoneCongestionArray, '/fleet/congestion', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self._mask_pub = self.create_publisher(OccupancyGrid, '/fleet/congestion_mask', latched)
        # 마스크 하나를 keepout(돌아가기)·speed(천천히) 필터가 같이 쓴다. 어느 쪽을 켤지는 Nav2 설정에서 고른다
        self._info_pubs = []
        for topic, ftype, base, mult in (
                ('/fleet/congestion_keepout_info', FILTER_KEEPOUT, 0.0, 1.0),
                ('/fleet/congestion_speed_info', FILTER_SPEED_PERCENT, speed_base, speed_multiplier)):
            pub = self.create_publisher(CostmapFilterInfo, topic, latched)
            info = CostmapFilterInfo(type=ftype, filter_mask_topic='/fleet/congestion_mask',
                                     base=float(base), multiplier=float(mult))
            info.header.frame_id = self._frame_id
            pub.publish(info)
            self._info_pubs.append(pub)

        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'혼잡도: 구역 {len(zones)}개 ({zones_file}), 손님 {len(ids)}명, {rate_hz} Hz')

    def _on_odom(self, oid: str, msg: Odometry):
        p = msg.pose.pose.position
        x, y, _ = world_to_map((p.x, p.y, 0.0), self._origin)
        self._people[oid] = (self.get_clock().now().nanoseconds * 1e-9, (x, y))

    def _tick(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        people = [xy for seen, xy in self._people.values() if t - seen <= self._stale_s]
        values = self._tracker.update(people)
        stamp = now.to_msg()
        self._congestion_pub.publish(to_array_msg(values, self._areas, self._frame_id, stamp))
        mask = make_mask(self._tracker.zones, [v.value for v in values], self._resolution, self._max_value)
        self._mask_pub.publish(to_grid_msg(mask, self._frame_id, stamp))


def main(args=None):
    rclpy.init(args=args)
    node = CongestionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
