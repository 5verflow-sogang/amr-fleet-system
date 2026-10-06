"""로봇 측 구역 진입 처리 노드. amr_fleet_manager(#57)의 ZoneRequest/ZoneGrant 서버와 짝이다.

계획 경로(/<ns>/plan)와 현재 위치(map_frame → base_frame TF)로 구역 경계까지 남은 거리를 계산해
ZoneRequest를 보내고, 허가 전까지는 cmd_vel을 감속·정지시킨다. 흐름은 README '동작 규칙' 참고.

cmd_vel 체인에서의 위치는 README 'cmd_vel 체인' 절 참고 — 최종 cmd_vel 발행은 이 노드 몫이
아니다 (안전 노드가 가로채는 게 팀 결정, project-spec.md '안전 정지 책임' 절).
"""

from __future__ import annotations

from typing import List, Optional

import rclpy
from amr_msgs.msg import ZoneGrant, ZoneRequest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .core import TYPE_RELEASE, ZoneClientConfig, ZoneClientCore
from .zones_config import load_zone_configs


class ZoneClientNode(Node):

    def __init__(self):
        super().__init__('zone_client')
        p = self.declare_parameter
        self._robot_id = p('robot_id', 'amr_01').value
        zones_file = p('zones_file', '').value
        self._map_frame = p('map_frame', 'map').value
        self._base_frame = p('base_frame', 'base_footprint').value
        plan_topic = p('plan_topic', 'plan').value
        # 서버 → 로봇(zone_grants)은 장애 주입기가 중계하므로 네임스페이스 안 상대 이름을 기본값으로 쓴다.
        # zone_requests(로봇 → 서버)는 중계되지 않는 절대 토픽이다 (fleet_manager_node.py, api.md 4-2절)
        zone_grants_topic = p('zone_grants_topic', 'fleet/zone_grants').value
        zone_requests_topic = p('zone_requests_topic', '/fleet/zone_requests').value
        # cmd_vel 체인: Nav2 controller_server(향후 velocity_smoother)의 출력을 받아 구역 상한만 걸고,
        # amr_safety에 넘긴다. 최종 cmd_vel 발행(가로채기)은 안전 노드의 몫이다 (9/28 회의,
        # project-spec.md '안전 정지 책임' 절, api.md 4-4절) — 그래서 출력 기본값을 'cmd_vel'로 두지
        # 않는다. amr_safety가 구현되기 전에는 amr_bringup이 이 토픽을 로봇 cmd_vel로 임시 리맵한다
        cmd_vel_in_topic = p('cmd_vel_in_topic', 'cmd_vel_nav').value
        cmd_vel_out_topic = p('cmd_vel_out_topic', 'cmd_vel_zone_limited').value
        update_rate_hz = p('update_rate_hz', 10.0).value

        zones = load_zone_configs(zones_file)
        self._core = ZoneClientCore(ZoneClientConfig(
            robot_id=self._robot_id,
            zones=zones,
            trigger_distance_m=p('trigger_distance_m', 3.0).value,
            stop_margin_m=p('stop_margin_m', 0.5).value,
            decel_mps2=p('decel_mps2', 1.0).value,
            resend_period_s=p('resend_period_s', 1.0).value,
        ))

        self._path_xy: List[tuple] = []
        self._speed_cap: Optional[float] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._request_pub = self.create_publisher(ZoneRequest, zone_requests_topic, qos)
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_out_topic, 10)
        self.create_subscription(ZoneGrant, zone_grants_topic, self._on_grant, qos)
        self.create_subscription(Path, plan_topic, self._on_path, 10)
        self.create_subscription(Twist, cmd_vel_in_topic, self._on_cmd_vel, 10)
        self.create_timer(1.0 / update_rate_hz, self._update)

        self.get_logger().info(
            f'zone_client({self._robot_id}): 구역 {zones and [z.zone_id for z in zones] or "없음"}, '
            f'구독 {zone_grants_topic} (허가), {plan_topic} (경로) · 발행 {zone_requests_topic}')

    def _on_path(self, msg: Path):
        self._path_xy = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def _on_grant(self, msg: ZoneGrant):
        lease_expiry = None
        if msg.grant == ZoneGrant.GRANT_GRANTED:
            lease_expiry = Time.from_msg(msg.lease_expiry).nanoseconds * 1e-9
        warning = self._core.on_grant(
            msg.robot_id, msg.zone_id, msg.request_id, msg.grant, lease_expiry, msg.reason, msg.holder_robot_id)
        if warning:
            self.get_logger().warn(warning)

    def _on_cmd_vel(self, msg: Twist):
        out = Twist()
        out.linear.x = min(msg.linear.x, self._speed_cap) if self._speed_cap is not None else msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular = msg.angular
        self._cmd_vel_pub.publish(out)

    def _lookup_robot_xy(self):
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f'{self._map_frame}→{self._base_frame} TF 없음: {e}', throttle_duration_sec=5.0)
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _update(self):
        robot_xy = self._lookup_robot_xy()
        if robot_xy is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        outgoing, speed_cap = self._core.update(now, self._path_xy, robot_xy)
        self._speed_cap = speed_cap
        for item in outgoing:
            msg = ZoneRequest()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.request_id = item.request_id
            msg.robot_id = self._robot_id
            msg.zone_id = item.zone_id
            msg.type = item.type
            self._request_pub.publish(msg)
            if item.type == TYPE_RELEASE:
                self.get_logger().info(f'구역 {item.zone_id} 반납 (request_id={item.request_id})')


def main(args=None):
    rclpy.init(args=args)
    node = ZoneClientNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
