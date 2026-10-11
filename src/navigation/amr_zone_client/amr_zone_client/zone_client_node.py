"""로봇 측 구역 진입 처리 노드. amr_fleet_manager(#57)의 ZoneRequest/ZoneGrant 서버와 짝이다.

계획 경로(/<ns>/plan)와 현재 위치(map_frame → base_frame TF)로 구역 경계까지 남은 거리를 계산해
ZoneRequest를 보내고, 허가 전까지는 cmd_vel을 감속·정지시킨다. 흐름은 README '동작 규칙' 참고.

구역 상한에 더해 safety/status(SafetyStatus.speed_limit)도 반영한다(#69·#70) - 둘 중 더
작은 쪽을 쓰고, 끊기면 제한 없음으로 본다. README 'SafetyStatus 연동' 절 참고.

cmd_vel 체인에서의 위치는 README 'cmd_vel 체인' 절 참고 — 최종 cmd_vel 발행은 이 노드 몫이
아니다 (안전 노드가 가로채는 게 팀 결정, project-spec.md '안전 정지 책임' 절).
"""

from __future__ import annotations

from typing import List, Optional

import rclpy
from amr_msgs.msg import SafetyStatus, ZoneGrant, ZoneRequest
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
        # 기본값은 노드 네임스페이스에서 가져온다(/amr_03 -> amr_03). 네임스페이스가 없으면(테스트 등)
        # amr_01로 떨어진다. 파라미터로 주면 그 값이 우선한다 (#66)
        default_robot_id = self.get_namespace().strip('/') or 'amr_01'
        self._robot_id = p('robot_id', default_robot_id).value
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

        # SafetyStatus 연동 (#69·#70): 네임스페이스 안 상대 이름(amr_msgs/msg/SafetyStatus.msg
        # 주석 "토픽: safety/status"). 끊기면(이 시간 동안 못 받으면) 제한 없음으로 본다 - 최종
        # 게이트가 amr_safety라서다 (docs/algorithms/navigation.md '5-1. speed_limit 연동').
        safety_status_topic = p('safety_status_topic', 'safety/status').value
        safety_status_timeout_s = p('safety_status_timeout_s', 1.0).value

        # lookahead_m: 0.0(기본)이면 core가 trigger_distance_m + 여유로 알아서 정한다. ROS 파라미터는
        # None을 못 담아서 0.0을 "안 줌"의 신호로 쓴다 (#66)
        lookahead_m = p('lookahead_m', 0.0).value

        zones = load_zone_configs(zones_file)
        self._core = ZoneClientCore(ZoneClientConfig(
            robot_id=self._robot_id,
            zones=zones,
            trigger_distance_m=p('trigger_distance_m', 3.0).value,
            stop_margin_m=p('stop_margin_m', 0.5).value,
            decel_mps2=p('decel_mps2', 1.0).value,
            resend_period_s=p('resend_period_s', 1.0).value,
            retry_period_s=p('retry_period_s', 5.0).value,
            lookahead_m=lookahead_m if lookahead_m > 0.0 else None,
            max_linear_velocity_mps=p('max_linear_velocity_mps', 2.0).value,
        ))

        self._path_xy: List[tuple] = []
        self._speed_cap: Optional[float] = None
        self._safety_status_timeout_s = safety_status_timeout_s
        self._safety_speed_limit: Optional[float] = None
        self._safety_status_last_t: Optional[float] = None
        # speed_limit 램프 시작점을 낮추는 데 쓴다 (core.update의 current_speed_mps,
        # #93 연우 리뷰) - 마지막으로 내보낸(상한 적용 후) cmd_vel.linear.x.
        self._last_cmd_vel_mps: float = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._request_pub = self.create_publisher(ZoneRequest, zone_requests_topic, qos)
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_out_topic, 10)
        self.create_subscription(ZoneGrant, zone_grants_topic, self._on_grant, qos)
        self.create_subscription(Path, plan_topic, self._on_path, 10)
        self.create_subscription(Twist, cmd_vel_in_topic, self._on_cmd_vel, 10)
        self.create_subscription(SafetyStatus, safety_status_topic, self._on_safety_status, 10)
        self.create_timer(1.0 / update_rate_hz, self._update)

        self.get_logger().info(
            f'zone_client({self._robot_id}): 구역 {zones and [z.zone_id for z in zones] or "없음"}, '
            f'구독 {zone_grants_topic} (허가), {plan_topic} (경로), {safety_status_topic} '
            f'(speed_limit) · 발행 {zone_requests_topic}')

    def _on_path(self, msg: Path):
        self._path_xy = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def _on_grant(self, msg: ZoneGrant):
        lease_expiry = None
        if msg.grant == ZoneGrant.GRANT_GRANTED:
            lease_expiry = Time.from_msg(msg.lease_expiry).nanoseconds * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        warning = self._core.on_grant(
            now, msg.robot_id, msg.zone_id, msg.request_id, msg.grant, lease_expiry, msg.reason,
            msg.holder_robot_id)
        if warning:
            self.get_logger().warn(warning)

    def _on_safety_status(self, msg: SafetyStatus):
        self._safety_speed_limit = msg.speed_limit
        self._safety_status_last_t = self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd_vel(self, msg: Twist):
        out = Twist()
        out.linear.x = min(msg.linear.x, self._speed_cap) if self._speed_cap is not None else msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular = msg.angular
        self._cmd_vel_pub.publish(out)
        self._last_cmd_vel_mps = out.linear.x

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
            # 위치를 모르면 fail-closed: 상한을 0으로 묶어 멈춰 세운다. 경고는 TF 조회 쪽에서
            # 이미 throttle해서 남긴다 (#66)
            self._speed_cap = 0.0
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        # 끊김(한 번도 못 받았거나 timeout 넘게 조용함) 판단은 노드 몫이다 - core는 ROS 비의존
        # 이라 타임스탬프 비교를 모른다. 끊겼으면 None(제한 없음)을 넘긴다 (#69).
        safety_limit = self._safety_speed_limit
        if (self._safety_status_last_t is None
                or now - self._safety_status_last_t > self._safety_status_timeout_s):
            safety_limit = None
        outgoing, speed_cap = self._core.update(
            now, self._path_xy, robot_xy, safety_limit, self._last_cmd_vel_mps)
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
