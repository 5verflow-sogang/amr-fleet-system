"""로봇 측 구역 진입 처리 노드. amr_fleet_manager(#57)의 ZoneRequest/ZoneGrant 서버와 짝이다.

계획 경로(/<ns>/plan)와 현재 위치(map_frame → base_frame TF)로 구역 경계까지 남은 거리를 계산해
ZoneRequest를 보내고, 허가 전까지는 cmd_vel을 감속·정지시킨다. 흐름은 README '동작 규칙' 참고.

구역 상한에 더해 safety/status(SafetyStatus.speed_limit)도 반영한다(#69·#70) - 둘 중 더
작은 쪽을 쓰고, 끊기면 제한 없음으로 본다. README 'SafetyStatus 연동' 절 참고.

cmd_vel 체인에서의 위치는 README 'cmd_vel 체인' 절 참고 — 최종 cmd_vel 발행은 이 노드 몫이
아니다 (안전 노드가 가로채는 게 팀 결정, project-spec.md '안전 정지 책임' 절).

BACKING(교착 해소, 10/11 결정) 중에는 cmd_vel_nav를 무시하고 core가 낸 후진 속도를 직접
cmd_vel_zone_limited로 낸다 - README '동작 규칙' 절, core.py 모듈 설명 참고.
"""

from __future__ import annotations

import math
import signal
from typing import List, Optional

import rclpy
from amr_msgs.msg import SafetyStatus, ZoneGrant, ZoneRequest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .core import TYPE_RELEASE, ZoneClientConfig, ZoneClientCore
from .zones_config import load_zone_configs, transform_zones

_STATE_TOPIC = 'zone_client/state'
_STATE_PUBLISH_PERIOD_S = 1.0


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
        # zones.yaml area_world의 좌표계. 비우면 map_frame 좌표로 그대로 본다 (TF 없는 테스트).
        # 'world'면 TF map ← world로 바꿔 쓴다 - 지도 원점이 world와 달라 그대로 쓰면 구역이 어긋난다
        self._zones_frame = p('zones_frame', '').value
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
        self._raw_zones = zones
        # zones_frame이 있으면 TF가 들어온 뒤(_update) 구역을 map 좌표로 바꿔 core를 만든다. 그때까지 None
        self._core: Optional[ZoneClientCore] = None
        self._core_params = dict(
            robot_id=self._robot_id,
            trigger_distance_m=p('trigger_distance_m', 3.0).value,
            stop_margin_m=p('stop_margin_m', 0.5).value,
            waiting_margin_m=p('waiting_margin_m', 0.15).value,
            decel_mps2=p('decel_mps2', 1.0).value,
            resend_period_s=p('resend_period_s', 1.0).value,
            retry_period_s=p('retry_period_s', 5.0).value,
            lookahead_m=lookahead_m if lookahead_m > 0.0 else None,
            max_linear_velocity_mps=p('max_linear_velocity_mps', 2.0).value,
            backing_speed_mps=p('backing_speed_mps', 0.15).value,
            backing_margin_m=p('backing_margin_m', 0.3).value,
            backing_max_distance_m=p('backing_max_distance_m', 6.0).value,
            backing_timeout_s=p('backing_timeout_s', 45.0).value,
        )
        if not self._zones_frame or self._zones_frame == self._map_frame:
            self._core = ZoneClientCore(ZoneClientConfig(zones=zones, **self._core_params))

        self._path_xy: List[tuple] = []
        self._speed_cap: Optional[float] = None
        self._safety_status_timeout_s = safety_status_timeout_s
        self._safety_speed_limit: Optional[float] = None
        self._safety_status_last_t: Optional[float] = None
        # speed_limit 램프 시작점을 낮추는 데 쓴다 (core.update의 current_speed_mps,
        # #93 연우 리뷰) - 마지막으로 내보낸(상한 적용 후) cmd_vel.linear.x.
        self._last_cmd_vel_mps: float = 0.0
        # None이 아니면 BACKING 중 - cmd_vel_nav를 무시하고 이 값을 직접 낸다(10/11 결정).
        self._backing_linear_x: Optional[float] = None
        self._last_state_label: Optional[str] = None
        self._last_state_pub_t: Optional[float] = None
        # 종료 중 콜백이 이미 닫히는 중인 컨텍스트(clock·TF·publisher)를 안 쓰게 막는 플래그
        # (#146 CI - test_zone_client_launch 간헐적 segfault). main()의 종료 처리가 destroy_node()
        # 전에 켠다 - prepare_shutdown() 참고.
        self._shutting_down = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._request_pub = self.create_publisher(ZoneRequest, zone_requests_topic, qos)
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_out_topic, 10)
        self._state_pub = self.create_publisher(String, _STATE_TOPIC, 10)
        self.create_subscription(ZoneGrant, zone_grants_topic, self._on_grant, qos)
        self.create_subscription(Path, plan_topic, self._on_path, 10)
        self.create_subscription(Twist, cmd_vel_in_topic, self._on_cmd_vel, 10)
        self.create_subscription(SafetyStatus, safety_status_topic, self._on_safety_status, 10)
        self._update_timer = self.create_timer(1.0 / update_rate_hz, self._update)

        self.get_logger().info(
            f'zone_client({self._robot_id}): 구역 {zones and [z.zone_id for z in zones] or "없음"}, '
            f'구독 {zone_grants_topic} (허가), {plan_topic} (경로), {safety_status_topic} '
            f'(speed_limit) · 발행 {zone_requests_topic}, {_STATE_TOPIC} (상태)')

    def _on_path(self, msg: Path):
        if self._shutting_down:
            return
        self._path_xy = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def _on_grant(self, msg: ZoneGrant):
        if self._shutting_down:
            return
        if self._core is None:
            # 구역 좌표를 아직 못 바꿨다 = 아직 요청을 보낸 적이 없다. 받을 답이 없다
            return
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
        if self._shutting_down:
            return
        self._safety_speed_limit = msg.speed_limit
        self._safety_status_last_t = self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd_vel(self, msg: Twist):
        if self._shutting_down:
            return
        if self._backing_linear_x is not None:
            return  # BACKING 중에는 cmd_vel_nav를 무시한다(10/11 결정) - _update가 직접 낸다
        out = Twist()
        out.linear.x = min(msg.linear.x, self._speed_cap) if self._speed_cap is not None else msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular = msg.angular
        self._cmd_vel_pub.publish(out)
        self._last_cmd_vel_mps = out.linear.x

    def _set_speed_cap(self, cap: Optional[float]):
        """속도 상한을 바꾼다. 0이 아닌 값(제한 없음 포함)에서 0으로 바뀌는 순간에만 정지 명령을 한 번 낸다.

        상한은 cmd_vel_nav가 들어올 때(_on_cmd_vel)만 적용되므로, Nav2가 0이 아닌 명령을 낸 뒤 발행을
        멈추면 상한이 0이 돼도 마지막 명령이 그대로 남는다(fleet_deadlock 조사). 계속 0인 동안은 다시 내지
        않는다. BACKING 중이면 _update가 후진 명령을 직접 내므로 건너뛴다.
        """
        dropped_to_zero = cap is not None and cap <= 0.0 and (self._speed_cap is None or self._speed_cap > 0.0)
        self._speed_cap = cap
        if dropped_to_zero and self._backing_linear_x is None:
            self._publish_cmd(0.0)

    def _publish_cmd(self, linear_x: float):
        out = Twist()
        out.linear.x = linear_x
        self._cmd_vel_pub.publish(out)
        self._last_cmd_vel_mps = linear_x

    def _lookup_robot_xy(self):
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f'{self._map_frame}→{self._base_frame} TF 없음: {e}', throttle_duration_sec=5.0)
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _ensure_core(self) -> bool:
        """zones_frame → map_frame TF로 구역을 바꿔 core를 만든다. 아직 TF가 없으면 False."""
        if self._core is not None:
            return True
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._zones_frame, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(
                f'구역 좌표 변환 대기 ({self._zones_frame}→{self._map_frame} TF 없음, 그동안 정지): {e}',
                throttle_duration_sec=5.0)
            return False
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        zones = transform_zones(self._raw_zones, t.x, t.y, yaw)
        self._core = ZoneClientCore(ZoneClientConfig(zones=zones, **self._core_params))
        self.get_logger().info(
            f'구역 좌표 {self._zones_frame}→{self._map_frame} 변환 (dx={t.x:.3f}, dy={t.y:.3f}, '
            f'dyaw={yaw:.5f}): ' + ', '.join(f'{z.zone_id} {tuple(round(v, 3) for v in z.area)}' for z in zones))
        return True

    def _update(self):
        if self._shutting_down:
            return
        if not self._ensure_core():
            # 구역이 어디인지 모르면 fail-closed: 위치를 모를 때와 같은 규칙으로 상한을 0으로 묶는다
            self._set_speed_cap(0.0)
            return
        robot_xy = self._lookup_robot_xy()
        if robot_xy is None:
            # 위치를 모르면 fail-closed: 상한을 0으로 묶어 멈춰 세운다. 경고는 TF 조회 쪽에서
            # 이미 throttle해서 남긴다 (#66). BACKING 중이면(위치가 있어야 후진 거리를 재므로)
            # 같은 규칙으로 바로 정지한다 - core 상태는 그대로 두고(이번 틱만 건너뜀) TF가
            # 돌아오면 이어서 재개한다.
            self._set_speed_cap(0.0)
            if self._backing_linear_x is not None:
                self._publish_cmd(0.0)
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        # 끊김(한 번도 못 받았거나 timeout 넘게 조용함) 판단은 노드 몫이다 - core는 ROS 비의존
        # 이라 타임스탬프 비교를 모른다. 끊겼으면 None(제한 없음)을 넘긴다 (#69).
        safety_limit = self._safety_speed_limit
        if (self._safety_status_last_t is None
                or now - self._safety_status_last_t > self._safety_status_timeout_s):
            safety_limit = None
        outgoing, speed_cap, backing_linear_x, errors = self._core.update(
            now, self._path_xy, robot_xy, safety_limit, self._last_cmd_vel_mps)
        self._backing_linear_x = backing_linear_x
        self._set_speed_cap(speed_cap)
        for error in errors:
            self.get_logger().error(error)
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
        if backing_linear_x is not None:
            # BACKING 중 - cmd_vel_nav와 무관하게 이 틱의 후진 속도를 바로 낸다
            self._publish_cmd(backing_linear_x)
        self._publish_state(now)

    def _publish_state(self, now: float):
        label = self._core.state_label(now)
        changed = label != self._last_state_label
        if not changed and self._last_state_pub_t is not None \
                and now - self._last_state_pub_t < _STATE_PUBLISH_PERIOD_S:
            return
        self._state_pub.publish(String(data=label))
        self._last_state_label = label
        self._last_state_pub_t = now

    def prepare_shutdown(self):
        """destroy_node() 전에 콜백이 더는 clock·TF·publisher를 안 쓰게 막는다 (#146 CI -
        test_zone_client_launch 간헐적 segfault). 타이머를 먼저 지워 _update()가 더는 안 불리게
        하고, 플래그를 켜서 그사이 들어온(구독 콜백 큐에 남아 있던) 호출도 바로 반환하게 한다."""
        self._shutting_down = True
        if self._update_timer is not None:
            self.destroy_timer(self._update_timer)
            self._update_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = ZoneClientNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C는 터미널과 launch에서 SIGINT가 두 번 온다. 정리 중에 끊기지 않게 이후 신호는
        # 무시한다(amr_fleet_decentralized task_auction_node와 같은 방식, #146 CI)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.prepare_shutdown()
        node.destroy_node()
        # ok() 확인과 shutdown() 사이에 SIGINT 처리기가 먼저 종료하면 "already called" 예외로
        # 종료 코드 1이 난다 - try_shutdown은 이미 종료됐으면 그냥 넘어간다
        rclpy.try_shutdown()
