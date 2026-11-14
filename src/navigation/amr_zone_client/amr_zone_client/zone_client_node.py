"""로봇 측 구역 진입 처리 노드. amr_fleet_manager(#57)의 ZoneRequest/ZoneGrant 서버와 짝이다.

계획 경로(/<ns>/plan)와 현재 위치(robot_pose_topic, map 기준)로 구역 경계까지 남은 거리를 계산해
ZoneRequest를 보내고, 허가 전까지는 cmd_vel을 감속·정지시킨다. 흐름은 README '동작 규칙' 참고.

위치는 TF가 아니라 토픽으로 받는다(perf/zone-client-pose-topic, 하연 분석) - 공용 Buffer()+
TransformListener()는 /tf를 구독해 모든 로봇의 TF를 다 받아 처리하므로, 구역 요청이 0건이어도
로봇 1대당 약 0.3 코어를 쓰고 있었다. map 기준 위치는 로봇마다 자기 ekf_global만 내는 네임스페이스
안 상대 토픽(robot_pose_topic, 기본 odometry/filtered)으로 이미 나오고 있어 그걸 직접 구독한다
(amr_localization launch 참고 - amcl_pose는 AMCL이 update_min_d(0.1 m)만큼 움직이기 전엔 안 보내
너무 뜸하다. odometry/filtered는 ekf_global이 amcl_pose+오도메트리+IMU를 계속 융합해 내는 map
기준 추정치라 지금 TF 조회가 주던 것과 같다). TF는 zones_frame(세계 좌표 → map, 정적) 변환에만
남겨 뒀고, 그 변환이 끝나면(_ensure_core) 리스너를 바로 치운다 - 평소(zones_frame 비움)에는 TF를
전혀 안 쓴다. 토픽을 잘못 물려서(예: odom 기준 ekf_local) 좌표계가 다른 위치가 들어오는 걸 조용히
놓치지 않으려고, header.frame_id가 map_frame이 아니면 그 메시지는 버리고 위치가 끊긴 것처럼
fail-closed로 본다(_on_robot_pose). 위치 자체는 한 번도 못 받았을 때만 fail-closed다. 받은 적이
있으면 robot_pose_timeout_s보다 오래돼도 속도를 묶지 않고 마지막 값을 그대로 쓴다 - 예전 TF
조회(lookup_transform(..., Time()))가 버퍼의 최신 값을 나이와 상관없이 주던 것과 같다. 끊겼다는
사실은 경고로만 남긴다 (#271 하연 리뷰: CPU 포화로 EKF 주기가 느려질 때 속도를 0으로 묶으면
구역과 상관없는 정지가 새 정체를 만든다 - 이 PR은 성능 개선이라 동작은 예전과 같아야 한다).

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
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
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
        # 로봇 위치(map 기준) 토픽. 네임스페이스 안 상대 이름 - ekf_global 출력(odometry/filtered,
        # amr_localization launch)이 기본이다. amcl_pose(AMCL 원출력)는 update_min_d(0.1 m)만큼
        # 움직이기 전엔 안 보내 너무 뜸하다 - odometry/filtered가 TF(map→base_frame)가 주던 것과
        # 같은, 계속 융합되는 추정치다.
        robot_pose_topic = p('robot_pose_topic', 'odometry/filtered').value
        # 한 번도 못 받았으면 fail-closed(TF 조회가 아예 없는 프레임에 실패하던 것과 같음). 받은 적이
        # 있으면 이 시간보다 오래돼도 마지막 값을 그대로 쓴다 - TF 조회(lookup_transform(..., Time()))가
        # 버퍼의 최신 값을 나이와 상관없이 주던 것과 같다. 오래 끊기면 경고만 남긴다 (#271 하연 리뷰:
        # CPU 포화로 EKF 주기가 느려질 때 속도를 0으로 묶으면 구역과 상관없는 정지가 새 정체를 만든다).
        robot_pose_timeout_s = p('robot_pose_timeout_s', 1.0).value
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
            drop_grace_s=p('drop_grace_s', 3.0).value,
            lookahead_m=lookahead_m if lookahead_m > 0.0 else None,
            max_linear_velocity_mps=p('max_linear_velocity_mps', 2.0).value,
            backing_speed_mps=p('backing_speed_mps', 0.15).value,
            backing_margin_m=p('backing_margin_m', 0.3).value,
            backing_max_distance_m=p('backing_max_distance_m', 6.0).value,
            backing_timeout_s=p('backing_timeout_s', 45.0).value,
            yield_distance_m=p('yield_distance_m', 0.6).value,
            yield_hold_s=p('yield_hold_s', 8.0).value,
            yield_timeout_s=p('yield_timeout_s', 10.0).value,
        )
        # 대치 양보 중 비킬 쪽 확인용 스캔 (상대 토픽 scan). 양보 중이 아니면 받아 두기만 한다
        self._scan: Optional[LaserScan] = None
        self.create_subscription(LaserScan, p('scan_topic', 'scan').value, self._on_scan, qos_profile_sensor_data)
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

        # 로봇 위치(map 기준) - 콜백에서 최신 값만 저장하고, 끊김 판단(robot_pose_timeout_s)은
        # safety_status와 같은 패턴으로 노드가 한다(core는 ROS 비의존이라 타임스탬프를 모른다).
        self._robot_pose_topic = robot_pose_topic
        self._robot_pose_timeout_s = robot_pose_timeout_s
        self._last_pose_xy: Optional[tuple] = None
        self._last_pose_t: Optional[float] = None

        # TF는 zones_frame(세계 좌표 → map, 정적) 변환에만 쓴다 - 평소(zones_frame 비움, self._core가
        # 이미 위에서 만들어짐)에는 전혀 안 띄운다. 변환이 끝나면(_ensure_core) 바로 치운다 - 공용
        # /tf를 구독해 모든 로봇의 TF를 받는 TransformListener를 평소에 계속 띄워 두는 게 이 노드의
        # CPU 사용량을 키운 원인이었다(perf/zone-client-pose-topic, 하연 분석).
        self._tf_buffer: Optional[Buffer] = None
        self._tf_listener: Optional[TransformListener] = None
        if self._core is None:
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
        self.create_subscription(Odometry, robot_pose_topic, self._on_robot_pose, 10)
        self._update_timer = self.create_timer(1.0 / update_rate_hz, self._update)

        self.get_logger().info(
            f'zone_client({self._robot_id}): 구역 {zones and [z.zone_id for z in zones] or "없음"}, '
            f'구독 {zone_grants_topic} (허가), {plan_topic} (경로), {robot_pose_topic} (위치), '
            f'{safety_status_topic} (speed_limit) · 발행 {zone_requests_topic}, {_STATE_TOPIC} (상태)')

    def _on_scan(self, msg: LaserScan):
        self._scan = msg

    def _yield_blocked(self, direction: float) -> bool:
        """비키는 방향(+1 앞, -1 뒤) 차체 끝에서 yield_distance_m + 여유 안, 옆으로 차체 반폭 + 여유 안에 스캔 점이 있나.
        차체 0.6 x 0.4 m(nav2_params footprint), LiDAR는 중심에서 앞으로 0.15 m(amr_description extrinsics.yaml).
        스캔이 없으면 막힌 것으로 본다(fail-closed)."""
        scan = self._scan
        if scan is None:
            return True
        half_length, half_width, lidar_x, margin = 0.3, 0.2, 0.15, 0.1
        reach = half_length + self._core_params['yield_distance_m'] + margin
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min <= r <= scan.range_max:
                x = r * math.cos(angle) + lidar_x
                y = r * math.sin(angle)
                along = direction * x
                if half_length < along <= reach and abs(y) <= half_width + margin:
                    return True
            angle += scan.angle_increment
        return False

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

    def _on_robot_pose(self, msg: Odometry):
        if self._shutting_down:
            return
        # ekf_global은 map 기준으로 낸다고 가정하는데, 실수로 ekf_local(odom 기준) 쪽을 물리거나
        # robot_pose_topic을 잘못 설정하면 좌표계가 달라 구역 판단이 조용히 틀려진다. header.frame_id가
        # map_frame(기본 'map', amr_localization launch에서 네임스페이스 접두어 없이 공유)이 아니면
        # 이 값은 버리고 위치가 안 온 것처럼 fail-closed로 본다 (#66 하연 리뷰 후속).
        if msg.header.frame_id != self._map_frame:
            self.get_logger().warn(
                f'{self._robot_pose_topic} frame_id가 {self._map_frame!r}이 아님'
                f'({msg.header.frame_id!r}) - 이 위치는 버림',
                throttle_duration_sec=5.0)
            return
        # 최신 값만 저장한다 - 구역 판단은 지금처럼 _update(update_rate_hz) 주기로 한다.
        self._last_pose_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._last_pose_t = self.get_clock().now().nanoseconds * 1e-9

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

    def _robot_xy(self):
        """로봇 위치(map 기준). 한 번도 못 받았으면 None(fail-closed) - 전에 TF 조회가 없는 프레임에
        TransformException을 내던 것과 같다. 받은 적이 있으면 오래됐어도 그 마지막 값을 그대로 쓴다 -
        전에 TF 조회(lookup_transform(..., Time()))가 버퍼의 가장 최근 값을 나이와 상관없이 주던 것과
        같다 (#271 하연 리뷰: robot_pose_timeout_s로 끊겼다고 속도를 0으로 묶으면, CPU 포화로 EKF
        주기가 느려질 때 구역과 상관없는 정지가 생겨 새 정체를 만든다 - 이 PR은 성능 개선이라 동작은
        예전과 같아야 한다). 오래된 값을 쓰고 있다는 사실만 경고로 남긴다.
        """
        if self._last_pose_xy is None:
            return None
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_pose_t > self._robot_pose_timeout_s:
            self.get_logger().warn(
                f'{self._robot_pose_topic} 위치가 {self._robot_pose_timeout_s:g}초 넘게 안 옴 - '
                f'마지막 위치를 그대로 쓴다', throttle_duration_sec=5.0)
        return self._last_pose_xy

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
        # 변환이 끝났다 - 더는 안 쓰니 /tf·/tf_static 구독을 치운다(이 변환 하나만 쓰려고 평소에도
        # 공용 TF를 계속 받는 게 CPU를 키운 원인이었다, perf/zone-client-pose-topic).
        self.destroy_subscription(self._tf_listener.tf_sub)
        self.destroy_subscription(self._tf_listener.tf_static_sub)
        self._tf_listener = None
        self._tf_buffer = None
        return True

    def _update(self):
        if self._shutting_down:
            return
        if not self._ensure_core():
            # 구역이 어디인지 모르면 fail-closed: 위치를 모를 때와 같은 규칙으로 상한을 0으로 묶는다
            self._set_speed_cap(0.0)
            return
        robot_xy = self._robot_xy()
        if robot_xy is None:
            # 위치를 한 번도 못 받았으면 fail-closed: 상한을 0으로 묶어 멈춰 세운다 (받은 적이
            # 있으면 robot_xy는 끊겨도 마지막 값을 주므로 여기 안 들어온다 - #271 하연 리뷰).
            # 경고는 위치 쪽에서 이미 throttle해서 남긴다 (#66). BACKING 중이면(위치가 있어야
            # 후진 거리를 재므로) 같은 규칙으로 바로 정지한다 - core 상태는 그대로 두고(이번 틱만
            # 건너뜀) 위치가 돌아오면 이어서 재개한다.
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
        direction = self._core.yield_direction
        outgoing, speed_cap, backing_linear_x, errors = self._core.update(
            now, self._path_xy, robot_xy, safety_limit, self._last_cmd_vel_mps,
            yield_blocked=direction is not None and self._yield_blocked(direction))
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
