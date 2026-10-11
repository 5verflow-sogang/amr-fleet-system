"""안전 노드: 장애물·센서 상태로 안전 단계를 정하고, 로봇의 최종 cmd_vel을 낸다.

구독: scan (필터 LiDAR), perception/obstacles, perception/collision_risks, odom, cmd_vel_zone_limited
발행: cmd_vel (최종, 9/28 합의), safety/status (바뀌면 즉시·그 외 10 Hz), safety/markers (RViz)
서비스: safety/estop (std_srvs/SetBool: true 걸기, false 해제)

판정 규칙은 amr_perception_core.safety.SafetyMonitor (단계표와 근거는 그 모듈 설명).
- 긴급정지는 LiDAR 원본 점으로, 지금 명령대로 움직일 때 차체가 지나갈 자리를 본다 (collision.path_hazard).
  진행 방향 앞 critical_distance 띠·옆 side_margin. 멀어지는 명령은 막지 않는다
- TTC는 amr_tracking의 collision_risks, "움직이는 장애물 근접"은 perception/obstacles로 본다
- 판정은 명령·스캔이 올 때마다, 그리고 rate [Hz] 타이머로 한다 (센서가 끊겨도 판정이 돈다).
  명령이 cmd_timeout 동안 없으면 정지 명령(0)으로 보고 판정한다

cmd_vel 게이트 (gate.limit_velocity):
- 정지(CRITICAL·ESTOP): 입력이 없어도 status_period마다 0을 낸다. 회전도 막는다
- WARNING: 선속도 크기를 0.6 m/s 이하로 (곡률 유지)
- 입력 명령이 cmd_timeout 동안 없으면 0을 한 번 낸다 (Gazebo DiffDrive는 마지막 명령을 계속 따른다)
"""

import math
import signal
from typing import Dict, List, Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from amr_msgs.msg import CollisionRiskArray, SafetyStatus, TrackedObstacleArray
from amr_perception_core.clustering import transform_points_2d
from amr_perception_core.collision import footprint_clearance, path_hazard
from amr_perception_core.safety import (
    LEVEL_NAMES,
    ObstacleInfo,
    SafetyConfig,
    SafetyMonitor,
    SafetyState,
)

from .gate import limit_velocity, min_clearance, path_horizon, scan_body_points
from .markers import safety_markers


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def obstacle_infos(obstacles: TrackedObstacleArray, ttc_by_id: Dict[int, float], yaw: float,
                   translation, length: float, width: float) -> List[ObstacleInfo]:
    """추적 장애물 → 판정 입력. yaw, translation: obstacles 좌표계 → base_footprint 변환.

    clearance = 차체에서 장애물 중심까지 거리 − 장애물 반경 (0 아래로 내려가지 않는다).
    TTC가 없는 트랙(collision_risks에 없음)은 inf.
    """
    if not obstacles.obstacles:
        return []
    centers = np.array([(o.pose.pose.position.x, o.pose.pose.position.y)
                        for o in obstacles.obstacles])
    body = transform_points_2d(centers, yaw, translation)
    distances = footprint_clearance(body, length, width)
    return [ObstacleInfo(track_id=o.track_id,
                         clearance=max(float(d) - float(o.radius), 0.0),
                         ttc=ttc_by_id.get(o.track_id, math.inf),
                         is_dynamic=bool(o.is_dynamic))
            for o, d in zip(obstacles.obstacles, distances)]


def to_status_msg(state: SafetyState, stamp) -> SafetyStatus:
    msg = SafetyStatus()
    msg.header.stamp = stamp
    msg.level = state.level
    msg.reason = state.reason
    msg.detail = state.detail
    msg.related_track_id = state.related_track_id
    msg.min_obstacle_distance = float(state.min_obstacle_distance)
    msg.speed_limit = float(state.speed_limit)
    return msg


class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety')
        p = self.declare_parameter
        self.config = SafetyConfig(
            critical_distance=p('critical_distance', 0.3).value,
            ttc_critical=p('ttc_critical', 1.0).value,
            ttc_warning=p('ttc_warning', 2.0).value,
            dynamic_warning_distance=p('dynamic_warning_distance', 1.5).value,
            warning_speed_limit=p('warning_speed_limit', 0.6).value,
            release_hold_s=p('release_hold_s', 0.5).value,
            scan_timeout_s=p('scan_timeout', 0.5).value,
            tracks_timeout_s=p('tracks_timeout', 1.0).value,
            odom_timeout_s=p('odom_timeout', 1.0).value,
        )
        self.monitor = SafetyMonitor(self.config)
        self.length = p('robot_length', 0.6).value
        self.width = p('robot_width', 0.4).value
        base_frame = p('base_frame', '').value
        ns = self.get_namespace().strip('/')
        self.base_frame = base_frame or (f'{ns}/base_footprint' if ns else 'base_footprint')
        self.cmd_timeout = p('cmd_timeout', 0.5).value
        # 진행 경로 판정 (path_hazard)
        self.side_margin = p('side_margin', 0.05).value
        self.reaction_time = p('reaction_time', 0.2).value
        self.decel = p('decel', 1.0).value
        self.path_horizon_min = p('path_horizon_min', 0.5).value
        self.status_period = p('status_period', 0.1).value
        self.publish_markers = p('publish_markers', True).value
        cmd_in = p('cmd_vel_in_topic', 'cmd_vel_zone_limited').value
        cmd_out = p('cmd_vel_out_topic', 'cmd_vel').value
        rate = p('rate', 20.0).value

        # 마지막 수신 시각 [s] (노드 시계)
        self.t_scan: Optional[float] = None
        self.t_obstacles: Optional[float] = None
        self.t_risks: Optional[float] = None
        self.t_odom: Optional[float] = None
        self.t_cmd: Optional[float] = None
        self.last_cmd = (0.0, 0.0)       # 마지막 입력 명령 (선속도, 각속도)
        self.scan_points: Optional[np.ndarray] = None   # 몸체 좌표 스캔 점
        self.obstacles: Optional[TrackedObstacleArray] = None
        self.ttc_by_id: Dict[int, float] = {}
        self.scan_tf = None              # 스캔 프레임 → base_footprint (고정이라 한 번 구해 둔다)
        self.state: Optional[SafetyState] = None
        self.last_status_t = -math.inf
        self.cmd_zero_sent = True        # 입력 끊김 뒤 0을 이미 냈는지

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, cmd_out, 10)
        self.status_pub = self.create_publisher(SafetyStatus, 'safety/status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'safety/markers', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(TrackedObstacleArray, 'perception/obstacles',
                                 self.on_obstacles, 10)
        self.create_subscription(CollisionRiskArray, 'perception/collision_risks',
                                 self.on_risks, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Twist, cmd_in, self.on_cmd, 10)
        self.create_service(SetBool, 'safety/estop', self.on_estop)
        self.create_timer(1.0 / rate, self.evaluate)
        self.get_logger().info(
            f'safety: {cmd_in} → {cmd_out}, 긴급정지 {self.config.critical_distance} m, '
            f'TTC {self.config.ttc_critical}/{self.config.ttc_warning} s, 차체 '
            f'{self.length}×{self.width} m ({self.base_frame})')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ 입력
    def on_scan(self, scan: LaserScan):
        if self.scan_tf is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, scan.header.frame_id,
                                                     Time())
            except TransformException as e:
                # 스캔을 못 쓰면 받지 않은 것으로 둔다 → scan_timeout 뒤 정지
                self.get_logger().warn(f'스캔 TF 없음: {e}', throttle_duration_sec=5.0)
                return
            q, t = tf.transform.rotation, tf.transform.translation
            self.scan_tf = (yaw_from_quaternion(q.x, q.y, q.z, q.w), (t.x, t.y))
        yaw, translation = self.scan_tf
        try:
            self.scan_points = scan_body_points(
                scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
                scan.range_max, yaw, translation)
        except ValueError as e:
            self.get_logger().warn(f'스캔 무시: {e}', throttle_duration_sec=5.0)
            return
        self.t_scan = self.now()
        self.evaluate()            # 긴급정지는 다음 타이머를 기다리지 않는다

    def on_obstacles(self, msg: TrackedObstacleArray):
        self.obstacles = msg
        self.t_obstacles = self.now()

    def on_risks(self, msg: CollisionRiskArray):
        self.ttc_by_id = {r.track_id: float(r.ttc) for r in msg.risks}
        self.t_risks = self.now()

    def on_odom(self, msg: Odometry):
        self.t_odom = self.now()

    def on_cmd(self, msg: Twist):
        self.t_cmd = self.now()
        self.cmd_zero_sent = False
        self.last_cmd = (msg.linear.x, msg.angular.z)
        self.evaluate()            # 이 명령의 진행 경로로 판정한 뒤 게이트에 건다
        limit = self.state.speed_limit if self.state is not None else 0.0
        self.publish_cmd(*limit_velocity(msg.linear.x, msg.angular.z, limit))

    def on_estop(self, request: SetBool.Request, response: SetBool.Response):
        self.monitor.set_estop(request.data)
        self.get_logger().warn(f'E-Stop {"걸림" if request.data else "해제 요청"}')
        self.evaluate()
        response.success = True
        response.message = ('E-Stop 걸림' if request.data
                            else f'E-Stop 해제 ({self.config.release_hold_s} s 뒤 단계 재판정)')
        return response

    # ------------------------------------------------------------------ 판정·발행
    def _age(self, t: Optional[float], now: float) -> Optional[float]:
        return None if t is None else max(now - t, 0.0)

    def evaluate(self):
        now = self.now()
        tracks_age = None
        if self.t_obstacles is not None and self.t_risks is not None:
            tracks_age = max(self._age(self.t_obstacles, now), self._age(self.t_risks, now))
        infos: List[ObstacleInfo] = []
        if tracks_age is not None and tracks_age <= self.config.tracks_timeout_s:
            try:
                infos = self._obstacle_infos()
            except TransformException as e:
                # 장애물을 몸체 기준으로 못 옮기면 추적이 없는 것과 같다 → 저속
                self.get_logger().warn(f'장애물 TF 없음: {e}', throttle_duration_sec=5.0)
                tracks_age = None
        scan_clearance, path_clearance = None, None
        if self.scan_points is not None:
            scan_clearance = min_clearance(self.scan_points, self.length, self.width)
            vx, wz = self.current_command(now)
            path_clearance = path_hazard(
                self.scan_points, vx, wz, self.length, self.width,
                self.config.critical_distance, self.side_margin,
                path_horizon(vx, self.reaction_time, self.decel, self.path_horizon_min))
        state = self.monitor.update(
            now, scan_clearance, infos, self._age(self.t_scan, now), tracks_age,
            self._age(self.t_odom, now), path_clearance)
        self._publish_state(state, now)

    def current_command(self, now: float):
        """판정에 쓸 명령. 입력이 cmd_timeout 동안 없으면 정지(0)로 본다."""
        if self.t_cmd is None or now - self.t_cmd > self.cmd_timeout:
            return 0.0, 0.0
        return self.last_cmd

    def _obstacle_infos(self) -> List[ObstacleInfo]:
        if not self.obstacles.obstacles:
            return []
        tf = self.tf_buffer.lookup_transform(self.base_frame, self.obstacles.header.frame_id,
                                             Time())
        q, t = tf.transform.rotation, tf.transform.translation
        return obstacle_infos(self.obstacles, self.ttc_by_id,
                              yaw_from_quaternion(q.x, q.y, q.z, q.w), (t.x, t.y),
                              self.length, self.width)

    def _publish_state(self, state: SafetyState, now: float):
        previous = self.state
        self.state = state
        changed = previous is None or (previous.level, previous.reason, previous.speed_limit) != (
            state.level, state.reason, state.speed_limit)
        stopping = state.speed_limit <= 0.0
        if changed:
            # rclpy는 한 호출 위치의 로그 단계가 바뀌면 예외를 던진다 → 호출을 나눈다
            text = f'{LEVEL_NAMES[state.level]} (limit {state.speed_limit}): {state.detail}'
            if state.level > 0:
                self.get_logger().warn(text)
            else:
                self.get_logger().info(text)
        if changed or now - self.last_status_t >= self.status_period - 1e-9:
            self.last_status_t = now
            self.status_pub.publish(to_status_msg(state, self.get_clock().now().to_msg()))
            if self.publish_markers:
                self.marker_pub.publish(safety_markers(
                    state, self.base_frame, self.length, self.width,
                    self.config.critical_distance, self.config.dynamic_warning_distance))
            if stopping:
                self.publish_cmd(0.0, 0.0)     # 입력이 없어도 정지 명령을 계속 낸다
        # 입력 명령이 끊기면 0을 한 번 (마지막 명령으로 계속 가지 않게)
        if (not self.cmd_zero_sent and self.t_cmd is not None
                and now - self.t_cmd > self.cmd_timeout):
            self.publish_cmd(0.0, 0.0)
            self.cmd_zero_sent = True

    def publish_cmd(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear_x), float(angular_z)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if rclpy.ok():
            node.publish_cmd(0.0, 0.0)     # 안전 노드가 내려가면 로봇도 선다
        node.destroy_node()
