"""Kidnapped Robot 감지·복구 노드 (docs/algorithms/localization.md 4절).

    ros2 run amr_localization kidnap_recovery --ros-args -r __ns:=/amr_01 -p use_sim_time:=true

감지: EKF 전역 자세(odometry/filtered)에서 스캔 끝점을 지도에 찍어 점유 셀에 붙는 비율(일치율)을 잰다.
      일치율이 lost_ratio 아래로 lost_checks번 연속 떨어지면 위치를 잃은 것으로 본다.
복구: 1차 국소 재초기화(마지막으로 확실했던 자세 주변, initialpose) → 2차부터 AMCL 전역 재초기화.
      각 시도마다 움직이지 않고 갱신 요청 → (허용 시) 제자리 회전 → 수렴 확인.

구독: /map, scan, odometry/filtered, amcl_pose
호출: reinitialize_global_localization, request_nomotion_update (AMCL, std_srvs/Empty)
발행: cmd_vel_recovery (탐색 중에만), initialpose (국소 재초기화),
      kidnap_recovery/state (std_msgs/String, tracking·settling·rotating·failed 단어만. BT가 비교한다),
      kidnap_recovery/match_ratio (std_msgs/Float32, 스캔-지도 일치율)
      최종 cmd_vel은 amr_safety 하나만 낸다(9/28). cmd_vel_recovery는 twist_mux에서 내비(cmd_vel_nav)보다 우선하게
      넣어 구역 허가·안전 노드를 그대로 거친다(#79 이진 리뷰). 복구 중 내비 목표 중지는 BT가 한다
"""

import math
import os

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import Empty

from .eval.trajectory import quaternion_from_yaw, yaw_from_quaternion
from .kidnap import (LOCAL_REINIT, NOMOTION, REINIT, ROTATING, STOP, TRACKING, KidnapRecovery, RecoveryConfig,
                     endpoints, grid_from_occupancy, match_ratio)


class KidnapRecoveryNode(Node):

    def __init__(self):
        super().__init__('kidnap_recovery')
        d = self.declare_parameter
        self.check_hz = d('check_hz', 2.0).value
        self.match_tol = d('match_tol', 0.1).value
        self.rotate_speed = d('rotate_speed', 0.5).value
        # 탐색 단계에서 앞이 비어 있으면 천천히 전진한다 (반복 구조에서 통로 방향 위치를 찾으려면 움직여야 한다)
        self.explore_forward = d('explore_forward', False).value
        self.explore_speed = d('explore_speed', 0.2).value
        self.front_clearance = d('front_clearance', 1.0).value
        self.base_frame = d('base_frame', 'base_footprint').value
        self.events_file = d('events_file', '').value
        cfg = RecoveryConfig(
            lost_ratio=d('lost_ratio', 0.5).value, lost_checks=d('lost_checks', 3).value,
            ok_ratio=d('ok_ratio', 0.8).value, ok_std_m=d('ok_std_m', 0.15).value, ok_checks=d('ok_checks', 3).value,
            settle_s=d('settle_s', 10.0).value, allow_rotate=d('allow_rotate', True).value,
            rotate_s=d('rotate_s', 15.0).value, max_attempts=d('max_attempts', 3).value,
            local_first=d('local_first', True).value)
        # 국소 재초기화 때 뿌리는 파티클 범위 (1σ). 바퀴 미끄러짐·충돌로 몇 m 어긋난 경우를 덮는다.
        # 1.0 m에서는 1.9 m 이동을 못 찾았고(통로 방향 1.5 m 오차 남음) 2.5 m에서 9.5초에 복구했다 (10/6)
        self.local_std_m = d('local_std_m', 2.5).value
        self.local_std_yaw = d('local_std_yaw', 0.8).value
        self.last_good = None
        self.machine = KidnapRecovery(cfg)
        ns = self.get_namespace().strip('/')
        self.prefix = f'{ns}/' if ns else ''

        self.grid = None
        self.scan = None
        self.pose = None
        self.amcl_std = math.inf
        self.sensor_offset = None
        self.ratio = float('nan')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odometry/filtered', self._on_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, 10)
        self.reinit = self.create_client(Empty, 'reinitialize_global_localization')
        self.nomotion = self.create_client(Empty, 'request_nomotion_update')
        self.cmd_pub = self.create_publisher(Twist, d('cmd_vel_topic', 'cmd_vel_recovery').value, 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.state_pub = self.create_publisher(String, 'kidnap_recovery/state', 10)
        self.ratio_pub = self.create_publisher(Float32, 'kidnap_recovery/match_ratio', 10)
        self.create_timer(1.0 / self.check_hz, self._check)
        self.create_timer(0.1, self._drive)
        self.get_logger().info('Kidnapped 감지 시작 (지도·스캔 대기)')

    def _on_map(self, msg: OccupancyGrid):
        info = msg.info
        self.grid = grid_from_occupancy(msg.data, info.width, info.height, info.resolution,
                                        info.origin.position.x, info.origin.position.y)
        self.get_logger().info(f'지도 {info.width}x{info.height} @ {info.resolution} m 거리장 준비')

    def _on_scan(self, msg: LaserScan):
        self.scan = msg
        if self.sensor_offset is None:
            try:
                t = self.tf_buffer.lookup_transform(self.prefix + self.base_frame, msg.header.frame_id,
                                                    rclpy.time.Time())
                q = t.transform.rotation
                self.sensor_offset = (t.transform.translation.x, t.transform.translation.y,
                                      yaw_from_quaternion(q.x, q.y, q.z, q.w))
            except tf2_ros.TransformException:
                pass

    def _on_pose(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y,
                     yaw_from_quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        c = msg.pose.covariance
        self.amcl_std = math.sqrt(max(c[0], c[7], 0.0))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _check(self):
        if self.grid is None or self.scan is None or self.pose is None or self.sensor_offset is None:
            return
        s = self.scan
        pts = endpoints(self.pose, self.sensor_offset, s.ranges, s.angle_min, s.angle_increment,
                        s.range_min, s.range_max)
        self.ratio = match_ratio(self.grid, pts, self.match_tol)
        if self.machine.state == TRACKING and self.ratio >= self.machine.config.ok_ratio:
            self.last_good = self.pose
        n_events = len(self.machine.events)
        for action in self.machine.step(self._now(), self.ratio, self.amcl_std):
            if action == LOCAL_REINIT:
                self._local_reinit()
            elif action == REINIT and self.reinit.service_is_ready():
                self.reinit.call_async(Empty.Request())
            elif action == NOMOTION and self.nomotion.service_is_ready():
                self.nomotion.call_async(Empty.Request())
            elif action == STOP:
                self.cmd_pub.publish(Twist())
        for e in self.machine.events[n_events:]:
            self.get_logger().warn(f'[kidnap] {e.kind} {e.detail}')
            self._log_event(e)
        self.state_pub.publish(String(data=self.machine.state))
        self.ratio_pub.publish(Float32(data=float(self.ratio)))

    def _local_reinit(self):
        """마지막으로 확실했던 자세 주변에 파티클을 넓게 뿌린다 (AMCL initialpose). 없으면 지금 자세."""
        x, y, yaw = self.last_good or self.pose
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        _, _, qz, qw = quaternion_from_yaw(yaw)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
        cov = [0.0] * 36
        cov[0] = cov[7] = self.local_std_m ** 2
        cov[35] = self.local_std_yaw ** 2
        msg.pose.covariance = cov
        self.initialpose_pub.publish(msg)

    def _front_free(self) -> bool:
        """스캔 앞쪽 ±20° 안 최소 거리가 front_clearance보다 큰지 (센서 앞 = 로봇 앞으로 본다)."""
        s = self.scan
        if s is None:
            return False
        lim = math.radians(20.0)
        front = [r for k, r in enumerate(s.ranges)
                 if abs(s.angle_min + k * s.angle_increment) <= lim and math.isfinite(r) and r > s.range_min]
        return bool(front) and min(front) > self.front_clearance

    def _drive(self):
        # 탐색 중에만 낸다. twist_mux에서 내비보다 우선하므로 다른 때는 아무것도 내지 않는다
        if self.machine.state == ROTATING:
            cmd = Twist()
            if self.explore_forward and self._front_free():
                cmd.linear.x = float(self.explore_speed)
            else:
                cmd.angular.z = float(self.rotate_speed)
            self.cmd_pub.publish(cmd)

    def _log_event(self, e):
        if not self.events_file:
            return
        new = not os.path.exists(self.events_file)
        os.makedirs(os.path.dirname(self.events_file) or '.', exist_ok=True)
        with open(self.events_file, 'a', encoding='utf-8') as f:
            if new:
                f.write('time,event,detail,ratio,amcl_std\n')
            f.write(f'{e.t:.3f},{e.kind},{e.detail},{self.ratio:.3f},{self.amcl_std:.3f}\n')


def main(args=None):
    rclpy.init(args=args)
    node = KidnapRecoveryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
