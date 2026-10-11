"""LiDAR 스캔(또는 위치 측정) → 다중 장애물 KF 추적 → perception/obstacles, perception/obstacle_markers.

measurement_source:
- scan (기본): 필터된 스캔(amr_odometry scan_filter)을 군집화하고, 크기 규칙(R1)으로 벽·랙 면을 빼고,
  다리 두 개를 한 사람으로 묶은 뒤, 스캔 촬영 시각의 TF로 fixed_frame(odom)에 옮겨 추적한다.
  군집화를 이 노드 안에서 하므로 군집 반경을 그대로 트랙 반경으로 쓴다 (perception.md 1-3절 C안).
- poses: geometry_msgs/PoseArray 위치 측정 (synthetic_obstacles_node 시험용). 반경은 obstacle_radius 고정.

출력은 odom 좌표계다 (DWA가 odom에서 동작하고, map은 위치 보정 때 점프해 가짜 속도가 생긴다).
is_dynamic은 추적 속력으로 정한다(R3, tracker.py). 지도 대조(R2)는 아직 없어 기둥·랙 끝도 정적 트랙으로 나간다.
"""

import math
import signal
import time

from geometry_msgs.msg import PoseArray
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from amr_msgs.msg import TrackedObstacle, TrackedObstacleArray
from amr_perception_core.clustering import ClusteringConfig, detect_obstacles, transform_points_2d
from amr_perception_core.kalman import KalmanConfig
from amr_perception_core.tracker import MultiTargetTracker, Track, TrackerConfig

from .markers import track_markers

# 추정하지 않는 성분(z, 회전)의 분산. docs/api.md 4-3절
UNKNOWN_VARIANCE = 1e6


def covariance_6x6(xy_block: np.ndarray) -> list:
    """2x2 (x, y) 공분산을 ROS 6x6 공분산(행 우선 36개)에 넣는다. 나머지 대각은 UNKNOWN_VARIANCE."""
    cov = np.diag([0.0, 0.0] + [UNKNOWN_VARIANCE] * 4)
    cov[:2, :2] = xy_block
    return cov.ravel().tolist()


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """쿼터니언의 z축 회전(yaw) [rad]. LiDAR가 수평이라 2D 변환에는 yaw만 쓴다."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class TrackingNode(Node):

    def __init__(self):
        super().__init__('tracking')
        p = self.declare_parameter
        kalman = KalmanConfig(
            accel_noise_std=p('accel_noise_std', 1.0).value,
            measurement_noise_std=p('measurement_noise_std', 0.05).value,
            initial_velocity_std=p('initial_velocity_std', 1.5).value,
        )
        self.tracker = MultiTargetTracker(TrackerConfig(
            kalman=kalman,
            gate_mahalanobis_sq=p('gate_mahalanobis_sq', 13.82).value,
            confirm_hits=p('confirm_hits', 3).value,
            max_coast_time=p('max_coast_time', 2.0).value,
            dynamic_speed_threshold=p('dynamic_speed_threshold', 0.25).value,
            dynamic_confirm_frames=p('dynamic_confirm_frames', 5).value,
            radius_smoothing=p('radius_smoothing', 0.3).value,
        ))
        self.clustering = ClusteringConfig(
            breakpoint_lambda_deg=p('breakpoint_lambda_deg', 10.0).value,
            range_noise_std=p('range_noise_std', 0.03).value,
            max_gap_beams=p('max_gap_beams', 2).value,
            min_points=p('min_points', 3).value,
            max_obstacle_extent=p('max_obstacle_extent', 2.0).value,
            leg_max_extent=p('leg_max_extent', 0.35).value,
            leg_merge_distance=p('leg_merge_distance', 0.6).value,
        )
        # 반경 측정이 없을 때(poses 입력) 쓰는 고정 반경
        self.radius = p('obstacle_radius', 0.3).value
        source = p('measurement_source', 'scan').value
        fixed_frame = p('fixed_frame', '').value
        # 비우면 노드 네임스페이스로 만든다: /amr_01 → amr_01/odom (TF 프레임 이름에 접두어가 붙는다)
        ns = self.get_namespace().strip('/')
        self.fixed_frame = fixed_frame or (f'{ns}/odom' if ns else 'odom')
        self.tf_timeout = Duration(seconds=p('tf_timeout', 0.1).value)
        self.publish_markers = p('publish_markers', True).value
        self.stats_period = p('stats_period', 5.0).value
        self._reset_stats()

        self.pub = self.create_publisher(TrackedObstacleArray, 'perception/obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'perception/obstacle_markers', 10)
        if source == 'scan':
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
            detail = f'scan → {self.fixed_frame}'
        elif source == 'poses':
            self.create_subscription(PoseArray, 'perception/obstacle_measurements',
                                     self.on_measurements, qos_profile_sensor_data)
            detail = 'perception/obstacle_measurements'
        else:
            raise ValueError(f"measurement_source는 'scan' 또는 'poses'여야 한다: {source}")
        self.get_logger().info(
            f'tracking: {detail} → perception/obstacles '
            f'(σa {kalman.accel_noise_std} m/s², σz {kalman.measurement_noise_std} m)')

    # ------------------------------------------------------------------ 입력
    def on_scan(self, scan: LaserScan):
        start = time.monotonic()
        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame, scan.header.frame_id, Time.from_msg(scan.header.stamp),
                timeout=self.tf_timeout)
        except TransformException as e:
            self.stats['tf_fail'] += 1
            self.get_logger().warn(f'TF 없음, 스캔 건너뜀: {e}', throttle_duration_sec=5.0)
            return
        clusters = detect_obstacles(scan.ranges, scan.angle_min, scan.angle_increment,
                                    self.clustering)
        q, t = tf.transform.rotation, tf.transform.translation
        points = np.array([c.centroid for c in clusters]).reshape(-1, 2)
        points = transform_points_2d(points, yaw_from_quaternion(q.x, q.y, q.z, q.w), (t.x, t.y))
        header = scan.header
        header.frame_id = self.fixed_frame    # 시각은 스캔 촬영 시각 그대로
        self.stats['candidates'] += len(clusters)
        self.track_and_publish(header, points, [c.radius for c in clusters])
        self.stats['proc_s'] += time.monotonic() - start

    def on_measurements(self, msg: PoseArray):
        zs = [(pose.position.x, pose.position.y) for pose in msg.poses]
        self.track_and_publish(msg.header, zs, None)

    # ------------------------------------------------------------------ 추적·발행
    def track_and_publish(self, header, measurements, radii):
        t = Time.from_msg(header.stamp).nanoseconds * 1e-9
        last = self.tracker.last_time
        if last is not None and t < last:
            # 시뮬레이션 리셋·rosbag 반복. 그대로 두면 이후 모든 프레임이 거부된다
            self.get_logger().warn(f'측정 시각이 거꾸로 감 ({t:.3f} < {last:.3f} s) → 트랙 초기화')
            self.tracker.reset()
        try:
            tracks = self.tracker.step(t, measurements, radii)
        except ValueError as e:
            self.get_logger().warn(f'측정 프레임 무시: {e}', throttle_duration_sec=5.0)
            return
        out = TrackedObstacleArray()
        out.header = header
        out.obstacles = [self.to_msg(track) for track in tracks]
        self.pub.publish(out)      # 장애물이 없어도 빈 배열로 발행 (api.md 4-3절)
        if self.publish_markers:
            self.marker_pub.publish(track_markers(out))
        self.stats['frames'] += 1
        self.stats['tracks'] += len(tracks)
        self.stats['dynamic'] += sum(track.is_dynamic for track in tracks)
        self._log_stats()

    def to_msg(self, track: Track) -> TrackedObstacle:
        kf = track.kf
        obstacle = TrackedObstacle()
        obstacle.track_id = track.track_id
        obstacle.source = TrackedObstacle.SOURCE_LIDAR
        obstacle.class_name = 'unknown'   # 카메라 연관 전
        obstacle.confidence = 0.0
        obstacle.is_dynamic = track.is_dynamic
        pose = obstacle.pose.pose
        pose.position.x, pose.position.y = kf.position.tolist()
        pose.orientation.w = 1.0
        obstacle.pose.covariance = covariance_6x6(kf.position_covariance)
        twist = obstacle.twist.twist
        twist.linear.x, twist.linear.y = kf.velocity.tolist()
        obstacle.twist.covariance = covariance_6x6(kf.velocity_covariance)
        obstacle.radius = float(track.radius if track.radius is not None else self.radius)
        return obstacle

    # ------------------------------------------------------------------ 기록
    def _reset_stats(self):
        self.stats = {'frames': 0, 'candidates': 0, 'tracks': 0, 'dynamic': 0, 'tf_fail': 0,
                      'proc_s': 0.0}
        self.stats_start = time.monotonic()

    def _log_stats(self):
        """프레임당 후보·확정 트랙·동적 트랙 수와 처리 시간 (가짜 트랙 수, 계산량 확인용)."""
        elapsed = time.monotonic() - self.stats_start
        if elapsed < self.stats_period:
            return
        s, n = self.stats, max(self.stats['frames'], 1)
        self.get_logger().info(
            f'{s["frames"] / elapsed:.1f} Hz, 프레임당 후보 {s["candidates"] / n:.1f}·'
            f'트랙 {s["tracks"] / n:.1f}·동적 {s["dynamic"] / n:.1f}, '
            f'처리 {1000 * s["proc_s"] / n:.1f} ms, TF 실패 {s["tf_fail"]}')
        self._reset_stats()


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
