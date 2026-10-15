"""LiDAR 스캔(또는 위치 측정) → 다중 장애물 KF 추적 → perception/obstacles, perception/obstacle_markers,
perception/collision_risks.

measurement_source:
- scan (기본): 필터된 스캔(amr_odometry scan_filter)을 군집화하고, 크기 규칙(R1)으로 벽·랙 면을 빼고,
  다리 두 개를 한 사람으로 묶은 뒤, 스캔 촬영 시각의 TF로 fixed_frame(odom)에 옮겨 추적한다.
  군집화를 이 노드 안에서 하므로 군집 반경을 그대로 트랙 반경으로 쓴다 (perception.md 1-3절 C안).
- poses: geometry_msgs/PoseArray 위치 측정 (synthetic_obstacles_node 시험용). 반경은 obstacle_radius 고정.

출력은 odom 좌표계다 (DWA가 odom에서 동작하고, map은 위치 보정 때 점프해 가짜 속도가 생긴다).
is_dynamic은 추적 속력(R3, tracker.py)에 빈 공간 침입 증거(use_free_space, free_space.py)를 더해 정한다.
로봇이 움직이면 벽·랙 조각의 중심이 미끄러져 속력이 생기는데, 이 조각은 빔이 지나간 빈 칸에 들어가지 않는다.

R2 정적 지도 대조 (use_static_map): 군집화 전에 지도의 막힌 칸(부풀림 map_inflation) 위 스캔 점을 뺀다
(amr_perception_core static_map.py). 스캔 촬영 시각의 TF(map → lidar_link)가 필요하다. 지도나 TF가 없으면
빼지 않고 그대로 추적한다 (기둥·랙 끝이 정적 트랙으로 나가는 R1만의 동작).

충돌 위험(TTC): 트랙마다 이 로봇의 차체(사각형)에 닿기까지 남은 시간 (collision.footprint_time_to_collision).
로봇 위치는 측정 시각의 TF(fixed_frame → base_frame), 로봇 속도는 odom 토픽의 선속도다.
로봇 위치를 모르면 collision_risks를 내지 않는다 (amr_safety가 끊김으로 보고 감속한다).

카메라 연관 (use_camera_association): perception/objects(YOLO 3D 검출, map)를 트랙(odom)에 짝지어
class_name·confidence·source를 채운다 (amr_perception_core association.py). 검출은 검출 노드가 쓴 것과 같은
촬영 시각의 map → odom으로 되돌리므로 위치추정 오차가 상쇄된다. 클래스는 근거가 쌓여야 붙고, 카메라 시야를
벗어나도 근거가 줄어들 때까지 유지한다. source는 최근 camera_recent_s 안에 짝지어졌을 때만 LIDAR_CAMERA다.
클래스별 최소 반경(min_radius_classes·min_radius_values)을 둔다. 사람은 LiDAR 높이에서 다리만 보여 반경이
0.12 m 정도로 작게 잡혔다.
"""

import math
import signal
import time

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from amr_msgs.msg import (
    CollisionRisk,
    CollisionRiskArray,
    DetectedObject3DArray,
    TrackedObstacle,
    TrackedObstacleArray,
)
from amr_perception_core.association import (
    associate,
    AssociationConfig,
    ClassLabeler,
    FixedDetection,
    TrackSnapshot,
)
from amr_perception_core.clustering import (
    ClusteringConfig,
    detect_obstacles,
    scan_to_points,
    transform_points_2d,
)
from amr_perception_core.collision import footprint_time_to_collision
from amr_perception_core.free_space import FreeSpaceGrid
from amr_perception_core.kalman import KalmanConfig
from amr_perception_core.static_map import StaticMapMask
from amr_perception_core.tracker import MultiTargetTracker, Track, TrackerConfig

from .markers import track_markers

# 추정하지 않는 성분(z, 회전)의 분산. docs/api.md 4-3절
UNKNOWN_VARIANCE = 1e6
# map_server의 /map은 transient local (늦게 뜬 노드도 받는다)
MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def covariance_6x6(xy_block: np.ndarray) -> list:
    """2x2 (x, y) 공분산을 ROS 6x6 공분산(행 우선 36개)에 넣는다. 나머지 대각은 UNKNOWN_VARIANCE."""
    cov = np.diag([0.0, 0.0] + [UNKNOWN_VARIANCE] * 4)
    cov[:2, :2] = xy_block
    return cov.ravel().tolist()


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """쿼터니언의 z축 회전(yaw) [rad]. LiDAR가 수평이라 2D 변환에는 yaw만 쓴다."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def detections_in_fixed(msg: DetectedObject3DArray, yaw: float, translation) -> list:
    """검출(msg.header.frame_id, 보통 map) → 고정 좌표계. 위치는 회전 + 평행이동, xy 공분산은 R C Rᵀ."""
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    offset = np.asarray(translation, dtype=float)
    out = []
    for obj in msg.objects:
        p = obj.pose.pose.position
        cov = np.asarray(obj.pose.covariance, dtype=float).reshape(6, 6)[:2, :2]
        out.append(FixedDetection(obj.class_name, float(obj.confidence),
                                  rot @ np.array([p.x, p.y]) + offset, rot @ cov @ rot.T))
    return out


def robot_velocity_in_fixed(linear_x: float, linear_y: float, yaw: float):
    """odom 메시지의 몸체 기준 선속도(child_frame_id 축) → fixed_frame 축 속도."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * linear_x - s * linear_y, s * linear_x + c * linear_y)


def collision_risks(obstacles: TrackedObstacleArray, robot_xy, robot_yaw: float, robot_velocity,
                    length: float, width: float, horizon: float) -> CollisionRiskArray:
    """트랙마다 차체(사각형) TTC. 위치·속도는 obstacles.header.frame_id 기준, 계산은 몸체 축으로 돌려서."""
    out = CollisionRiskArray()
    out.header = obstacles.header
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)

    def to_body(x, y):
        return (c * x + s * y, -s * x + c * y)

    for obs in obstacles.obstacles:
        p, v = obs.pose.pose.position, obs.twist.twist.linear
        ttc = footprint_time_to_collision(
            to_body(p.x - robot_xy[0], p.y - robot_xy[1]),
            to_body(v.x - robot_velocity[0], v.y - robot_velocity[1]),
            float(obs.radius), length, width, horizon)
        out.risks.append(CollisionRisk(track_id=obs.track_id, ttc=float(ttc)))
    return out


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
            dynamic_evidence_frames=p('dynamic_evidence_frames', 3).value,
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
        # 충돌 위험(TTC). 비우면 <ns>/base_footprint. 차체는 명세 0.6 × 0.4 m
        base_frame = p('base_frame', '').value
        self.base_frame = base_frame or (f'{ns}/base_footprint' if ns else 'base_footprint')
        self.robot_length = p('robot_length', 0.6).value
        self.robot_width = p('robot_width', 0.4).value
        self.ttc_horizon = p('ttc_horizon', 5.0).value
        self.odom_timeout = p('odom_timeout', 0.5).value
        # R2 정적 지도 대조
        self.map_frame = p('map_frame', 'map').value
        self.map_inflation = p('map_inflation', 0.15).value
        self.map_occupied_threshold = p('map_occupied_threshold', 65).value
        self.map_align_max_shift = p('map_align_max_shift', 0.3).value
        self.map_align_step = p('map_align_step', 0.05).value
        self.static_map: StaticMapMask = None
        # 빈 공간 침입 증거 (동적 판정)
        self.free_space = None
        if p('use_free_space', True).value:
            self.free_space = FreeSpaceGrid(
                resolution=p('free_space_resolution', 0.1).value,
                size=p('free_space_size', 30.0).value,
                max_range=p('free_space_max_range', 12.0).value,
                endpoint_margin=p('free_space_endpoint_margin', 0.2).value,
                free_memory=p('free_space_memory', 3.0).value,
                hit_quiet=p('free_space_hit_quiet', 3.0).value,
                hit_clearance=p('free_space_hit_clearance', 0.2).value)
        self.free_evidence_min_points = p('free_evidence_min_points', 2).value
        # 카메라 연관
        self.association = AssociationConfig(
            gate_mahalanobis_sq=p('assoc_gate_mahalanobis_sq', 9.21).value,
            max_distance=p('assoc_max_distance', 1.0).value,
            min_confidence=p('assoc_min_confidence', 0.5).value,
            evidence_tau=p('assoc_evidence_tau', 5.0).value,
            min_evidence=p('assoc_min_evidence', 1.0).value,
            release_evidence=p('assoc_release_evidence', 0.3).value,
            camera_recent_s=p('assoc_camera_recent_s', 1.0).value)
        self.labeler = ClassLabeler(self.association)
        # 클래스별 최소 반경 [m] (ROS 파라미터에 dict가 없어 같은 길이의 목록 두 개)
        classes = list(p('min_radius_classes', ['person']).value)
        values = list(p('min_radius_values', [0.25]).value)
        if len(classes) != len(values):
            raise ValueError('min_radius_classes와 min_radius_values의 길이가 다르다')
        self.class_min_radius = dict(zip(classes, values))
        self.last_odom = None          # (수신 시각 [s], 몸체 기준 vx, vy)
        self._reset_stats()

        self.pub = self.create_publisher(TrackedObstacleArray, 'perception/obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'perception/obstacle_markers', 10)
        self.risk_pub = self.create_publisher(CollisionRiskArray, 'perception/collision_risks', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        if p('use_camera_association', True).value:
            self.create_subscription(DetectedObject3DArray,
                                     p('detections_topic', 'perception/objects').value,
                                     self.on_detections, 10)
        if p('use_static_map', False).value:
            self.create_subscription(OccupancyGrid, p('map_topic', '/map').value, self.on_map,
                                     MAP_QOS)
        if source == 'scan':
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
    def on_map(self, msg: OccupancyGrid):
        o = msg.info.origin
        self.static_map = StaticMapMask.from_grid_message(
            msg.info.width, msg.info.height, msg.info.resolution,
            (o.position.x, o.position.y,
             yaw_from_quaternion(o.orientation.x, o.orientation.y, o.orientation.z,
                                 o.orientation.w)),
            msg.data, occupied_threshold=self.map_occupied_threshold,
            inflation=self.map_inflation)
        self.get_logger().info(
            f'정적 지도 받음 ({msg.info.width}×{msg.info.height}, {msg.info.resolution} m, '
            f'부풀림 {self.map_inflation} m): 지도 위 스캔 점을 추적에서 뺀다')

    def remove_static_points(self, scan: LaserScan, odom_points: np.ndarray, valid) -> list:
        """지도의 막힌 칸 위 빔을 NaN(무효)으로 바꾼 거리 배열. 지도·TF가 없으면 그대로.

        odom_points: 스캔 촬영 시각 TF로 fixed_frame에 옮긴 빔별 점. map → fixed_frame은 가장 최근 값을
        쓴다. 위치추정(ekf_global)의 map → odom은 스캔보다 늦게 나와 촬영 시각 값을 자주 못 구하는데
        (Gazebo, 5초에 3~6번), 천천히만 바뀌므로 최근 값이면 충분하다.
        """
        if self.static_map is None:
            return scan.ranges
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.fixed_frame, Time())
        except TransformException as e:
            self.stats['map_tf_fail'] += 1
            self.get_logger().warn(f'map TF 없음, 지도 대조 건너뜀: {e}', throttle_duration_sec=5.0)
            return scan.ranges
        q, t = tf.transform.rotation, tf.transform.translation
        in_map = transform_points_2d(odom_points, yaw_from_quaternion(q.x, q.y, q.z, q.w),
                                     (t.x, t.y))
        if self.map_align_max_shift > 0.0:
            # 위치추정과 지도의 어긋남을 이 스캔으로 보정한다 (static_map.best_offset)
            dx, dy, _, _ = self.static_map.best_offset(
                in_map[valid], self.map_align_max_shift, self.map_align_step)
            in_map = in_map + (dx, dy)
            self.stats['align_shift'] += math.hypot(dx, dy)
        static = self.static_map.is_static(in_map) & valid
        self.stats['beams'] += int(valid.sum())
        self.stats['static_beams'] += int(static.sum())
        ranges = np.asarray(scan.ranges, dtype=float)
        ranges[static] = np.nan
        return ranges

    def on_detections(self, msg: DetectedObject3DArray):
        """카메라 검출을 확정 트랙에 짝지어 클래스 근거를 쌓는다."""
        tracks = self.tracker.confirmed_tracks
        if not msg.objects or not tracks or self.tracker.last_time is None:
            return
        # 검출 노드가 map으로 옮길 때 쓴 촬영 시각의 map → odom을 그대로 되돌린다 (위치추정 오차 상쇄).
        # 그 시각 값이 없으면 최근 값을 쓴다 (map → odom은 천천히 바뀐다)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(self.fixed_frame, msg.header.frame_id,
                                                     Time())
            except TransformException as e:
                self.stats['assoc_tf_fail'] += 1
                self.get_logger().warn(f'검출 TF 없음, 카메라 연관 건너뜀: {e}',
                                       throttle_duration_sec=5.0)
                return
        q, t = tf.transform.rotation, tf.transform.translation
        detections = detections_in_fixed(msg, yaw_from_quaternion(q.x, q.y, q.z, q.w),
                                         (t.x, t.y))
        t_detection = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        snapshots = [TrackSnapshot(track.track_id, track.kf.position, track.kf.velocity,
                                   track.kf.position_covariance, track.kf.velocity_covariance,
                                   self.tracker.last_time) for track in tracks]
        pairs = associate(snapshots, detections, t_detection, self.association)
        for ti, di in pairs:
            self.labeler.update(snapshots[ti].track_id, detections[di].class_name,
                                detections[di].confidence, t_detection)
        self.stats['detections'] += len(detections)
        self.stats['assoc'] += len(pairs)

    def on_odom(self, msg: Odometry):
        v = msg.twist.twist.linear
        self.last_odom = (self.get_clock().now().nanoseconds * 1e-9, v.x, v.y)

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
        t_tf = time.monotonic()
        q, t = tf.transform.rotation, tf.transform.translation
        scan_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        if self.static_map is not None:
            beam_points, valid = scan_to_points(scan.ranges, scan.angle_min, scan.angle_increment)
            ranges = self.remove_static_points(
                scan, transform_points_2d(beam_points, scan_yaw, (t.x, t.y)), valid)
        else:
            ranges = scan.ranges
        t_map = time.monotonic()
        clusters = detect_obstacles(ranges, scan.angle_min, scan.angle_increment, self.clustering)
        t_cluster = time.monotonic()
        points = np.array([c.centroid for c in clusters]).reshape(-1, 2)
        points = transform_points_2d(points, scan_yaw, (t.x, t.y))
        evidence = None
        if self.free_space is not None:
            now = Time.from_msg(scan.header.stamp).nanoseconds * 1e-9
            if self.tracker.last_time is not None and now < self.tracker.last_time:
                self.free_space = FreeSpaceGrid(**self._free_space_args())   # 시각이 거꾸로 감
            # 이번 스캔을 넣기 전 격자로 판정한다 (군집 점이 방금 전까지 빈 칸이었는가)
            evidence = [int(self.free_space.evidence(
                transform_points_2d(c.points, scan_yaw, (t.x, t.y)), now).sum())
                >= self.free_evidence_min_points for c in clusters]
            angles = scan_yaw + scan.angle_min + scan.angle_increment * np.arange(len(scan.ranges))
            self.free_space.integrate((t.x, t.y), np.c_[np.cos(angles), np.sin(angles)],
                                      np.asarray(scan.ranges, dtype=float), now)
            self.stats['evidence'] += sum(evidence)
        t_free = time.monotonic()
        header = scan.header
        header.frame_id = self.fixed_frame    # 시각은 스캔 촬영 시각 그대로
        self.stats['candidates'] += len(clusters)
        self.track_and_publish(header, points, [c.radius for c in clusters], evidence)
        end = time.monotonic()
        self.stats['proc_s'] += end - start
        # 단계별 [s]: TF 기다림(계산 아님), 지도 대조, 군집화, 빈 공간, 추적·발행(TTC 포함)
        for key, value in (('tf_s', t_tf - start), ('map_s', t_map - t_tf),
                           ('cluster_s', t_cluster - t_map), ('free_s', t_free - t_cluster),
                           ('track_s', end - t_free)):
            self.stats[key] += value

    def on_measurements(self, msg: PoseArray):
        zs = [(pose.position.x, pose.position.y) for pose in msg.poses]
        self.track_and_publish(msg.header, zs, None)

    # ------------------------------------------------------------------ 추적·발행
    def _free_space_args(self) -> dict:
        g = self.free_space
        return dict(resolution=g.resolution, size=g.cells * g.resolution, max_range=g.max_range,
                    endpoint_margin=g.endpoint_margin, free_memory=g.free_memory,
                    hit_quiet=g.hit_quiet, hit_clearance=g.hit_clearance)

    def track_and_publish(self, header, measurements, radii, evidence=None):
        t = Time.from_msg(header.stamp).nanoseconds * 1e-9
        last = self.tracker.last_time
        if last is not None and t < last:
            # 시뮬레이션 리셋·rosbag 반복. 그대로 두면 이후 모든 프레임이 거부된다
            self.get_logger().warn(f'측정 시각이 거꾸로 감 ({t:.3f} < {last:.3f} s) → 트랙 초기화')
            self.tracker.reset()
        try:
            tracks = self.tracker.step(t, measurements, radii, evidence)
        except ValueError as e:
            self.get_logger().warn(f'측정 프레임 무시: {e}', throttle_duration_sec=5.0)
            return
        self.labeler.prune(track.track_id for track in self.tracker.tracks)
        out = TrackedObstacleArray()
        out.header = header
        out.obstacles = [self.to_msg(track, t) for track in tracks]
        self.pub.publish(out)      # 장애물이 없어도 빈 배열로 발행 (api.md 4-3절)
        self.publish_risks(out)
        if self.publish_markers:
            self.marker_pub.publish(track_markers(out))
        self.stats['frames'] += 1
        self.stats['tracks'] += len(tracks)
        self.stats['dynamic'] += sum(track.is_dynamic for track in tracks)
        self._log_stats()

    def publish_risks(self, obstacles: TrackedObstacleArray):
        """측정 시각의 로봇 위치·속도로 트랙별 TTC를 낸다. 로봇 위치를 모르면 내지 않는다."""
        try:
            tf = self.tf_buffer.lookup_transform(
                obstacles.header.frame_id, self.base_frame,
                Time.from_msg(obstacles.header.stamp))   # 스캔 TF를 이미 기다렸으니 다시 안 기다린다
        except TransformException as e:
            self.stats['risk_skip'] += 1
            self.get_logger().warn(f'로봇 위치 TF 없음, 충돌 위험 건너뜀: {e}',
                                   throttle_duration_sec=5.0)
            return
        q, t = tf.transform.rotation, tf.transform.translation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        velocity = (0.0, 0.0)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_odom is not None and now - self.last_odom[0] <= self.odom_timeout:
            velocity = robot_velocity_in_fixed(self.last_odom[1], self.last_odom[2], yaw)
        else:
            # 정지로 보면 로봇이 다가가는 위험을 놓친다. amr_safety가 odom 끊김을 따로 감속으로 처리한다
            self.get_logger().warn('odom 끊김: 로봇 속도 0으로 TTC 계산', throttle_duration_sec=5.0)
        self.risk_pub.publish(collision_risks(obstacles, (t.x, t.y), yaw, velocity,
                                              self.robot_length, self.robot_width,
                                              self.ttc_horizon))

    def to_msg(self, track: Track, t: float) -> TrackedObstacle:
        kf = track.kf
        obstacle = TrackedObstacle()
        obstacle.track_id = track.track_id
        label = self.labeler.label(track.track_id, t)
        obstacle.source = (TrackedObstacle.SOURCE_LIDAR_CAMERA if label.camera_recent
                           else TrackedObstacle.SOURCE_LIDAR)
        obstacle.class_name = label.class_name
        obstacle.confidence = float(label.confidence)
        obstacle.is_dynamic = track.is_dynamic
        pose = obstacle.pose.pose
        pose.position.x, pose.position.y = kf.position.tolist()
        pose.orientation.w = 1.0
        obstacle.pose.covariance = covariance_6x6(kf.position_covariance)
        twist = obstacle.twist.twist
        twist.linear.x, twist.linear.y = kf.velocity.tolist()
        obstacle.twist.covariance = covariance_6x6(kf.velocity_covariance)
        radius = track.radius if track.radius is not None else self.radius
        obstacle.radius = float(max(radius, self.class_min_radius.get(label.class_name, 0.0)))
        return obstacle

    # ------------------------------------------------------------------ 기록
    def _reset_stats(self):
        self.stats = {'frames': 0, 'candidates': 0, 'tracks': 0, 'dynamic': 0, 'tf_fail': 0,
                      'risk_skip': 0, 'map_tf_fail': 0, 'beams': 0, 'static_beams': 0,
                      'align_shift': 0.0, 'evidence': 0, 'detections': 0, 'assoc': 0,
                      'assoc_tf_fail': 0, 'proc_s': 0.0, 'tf_s': 0.0,
                      'map_s': 0.0, 'cluster_s': 0.0, 'free_s': 0.0, 'track_s': 0.0}
        self.stats_start = time.monotonic()

    def _log_stats(self):
        """프레임당 후보·확정 트랙·동적 트랙 수와 처리 시간 (가짜 트랙 수, 계산량 확인용)."""
        elapsed = time.monotonic() - self.stats_start
        if elapsed < self.stats_period:
            return
        s, n = self.stats, max(self.stats['frames'], 1)
        self.get_logger().info(
            f'{s["frames"] / elapsed:.1f} Hz, 프레임당 후보 {s["candidates"] / n:.1f}·'
            f'트랙 {s["tracks"] / n:.1f}·동적 {s["dynamic"] / n:.1f}·'
            f'빈칸 침입 {s["evidence"] / n:.1f}, '
            f'처리 {1000 * s["proc_s"] / n:.1f} ms (TF 대기 {1000 * s["tf_s"] / n:.0f}·'
            f'지도 {1000 * s["map_s"] / n:.0f}·군집 {1000 * s["cluster_s"] / n:.0f}·'
            f'빈칸 {1000 * s["free_s"] / n:.0f}·추적 {1000 * s["track_s"] / n:.0f}), '
            f'TF 실패 {s["tf_fail"]}, '
            f'TTC 건너뜀 {s["risk_skip"]}, '
            f'지도 위 빔 {100 * s["static_beams"] / max(s["beams"], 1):.0f}% '
            f'(보정 평균 {s["align_shift"] / n:.2f} m, map TF 실패 {s["map_tf_fail"]}), '
            f'카메라 검출 {s["detections"]}개 중 연관 {s["assoc"]} (TF 실패 {s["assoc_tf_fail"]})')
        self._reset_stats()


def make_executor(node: Node) -> MultiThreadedExecutor:
    """TF를 콜백과 다른 스레드에서 받는 executor.

    단일 스레드(rclpy.spin)면 /tf 수신이 콜백과 같은 줄에 서서, 콜백이 무겁거나 lookup_transform(timeout=)으로
    기다리는 동안 TF가 쌓여 버퍼가 뒤처진다 (Mac Docker에서 0.35~0.6 s, "TF 없음" 반복).
    TransformListener는 /tf 구독을 Reentrant 그룹에 두므로 두 번째 스레드가 받는다. 이 노드의 콜백은 기본
    그룹(MutuallyExclusive)이라 지금처럼 하나씩 실행되어 공유 상태에 락이 필요 없다.
    TransformListener(spin_thread=True)는 같은 노드를 다른 executor에 붙였다가 rclpy.spin이 도로 가져가서
    (노드 하나 = executor 하나) Humble에서 효과가 없다.
    """
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    return executor


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    executor = make_executor(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        executor.shutdown()
        node.destroy_node()
