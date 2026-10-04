"""perception/obstacle_measurements → 다중 장애물 KF 추적 → perception/obstacles.

측정은 장애물 위치(LiDAR 군집 중심)를 geometry_msgs/PoseArray로 받는다. 위치만 쓰고 방향은 무시한다.
측정은 고정 좌표계(odom 또는 map) 기준이어야 한다. 로봇 기준 좌표면 로봇의 움직임이 장애물 속도로 잡힌다.
LiDAR 군집화 노드가 이 토픽을 발행할 예정이고, 그전에는 synthetic_obstacles_node로 시험한다.
radius는 군집 반경이 나오기 전까지 고정값, is_dynamic은 속도 기반 임시 규칙(tracker.py)이다.
"""

import signal

from geometry_msgs.msg import PoseArray
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from amr_msgs.msg import TrackedObstacle, TrackedObstacleArray
from amr_perception_core.kalman import KalmanConfig
from amr_perception_core.tracker import MultiTargetTracker, TrackerConfig, Track

# 추정하지 않는 성분(z, 회전)의 분산. docs/api.md 4-3절
UNKNOWN_VARIANCE = 1e6


def covariance_6x6(xy_block: np.ndarray) -> list:
    """2x2 (x, y) 공분산을 ROS 6x6 공분산(행 우선 36개)에 넣는다. 나머지 대각은 UNKNOWN_VARIANCE."""
    cov = np.diag([0.0, 0.0] + [UNKNOWN_VARIANCE] * 4)
    cov[:2, :2] = xy_block
    return cov.ravel().tolist()


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
            max_coast_time=p('max_coast_time', 0.5).value,
            dynamic_speed_threshold=p('dynamic_speed_threshold', 0.25).value,
            dynamic_confirm_frames=p('dynamic_confirm_frames', 5).value,
        ))
        # 임시 고정 반경 (LiDAR 군집 반경으로 교체)
        self.radius = p('obstacle_radius', 0.3).value
        self.pub = self.create_publisher(TrackedObstacleArray, 'perception/obstacles', 10)
        self.create_subscription(
            PoseArray, 'perception/obstacle_measurements', self.on_measurements,
            qos_profile_sensor_data)
        self.get_logger().info(
            'tracking: perception/obstacle_measurements → perception/obstacles '
            f'(σa {kalman.accel_noise_std} m/s², σz {kalman.measurement_noise_std} m)')

    def on_measurements(self, msg: PoseArray):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        zs = [(pose.position.x, pose.position.y) for pose in msg.poses]
        last = self.tracker.last_time
        if last is not None and t < last:
            # 시뮬레이션 리셋·rosbag 반복 재생. 그대로 두면 이후 모든 프레임이 거부된다
            self.get_logger().warn(f'측정 시각이 거꾸로 감 ({t:.3f} < {last:.3f} s) → 트랙 초기화')
            self.tracker.reset()
        try:
            tracks = self.tracker.step(t, zs)
        except ValueError as e:
            self.get_logger().warn(f'측정 프레임 무시: {e}', throttle_duration_sec=5.0)
            return
        out = TrackedObstacleArray()
        out.header = msg.header    # 측정 시각·좌표계를 그대로 쓴다
        out.obstacles = [self.to_msg(track) for track in tracks]
        self.pub.publish(out)      # 장애물이 없어도 빈 배열로 발행 (api.md 4-3절)

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
        obstacle.radius = float(self.radius)
        return obstacle


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
