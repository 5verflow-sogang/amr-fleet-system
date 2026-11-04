"""camera/depth/image_raw + camera/camera_info → camera/depth/points (포인트클라우드 변환·복셀 다운샘플).

점은 Depth 영상의 frame_id(광학 프레임 <ns>/camera_optical_frame) 기준이다. 받는 쪽이 TF로 바꾼다.
Gazebo의 camera/points는 점 좌표축이 frame_id와 달라 쓰지 않는다 (amr_description README "센서 검증").
토픽은 상대 이름이고 네임스페이스는 launch가 붙인다.
"""

import signal
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from amr_odometry.depth_cloud import decode_depth, depth_to_cloud, DepthCloudConfig, rate_limited

XYZ_FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate('xyz')]


def to_cloud_msg(header, points: np.ndarray) -> PointCloud2:
    msg = PointCloud2()
    msg.header = header
    msg.height, msg.width = 1, len(points)
    msg.fields = XYZ_FIELDS
    msg.is_bigendian = False
    msg.point_step, msg.row_step = 12, 12 * len(points)
    msg.is_dense = True
    msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
    return msg


class DepthCloudNode(Node):

    def __init__(self):
        super().__init__('depth_cloud')
        p = self.declare_parameter
        self.config = DepthCloudConfig(
            range_min=p('range_min', 0.3).value,
            range_max=p('range_max', 10.0).value,
            stride=p('stride', 2).value,
            voxel_size=p('voxel_size', 0.05).value,
            min_points_per_voxel=p('min_points_per_voxel', 1).value,
        )
        # Depth는 30 Hz로 오지만 이 주기까지만 처리한다 (영상 시각 기준, 0이면 전부)
        self.max_rate_hz = p('max_rate_hz', 10.0).value
        self.k = None
        self.last_stamp = None
        self.stats = [0, 0, 0, 0.0]     # 처리 장수, 입력 픽셀, 출력 점, 처리 시간 합 [s] (10초마다 로그)
        self.pub = self.create_publisher(PointCloud2, 'camera/depth/points', 10)
        self.create_subscription(CameraInfo, 'camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, 'camera/depth/image_raw', self.on_depth, qos_profile_sensor_data)
        self.create_timer(10.0, self.log_stats)
        self.get_logger().info(f'depth cloud: camera/depth/image_raw → camera/depth/points, {self.config}, '
                               f'최대 {self.max_rate_hz} Hz')

    def on_info(self, msg: CameraInfo):
        self.k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_depth(self, msg: Image):
        if self.k is None:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if rate_limited(stamp, self.last_stamp, self.max_rate_hz):
            return
        self.last_stamp = stamp
        t0 = time.perf_counter()
        try:
            depth = decode_depth(bytes(msg.data), msg.encoding, msg.width, msg.height, msg.step,
                                 bool(msg.is_bigendian))
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=10.0)
            return
        cloud = depth_to_cloud(depth, *self.k, self.config)
        self.pub.publish(to_cloud_msg(msg.header, cloud))
        s = self.stats
        s[0] += 1
        s[1] += msg.width * msg.height
        s[2] += len(cloud)
        s[3] += time.perf_counter() - t0

    def log_stats(self):
        n, pixels, points, dt = self.stats
        if n:
            self.get_logger().info(f'{n}장: 픽셀 {pixels // n} → 점 {points // n} (평균), 처리 {1000 * dt / n:.1f} ms/장')
        self.stats = [0, 0, 0, 0.0]


def main(args=None):
    rclpy.init(args=args)
    node = DepthCloudNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C는 터미널과 launch에서 SIGINT가 두 번 온다. 정리 중에 끊기지 않게 이후 신호는 무시한다
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
