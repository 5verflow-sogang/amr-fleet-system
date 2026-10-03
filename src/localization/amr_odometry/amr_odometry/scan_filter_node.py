"""scan_raw (Gazebo 원본) → scan (거리·각도·아웃라이어 필터).

인계 계약(명세서 8-2절)의 /scan이다. 토픽은 상대 이름이고 네임스페이스는 launch가 붙인다.
"""

import signal

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from amr_odometry.scan_filter import filter_scan, ScanFilterConfig


class ScanFilterNode(Node):

    def __init__(self):
        super().__init__('scan_filter')
        p = self.declare_parameter
        self.config = ScanFilterConfig(
            range_min=p('range_min', 0.12).value,
            range_max=p('range_max', 25.0).value,
            angle_min=p('angle_min', -np.pi).value,
            angle_max=p('angle_max', np.pi).value,
            outlier_window=p('outlier_window', 2).value,
            outlier_distance=p('outlier_distance', 0.2).value,
            outlier_ratio=p('outlier_ratio', 0.05).value,
            outlier_min_neighbors=p('outlier_min_neighbors', 1).value,
        )
        self.pub = self.create_publisher(LaserScan, 'scan', 10)
        self.create_subscription(LaserScan, 'scan_raw', self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(f'scan filter: scan_raw → scan, {self.config}')

    def on_scan(self, msg: LaserScan):
        ranges = filter_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                             msg.range_min, msg.range_max, self.config)
        out = LaserScan()
        out.header = msg.header
        out.angle_min, out.angle_max = msg.angle_min, msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment, out.scan_time = msg.time_increment, msg.scan_time
        out.range_min = max(msg.range_min, self.config.range_min)
        out.range_max = min(msg.range_max, self.config.range_max)
        out.ranges = ranges.tolist()
        if len(msg.intensities) == len(ranges):
            out.intensities = msg.intensities
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
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
