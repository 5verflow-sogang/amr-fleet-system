"""시험용 장애물 위치 측정 발행기. LiDAR 군집화 노드가 나오기 전 tracking_node를 시험할 때 쓴다.

perception/obstacle_measurements(PoseArray)로 장애물 2개의 잡음 섞인 위치를 발행한다.
- 직선 왕복: x = 5 sin(t / 5), 최대 1.0 m/s (명세 회피 시나리오 속도). 끝에서 감속해 방향을 바꾼다
- 원 궤도: 중심 (0, −4), 반경 3 m, 0.8 m/s
"""

import math
import signal

from geometry_msgs.msg import Pose, PoseArray
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def obstacle_positions(t: float):
    """시각 t [s]의 두 장애물 진짜 위치."""
    w = 0.8 / 3.0
    return [(5.0 * math.sin(t / 5.0), 2.0), (3.0 * math.cos(w * t), -4.0 + 3.0 * math.sin(w * t))]


class SyntheticObstaclesNode(Node):

    def __init__(self):
        super().__init__('synthetic_obstacles')
        p = self.declare_parameter
        self.frame_id = p('frame_id', 'odom').value
        self.noise_std = p('noise_std', 0.05).value
        self.rng = np.random.default_rng(p('seed', 0).value)
        self.start = self.get_clock().now()
        self.pub = self.create_publisher(PoseArray, 'perception/obstacle_measurements', 10)
        self.create_timer(1.0 / p('rate', 10.0).value, self.on_timer)

    def on_timer(self):
        now = self.get_clock().now()
        t = (now - self.start).nanoseconds * 1e-9
        msg = PoseArray()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        for x, y in obstacle_positions(t):
            pose = Pose()
            noise = self.rng.normal(0.0, self.noise_std, 2)
            pose.position.x, pose.position.y = float(x + noise[0]), float(y + noise[1])
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticObstaclesNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
