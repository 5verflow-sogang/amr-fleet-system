#!/usr/bin/env python3
"""Gazebo 시계(/clock_raw, 약 1000 Hz)를 받아 rate_hz(기본 200)로 줄여 /clock으로 낸다.

world.launch.py가 시계 브리지 출력을 /clock_raw로 바꾸고 이 노드를 같이 띄운다. 1000 Hz를 받는 건 이 노드
하나뿐이고 나머지 노드는 200 Hz만 받는다. 시간 단위가 5 ms가 되지만 제어(20 Hz)·EKF(50 Hz)·BT(100 Hz)
주기보다 짧아 영향이 없다. Gazebo 물리 스텝(1 ms)은 그대로다. 규칙은 amr_gazebo.clock_throttle.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

from amr_gazebo.clock_throttle import should_publish


class ClockThrottle(Node):

    def __init__(self):
        # 이 노드는 시계를 만드는 쪽이라 use_sim_time을 쓰지 않는다
        super().__init__('clock_throttle')
        rate_hz = float(self.declare_parameter('rate_hz', 200.0).value)
        self._period_ns = int(1e9 / rate_hz)
        self._last_ns = None
        # 늦은 시계는 의미가 없다: 받는 쪽은 최신 1개만 (rclcpp ClockQoS와 같은 발상)
        self._pub = self.create_publisher(Clock, '/clock', QoSProfile(depth=1))
        self.create_subscription(Clock, '/clock_raw', self._on_clock,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info(f'/clock_raw → /clock {rate_hz:g} Hz')

    def _on_clock(self, msg: Clock):
        now_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if should_publish(self._last_ns, now_ns, self._period_ns):
            self._last_ns = now_ns
            self._pub.publish(msg)


def main():
    rclpy.init()
    node = ClockThrottle()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
