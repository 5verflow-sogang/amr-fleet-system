#!/usr/bin/env python3
"""slam_toolbox의 map → amr_01/odom 보정이 크게 바뀌는 순간(루프 클로저 후보)을 기록한다.

매핑 중 map → odom은 스캔 매칭으로 조금씩 바뀌고, 루프 클로저로 그래프를 다시 풀면 한 번에 크게 바뀐다.
0.1 s(sim)마다 읽어 직전 대비 2 cm 또는 0.5° 넘게 바뀌면 그때 로봇 위치(GT)와 함께 출력한다.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Logger(Node):
    def __init__(self):
        super().__init__('tf_jump_logger', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.gt = None
        self.create_subscription(Odometry, '/amr_01/ground_truth/odom', self.on_gt, 10)
        self.last = None
        self.jumps = 0
        self.create_timer(0.1, self.tick)

    def on_gt(self, m):
        self.gt = (m.pose.pose.position.x, m.pose.pose.position.y)

    def tick(self):
        try:
            t = self.buf.lookup_transform('map', 'amr_01/odom', rclpy.time.Time())
        except Exception:
            return
        cur = (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))
        if self.last is not None:
            dd = math.hypot(cur[0] - self.last[0], cur[1] - self.last[1])
            da = abs(math.degrees(math.atan2(math.sin(cur[2] - self.last[2]), math.cos(cur[2] - self.last[2]))))
            if dd > 0.02 or da > 0.5:
                self.jumps += 1
                gt = f'({self.gt[0]:.1f}, {self.gt[1]:.1f})' if self.gt else '?'
                print(f'JUMP t={self.get_clock().now().nanoseconds * 1e-9:.1f} 이동 {dd * 100:.1f} cm, '
                      f'회전 {da:.2f}°, 로봇 world {gt}, map→odom=({cur[0]:.3f}, {cur[1]:.3f}, '
                      f'{math.degrees(cur[2]):.2f}°)', flush=True)
        self.last = cur


def main():
    rclpy.init()
    n = Logger()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    if n.last:
        print(f'FINAL map→odom=({n.last[0]:.3f}, {n.last[1]:.3f}, {math.degrees(n.last[2]):.2f}°), '
              f'큰 변화 {n.jumps}번', flush=True)


if __name__ == '__main__':
    main()
