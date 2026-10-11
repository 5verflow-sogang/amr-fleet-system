"""고정 경로를 내고 GT 궤적을 기록한다 (Nav2 없이 path_tracker_node 측정용).

    python3 path_publisher.py --out <결과 폴더> [--ns amr_01] [--frame amr_01/odom] [--start-x 0 --start-y 1.5]

- /<ns>/plan: path_def의 경로를 시작 직후 3번 낸다. 추종기는 새 경로를 받을 때마다 진행 상태를
  초기화하므로 주행 중에는 다시 내지 않는다
- /<ns>/ground_truth/odom (frame world)을 <out>/gt.csv로 기록한다. 경로는 odom 프레임이므로 출발 자세
  (start_x, start_y, yaw 0)를 빼서 같은 프레임으로 적는다
- 로봇이 움직였다가 경로 끝 근처에서 2초(시뮬레이션 시간) 동안 서 있으면, 또는 timeout(실제 시간)이 지나면 끝낸다
"""

import argparse
import csv
import math
import os
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node

from path_def import build_path


class PathPublisher(Node):

    def __init__(self, args):
        super().__init__('cte_path_publisher')
        self.args = args
        self.path = build_path()
        self.pub = self.create_publisher(Path, f'/{args.ns}/plan', 10)
        self.create_subscription(Odometry, f'/{args.ns}/ground_truth/odom', self.on_gt, 50)
        os.makedirs(args.out, exist_ok=True)
        self.gt_file = open(os.path.join(args.out, 'gt.csv'), 'w', newline='')
        self.gt = csv.writer(self.gt_file)
        self.gt.writerow(['timestamp', 'x', 'y', 'yaw', 'v'])
        self.sent = 0
        self.moved = False
        self.still_since = None
        self.done = False
        self.create_timer(0.5, self.publish_path)

    def publish_path(self):
        if self.sent >= 3:
            return
        msg = Path()
        msg.header.frame_id = self.args.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, _ in self.path:
            p = PoseStamped()
            p.header.frame_id = self.args.frame
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.pub.publish(msg)
        self.sent += 1

    def on_gt(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x - self.args.start_x
        y = msg.pose.pose.position.y - self.args.start_y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.gt.writerow([f'{t:.9f}', f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}', f'{v:.6f}'])

        if v > 0.1:
            self.moved = True
        end_x, end_y, _ = self.path[-1]
        near_end = math.hypot(x - end_x, y - end_y) < 0.5
        if self.moved and near_end and v < 0.01:
            if self.still_since is None:
                self.still_since = t
            elif t - self.still_since > 2.0:
                self.done = True
        else:
            self.still_since = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    parser.add_argument('--ns', default='amr_01')
    parser.add_argument('--frame', default='amr_01/odom')
    parser.add_argument('--start-x', type=float, default=0.0)
    parser.add_argument('--start-y', type=float, default=1.5)
    parser.add_argument('--timeout', type=float, default=600.0, help='실제 시간 [s]')
    args = parser.parse_args()

    rclpy.init()
    node = PathPublisher(args)
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and not node.done and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.gt_file.close()
    node.get_logger().info('경로 끝 정지 확인' if node.done else 'timeout')
    done = node.done
    node.destroy_node()
    rclpy.shutdown()
    return 0 if done else 1


if __name__ == '__main__':
    raise SystemExit(main())
