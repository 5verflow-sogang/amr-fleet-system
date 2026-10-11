"""10 m 사각형을 한 바퀴 돌아 출발점으로 돌아온 뒤 휠 오도메트리 오차를 잰다 (D9 드리프트 실험).

주행은 GT(/amr_01/ground_truth/odom)로 경로를 따라가서 매번 같은 사각형을 돈다. 그래서 차이는 오도메트리 오차만이다.
오차는 출발 자세 기준으로 "휠 odom이 생각한 이동" − "실제 이동"이다.

    python3 drift_square.py --dir ccw --label seed=3 --csv results.csv
"""

import argparse
import csv
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter

V = 0.5            # 직선 속도 [m/s]
W = 0.5            # 제자리 회전 속도 [rad/s]
YAW_TOL = 0.01     # 회전 완료 판정 [rad]
POS_TOL = 0.02     # 꼭짓점 도착 판정 [m]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pose(msg):
    p = msg.pose.pose
    return p.position.x, p.position.y, yaw_of(p.orientation)


def to_frame(origin, x, y):
    """world 점을 origin(x, y, yaw) 기준 좌표로."""
    ox, oy, oyaw = origin
    dx, dy = x - ox, y - oy
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    return c * dx - s * dy, s * dx + c * dy


class Driver:
    def __init__(self, node):
        self.node = node
        self.gt = None
        self.wheel = None
        self.path = 0.0
        node.create_subscription(Odometry, '/amr_01/ground_truth/odom', self._on_gt, 50)
        node.create_subscription(Odometry, '/amr_01/odom', self._on_wheel, 50)
        self.pub = node.create_publisher(Twist, '/amr_01/cmd_vel', 10)

    def _on_gt(self, m):
        if self.gt is not None:
            a, b = pose(self.gt), pose(m)
            self.path += math.hypot(b[0] - a[0], b[1] - a[1])
        self.gt = m

    def _on_wheel(self, m):
        self.wheel = m

    def spin(self, sec=0.02):
        rclpy.spin_once(self.node, timeout_sec=sec)

    def cmd(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = v, w
        self.pub.publish(t)

    def stop(self, sec=1.5):
        end = time.time() + sec
        while time.time() < end:
            self.cmd(0.0, 0.0)
            self.spin(0.05)

    def turn_to(self, yaw):
        while True:
            self.spin()
            err = wrap(yaw - pose(self.gt)[2])
            if abs(err) < YAW_TOL:
                break
            self.cmd(0.0, math.copysign(min(W, max(0.08, 1.5 * abs(err))), err))
        self.stop(0.5)

    def drive_to(self, x, y):
        """(x, y)를 향해 돌고 직선으로 간다. 진행 방향 오차를 바로잡으며 도착하면 멈춘다."""
        gx, gy, _ = pose(self.gt)
        self.turn_to(math.atan2(y - gy, x - gx))
        sx, sy = gx, gy
        heading = math.atan2(y - sy, x - sx)
        length = math.hypot(x - sx, y - sy)
        while True:
            self.spin()
            px, py, pyaw = pose(self.gt)
            along = (px - sx) * math.cos(heading) + (py - sy) * math.sin(heading)
            cross = -(px - sx) * math.sin(heading) + (py - sy) * math.cos(heading)
            remain = length - along
            if remain < POS_TOL:
                break
            v = min(V, max(0.05, 0.8 * remain))
            w = max(-0.5, min(0.5, 2.0 * wrap(heading - pyaw) - 3.0 * cross))
            self.cmd(v, w)
        self.stop(0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', choices=['ccw', 'cw'], default='ccw')
    ap.add_argument('--side', type=float, default=10.0)
    ap.add_argument('--label', default='')
    ap.add_argument('--csv', default='')
    ap.add_argument('--face', type=float, default=None,
                    help='출발 전에 이 방향(world, 도)으로 돈다. 같은 자리에서 ccw·cw 사각형이 모두 통로 안에 들도록')
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('drift_square', parameter_overrides=[Parameter('use_sim_time', value=True)])
    d = Driver(node)
    deadline = time.time() + 30
    while d.gt is None or d.wheel is None:
        d.spin(0.2)
        if time.time() > deadline:
            raise SystemExit('GT 또는 /amr_01/odom이 안 들어온다')
    if a.face is not None:
        d.turn_to(math.radians(a.face))
    d.stop(1.0)
    g0, w0 = pose(d.gt), pose(d.wheel)
    d.path = 0.0

    # 출발 자세 기준 사각형 꼭짓점 (+x = 출발 방향). ccw는 왼쪽(+y)으로 돈다
    s = a.side
    sign = 1.0 if a.dir == 'ccw' else -1.0
    corners = [(s, 0.0), (s, sign * s), (0.0, sign * s), (0.0, 0.0)]
    c, si = math.cos(g0[2]), math.sin(g0[2])
    t0 = time.time()
    for lx, ly in corners:
        d.drive_to(g0[0] + c * lx - si * ly, g0[1] + si * lx + c * ly)
    d.turn_to(g0[2])
    d.stop(1.5)

    g1, w1 = pose(d.gt), pose(d.wheel)
    gdx, gdy = to_frame(g0, g1[0], g1[1])
    wdx, wdy = to_frame(w0, w1[0], w1[1])
    ex, ey = wdx - gdx, wdy - gdy
    epos = math.hypot(ex, ey)
    eyaw = wrap((w1[2] - w0[2]) - (g1[2] - g0[2]))
    cov = d.wheel.pose.covariance
    row = {
        'label': a.label, 'dir': a.dir, 'path_m': round(d.path, 2),
        'gt_return_cm': round(math.hypot(gdx, gdy) * 100, 1),
        'err_x_cm': round(ex * 100, 1), 'err_y_cm': round(ey * 100, 1),
        'err_pos_cm': round(epos * 100, 1), 'err_pos_pct': round(epos / d.path * 100, 2),
        'err_yaw_deg': round(math.degrees(eyaw), 2),
        'rep_sx_cm': round(math.sqrt(max(cov[0], 0)) * 100, 1),
        'rep_sy_cm': round(math.sqrt(max(cov[7], 0)) * 100, 1),
        'rep_syaw_deg': round(math.degrees(math.sqrt(max(cov[35], 0))), 2),
        'wall_s': round(time.time() - t0, 1),
    }
    print('RESULT ' + ' '.join(f'{k}={v}' for k, v in row.items()), flush=True)
    if a.csv:
        new = not os.path.exists(a.csv)
        with open(a.csv, 'a', newline='') as f:
            wr = csv.DictWriter(f, fieldnames=list(row))
            if new:
                wr.writeheader()
            wr.writerow(row)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
