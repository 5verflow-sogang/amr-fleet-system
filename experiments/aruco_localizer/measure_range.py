"""ArUco 위치 노드(aruco_localizer) 거리별 인식률·위치 오차 (Gazebo GT 기준).

로봇을 마커판 정면 앞 d m에 (옆으로 lateral m, 판 쪽에서 heading° 돌려) 순간 이동시키고, hold_s 동안 나온
aruco_pose를 GT와 비교한다. 인식률 = 받은 자세 수 / (hold_s × max_hz).

    python3 measure_range.py --stations rack_a5 dock_1 --distances 1 2 3 4 5 --csv range.csv

로봇 1대 sim + 카메라, aruco_localizer only_when_lost:=false가 떠 있어야 한다. 순간 이동 뒤 settle_s 기다린다.
"""

import argparse
import csv
import math
import os
import statistics as st
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from amr_docking.markers import load_stations
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stations', nargs='+', default=['rack_a5', 'dock_1'])
    ap.add_argument('--distances', nargs='+', type=float, default=[1, 2, 3, 4, 5])
    ap.add_argument('--laterals', nargs='+', type=float, default=[0.0])
    ap.add_argument('--headings', nargs='+', type=float, default=[0.0], help='판을 바라보는 방향에서 돌림 [deg]')
    ap.add_argument('--hold-s', type=float, default=4.0)
    ap.add_argument('--settle-s', type=float, default=2.0)
    ap.add_argument('--max-hz', type=float, default=5.0)
    ap.add_argument('--robot', default='amr_01')
    ap.add_argument('--world', default='warehouse')
    ap.add_argument('--csv', default='')
    a = ap.parse_args()

    stations = load_stations(os.path.join(get_package_share_directory('amr_docking'), 'config',
                                          'dock_stations.yaml'))
    rclpy.init()
    node = rclpy.create_node('aruco_range')
    tf = Buffer()
    TransformListener(tf, node)
    got, gt = [], {}
    node.create_subscription(PoseWithCovarianceStamped, f'/{a.robot}/aruco_pose', got.append, 50)
    node.create_subscription(Odometry, f'/{a.robot}/ground_truth/odom', lambda m: gt.update(p=m.pose.pose), 10)

    def spin(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    spin(2.0)
    origin = tf.lookup_transform('world', 'map', rclpy.time.Time()).transform
    ox, oy, oyaw = origin.translation.x, origin.translation.y, yaw_of(origin.rotation)

    def to_world(x, y, yaw):
        c, s = math.cos(oyaw), math.sin(oyaw)
        return ox + c * x - s * y, oy + s * x + c * y, wrap(oyaw + yaw)

    def teleport(x, y, yaw):
        req = (f'name: "{a.robot}", position: {{x: {x:.4f}, y: {y:.4f}, z: 0.02}}, '
               f'orientation: {{z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')
        for _ in range(3):
            out = subprocess.run(['ign', 'service', '-s', f'/world/{a.world}/set_pose', '--reqtype',
                                  'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean', '--timeout',
                                  '10000', '--req', req], capture_output=True, text=True, timeout=20)
            if 'true' in out.stdout:
                return True
        return False

    rows = []
    for name in a.stations:
        s = stations[name]
        bx, by, _ = s.board_center
        n = s.board_yaw
        for d in a.distances:
            for lat in a.laterals:
                for hd in a.headings:
                    x = bx + d * math.cos(n) - lat * math.sin(n)
                    y = by + d * math.sin(n) + lat * math.cos(n)
                    yaw = wrap(math.atan2(by - y, bx - x) + math.radians(hd))
                    if not teleport(x, y, yaw):
                        print(f'{name} d={d}: 순간 이동 실패', flush=True)
                        continue
                    spin(a.settle_s)
                    got.clear()
                    spin(a.hold_s)
                    p = gt['p']
                    gx, gy, gyaw = p.position.x, p.position.y, yaw_of(p.orientation)
                    errs, yerrs, sig = [], [], []
                    for m in got:
                        wx, wy, wyaw = to_world(m.pose.pose.position.x, m.pose.pose.position.y,
                                                yaw_of(m.pose.pose.orientation))
                        errs.append(math.hypot(wx - gx, wy - gy))
                        yerrs.append(abs(math.degrees(wrap(wyaw - gyaw))))
                        sig.append(math.sqrt(m.pose.covariance[0]))
                    row = {
                        'station': name, 'board': 'large' if s.board.spacing > 0.5 else 'small',
                        'distance_m': d, 'lateral_m': lat, 'heading_deg': hd,
                        'rate': round(len(got) / (a.hold_s * a.max_hz), 2),
                        'pos_err_cm_mean': round(st.mean(errs) * 100, 1) if errs else '',
                        'pos_err_cm_max': round(max(errs) * 100, 1) if errs else '',
                        'yaw_err_deg_mean': round(st.mean(yerrs), 2) if yerrs else '',
                        'yaw_err_deg_max': round(max(yerrs), 2) if yerrs else '',
                        'sigma_xy_cm': round(st.mean(sig) * 100, 1) if sig else '',
                    }
                    rows.append(row)
                    print(row, flush=True)
    if a.csv and rows:
        with open(a.csv, 'w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except TransformException as e:
        raise SystemExit(f'TF world → map 없음: {e}')
