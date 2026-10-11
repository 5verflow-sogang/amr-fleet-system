"""도킹 KPI 실험: Nav2가 스테이션 근처에 세운 것처럼 로봇을 흩어 놓고 docking/dock을 불러, 끝난 자세를 GT로 잰다.

명세 KPI: 도킹 위치 2 cm · 방향 1도. 오차는 dock_stations.yaml의 도킹 자세 − Gazebo GT(/amr_01/ground_truth/odom)다.
시작 자세는 도킹 자세 기준 (남은 거리 along, 옆 lateral, 방향 heading)을 균등 분포로 뽑는다.
Nav2 기본 도착 허용치(xy 0.25 m, yaw 0.25 rad)보다 조금 좁게 잡았다.

    python3 dock_trials.py --stations rack_a4 rack_c4 charger_c2 dock_2 --per-station 6 --csv results.csv
"""

import argparse
import csv
import math
import os
import random
import subprocess
import time

from amr_docking.markers import load_stations
from amr_msgs.action import Dock
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient

ROBOT = 'amr_01'
WORLD = 'warehouse'


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def teleport(x, y, yaw):
    req = (f'name: "{ROBOT}", position: {{x: {x:.4f}, y: {y:.4f}, z: 0.02}}, '
           f'orientation: {{z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')
    for _ in range(3):   # 먼 곳으로 옮길 때 서비스가 가끔 시간 초과된다
        out = subprocess.run(
            ['ign', 'service', '-s', f'/world/{WORLD}/set_pose', '--reqtype', 'ignition.msgs.Pose',
             '--reptype', 'ignition.msgs.Boolean', '--timeout', '10000', '--req', req],
            capture_output=True, text=True, timeout=20)
        if 'true' in out.stdout:
            return
    raise RuntimeError(f'set_pose 실패: {out.stdout} {out.stderr}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stations', nargs='+', default=['rack_a4', 'rack_c4', 'charger_c2', 'dock_2'])
    ap.add_argument('--per-station', type=int, default=6)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--along', nargs=2, type=float, default=[-0.15, 0.25])
    ap.add_argument('--lateral', type=float, default=0.15)
    ap.add_argument('--heading-deg', type=float, default=10.0)
    ap.add_argument('--csv', default='')
    a = ap.parse_args()

    stations = load_stations(os.path.join(get_package_share_directory('amr_docking'), 'config', 'dock_stations.yaml'))
    rng = random.Random(a.seed)
    rclpy.init()
    node = rclpy.create_node('dock_trials')
    gt = {}
    node.create_subscription(Odometry, f'/{ROBOT}/ground_truth/odom', lambda m: gt.update(msg=m), 10)
    client = ActionClient(node, Dock, f'/{ROBOT}/docking/dock')
    if not client.wait_for_server(timeout_sec=30.0):
        raise SystemExit('docking/dock 서버 없음')

    rows = []
    for name in a.stations:
        s = stations[name]
        for i in range(a.per_station):
            along = rng.uniform(*a.along)
            lateral = rng.uniform(-a.lateral, a.lateral)
            heading = math.radians(rng.uniform(-a.heading_deg, a.heading_deg))
            c, sn = math.cos(s.yaw), math.sin(s.yaw)
            x = s.x - c * along - sn * lateral
            y = s.y - sn * along + c * lateral
            teleport(x, y, s.yaw + heading)
            end = time.monotonic() + 2.0   # 멈춰 자리 잡을 때까지
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)

            t0 = time.monotonic()
            goal = Dock.Goal()
            goal.station_id = name
            fut = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
            handle = fut.result()
            res_fut = handle.get_result_async()
            rclpy.spin_until_future_complete(node, res_fut, timeout_sec=180.0)
            result = res_fut.result().result
            wall = time.monotonic() - t0
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            p = gt['msg'].pose.pose
            dx, dy = p.position.x - s.x, p.position.y - s.y
            gt_along = -(c * dx + sn * dy)        # 양수면 아직 덜 들어감
            gt_lateral = -sn * dx + c * dy
            gt_yaw = wrap(yaw_of(p.orientation) - s.yaw)
            row = {
                'station': name, 'trial': i,
                'start_along_cm': round(along * 100, 1), 'start_lateral_cm': round(lateral * 100, 1),
                'start_heading_deg': round(math.degrees(heading), 1),
                'success': int(result.success), 'error_code': result.error_code,
                'vision_pos_cm': round(result.position_error * 100, 2),
                'vision_yaw_deg': round(math.degrees(result.yaw_error), 3),
                'gt_along_cm': round(gt_along * 100, 2), 'gt_lateral_cm': round(gt_lateral * 100, 2),
                'gt_pos_cm': round(math.hypot(gt_along, gt_lateral) * 100, 2),
                'gt_yaw_deg': round(math.degrees(gt_yaw), 3),
                'wall_s': round(wall, 1), 'message': result.message,
            }
            rows.append(row)
            print('RESULT ' + ' '.join(f'{k}={v}' for k, v in row.items()), flush=True)
            if a.csv:
                new = not os.path.exists(a.csv)
                with open(a.csv, 'a', newline='') as f:
                    w = csv.DictWriter(f, fieldnames=list(row))
                    if new:
                        w.writeheader()
                    w.writerow(row)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
