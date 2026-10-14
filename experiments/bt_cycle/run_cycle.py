"""배송 작업을 Fleet 서버에 요청하고 TaskStatus를 끝(COMPLETED·FAILED)까지 따라간다 (4주차 작업 1·3).

작업 1: 한 번 돌려 COMPLETED를 본다. 작업 3: N번 반복해 단일 작업 성공률을 잰다 (97 % 판정에는 34회 이상).
경로는 (픽업, 드롭오프) 스테이션 쌍 목록을 차례로 돈다. 끝 자세는 Gazebo GT로 드롭오프 도킹 자세와 비교한다.

    python3 run_cycle.py --pairs rack_c6:rack_b6 rack_b6:rack_c6 --repeat 1 --csv results.csv

로봇 1대(amr_01), system.launch.py nav2:=true docking:=true enable_camera:=true num_robots:=1이 떠 있어야 한다.
작업 좌표는 map 기준이다 (stations.yaml은 world → map_origin_in_world로 바꾼다, system.launch.py 기본 [0, 1.5, 0]).
"""

import argparse
import csv
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from amr_msgs.msg import Task, TaskStatus
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
import yaml

STATES = {0: 'PENDING', 1: 'IN_PROGRESS', 2: 'COMPLETED', 3: 'FAILED'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pairs', nargs='+', default=['rack_c6:rack_b6', 'rack_b6:rack_c6'],
                    help='픽업:드롭오프 스테이션 쌍. 차례로 돈다')
    ap.add_argument('--repeat', type=int, default=1, help='쌍 목록을 몇 바퀴 돌지')
    ap.add_argument('--origin', nargs=3, type=float, default=[0.0, 1.5, 0.0], help='map_origin_in_world')
    ap.add_argument('--timeout', type=float, default=900.0, help='작업 하나의 시간 상한 [s, 실제 시간]')
    ap.add_argument('--robot', default='amr_01')
    ap.add_argument('--csv', default='')
    a = ap.parse_args()

    stations = yaml.safe_load(open(os.path.join(
        get_package_share_directory('amr_fleet_manager'), 'config', 'stations.yaml'), encoding='utf-8'))['stations']
    ox, oy, oyaw = a.origin

    def pose_of(name):
        x, y, yaw = stations[name]
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        dx, dy = x - ox, y - oy
        p = Task().pickup_pose
        p.position.x, p.position.y = c * dx - s * dy, s * dx + c * dy
        p.orientation.z, p.orientation.w = math.sin((yaw - oyaw) / 2), math.cos((yaw - oyaw) / 2)
        return p

    rclpy.init()
    node = rclpy.create_node('bt_cycle')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(Task, '/fleet/task_requests', qos)
    current = {'id': None, 'events': []}
    gt = {}

    def on_status(m):
        if m.task_id != current['id']:
            return
        current['events'].append((time.monotonic(), m))
        print(f'  {time.strftime("%H:%M:%S")} {STATES.get(m.state, m.state):11s} {m.phase:16s} '
              f'dist {m.distance_traveled:6.1f} m  code {m.error.code} {m.error.message}', flush=True)

    node.create_subscription(TaskStatus, '/fleet/task_status', on_status, qos)
    node.create_subscription(Odometry, f'/{a.robot}/ground_truth/odom', lambda m: gt.update(p=m.pose.pose), 10)
    end = time.monotonic() + 30
    while pub.get_subscription_count() == 0 and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    run_tag = time.strftime('%m%d%H%M')
    n = 0
    for _ in range(a.repeat):
        for pair in a.pairs:
            pickup, dropoff = pair.split(':')
            n += 1
            task_id = f'cycle_{run_tag}_{n:03d}'
            current['id'], current['events'] = task_id, []
            t = Task()
            t.task_id, t.task_type, t.item_type = task_id, Task.TYPE_DELIVERY, Task.ITEM_SMALL
            t.pickup_station_id, t.pickup_pose = pickup, pose_of(pickup)
            t.dropoff_station_id, t.dropoff_pose = dropoff, pose_of(dropoff)
            print(f'#{n} {task_id}: {pickup} → {dropoff}', flush=True)
            t0 = time.monotonic()
            pub.publish(t)
            while time.monotonic() - t0 < a.timeout:
                rclpy.spin_once(node, timeout_sec=0.2)
                ev = current['events']
                if ev and ev[-1][1].state in (2, 3):
                    break
            for _ in range(10):   # 끝 자세가 GT에 반영되게 잠깐
                rclpy.spin_once(node, timeout_sec=0.1)
            ev = current['events']
            last = ev[-1][1] if ev else None
            phases = [m.phase for _, m in ev if m.state == 1]
            x, y, yaw = stations[dropoff]
            p = gt.get('p')
            row = {
                'n': n, 'task_id': task_id, 'pickup': pickup, 'dropoff': dropoff,
                'result': STATES.get(last.state, '') if last else 'NO_STATUS',
                'phase': last.phase if last else '', 'error_code': last.error.code if last else '',
                'error_message': (last.error.message if last else '').replace(',', ';'),
                'wall_s': round(time.monotonic() - t0, 1),
                'distance_m': round(last.distance_traveled, 2) if last else '',
                'phases': ' '.join(phases),
                'end_pos_err_cm': round(math.hypot(p.position.x - x, p.position.y - y) * 100, 2) if p else '',
                'end_yaw_err_deg': round(math.degrees(math.atan2(math.sin(yaw_of(p.orientation) - yaw),
                                                                 math.cos(yaw_of(p.orientation) - yaw))), 3) if p else '',
            }
            print(f'  → {row["result"]} {row["wall_s"]} s, 끝 자세 오차 {row["end_pos_err_cm"]} cm '
                  f'{row["end_yaw_err_deg"]}°', flush=True)
            if a.csv:
                new = not os.path.exists(a.csv)
                with open(a.csv, 'a', newline='', encoding='utf-8') as f:
                    w = csv.DictWriter(f, fieldnames=list(row))
                    if new:
                        w.writeheader()
                    w.writerow(row)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
