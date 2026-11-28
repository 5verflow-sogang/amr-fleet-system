#!/usr/bin/env python3
"""zone_pass record:=true rosbag → CSV (animate.py 입력). ROS가 있는 컨테이너에서 돌린다 (amr_msgs 빌드 필요).

    python3 experiments/zone_capacity/bag_to_csv.py <run_dir>

<run_dir>/video/에 robot.csv(t,x,y,yaw,v), guests.csv(t,id,x,y), safety.csv(t,level,dist,detail), plan.csv(t,i,x,y)를 쓴다.
시각은 메시지 header.stamp (시뮬레이션 s)라 nav_kpi/gt_track.csv와 같은 축이다.
"""

import csv
import math
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def main():
    run = Path(sys.argv[1])
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run / 'rosbag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    out = run / 'video'
    out.mkdir(exist_ok=True)
    files = {k: (out / f'{k}.csv').open('w', newline='') for k in ('robot', 'guests', 'safety', 'plan')}
    w = {k: csv.writer(f) for k, f in files.items()}
    w['robot'].writerow(['t', 'x', 'y', 'yaw', 'v'])
    w['guests'].writerow(['t', 'id', 'x', 'y'])
    w['safety'].writerow(['t', 'level', 'dist', 'detail'])
    w['plan'].writerow(['t', 'i', 'x', 'y'])
    last_robot = -1.0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, types[topic])
        if topic == '/amr_01/ground_truth/odom':
            t = stamp(msg.header)
            if t - last_robot < 0.05:   # 20 Hz면 충분하다
                continue
            last_robot = t
            p, v = msg.pose.pose, msg.twist.twist.linear
            w['robot'].writerow([f'{t:.3f}', f'{p.position.x:.3f}', f'{p.position.y:.3f}',
                                 f'{yaw(p.orientation):.3f}', f'{math.hypot(v.x, v.y):.3f}'])
        elif topic.startswith('/dynamic_obstacles/'):
            p = msg.pose.pose.position
            w['guests'].writerow([f'{stamp(msg.header):.3f}', topic.split('/')[2], f'{p.x:.3f}', f'{p.y:.3f}'])
        elif topic == '/amr_01/safety/status':
            w['safety'].writerow([f'{stamp(msg.header):.3f}', msg.level, f'{msg.min_obstacle_distance:.2f}', msg.detail])
        elif topic == '/amr_01/plan':
            t = stamp(msg.header)
            for i, ps in enumerate(msg.poses[::5]):
                w['plan'].writerow([f'{t:.3f}', i, f'{ps.pose.position.x:.3f}', f'{ps.pose.position.y:.3f}'])
    for f in files.values():
        f.close()
    print(f'썼다: {out}')


if __name__ == '__main__':
    main()
