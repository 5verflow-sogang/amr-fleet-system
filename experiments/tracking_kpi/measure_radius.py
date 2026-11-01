#!/usr/bin/env python3
"""녹화(rosbag)에서 작업자를 잡은 트랙의 반경을 잰다. 평가 결과(frames.csv)에는 짝지어진 트랙의 반경이 없어서 쓴다.

    python3 measure_radius.py <rosbag 폴더> <amr_msgs/msg 폴더> [GT id, 기본 worker_approach]

- 로봇이 서 있는 장면(C)만 맞다: world → odom을 /tf_static의 고정 TF(출발 자세, yaw 0)로 본다
- 같은 시각(±0.05 s) GT 위치에서 0.5 m 안의 가장 가까운 트랙을 작업자 트랙으로 본다
"""
import bisect
import math
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


def main(argv):
    bag, msg_dir = Path(argv[1]).expanduser(), Path(argv[2]).expanduser()
    gid = argv[3] if len(argv) > 3 else 'worker_approach'
    store = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    for name in ('TrackedObstacle', 'TrackedObstacleArray'):
        types.update(get_types_from_msg((msg_dir / f'{name}.msg').read_text(), f'amr_msgs/msg/{name}'))
    store.register(types)
    tracks, gt, offset, scan = [], [], None, None
    topics = ('/amr_01/perception/obstacles', f'/dynamic_obstacles/{gid}/ground_truth/odom',
              '/tf_static', '/amr_01/scan')
    with AnyReader([bag], default_typestore=store) as reader:
        for conn, _, raw in reader.messages(connections=[c for c in reader.connections if c.topic in topics]):
            m = reader.deserialize(raw, conn.msgtype)
            if conn.topic == '/amr_01/scan':
                scan = scan or (len(m.ranges), math.degrees(m.angle_increment))
            elif conn.topic == '/tf_static':
                for tf in m.transforms:
                    if tf.header.frame_id == 'world' and tf.child_frame_id == 'amr_01/odom':
                        offset = (tf.transform.translation.x, tf.transform.translation.y)
            elif conn.topic.endswith('ground_truth/odom'):
                p = m.pose.pose.position
                gt.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y))
            else:
                stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                obs = [(o.pose.pose.position.x, o.pose.pose.position.y, o.radius) for o in m.obstacles]
                tracks.append((stamp, obs))
    print(f'LiDAR 빔 {scan[0]}개, 간격 {scan[1]:.2f}° → 빔 사이 거리: 7 m {7 * math.radians(scan[1]):.3f} m, '
          f'10 m {10 * math.radians(scan[1]):.3f} m')
    times = [g[0] for g in gt]
    radius = []
    for stamp, obs in tracks:
        i = bisect.bisect_left(times, stamp)
        if i >= len(gt) or abs(gt[i][0] - stamp) > 0.05:
            continue
        gx, gy = gt[i][1] - offset[0], gt[i][2] - offset[1]
        near = [o for o in obs if math.hypot(o[0] - gx, o[1] - gy) < 0.5]
        if near:
            radius.append(min(near, key=lambda o: math.hypot(o[0] - gx, o[1] - gy))[2])
    r = np.array(radius)
    print(f'{gid}를 잡은 트랙 반경: 중앙 {np.median(r):.3f} m, 5% {np.percentile(r, 5):.3f}, 95% {np.percentile(r, 95):.3f} m '
          f'(n={len(r)})')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
