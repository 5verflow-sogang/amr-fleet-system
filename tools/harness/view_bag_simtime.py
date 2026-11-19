#!/usr/bin/env python3
"""하네스 view_bag(수신 시각 = 실제 시간)을 시뮬레이션 시각 bag으로 바꾼다 (rviz_video.sh가 쓴다).

녹화 bag의 메시지 시각은 실제 시간이라 RTF(클라우드 약 0.1)만큼 느리고, RTF가 실행 중에 바뀌어 일정한 배속으로 틀면
시뮬레이션 속도가 들쭉날쭉하다. bag 안의 /clock(시뮬레이션 시계)으로 수신 시각을 시뮬레이션 시각으로 바꿔 새 bag에
쓰고, /clock·/clock_raw는 뺀다 - 새 bag을 ros2 bag play --clock으로 1배속 틀면 시뮬레이션 1 s가 화면 1 s다.

    python3 view_bag_simtime.py <view_bag> <출력 bag> [--start t0|<초>] [--pre <초>]

--start t0(기본)이면 /demo/t0 시각부터(--pre만큼 앞에서) 쓴다. 그 전 메시지 중 계속 보여야 하는 것(지도·고정 TF·costmap·
혼잡 마스크·전역 경로 - 마지막 것, /tf_static은 전부)은 시작 시각으로 옮겨 넣어 첫 화면부터 보이게 한다.
시작 시각(시뮬레이션 s)을 표준 출력 마지막 줄에 낸다.
"""

import argparse
import bisect
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header

CLOCKS = ('/clock', '/clock_raw')
# 시작 전 마지막 메시지를 시작 시각으로 옮겨 넣는 토픽 (latched·드물게 오는 것). /tf_static은 전부 옮긴다
CARRY_LAST = ('/map', '/fleet/congestion_mask', '/global_costmap/costmap', '/plan', '/fleet/congestion')


def _reader(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    return r


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--start', default='t0', help='t0 또는 시뮬레이션 시각 [s]')
    ap.add_argument('--pre', type=float, default=0.0, help='시작 시각보다 이만큼 앞부터 [s]')
    a = ap.parse_args(argv)

    # 1) /clock: 수신 시각 → 시뮬레이션 시각, /demo/t0
    clk, t0 = [], None
    r = _reader(a.bag)
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == '/clock':
            c = deserialize_message(data, Clock).clock
            clk.append((t, c.sec * 1_000_000_000 + c.nanosec))
        elif topic == '/demo/t0' and t0 is None:
            s = deserialize_message(data, Header).stamp
            t0 = s.sec * 1_000_000_000 + s.nanosec
    if len(clk) < 2:
        sys.exit('/clock이 bag에 없다')
    ct = [c[0] for c in clk]

    def sim(t):
        i = min(max(bisect.bisect_left(ct, t), 1), len(ct) - 1)
        (x0, s0), (x1, s1) = clk[i - 1], clk[i]
        return int(s0 + (s1 - s0) * (t - x0) / max(x1 - x0, 1))

    if a.start == 't0':
        if t0 is None:
            sys.exit('/demo/t0이 bag에 없다 (--start에 시각을 준다)')
        start = t0
    else:
        start = int(float(a.start) * 1e9)
    start -= int(a.pre * 1e9)

    # 2) 새 bag
    r = _reader(a.bag)
    topics = [t for t in r.get_all_topics_and_types() if t.name not in CLOCKS]
    w = rosbag2_py.SequentialWriter()
    if os.path.exists(a.out):
        sys.exit(f'출력이 이미 있다: {a.out}')
    w.open(rosbag2_py.StorageOptions(uri=a.out, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    for t in topics:
        w.create_topic(rosbag2_py.TopicMetadata(name=t.name, type=t.type, serialization_format=t.serialization_format,
                                                offered_qos_profiles=t.offered_qos_profiles))
    carry, static = {}, []
    written = 0
    while r.has_next():
        topic, data, t = r.read_next()
        if topic in CLOCKS:
            continue
        s = sim(t)
        if s < start:
            if topic == '/tf_static':
                static.append(data)
            elif topic.endswith(CARRY_LAST):
                carry[topic] = data
            continue
        if static or carry:
            for d in static:
                w.write('/tf_static', d, start)
            for tp, d in carry.items():
                w.write(tp, d, start)
            written += len(static) + len(carry)
            static, carry = [], {}
        w.write(topic, data, max(s, start))
        written += 1
    del w
    print(f'{written} 메시지, 시작 시뮬레이션 {start / 1e9:.3f} s (t0 {t0 / 1e9 if t0 else float("nan"):.3f} s)')
    print(f'{start / 1e9:.3f}')


if __name__ == '__main__':
    main()
