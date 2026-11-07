#!/usr/bin/env python3
"""측정 bag(rosbag/) → 대시보드에 넣을 메시지 목록 (dashboard.jsonl). ROS가 있는 곳에서 돈다.

    python3 tools/demo/bag_export.py <실행 폴더>/rosbag --out <실행 폴더>/dashboard.jsonl
    (하네스 demo_restaurant 후처리가 부른다. 녹화는 ROS 없이 tools/demo/record_dashboard.py)

첫 줄: {"t0": /demo/t0 시뮬레이션 시각 또는 null, "scene": 장면 이름}
다음 줄마다: {"t": 시뮬레이션 시각(s), "topic": 토픽, "msg": rosbridge와 같은 JSON}
bag의 시각은 받은 실제 시각이라(RTF 0.07이면 14배 느리다), 직전 /clock 값을 메시지의 시뮬레이션 시각으로 쓴다.
대시보드가 rosbridge에 거는 throttle_rate(dashboard/index.html SUBS)와 같게 토픽마다 간격을 줄인다.
"""

import argparse
import array
import base64
import json
import math
import re

# 토픽 → 최소 간격 (s). dashboard/index.html SUBS의 throttle_rate와 같다
THROTTLE = [
    (r'^/fleet/robot_states$', 0.0), (r'^/fleet/task_table$', 0.5), (r'^/fleet/task_status$', 0.0),
    (r'^/fleet/congestion$', 0.5), (r'^/fleet/assignment_decisions$', 0.0), (r'^/clock$', 0.5),
    (r'^/amr_\d+/safety/status$', 0.2), (r'^/amr_\d+/plan$', 1.0),
]


def throttle_of(topic):
    for pattern, gap in THROTTLE:
        if re.match(pattern, topic):
            return gap
    return None                                   # 대시보드가 안 받는 토픽


def plain(value):
    """message_to_ordereddict 결과 → JSON 값. uint8[]는 rosbridge처럼 base64, NaN·inf는 null."""
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (bytes, bytearray)):
        return base64.b64encode(bytes(value)).decode()
    if isinstance(value, array.array):
        return base64.b64encode(value.tobytes()).decode() if value.typecode == 'B' else [plain(v) for v in value]
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if hasattr(value, 'tolist'):                  # numpy 배열
        return plain(value.tolist())
    return value


def export(bag, out, storage='sqlite3'):
    # ROS는 여기서만 부른다 (plain·throttle_of는 ROS 없이 시험한다)
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    keep = {t: throttle_of(t) for t in types if throttle_of(t) is not None or t == '/demo/t0'}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(keep)))
    classes = {t: get_message(types[t]) for t in keep}
    sim, last, lines, head, counts = None, {}, [], {'t0': None, 'scene': ''}, {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, classes[topic])
        if topic == '/clock':
            sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        if topic == '/demo/t0':
            head = {'t0': msg.stamp.sec + msg.stamp.nanosec * 1e-9, 'scene': msg.frame_id}
            continue
        if sim is None or sim - last.get(topic, -math.inf) < keep[topic]:
            continue
        last[topic] = sim
        counts[topic] = counts.get(topic, 0) + 1
        lines.append({'t': round(sim, 3), 'topic': topic, 'msg': plain(message_to_ordereddict(msg))})
    with open(out, 'w', encoding='utf-8') as f:
        f.write(json.dumps(head, ensure_ascii=False) + '\n')
        for line in lines:
            f.write(json.dumps(line, ensure_ascii=False, separators=(',', ':')) + '\n')
    return head, counts, (lines[0]['t'], lines[-1]['t']) if lines else (None, None)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag', help='rosbag 폴더')
    ap.add_argument('--out', required=True, help='jsonl')
    ap.add_argument('--storage', default='sqlite3')
    args = ap.parse_args(argv)
    head, counts, (first, last) = export(args.bag, args.out, args.storage)
    print(f'{args.out}: 시뮬레이션 {first} ~ {last} s, 0초 {head["t0"]}, 메시지 {sum(counts.values())}개')
    for topic, n in sorted(counts.items()):
        print(f'  {topic}: {n}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
