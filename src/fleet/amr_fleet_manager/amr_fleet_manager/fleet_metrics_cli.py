"""rosbag → Fleet 지표 CSV. 하네스가 실행마다 부른다.

    ros2 run amr_fleet_manager fleet_metrics <bag 폴더> --out <결과 폴더> --delay-ms 100

결과: <결과 폴더>/fleet_latency.csv, response_time.csv, task_outcomes.csv, robot_states.csv,
meta.json의 observation_cutoff.
<결과 폴더>/fault_events.csv에 로봇 강제종료가 있으면 recovery.csv도 쓴다.
로직은 fleet_metrics.py (ROS 없이 테스트), 여기는 bag 읽기만 한다.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from .fleet_metrics import (ACKS_TOPIC, ASSIGNMENTS_TOPIC, BIDS_TOPIC, MODES_TOPIC, ODOM_TOPIC, RELAY_TOPIC,
                            RELAYED_KEYS, REQUESTS_TOPIC, STATES_TOPIC, STATUS_TOPIC, ZONE_REQUESTS_TOPIC, Record,
                            convert, robot_ids)

ORIGINAL_TOPIC = re.compile(r'^/fleet/(' + '|'.join(RELAYED_KEYS) + r')$')


def wanted(topic: str) -> bool:
    return bool(topic in (ASSIGNMENTS_TOPIC, ACKS_TOPIC, REQUESTS_TOPIC, STATUS_TOPIC, STATES_TOPIC,
                          ZONE_REQUESTS_TOPIC, BIDS_TOPIC, MODES_TOPIC)
                or ORIGINAL_TOPIC.match(topic)
                or RELAY_TOPIC.match(topic) or ODOM_TOPIC.match(topic))


def read_bag(path: str) -> list[Record]:
    """bag에서 Fleet 지표에 필요한 토픽만 읽는다."""
    import rosbag2_py  # ROS 환경에서만 있다

    # Humble은 저장 방식을 직접 알려 줘야 해서 metadata.yaml에서 읽는다 (sqlite3 기본, mcap 선택)
    info = yaml.safe_load((Path(path) / 'metadata.yaml').read_text(encoding='utf-8'))
    storage_id = info['rosbag2_bagfile_information']['storage_identifier']

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage_id),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if wanted(t.name)}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))

    records = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        data = bytes(data)
        records.append(Record(topic, t_ns * 1e-9, data, deserialize_message(data, types[topic])))
    return records


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='rosbag → fleet_latency.csv, response_time.csv')
    parser.add_argument('bag', help='rosbag2 폴더 (metadata.yaml이 있는 곳)')
    parser.add_argument('--out', required=True, help='결과 폴더')
    parser.add_argument('--delay-ms', type=float, required=True, help='이 실행의 주입 지연 (관측 마감 계산)')
    parser.add_argument('--grace-s', type=float, default=1.0, help='관측 마감 여유 시간 (s)')
    parser.add_argument('--resend-grace-s', type=float, default=5.0,
                        help='Ack 관측 마감에 더할 시간 = 서버 resend.interval_s × resend.max_resends (s)')
    robots = parser.add_mutually_exclusive_group()
    robots.add_argument('--robots', nargs='*', help='로봇 목록. 주지 않으면 bag에 중계본이 있는 로봇 전부')
    # 목록을 정해 줘야 중계본을 하나도 못 받은 로봇이 손실 100%로 잡힌다 (bag에 없으면 빠진다)
    robots.add_argument('--num-robots', type=int, help='amr_01부터 이 수만큼을 로봇 목록으로 (--robots 대신)')
    parser.add_argument('--meta', help='observation_cutoff를 쓸 meta.json (기본: <결과 폴더>/meta.json)')
    args = parser.parse_args(argv)

    records = read_bag(args.bag)
    out = Path(args.out)
    robots = robot_ids(args.num_robots) if args.num_robots is not None else args.robots
    cutoff = convert(records, out, args.delay_ms, args.grace_s, args.resend_grace_s, robots,
                     Path(args.meta) if args.meta else None)
    print(f'{len(records)}건 → {out}/fleet_latency.csv, response_time.csv '
          f'(cutoff={cutoff.latency:.3f}, ack_cutoff={cutoff.ack:.3f})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
