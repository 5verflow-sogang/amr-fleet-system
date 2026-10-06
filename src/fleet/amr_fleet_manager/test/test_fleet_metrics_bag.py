"""실제 rosbag2 파일을 써서 fleet_metrics 실행 파일이 읽고 변환하는지 확인한다. ROS 환경(CI)에서만 돈다."""

import csv
import json

import pytest

rosbag2_py = pytest.importorskip('rosbag2_py')

from amr_msgs.msg import FleetHeartbeat, TaskAssignment, TaskAssignmentAck  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.serialization import serialize_message  # noqa: E402

from amr_fleet_manager.fleet_metrics_cli import main  # noqa: E402

SEC = 1_000_000_000


def write_bag(path, messages):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    created = set()
    for topic, msg, t_ns in messages:
        if topic not in created:
            type_name = f'{type(msg).__module__.split(".")[0]}/msg/{type(msg).__name__}'
            writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=type_name, serialization_format='cdr'))
            created.add(topic)
        writer.write(topic, serialize_message(msg), t_ns)
    del writer  # 닫아야 metadata.yaml이 써진다


def test_bag_to_csv(tmp_path):
    hb = FleetHeartbeat(server_id='fleet_manager', epoch=1, seq=0)
    cmd = TaskAssignment(assignment_id=1, robot_id='amr_01', action=TaskAssignment.ACTION_ASSIGN)
    cmd.header.stamp = Time(sec=100)
    cmd.task.task_id = 'task_s0_0001'
    moving = Odometry()
    moving.header.stamp = Time(sec=100, nanosec=400_000_000)
    moving.twist.twist.linear.x = 0.3

    bag = tmp_path / 'rosbag'
    write_bag(bag, [
        ('/fleet/heartbeat', hb, 10 * SEC),
        ('/amr_01/fleet/heartbeat', hb, 10 * SEC + 150_000_000),
        ('/fleet/task_assignments', cmd, 11 * SEC),
        ('/amr_01/fleet/task_assignments', cmd, 11 * SEC + 160_000_000),
        ('/fleet/task_assignment_acks',
         TaskAssignmentAck(robot_id='amr_01', assignment_id=1, assignment_stamp=Time(sec=100)),
         11 * SEC + 200_000_000),
        ('/amr_01/odom', moving, 11 * SEC + 500_000_000),
        # 관측 마감을 넉넉히 뒤로 미는 마지막 기록
        ('/fleet/heartbeat', FleetHeartbeat(server_id='fleet_manager', epoch=1, seq=1), 30 * SEC),
    ])

    out = tmp_path / 'run'
    assert main([str(bag), '--out', str(out), '--delay-ms', '150']) == 0

    with (out / 'fleet_latency.csv').open(newline='') as f:
        latency = {(r['topic'], r['key']): r for r in csv.DictReader(f)}
    assert latency[('heartbeat', '1:0')]['latency_ms'] == '150.000'
    assert latency[('task_assignments', '1')]['latency_ms'] == '160.000'
    assert latency[('heartbeat', '1:1')]['delivered'] == ''

    with (out / 'response_time.csv').open(newline='') as f:
        [row] = list(csv.DictReader(f))
    assert (row['ack_rtt_ms'], row['latency_ms'], row['latency_sim_ms']) == ('200.000', '500.000', '400.000')
    assert row['task_id'] == 'task_s0_0001'

    assert 'observation_cutoff' in json.loads((out / 'meta.json').read_text())
