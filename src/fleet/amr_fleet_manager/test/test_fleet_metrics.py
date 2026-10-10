"""rosbag 기록 → Fleet 지표 변환 로직. rclpy 없이 메시지 대신 SimpleNamespace로 돈다."""

import csv
import json
from types import SimpleNamespace as NS

import pytest

from amr_fleet_manager.fleet_metrics import (FLEET_LATENCY_HEADER, RESPONSE_TIME_HEADER, Cutoff, Record, convert,
                                             fleet_latency, observation_start, response_time)


def stamp(sec, nanosec=0):
    return NS(sec=sec, nanosec=nanosec)


def heartbeat(seq, epoch=1):
    return NS(epoch=epoch, seq=seq)


def assignment(assignment_id, robot, sec, task_id='t1', action=0):
    return NS(assignment_id=assignment_id, robot_id=robot, action=action, header=NS(stamp=stamp(sec)),
              task=NS(task_id=task_id))


def ack(assignment_id, robot, sec):
    return NS(assignment_id=assignment_id, robot_id=robot, assignment_stamp=stamp(sec))


def odom(vx, sec, wz=0.0):
    return NS(header=NS(stamp=stamp(sec)), twist=NS(twist=NS(linear=NS(x=vx, y=0.0), angular=NS(z=wz))))


def rec(topic, t, msg, data=None):
    # 같은 메시지면 같은 바이트 (원본·중계본 짝짓기)
    return Record(topic, t, data if data is not None else repr(vars(msg)).encode(), msg)


CUTOFF = Cutoff(end=100.0, delay_ms=200.0, grace_s=1.0, resend_grace_s=5.0)


def test_cutoff():
    assert CUTOFF.latency == pytest.approx(98.8)
    assert CUTOFF.ack == pytest.approx(93.8)
    assert CUTOFF.as_meta()['ack_cutoff'] == pytest.approx(93.8)


def test_fleet_latency_delivered_lost_and_cutoff():
    hb1, hb2, hb3 = heartbeat(0), heartbeat(1), heartbeat(2)
    records = [
        rec('/fleet/heartbeat', 10.0, hb1),
        rec('/amr_01/fleet/heartbeat', 10.2, hb1),
        rec('/amr_02/fleet/heartbeat', 10.25, hb1),
        rec('/fleet/heartbeat', 20.0, hb2),
        rec('/amr_01/fleet/heartbeat', 20.2, hb2),       # amr_02는 손실
        rec('/fleet/heartbeat', 99.0, hb3),               # 관측 마감(98.8) 뒤
    ]
    rows = fleet_latency(records, CUTOFF)
    table = {(r[1], r[2]): r for r in rows}
    assert len(rows) == 6
    assert table[('amr_01', '1:0')][4:] == ['10.200000', '200.000', '1']
    assert table[('amr_02', '1:1')][4:] == ['', '', '0']
    assert table[('amr_01', '1:2')][6] == ''               # 판정 안 함
    assert all(r[0] == 'heartbeat' for r in rows)


def test_fleet_latency_matches_by_bytes_not_key():
    # 로봇마다 request_id가 겹쳐도 바이트가 다르면 다른 메시지다
    a = NS(request_id=1, robot_id='amr_01')
    b = NS(request_id=1, robot_id='amr_02')
    records = [
        rec('/fleet/zone_grants', 1.0, a),
        rec('/fleet/zone_grants', 2.0, b),
        rec('/amr_01/fleet/zone_grants', 2.5, b),         # b가 먼저 도착
        rec('/amr_01/fleet/zone_grants', 3.0, a),
    ]
    rows = fleet_latency(records, CUTOFF, robots=['amr_01'])
    assert [(r[2], r[5]) for r in rows] == [('1', '2000.000'), ('1', '500.000')]


def test_fleet_latency_resend_uses_first():
    a = assignment(7, 'amr_01', 5)
    records = [
        rec('/fleet/task_assignments', 5.0, a),
        rec('/fleet/task_assignments', 6.0, a),          # 재전송: 같은 바이트
        rec('/amr_01/fleet/task_assignments', 6.1, a),
        rec('/amr_01/fleet/task_assignments', 6.3, a),
    ]
    [row] = fleet_latency(records, CUTOFF)
    assert row[:4] == ['task_assignments', 'amr_01', '7', '5.000000']
    assert row[5] == '1100.000'


def test_response_time_with_ack_resend_and_odom():
    a = assignment(3, 'amr_01', 50, task_id='task_s0_0001')
    records = [
        rec('/amr_01/odom', 9.0, odom(0.5, 49)),           # 명령 전 움직임은 무시
        rec('/fleet/task_assignments', 10.0, a),
        rec('/fleet/task_assignments', 11.0, a),
        rec('/amr_01/odom', 10.1, odom(0.0, 50)),
        rec('/amr_01/odom', 10.3, odom(0.0, 50, wz=0.1)),  # 제자리 회전도 응답
        rec('/fleet/task_assignment_acks', 11.2, ack(3, 'amr_01', 50)),
    ]
    [row] = response_time(records, CUTOFF)
    assert dict(zip(RESPONSE_TIME_HEADER, row)) == {
        'cmd_time': '10.000000', 'response_time': '10.300000', 'latency_ms': '300.000', 'robot': 'amr_01',
        'task_id': 'task_s0_0001', 'latency_sim_ms': '0.000', 'assignment_id': 3,
        'ack_time': '11.200000', 'ack_rtt_ms': '1200.000', 'resends': 1}


def test_response_time_separates_epochs_and_skips():
    records = [
        # 서버 재시작 전후로 assignment_id 1이 두 번: stamp로 가른다
        rec('/fleet/task_assignments', 10.0, assignment(1, 'amr_01', 10)),
        rec('/fleet/task_assignments', 30.0, assignment(1, 'amr_01', 30)),
        rec('/fleet/task_assignment_acks', 30.1, ack(1, 'amr_01', 30)),
        # 취소는 응답 시간 대상이 아니다
        rec('/fleet/task_assignments', 40.0, assignment(2, 'amr_01', 40, action=1)),
        # Ack 관측 마감(93.8) 뒤 명령은 행을 만들지 않는다
        rec('/fleet/task_assignments', 95.0, assignment(3, 'amr_01', 95)),
    ]
    rows = response_time(records, CUTOFF)
    assert [(r[0], r[7], r[9]) for r in rows] == [('10.000000', '', 0), ('30.000000', '30.100000', 0)]
    # odom이 없으면 응답 시각은 비운다 (가짜 로봇)
    assert rows[0][1] == '' and rows[0][2] == ''


def test_convert_writes_files_and_meta(tmp_path):
    hb = heartbeat(0)
    records = [rec('/fleet/heartbeat', 1.0, hb), rec('/amr_01/fleet/heartbeat', 1.1, hb),
               rec('/fleet/task_assignments', 2.0, assignment(1, 'amr_01', 2)), rec('/fleet/heartbeat', 20.0, heartbeat(1))]
    (tmp_path / 'meta.json').write_text(json.dumps({'scenario': 'smoke'}))
    convert(records, tmp_path, delay_ms=100.0)

    with (tmp_path / 'fleet_latency.csv').open(newline='') as f:
        assert next(csv.reader(f)) == FLEET_LATENCY_HEADER
    with (tmp_path / 'response_time.csv').open(newline='') as f:
        assert len(list(csv.reader(f))) == 2
    meta = json.loads((tmp_path / 'meta.json').read_text())
    assert meta['scenario'] == 'smoke'
    assert meta['observation_cutoff']['cutoff'] == pytest.approx(20.0 - 0.1 - 1.0)
    assert meta['observation_cutoff']['start'] == pytest.approx(1.1 - 0.1)

    with pytest.raises(ValueError):
        convert([], tmp_path, delay_ms=0.0)


def test_warmup_before_relay_subscribed_not_judged():
    # 녹화기가 원본(/fleet)을 먼저 구독하고 amr_02 중계본은 11초 무렵부터 녹화했다 (10/10 GCP 첫 하네스 실행)
    hb = [heartbeat(i) for i in range(5)]
    records = [
        rec('/fleet/heartbeat', 10.0, hb[0]), rec('/amr_01/fleet/heartbeat', 10.21, hb[0]),
        rec('/fleet/heartbeat', 10.6, hb[1]), rec('/amr_01/fleet/heartbeat', 10.81, hb[1]),
        rec('/fleet/heartbeat', 11.2, hb[2]), rec('/amr_01/fleet/heartbeat', 11.41, hb[2]),
        rec('/amr_02/fleet/heartbeat', 11.41, hb[2]),
        rec('/fleet/heartbeat', 20.0, hb[3]), rec('/amr_01/fleet/heartbeat', 20.21, hb[3]),
        rec('/amr_02/fleet/heartbeat', 20.21, hb[3]),
        rec('/fleet/heartbeat', 30.0, hb[4]), rec('/amr_01/fleet/heartbeat', 30.21, hb[4]),  # amr_02 진짜 손실
    ]
    start = observation_start(records, delay_ms=200.0)
    assert start == pytest.approx(11.21)
    cutoff = Cutoff(end=100.0, delay_ms=200.0, grace_s=1.0, resend_grace_s=5.0, start=start)
    table = {(r[1], r[2]): r[6] for r in fleet_latency(records, cutoff)}
    # 관측 시작 전에 보낸 세 건은 두 로봇 모두 판정하지 않는다 (amr_02가 구독 전이라 손실로 세면 안 된다).
    # 가장 늦게 구독된 링크의 첫 메시지도 빠진다 (실제 지연이 주입값보다 길어 보수적으로 자른다)
    keys = ('1:0', '1:1', '1:2', '1:3', '1:4')
    assert [table[('amr_01', k)] for k in keys] == ['', '', '', '1', '1']
    assert [table[('amr_02', k)] for k in keys] == ['', '', '', '1', '0']
    assert observation_start([rec('/fleet/heartbeat', 1.0, hb[0])], 0.0) is None
    # 드문 토픽(할당 명령)의 첫 중계본은 구독 시각이 아니므로 관측 시작에 쓰지 않는다
    late = assignment(1, 'amr_01', 50)
    assert observation_start(records + [rec('/fleet/task_assignments', 50.0, late),
                                        rec('/amr_01/fleet/task_assignments', 50.2, late)], 200.0) == pytest.approx(11.21)
