"""GCP 실행 줄 세우기(gcp_batch.py)의 판단 부분 (ROS·gh 없이)."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from harness.batch import (append_log, free_slots, parse_conditions, parse_slots, pending,  # noqa: E402
                           read_log, running, with_ref)

LIST = """
# 중앙집중 지연 스윕
fleet_fake -n 10 --set delay_ms=0   --label central_d0
fleet_fake -n 10 --set delay_ms=200 --label central_d200   # 주석
fleet_fake -n 10 --set delay_ms=500 --label central_d500 --pr 171
"""


def test_parse_conditions():
    conds = parse_conditions(LIST)
    assert [c.key for c in conds] == [
        'fleet_fake -n 10 --set delay_ms=0 --label central_d0',
        'fleet_fake -n 10 --set delay_ms=200 --label central_d200',
        'fleet_fake -n 10 --set delay_ms=500 --label central_d500 --pr 171',
    ]
    assert conds[1].args[:2] == ('fleet_fake', '-n')


@pytest.mark.parametrize('text', [
    'fleet_fake -n 10',                                         # 라벨 없음
    '-n 10 --label x',                                          # 시나리오 없음
    'fleet_fake --label x --slot b1',                           # 슬롯은 디스패처가 정한다
    'fleet_fake --label x --watch',
    'fleet_fake --label x\nfleet_fake --label x',               # 같은 조건 두 번
])
def test_parse_conditions_rejects(text):
    with pytest.raises(ValueError):
        parse_conditions(text)


def test_with_ref():
    args = ('fleet_fake', '--label', 'x')
    assert with_ref(args, 'dev', None)[-2:] == ('--ref', 'dev')
    assert with_ref(args, 'dev', '171')[-2:] == ('--pr', '171')
    assert with_ref(args + ('--ref', 'main'), 'dev', None) == args + ('--ref', 'main')
    with pytest.raises(ValueError):
        with_ref(args, None, None)


def test_parse_slots():
    assert parse_slots('b1, b2,b3') == ['b1', 'b2', 'b3']
    for bad in ('', 'b5', 'a,b1', 'b1,b1'):
        with pytest.raises(ValueError):
            parse_slots(bad)


def test_pending_and_running(tmp_path):
    conds = parse_conditions(LIST)
    log_path = tmp_path / 'log.csv'
    k0, k1, k2 = (c.key for c in conds)
    # 0: 성공, 1: 돌고 있음, 2: 아직 안 넣음
    append_log(log_path, {'key': k0, 'slot': 'b1', 'run_id': '1', 'status': 'dispatched'})
    append_log(log_path, {'key': k0, 'slot': 'b1', 'run_id': '1', 'status': 'completed', 'conclusion': 'success'})
    append_log(log_path, {'key': k1, 'slot': 'b2', 'run_id': '2', 'status': 'dispatched'})
    log = read_log(log_path)
    assert [c.key for c in pending(conds, log, retries=1)] == [k2]
    assert set(running(log)) == {'b2'}
    assert free_slots(['b1', 'b2', 'b3'], running(log)) == ['b1', 'b3']


def test_failed_condition_retried_after_fresh_ones_then_given_up(tmp_path):
    conds = parse_conditions(LIST)
    log_path = tmp_path / 'log.csv'
    k0 = conds[0].key
    append_log(log_path, {'key': k0, 'slot': 'b1', 'run_id': '1', 'status': 'dispatched'})
    append_log(log_path, {'key': k0, 'slot': 'b1', 'run_id': '1', 'status': 'completed', 'conclusion': 'failure'})
    # 실패 1회: 재시도 1번 남음, 처음 넣는 조건 뒤로
    assert [c.key for c in pending(conds, read_log(log_path), retries=1)] == [c.key for c in conds[1:]] + [k0]
    append_log(log_path, {'key': k0, 'slot': 'b1', 'run_id': '3', 'status': 'dispatched'})
    append_log(log_path, {'key': k0, 'slot': 'b1', 'run_id': '3', 'status': 'completed', 'conclusion': 'cancelled'})
    # 2회 실패: 더 넣지 않는다
    assert k0 not in [c.key for c in pending(conds, read_log(log_path), retries=1)]


def test_dispatch_failure_counts_as_attempt(tmp_path):
    """gcp_run.sh가 넣지 못한 것(입력 오류 등)도 시도로 세서 끝없이 반복하지 않는다."""
    conds = parse_conditions(LIST)
    log_path = tmp_path / 'log.csv'
    k0 = conds[0].key
    for _ in range(2):
        append_log(log_path, {'key': k0, 'slot': 'b1', 'status': 'dispatched'})
        append_log(log_path, {'key': k0, 'slot': 'b1', 'status': 'completed', 'conclusion': 'dispatch_failed'})
    log = read_log(log_path)
    assert k0 not in [c.key for c in pending(conds, log, retries=1)]
    assert running(log) == {}
