"""실행 목록(index.csv) 한 줄 만들기 (ROS 없이)."""

import csv
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from harness.index import INDEX_HEADER, append, index_row  # noqa: E402
from index_run import main  # noqa: E402


def make_run(root: Path, name: str, outcome='completed', **fault):
    run = root / name
    run.mkdir(parents=True)
    (run / 'meta.json').write_text(json.dumps({
        'scenario': 'fleet_fake', 'label': 'before_d100', 'outcome': outcome, 'git_sha': 'b918867',
        'fault': {'delay_ms': 100.0, 'loss_pct': 0.0, **fault},
    }))
    return run


def test_index_row(tmp_path):
    runs = [make_run(tmp_path, f'r{i}') for i in range(3)] + [make_run(tmp_path, 'r3', outcome='timeout')]
    row = index_row(runs, 'gs://b/runs/x/', 'https://github.com/a/b/actions/runs/1', date='2026-10-10T15:00:00Z')
    assert row == ['2026-10-10T15:00:00Z', 'fleet_fake', 'before_d100', '100', '0', '', '', '', '4', '3',
                   'b918867', 'gs://b/runs/x/', 'https://github.com/a/b/actions/runs/1', '']
    with pytest.raises(ValueError):
        index_row([tmp_path / 'nothing'], 'gs://b/')


def test_index_row_fault_conditions(tmp_path):
    run = make_run(tmp_path, 'r0', kill_robot='amr_03', kill_robot_at_s=120.0, relay_auction=True)
    row = dict(zip(INDEX_HEADER, index_row([run], 'gs://b/')))
    assert (row['kill_robot'], row['kill_robot_at_s'], row['relay_auction']) == ('amr_03', '120', 'true')


def test_append_writes_header_once(tmp_path):
    index = tmp_path / 'index.csv'
    append(index, ['a'] * len(INDEX_HEADER))
    append(index, ['b'] * len(INDEX_HEADER))
    rows = list(csv.reader(index.open()))
    assert rows[0] == INDEX_HEADER and [r[0] for r in rows[1:]] == ['a', 'b']


def test_append_migrates_old_header(tmp_path):
    """machine_type 열이 없던 예전 index.csv에 더하면 머리줄을 바꾸고 예전 줄은 빈칸으로 채운다."""
    index = tmp_path / 'index.csv'
    old_header = INDEX_HEADER[:-1]
    with index.open('w', newline='') as f:
        csv.writer(f).writerows([old_header, ['old'] * len(old_header)])
    append(index, ['new'] * len(INDEX_HEADER))
    rows = list(csv.reader(index.open()))
    assert rows[0] == INDEX_HEADER
    assert rows[1] == ['old'] * len(old_header) + ['']
    assert rows[2] == ['new'] * len(INDEX_HEADER)


def test_append_fixes_mixed_rows_under_stale_header(tmp_path):
    """10/16 버킷 모양: 머리줄 10열(장애 열 없음) 아래 10열 줄과 13열 줄이 섞여 있다."""
    index = tmp_path / 'index.csv'
    stale = ['date', 'scenario', 'label', 'delay_ms', 'loss_pct', 'repeats', 'completed', 'git_sha', 'path',
             'actions_url']
    old_row = ['2026-10-10', 'fleet_fake', 'before', '0', '0', '10', '10', 'abc', 'gs://x/', 'u1']
    mid_row = ['2026-10-15', 'fleet_deadlock', 'revoke', '100', '0', 'none', '-1', '', '1', '1', 'def',
               'gs://y/', 'u2']
    with index.open('w', newline='') as f:
        csv.writer(f).writerows([stale, old_row, mid_row])
    append(index, ['n'] * len(INDEX_HEADER))
    rows = [dict(zip(INDEX_HEADER, r)) for r in list(csv.reader(index.open()))[1:]]
    assert list(csv.reader(index.open()))[0] == INDEX_HEADER
    assert (rows[0]['repeats'], rows[0]['git_sha'], rows[0]['kill_robot'], rows[0]['machine_type']) \
        == ('10', 'abc', '', '')
    assert (rows[1]['kill_robot'], rows[1]['repeats'], rows[1]['actions_url'], rows[1]['machine_type']) \
        == ('none', '1', 'u2', '')
    assert all(len(r) == len(INDEX_HEADER) for r in csv.reader(index.open()))


def test_append_rejects_unknown_header(tmp_path):
    index = tmp_path / 'index.csv'
    index.write_text('date,something_else\n')
    with pytest.raises(ValueError):
        append(index, ['x'] * len(INDEX_HEADER))


def test_cli(tmp_path, capsys):
    out = tmp_path / 'results'
    make_run(out, 'fleet_fake-before_d100_20261010_00')
    (out / 'summary_fleet_fake-before_d100.md').write_text('## x\n')
    index = tmp_path / 'index.csv'
    assert main([str(out), '--index', str(index), '--path', 'gs://b/runs/y/', '--machine', 'c2d-standard-8']) == 0
    rows = list(csv.reader(index.open()))
    row = dict(zip(rows[0], rows[1]))
    assert [row[k] for k in ('scenario', 'label', 'delay_ms', 'loss_pct', 'repeats', 'completed', 'git_sha')] \
        == ['fleet_fake', 'before_d100', '100', '0', '1', '1', 'b918867']
    assert row['path'] == 'gs://b/runs/y/'
    assert row['machine_type'] == 'c2d-standard-8'
