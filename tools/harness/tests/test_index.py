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
                   'b918867', 'gs://b/runs/x/', 'https://github.com/a/b/actions/runs/1']
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


def test_cli(tmp_path, capsys):
    out = tmp_path / 'results'
    make_run(out, 'fleet_fake-before_d100_20261010_00')
    (out / 'summary_fleet_fake-before_d100.md').write_text('## x\n')
    index = tmp_path / 'index.csv'
    assert main([str(out), '--index', str(index), '--path', 'gs://b/runs/y/']) == 0
    rows = list(csv.reader(index.open()))
    row = dict(zip(rows[0], rows[1]))
    assert [row[k] for k in ('scenario', 'label', 'delay_ms', 'loss_pct', 'repeats', 'completed', 'git_sha')] \
        == ['fleet_fake', 'before_d100', '100', '0', '1', '1', 'b918867']
    assert row['path'] == 'gs://b/runs/y/'
