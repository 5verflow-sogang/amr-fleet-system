"""반복 사이 DDS 격리 (ROS 없이). 도메인 번호 범위, 실행 환경·meta 기록, 나눠 부른 반복의 요약 묶기를 본다."""

import json
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import run  # noqa: E402
from harness import runner  # noqa: E402
from harness.runner import DOMAIN_COUNT, DOMAIN_MIN, dds_domain, leftover_processes, run_once  # noqa: E402


def test_domain_stays_in_safe_range_and_never_zero():
    ids = [dds_domain(k) for k in range(250)]
    assert min(ids) == DOMAIN_MIN == 1
    assert max(ids) == DOMAIN_MIN + DOMAIN_COUNT - 1 <= 101
    # 연속한 시도(재시도 포함)는 서로 다른 도메인
    assert all(a != b for a, b in zip(ids, ids[1:]))


def bash_scenario(cmd: str) -> dict:
    return {'name': 'iso', 'processes': [{'name': 'p', 'cmd': cmd}], 'driver': {}}


def test_run_once_sets_domain_for_children_and_records_it(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    for name in ('IGN_PARTITION', 'GZ_PARTITION'):  # 테스트가 끝나면 원래대로 돌려놓게 등록
        monkeypatch.delenv(name, raising=False)
    resets = []
    monkeypatch.setattr(runner, 'dds_reset', lambda: resets.append(True))
    # 프로세스가 자기 환경을 적고 바로 끝난다 → 기동 실패로 처리되지만 meta.json은 남는다
    out = tmp_path / 'env.txt'
    scenario = bash_scenario(f'echo "$ROS_DOMAIN_ID $IGN_PARTITION" > {out}')
    meta = run_once(scenario, {'repeat': 0, 'seed': 1}, tmp_path / 'r0', domain_id=7)
    assert out.read_text().split() == ['7', 'harness_7']
    assert meta['dds'] == {'ros_domain_id': 7, 'reset': True}
    assert json.loads((tmp_path / 'r0' / 'meta.json').read_text())['dds']['ros_domain_id'] == 7
    assert resets == [True]


def test_run_once_without_isolation_keeps_shell_domain(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '42')
    monkeypatch.setattr(runner, 'dds_reset', lambda: (_ for _ in ()).throw(AssertionError('reset 호출됨')))
    meta = run_once(bash_scenario('true'), {'repeat': 0, 'seed': 1}, tmp_path / 'r0', reset_dds=False)
    assert meta['dds'] == {'ros_domain_id': 42, 'reset': False}
    assert os.environ['ROS_DOMAIN_ID'] == '42'


def test_split_calls_with_same_stamp_summarize_together(tmp_path, monkeypatch):
    """새 컨테이너 모드: -n 1 --repeat-offset i --stamp S를 여러 번 부르면 마지막 요약이 모든 반복을 묶는다."""
    calls = []

    def fake_run_once(scenario, params, run_dir, overrides, domain_id=None, reset_dds=True):
        calls.append((params['repeat'], params['seed'], domain_id))
        run_dir.mkdir(parents=True)
        meta = {'scenario': scenario['name'], 'label': '', 'outcome': 'completed', 'duration_wall_s': 1.0}
        (run_dir / 'meta.json').write_text(json.dumps(meta))
        return meta

    seen = []
    monkeypatch.setattr(run, 'run_once', fake_run_once)
    monkeypatch.setattr(run, 'dds_reset', lambda: None)
    monkeypatch.setattr(run, 'notify', lambda summary, run_dirs, path: seen.append([d.name for d in run_dirs]))
    (tmp_path / 'loc_basic_S_00_fail1').mkdir()  # 기동 실패 시도 폴더는 요약에서 빠져야 한다
    for i in range(3):
        run.main(['loc_basic', '-n', '1', '--repeat-offset', str(i), '--stamp', 'S', '--out', str(tmp_path)])
    assert [c[:2] for c in calls] == [(0, 1), (1, 2), (2, 3)]
    assert len({c[2] for c in calls}) == 3
    assert seen[-1] == ['loc_basic_S_00', 'loc_basic_S_01', 'loc_basic_S_02']


def test_leftover_processes_finds_surviving_simulator():
    # 정리를 빠져나간 시뮬레이터 흉내: 명령줄이 "ign gazebo ..."인 프로세스
    p = subprocess.Popen(['bash', '-c', 'exec -a "ign gazebo -s harness_test" sleep 30'])
    try:
        for _ in range(50):
            found = [a for a in leftover_processes() if 'harness_test' in a]
            if found:
                break
        assert found
    finally:
        p.kill()
        p.wait()
    assert not [a for a in leftover_processes() if 'harness_test' in a]


def test_run_once_records_ready_time(tmp_path, monkeypatch):
    monkeypatch.setattr(runner, 'dds_reset', lambda: None)
    scenario = {'name': 'iso', 'processes': [{'name': 'sim', 'cmd': 'sleep 5', 'settle_s': 0.2},
                                             {'name': 'quick', 'cmd': 'true'}], 'driver': {}}
    meta = run_once(scenario, {'repeat': 0, 'seed': 1}, tmp_path / 'r0', reset_dds=False)
    assert list(meta['ready_s']) == ['sim', 'quick']
    assert 'leftover_processes' in meta


def test_patch_yaml_changes_only_named_values(tmp_path):
    src = tmp_path / 'nav2_params.yaml'
    src.write_text('controller_server:\n  ros__parameters:\n    DWA:\n      max_v: 2.0\n      max_w: 1.5\n')
    out = runner.patch_yaml(src, {'controller_server.ros__parameters.DWA.max_v': 0.5}, tmp_path / 'p' / 'n.yaml')
    import yaml
    assert yaml.safe_load(out.read_text()) == {'controller_server': {'ros__parameters': {'DWA': {'max_v': 0.5, 'max_w': 1.5}}}}
    # 없는 키는 조용히 새로 만들지 않고 거부한다 (오타로 아무 효과 없는 실험을 막는다)
    import pytest
    with pytest.raises(KeyError):
        runner.patch_yaml(src, {'controller_server.ros__parameters.DWA.max_vv': 0.5}, tmp_path / 'q.yaml')


def test_ready_check_timeout_is_not_run_timeout(monkeypatch):
    # 준비 확인 명령이 멈춰도(TimeoutExpired) 실행 전체 시간 초과가 아니라 기동 실패(startup)로 끝나야 재시도된다
    monkeypatch.setattr(runner, 'run_cli', lambda *a, **k: (None, ''))
    monkeypatch.setattr(runner.time, 'sleep', lambda s: None)
    for wait in (lambda: runner.wait_topic('/x', 0.05), lambda: runner.wait_cmd('ros2 lifecycle get /amcl', 0.05)):
        try:
            wait()
        except runner.RunError as e:
            assert e.startup
        else:
            raise AssertionError('RunError가 나야 한다')


def test_run_cli_kills_whole_group_on_timeout(tmp_path):
    # 셸이 띄운 자식(sleep)까지 같이 죽어야 재시도마다 쌓이지 않는다
    marker = tmp_path / 'child_pid'
    code, out = runner.run_cli(f'sleep 30 & echo $! > {marker}; wait', timeout_s=0.5)
    assert code is None and out == ''
    pid = int(marker.read_text())
    import time
    for _ in range(40):
        # 죽은 자식은 부모(셸)가 먼저 죽어 init에 넘어가는데, CI 컨테이너의 1번 프로세스는 거두지 않아 좀비(Z)로 남는다.
        # 좀비는 아무것도 안 하므로 죽은 것으로 본다
        state = subprocess.run(['ps', '-o', 'stat=', '-p', str(pid)], capture_output=True, text=True).stdout.strip()
        if not state or state.startswith('Z'):
            break
        time.sleep(0.05)
    else:
        raise AssertionError(f'자식 프로세스가 남았다 (상태 {state})')
    assert runner.run_cli('echo hi') == (0, 'hi\n')
