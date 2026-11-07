"""식당 시연 녹화 지표 (demo_metrics, analyze) 가짜 데이터 시험.

    python3 -m pytest experiments/restaurant_demo -q
마지막 시험은 가짜 rosbag을 실제로 써서 analyze.py 전체를 돌린다 (rosbag2_py·amr_msgs가 있는 컨테이너에서만, 없으면 건너뛴다).
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze  # noqa: E402
import demo_metrics as dm  # noqa: E402


def row(task_id, assignee='', state=0, task_type=0, created=0.0, pickup='kitchen', dropoff='buffet_2', priority=100):
    return {'task_id': task_id, 'task_type': task_type, 'priority': priority, 'pickup': pickup, 'dropoff': dropoff,
            'created': created, 'assignee': assignee, 'state': state}


# ---------------------------------------------------------------- 작업표 접기
def test_fold_table_takes_first_assign_and_end_and_falls_back_to_first_seen():
    tasks = dm.fold_table([
        (11.0, [row('a', created=10.0), row('b')]),
        (12.0, [row('a', 'amr_01', 0), row('b')]),
        (13.0, [row('a', 'amr_01', 1), row('b', 'amr_02', 0)]),
        (20.0, [row('a', 'amr_01', 2), row('b', 'amr_02', 3)]),
        (21.0, [row('a', 'amr_01', 2), row('b', 'amr_02', 3)]),
    ])
    a, b = tasks['a'], tasks['b']
    assert (a.request, a.assign, a.wait, a.end, a.final_state) == (10.0, 12.0, 2.0, 20.0, 2)
    assert b.request == 11.0          # created_time 0 → 작업표에 처음 나온 시각
    assert (b.assign, b.assignee, b.end, b.final_state) == (13.0, 'amr_02', 20.0, 3)


def test_snapshots_out_of_order_are_sorted():
    tasks = dm.fold_table([(12.0, [row('a', 'amr_01')]), (11.0, [row('a')])])
    assert tasks['a'].first_seen == 11.0 and tasks['a'].assign == 12.0


# ---------------------------------------------------------------- 1-2
def _task(**kw):
    base = dict(task_id='d1', task_type=0, priority=100, pickup='kitchen', dropoff='buffet_2', created=100.0,
                first_seen=100.5, assign=101.0, assignee='amr_01', end=None, final_state=1)
    base.update(kw)
    return dm.TableTask(**base)


def test_arrival_is_first_unloading_phase_at_dropoff():
    st = [{'t': 130.0, 'task_id': 'd1', 'state': 1, 'phase': 'unloading'},
          {'t': 110.0, 'task_id': 'd1', 'state': 1, 'phase': 'move_to_dropoff'},
          {'t': 131.0, 'task_id': 'd1', 'state': 1, 'phase': 'unloading'},
          {'t': 125.0, 'task_id': 'other', 'state': 1, 'phase': 'unloading'}]
    assert dm.arrival_time(_task(), st, 'buffet_2') == 130.0


def test_arrival_at_pickup_station_uses_loading():
    task = _task(pickup='buffet_2', dropoff='t11')
    st = [{'t': 120.0, 'task_id': 'd1', 'state': 1, 'phase': 'unloading'},
          {'t': 115.0, 'task_id': 'd1', 'state': 1, 'phase': 'loading'}]
    assert dm.arrival_time(task, st, 'buffet_2') == 115.0


def test_arrival_falls_back_to_completed_only_for_dropoff():
    st = [{'t': 140.0, 'task_id': 'd1', 'state': 2, 'phase': ''}]
    assert dm.arrival_time(_task(), st, 'buffet_2') == 140.0
    assert dm.arrival_time(_task(pickup='buffet_2', dropoff='t11'), st, 'buffet_2') is None
    assert dm.arrival_time(_task(dropoff='buffet_1'), st, 'buffet_2') is None


def _track(segments, t0=100.0, hz=2.0):
    """[(길이 s, 속도 m/s)] → 2 Hz 자세 [(t, x, y)] (x로 곧게 간다)."""
    out, t, x = [(t0, 0.0, 0.0)], t0, 0.0
    for dur, v in segments:
        for _ in range(int(round(dur * hz))):
            t += 1 / hz
            x += v / hz
            out.append((t, x, 0.0))
    return out


def test_stops_counts_only_stops_during_driving():
    # 출발 전 3 s 대기 → 5 s 주행 → 2 s 정지 → 3 s 주행 → 0.5 s 정지(짧아서 안 셈) → 3 s 주행 → 4 s 정지 → 2 s 주행 → 1 s 도착 정차
    track = _track([(3, 0), (5, 0.4), (2, 0), (3, 0.4), (0.5, 0), (3, 0.4), (4, 0.01), (2, 0.4), (1, 0)])
    out = dm.stops(track, 100.0, 123.5)
    assert out['start_wait_s'] == pytest.approx(3.5)   # 첫 빠른 표본(구간 끝)까지
    assert out['stops'] == 2
    assert [d for _, d in out['intervals']] == [pytest.approx(2.0), pytest.approx(4.0)]
    assert out['stop_s'] == pytest.approx(6.0)
    assert out['end_stop_s'] == pytest.approx(1.0)
    assert out['stop_ratio'] == pytest.approx(6.0 / (123.5 - 103.5))   # 출발 ~ 도착 중 멈춘 비율


def test_stops_never_departing():
    out = dm.stops(_track([(10, 0)]), 100.0, 110.0)
    assert out['stops'] == 0 and out['start_wait_s'] == pytest.approx(10.0)


def test_scene12_reports_request_to_arrival_and_stops_for_the_station_tasks_only():
    tasks = {'d1': _task(), 'd2': _task(task_id='d2', dropoff='buffet_1', created=105.0)}
    st = [{'t': 118.0, 'task_id': 'd1', 'state': 1, 'phase': 'unloading'}]
    poses = {'amr_01': _track([(1, 0), (5, 0.4), (2, 0), (10, 0.4)])}
    rows = dm.scene12(tasks, st, poses, station='buffet_2')
    assert len(rows) == 1
    r = rows[0]
    assert (r['task_id'], r['robot'], r['request_to_arrival_s']) == ('d1', 'amr_01', 18.0)
    assert r['stops'] == 1


def test_scene12_unreached_station_reports_stops_until_the_end_of_the_recording():
    # 도착 못 함: 도착 시간은 비우고, 녹화 끝까지 멈춤을 센다. 끝까지 멈춰 있던 6 s도 멈춤 하나
    track = _track([(1, 0), (5, 0.4), (2, 0), (3, 0.4), (6, 0)])     # 100 ~ 117 s
    rows = dm.scene12({'d1': _task()}, [], {'amr_01': track})
    r = rows[0]
    assert r['arrive_s'] is None and r['request_to_arrival_s'] is None
    assert r['stops'] == 2
    assert [d for _, d in r['intervals']] == [pytest.approx(2.0), pytest.approx(6.0)]
    assert r['stop_s'] == pytest.approx(8.0)
    assert r['stop_ratio'] == pytest.approx(8.0 / (117.0 - 101.5), abs=1e-3)
    assert r['end_stop_s'] == 0.0


def test_scene12_unreached_uses_recording_end_when_given():
    # 로봇 자세가 117 s에서 끊겨도 녹화가 120 s까지면 120 s까지 멈춘 것으로 본다
    track = _track([(1, 0), (5, 0.4), (2, 0), (3, 0.4), (6, 0)])
    r = dm.scene12({'d1': _task()}, [], {'amr_01': track}, end=120.0)[0]
    assert r['stop_s'] == pytest.approx(2.0 + 9.0)


def test_scene12_unreached_without_robot_track_has_no_stop_metrics():
    rows = dm.scene12({'d1': _task(assignee='')}, [], {'amr_01': _track([(5, 0.4)])})
    assert rows[0]['arrive_s'] is None and 'stops' not in rows[0]


# ---------------------------------------------------------------- 2
def _t2(task_id, kind, request, assign, state=2):
    return dm.TableTask(task_id=task_id, task_type=dm.TYPE_COLLECT if kind == 'collect' else 0,
                        priority=200 if kind == 'collect' else 100, pickup='t07', dropoff='return_1', created=request,
                        first_seen=request, assign=assign, assignee='amr_01' if assign is not None else '',
                        end=None, final_state=state)


def test_collect_wait_mean_max_and_unassigned():
    tasks = {k.task_id: k for k in [
        _t2('c1', 'collect', 5, 9), _t2('c2', 'collect', 10, 30),
        _t2('c3', 'collect', 40, None, 0), _t2('n1', 'normal', 0, 1)]}
    out = dm.collect_wait(tasks)
    assert (out['collect_n'], out['collect_unassigned']) == (3, 1)
    assert out['collect_wait_mean_s'] == pytest.approx(12.0)
    assert out['collect_wait_max_s'] == pytest.approx(20.0)
    # 마감 뒤 할당은 미할당
    assert dm.collect_wait(tasks, cutoff=20)['collect_unassigned'] == 1   # c3은 요청이 마감 뒤라 빠지고 c2가 미할당
    assert dm.collect_wait(tasks, cutoff=20)['collect_n'] == 2


def test_starvation_matches_collect_aging_definition():
    # 일반 n1 0 s 요청·1 s 할당, n2 2 s 요청·50 s 할당, n3 60 s 요청·끝까지 미할당 (마감 100)
    tasks = {k.task_id: k for k in [
        _t2('n1', 'normal', 0, 1), _t2('n2', 'normal', 2, 50),
        _t2('n3', 'normal', 60, None, 0), _t2('c1', 'collect', 3, 4)]}
    assert dm.starvation(tasks, cutoff=100) == pytest.approx(48.0)            # 2 → 50
    assert dm.starvation(tasks, cutoff=100, window=(70, 90)) == pytest.approx(40.0)   # 60 → 100만 겹친다
    assert dm.starvation(tasks, cutoff=100, window=(55, 58)) == 0.0


def test_starvation_without_normal_tasks_is_zero():
    assert dm.starvation({'c1': _t2('c1', 'collect', 0, 5)}, cutoff=10) == 0.0


def test_scene2_assign_order_and_counts():
    tasks = {k.task_id: k for k in [
        _t2('n1', 'normal', 0, 1), _t2('c1', 'collect', 5, 6),
        _t2('n2', 'normal', 1, 20, 3)]}
    out = dm.scene2(tasks, cutoff=30)
    assert out['assign_order'] == ['n1', 'c1', 'n2']
    assert (out['completed'], out['failed'], out['normal_n']) == (2, 1, 2)


# ---------------------------------------------------------------- 폴더 이름·표
@pytest.mark.parametrize('name, want', [
    ('demo_1-2-on', ('1-2', 'on')), ('demo_1-2-off', ('1-2', 'off')), ('demo_2-on', ('2', 'on')),
    ('demo_2-1-off', ('2-1', 'off')), ('demo_1-2', None), ('run_1-2-on', None), ('demo_1-2-maybe', None),
    # 하네스·버킷 이름 (#319): s11 → 1-1, s12 → 1-2, s2… → 2 (1막·2막 이름 미정)
    ('demo_s11-on', ('1-1', 'on')), ('demo_s12-off', ('1-2', 'off')), ('demo_s2-on', ('2', 'on')),
    ('demo_s21-on', ('2', 'on')), ('demo_s2_act2-off', ('2', 'off')), ('demo_smoke', None)])
def test_parse_run_name(name, want):
    assert analyze.parse_run_name(name) == want


def _fake_bag(path):
    path.mkdir(parents=True)
    (path / 'metadata.yaml').write_text('x', encoding='utf-8')
    return path


def test_find_bag_prefers_measurement_bag_and_skips_camera_bag(tmp_path):
    run = tmp_path / 'demo_s12-on_20261107-090000_00'
    _fake_bag(run / 'camera_bag')            # mp4로 못 바꿔 남은 카메라 bag (이름순으로 먼저 온다)
    _fake_bag(run / 'rosbag')
    assert analyze.find_bag(run) == run / 'rosbag'
    # rosbag/ 이름이 아니어도 카메라 bag은 고르지 않는다
    other = tmp_path / 'other'
    _fake_bag(other / 'camera_bag')
    _fake_bag(other / 'measure')
    assert analyze.find_bag(other) == other / 'measure'
    only_camera = tmp_path / 'only_camera'
    _fake_bag(only_camera / 'camera_bag')
    assert analyze.find_bag(only_camera) is None


def test_run_dirs_oldest_first_by_harness_stamp(tmp_path):
    scene = tmp_path / 'demo_s12-on'
    names = ['demo_s12-on_20261107-101500_00', 'demo_s12-on_20261106-235900_01',
             'demo_s12-on_20261107-101500_01', 'demo_s12-on_20261106-235900_00']
    for n in names:
        (scene / n).mkdir(parents=True)
    _fake_bag(scene / 'camera_bag')          # 장면 폴더의 카메라 bag은 실행이 아니다
    assert [p.name for p in analyze.run_dirs(scene)] == sorted(names)


def test_run_dirs_scene_folder_with_bag_is_one_run(tmp_path):
    scene = tmp_path / 'demo_s12-on'
    _fake_bag(scene / 'rosbag')
    assert analyze.run_dirs(scene) == [scene]


def test_latest_keeps_the_last_run_per_scene_folder():
    rows = [{'scene_dir': 'demo_s12-on', 'run': 'a'}, {'scene_dir': 'demo_s12-on', 'run': 'b'},
            {'scene_dir': 'demo_s12-off', 'run': 'c'}]
    assert [r['run'] for r in analyze.latest(rows)] == ['b', 'c']


def test_table_md_marks_unreached_and_missing_bag():
    md = analyze.table_md([
        {'run': 'demo_1-2-on', 'scene12': [{'task_id': 'd1', 'request_to_arrival_s': None, 'stops': 2,
                                            'stop_s': 8.0, 'stop_ratio': 0.516, 'start_wait_s': 1.5}]},
        {'run': 'demo_2-on', 'error': 'rosbag 없음'}])
    assert '| demo_1-2-on | d1 | - | 2 (8.0) | 52% | 1.5 |' in md   # 도착 시간은 -, 멈춤 지표는 채운다
    assert 'demo_2-on: rosbag 없음' in md


# ---------------------------------------------------------------- 가짜 rosbag 끝까지
def _write_bag(path, scene, arrive=True):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    msgs = pytest.importorskip('amr_msgs.msg')
    from builtin_interfaces.msg import Time
    from rclpy.serialization import serialize_message

    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for i, (name, typ) in enumerate([('/fleet/task_table', 'FleetTaskTable'), ('/fleet/task_status', 'TaskStatus'),
                                     ('/fleet/robot_states', 'RobotState')]):
        w.create_topic(rosbag2_py.TopicMetadata(name=name, type=f'amr_msgs/msg/{typ}', serialization_format='cdr'))

    def stamp(t):
        return Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))

    def put(topic, msg, t):
        w.write(topic, serialize_message(msg), int(t * 1e9))

    def record(task_id, task_type, dropoff, created, assignee, state):
        task = msgs.Task(task_id=task_id, task_type=task_type, pickup_station_id='kitchen',
                         dropoff_station_id=dropoff, priority=200 if task_type == 3 else 100,
                         created_time=stamp(created))
        return msgs.TaskRecord(task=task, assignee_id=assignee, state=state)

    if scene == '1-2':
        for t in [10.5, 11.0, 12.0, 30.0]:
            assignee, state = ('', 0) if t < 11 else ('amr_01', 1 if t < 30 else 2)
            m = msgs.FleetTaskTable(records=[record('d1', 0, 'buffet_2', 10.0, assignee, state)])
            m.header.stamp = stamp(t)
            put('/fleet/task_table', m, t)
        x = 0.0
        for k in range(41):     # 10 s ~ 30 s, 2 Hz. 18~20 s 멈춤
            t = 10.0 + k * 0.5
            x += 0.0 if t <= 11.0 or 18.0 < t <= 20.0 else 0.2
            m = msgs.RobotState(robot_id='amr_01')
            m.header.stamp = stamp(t)
            m.pose.position.x = x
            put('/fleet/robot_states', m, t)
        if arrive:
            s = msgs.TaskStatus(task_id='d1', robot_id='amr_01', state=1, phase='unloading')
            s.header.stamp = stamp(28.0)
            put('/fleet/task_status', s, 28.0)
    else:
        rows = [(0.0, [record('n1', 0, 't03', 0.0, '', 0), record('c1', 3, 'return_1', 0.0, '', 0)]),
                (2.0, [record('n1', 0, 't03', 0.0, '', 0), record('c1', 3, 'return_1', 0.0, 'amr_01', 0)]),
                (12.0, [record('n1', 0, 't03', 0.0, 'amr_01', 0), record('c1', 3, 'return_1', 0.0, 'amr_01', 2)])]
        for t, recs in rows:
            m = msgs.FleetTaskTable(records=recs)
            m.header.stamp = stamp(100.0 + t)
            put('/fleet/task_table', m, 100.0 + t)
    w = None   # 닫는다 (Humble SequentialWriter는 close가 없고 소멸 때 닫힌다)


def test_analyze_end_to_end_on_fake_bags(tmp_path):
    _write_bag(tmp_path / 'demo_1-2-on' / 'bag', '1-2')
    _write_bag(tmp_path / 'demo_2-off' / 'bag', '2')
    (tmp_path / 'demo_2-on').mkdir()        # bag 없는 실행
    (tmp_path / 'notes').mkdir()            # 규칙 밖 폴더는 무시
    out = tmp_path / 'results.md'
    assert analyze.main([str(tmp_path), '--out', str(out)]) == 0
    import json
    res = {r['scene_dir']: r for r in json.loads(out.with_suffix('.json').read_text(encoding='utf-8'))}
    assert set(res) == {'demo_1-2-on', 'demo_2-off', 'demo_2-on'}
    r12 = res['demo_1-2-on']['scene12'][0]
    assert r12['request_to_arrival_s'] == pytest.approx(18.0)
    assert r12['stops'] == 1
    s2 = res['demo_2-off']['scene2']
    assert s2['collect_wait_max_s'] == pytest.approx(2.0)     # 요청은 created_time 0 → 처음 나온 100 s
    assert s2['starvation_s'] == pytest.approx(12.0)          # n1 100 → 112
    assert s2['assign_order'] == ['c1', 'n1']
    assert res['demo_2-on']['error'] == 'rosbag 없음'


def test_analyze_harness_layout_runs_latest_in_table_all_in_json(tmp_path):
    # runs/demo_s12-on/<실행 id>/{rosbag, camera_bag}: 실행 id마다 한 줄, 표에는 가장 최근만
    scene = tmp_path / 'demo_s12-on'
    old = scene / 'demo_s12-on_20261106-235900_00'
    new = scene / 'demo_s12-on_20261107-101500_00'
    _write_bag(old / 'rosbag', '1-2')
    _write_bag(new / 'rosbag', '1-2', arrive=False)    # 최근 실행은 buffet_2에 못 닿았다
    _fake_bag(new / 'camera_bag')                      # 읽으면 깨지는 가짜 - 고르면 안 된다
    _write_bag(tmp_path / 'demo_s2-off' / 'demo_s2-off_20261107-090000_00' / 'rosbag', '2')
    out = tmp_path / 'results.md'
    assert analyze.main([str(tmp_path), '--out', str(out)]) == 0
    import json
    rows = json.loads(out.with_suffix('.json').read_text(encoding='utf-8'))
    assert [r['run_id'] for r in rows if r['scene_dir'] == 'demo_s12-on'] == [old.name, new.name]
    by_id = {r['run_id']: r for r in rows}
    assert by_id[old.name]['scene'] == '1-2' and by_id[old.name]['bag'].endswith('rosbag')
    assert by_id[old.name]['scene12'][0]['request_to_arrival_s'] == pytest.approx(18.0)
    unreached = by_id[new.name]['scene12'][0]
    assert unreached['request_to_arrival_s'] is None
    assert unreached['stops'] == 1 and unreached['stop_s'] == pytest.approx(2.0)   # 녹화 끝(30 s)까지
    assert by_id['demo_s2-off_20261107-090000_00']['scene'] == '2'
    md = out.read_text(encoding='utf-8')
    assert f'demo_s12-on/{new.name}' in md and old.name not in md
    assert analyze.main([str(tmp_path), '--out', str(out), '--all']) == 0
    assert old.name in out.read_text(encoding='utf-8')


def test_analyze_stops_with_a_clear_message_when_no_scene_folder(tmp_path):
    (tmp_path / 'demo_smoke').mkdir()
    with pytest.raises(SystemExit, match='demo_s12-on'):
        analyze.main([str(tmp_path)])
