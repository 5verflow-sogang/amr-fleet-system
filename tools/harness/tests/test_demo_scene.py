"""식당 시연 장면 스크립트 (tools/harness/demo_scene.py) 단계 진행과 장면 2 장면 파일. ROS 없이 가짜 시계·작업표로 돈다."""

import math
import sys
from pathlib import Path

import pytest
import yaml

HARNESS = Path(__file__).resolve().parents[1]
REPO = HARNESS.parents[1]
sys.path.insert(0, str(HARNESS))

import demo_scene as ds  # noqa: E402

TASKS = HARNESS / 'tasks'
STATIONS = REPO / 'src' / 'fleet' / 'amr_fleet_manager' / 'config' / 'stations_restaurant.yaml'
ROBOTS = REPO / 'src' / 'bringup' / 'amr_bringup' / 'config' / 'robots_restaurant.yaml'


def stations():
    return {k: tuple(v) for k, v in yaml.safe_load(STATIONS.read_text(encoding='utf-8'))['stations'].items()}


def run_scene(steps, events, robots_at=None):
    """가짜 시계로 돌린다. events = {시각: 작업표 갱신 함수(sent, table)}, robots_at(t) = 로봇 자세 {robot: (x, y)}.
    0.5 s마다 tick, 낸 것을 [(시각, 종류, id)]로."""
    scene = ds.Scene(steps, {k: v[:2] for k, v in stations().items()})
    table, out, t = {}, [], 0.0
    while not scene.done and t < 400:
        for when, update in events.items():
            if abs(t - when) < 1e-9:
                update(scene.sent, table)
        for action in scene.tick(t, table, robots_at(t) if robots_at else {}):
            out.append((t, action[0], action[1].task_id if action[0] == 'task' else action[2]))
        t = round(t + 0.5, 3)
    return scene, out


def assign_all(sent, table):
    for task_id in sent:
        table[task_id] = 'amr_x'


# ---------------------------------------------------------------- 단계 진행
def at_station_after(name, t_arrive, robot='amr_x'):
    """t_arrive부터 robot이 그 자리에 있다 (after.near 시험용)."""
    xy = stations()[name][:2]
    return lambda t: {robot: xy} if t >= t_arrive else {robot: (xy[0] + 10.0, xy[1])}


def test_act1_order_and_times_with_fake_clock():
    steps = ds.load_scene(TASKS / 'demo_s2a.yaml', {'collect_priority': '200'})
    # 4 s에 busy 5건이 모두 할당된다. queue 4건은 계속 대기. 2호(a_busy_c01)는 20 s에 하역 자리에 온다
    scene, out = run_scene(steps, {4.0: assign_all}, at_station_after('c01', 20.0))
    busy = [o for o in out if o[2].startswith('a_busy')]
    queue = [o for o in out if o[2].startswith('a_wait')]
    assert [o[0] for o in busy] == [0.0, 0.5, 1.0, 1.5, 2.0]        # 0.5 s 간격 (배정 순서를 못 박는다)
    assert [o[0] for o in queue] == [4.0] * 4                      # 할당 확인 즉시
    assert [o[2] for o in queue] == ['a_wait_t09', 'a_wait_t15', 'a_wait_t04', 'a_wait_t02']
    # 대기 4건 + 3 s(7 s)로는 아직이고, 2호가 하역 1 m 앞에 온 20 s에 /demo/t0과 회수 호출 (t0이 먼저)
    assert out[-2:] == [(20.0, 't0', 'a_collect_t03'), (20.0, 'task', 'a_collect_t03')]
    assert scene.t0 == 20.0
    assert [s.tasks[0].priority for s in steps if s.t0] == [200]


def test_act1_off_fills_collect_priority_100():
    steps = ds.load_scene(TASKS / 'demo_s2a.yaml', {'collect_priority': '100'})
    collect = [t for s in steps for t in s.tasks if t.type == 'collect']
    assert [(t.task_id, t.priority, t.dropoff) for t in collect] == [('a_collect_t03', 100, 'return_1')]
    assert all(t.priority == 100 for s in steps for t in s.tasks if t.type == 'delivery')


def test_act2_delivery_then_collects_after_60s_and_when_robot_4_is_near():
    steps = ds.load_scene(TASKS / 'demo_s2b.yaml')
    # 시작 작업 0~2 s → 할당 확인(3 s) → 2인석 배달(3 s, 마지막 시작 작업 뒤 1 s) → 60 s 뒤(63 s)에도 4호가 멀어 기다림
    # → 4호(b_busy_t03)가 하역 2 m 앞에 온 69.5 s(배달 뒤 66.5 s, 11/10 on_v3은 66.7 s)에 0초, 회수 5건 2 s 간격
    scene, out = run_scene(steps, {3.0: assign_all}, at_station_after('t03', 69.5))
    assert [o for o in out if o[2] == 'b_wait_c01'] == [(3.0, 'task', 'b_wait_c01')]
    collects = [o for o in out if o[1] == 'task' and o[2].startswith('b_collect')]
    assert [o[0] for o in collects] == [69.5, 71.5, 73.5, 75.5, 77.5]
    assert [o[2] for o in collects] == ['b_collect_t01', 'b_collect_t06', 'b_collect_t08', 'b_collect_t12',
                                        'b_collect_t14']
    assert (69.5, 't0', 'b_collect_t01') in out and scene.t0 == 69.5 and scene.t0_by == 'near'
    assert {'t': 69.5, 'ev': 't0', 'step': 'collects', 'by': 'near'} in scene.log
    assert {t.priority for s in steps for t in s.tasks if t.type == 'collect'} == {200}
    # 4호가 일찍 와도 배달이 60 s 기다리기 전에는 0초가 아니다
    scene, out = run_scene(steps, {3.0: assign_all}, at_station_after('t03', 10.0))
    assert scene.t0 == 63.0 and scene.t0_by == 'near'


def test_act2_second_zero_falls_back_to_70s_after_the_delivery_if_robot_4_is_stuck():
    # 11/10 off_v3: 4호가 가운데 통로 손님 사이에 갇혀 near가 끝까지 안 맞았고, 150 s에 5호가 배달을 가져갔다.
    # 배달 요청(3 s) 뒤 70 s(73 s)에 0초로 하고 회수를 낸다
    steps = ds.load_scene(TASKS / 'demo_s2b.yaml')
    scene, out = run_scene(steps, {3.0: assign_all}, at_station_after('t03', 1e9))
    collects = [o[0] for o in out if o[1] == 'task' and o[2].startswith('b_collect')]
    assert collects == [73.0, 75.0, 77.0, 79.0, 81.0]
    assert scene.t0 == 73.0 and scene.t0_by == 'timeout'
    assert {'t': 73.0, 'ev': 't0', 'step': 'collects', 'by': 'timeout'} in scene.log


def test_act2_does_not_send_collects_if_the_delivery_got_assigned():
    # 배달이 0초 전에 할당되면(로봇이 일찍 빔) 기다리다 시간 제한(배달 뒤 70 s)에 보내고 scene.json에 timeout을 남긴다
    steps = ds.load_scene(TASKS / 'demo_s2b.yaml')
    scene, out = run_scene(steps, {3.0: assign_all, 10.0: assign_all}, at_station_after('t03', 69.5))
    first = next(o[0] for o in out if o[2] == 'b_collect_t01')
    assert first == 3.0 + 70.0
    assert {'t': 73.0, 'ev': 'step', 'step': 'collects', 'why': 'timeout'} in scene.log


def test_act1_second_zero_by_near_or_70s_after_the_queue():
    # 1막도 같은 대비: 2호가 막히면 대기 배달(4 s) 뒤 70 s(74 s)에 0초
    steps = ds.load_scene(TASKS / 'demo_s2a.yaml', {'collect_priority': '200'})
    scene, _ = run_scene(steps, {4.0: assign_all}, at_station_after('c01', 1e9))
    assert scene.t0 == 74.0 and scene.t0_by == 'timeout'
    scene, _ = run_scene(steps, {4.0: assign_all}, at_station_after('c01', 40.0))
    assert scene.t0 == 40.0 and scene.t0_by == 'near'


def test_scene12_second_zero_is_ok_without_a_position_condition():
    scene, _ = run_scene(ds.load_scene(TASKS / 'demo_s12.yaml'), {3.0: assign_all})
    assert scene.t0 == 3.5 and scene.t0_by == 'ok'


def test_condition_timeout_sends_anyway_and_logs_it():
    steps = ds.load_scene(TASKS / 'demo_s2a.yaml', {'collect_priority': '200'})
    scene, out = run_scene(steps, {})                 # 아무것도 할당되지 않는다
    queue_at = next(o[0] for o in out if o[2] == 'a_wait_t09')
    assert queue_at == 2.0 + 60.0                    # 마지막 busy 작업(2.0 s) 뒤 60 s
    assert any(e.get('step') == 'queue' and e['why'] == 'timeout' for e in scene.log)
    assert scene.done


@pytest.mark.parametrize('bad, message', [
    ({'type': 'teleport'}, 'type'), ({'priority': 300}, 'priority'), ({'id': 'x1'}, '중복')])
def test_load_scene_rejects_bad_tasks(tmp_path, bad, message):
    task = {'id': 'x2', 'type': 'delivery', 'pickup': 'stall_1', 'dropoff': 't01', **bad}
    path = tmp_path / 's.yaml'
    path.write_text(yaml.safe_dump({'steps': [{'name': 'a', 'tasks': [
        {'id': 'x1', 'type': 'delivery', 'pickup': 'stall_1', 'dropoff': 't02'}, task]}]}), encoding='utf-8')
    with pytest.raises(ValueError, match=message):
        ds.load_scene(path)


def test_load_scene_allows_one_t0_step(tmp_path):
    path = tmp_path / 's.yaml'
    step = {'tasks': [{'id': 'x', 'type': 'collect', 'pickup': 't01', 'dropoff': 'return_1'}], 't0': True}
    path.write_text(yaml.safe_dump({'steps': [{'name': 'a', **step},
                                              {'name': 'b', **step, 'tasks': [{**step['tasks'][0], 'id': 'y'}]}]}),
                    encoding='utf-8')
    with pytest.raises(ValueError, match='t0'):
        ds.load_scene(path)


# ---------------------------------------------------------------- 장면 2 장면 파일과 식당 자리
@pytest.mark.parametrize('scene', ['demo_s2a.yaml', 'demo_s2b.yaml'])
def test_scene_stations_exist_and_start_tasks_avoid_the_key_request(scene):
    st = stations()
    steps = ds.load_scene(TASKS / scene, {'collect_priority': '200'})
    names = {n for s in steps for t in s.tasks for n in (t.pickup, t.dropoff)}
    assert names <= set(st), names - set(st)
    core = {n for s in steps if s.t0 for t in s.tasks for n in (t.pickup, t.dropoff)}
    start = {n for s in steps if not s.t0 for t in s.tasks for n in (t.pickup, t.dropoff)}
    # 시작 작업은 핵심 요청과 같은 자리·스테이션 예약 1 m 안을 쓰지 않는다
    near = sorted((a, b) for a in start for b in core if ds.stations_near(st, a, b))
    assert near == []


def _busy_owners(scene, robots_file=ROBOTS):
    """nearest(로봇 → 픽업 직선 거리, 탐욕)로 busy 작업을 낸 순서대로 배정했을 때 {로봇: (작업 id, 직선 거리 합, path_m)}."""
    st = stations()
    robots = {k: tuple(v[:2]) for k, v in yaml.safe_load(robots_file.read_text(encoding='utf-8'))['robots'].items()}
    raw = yaml.safe_load((TASKS / scene).read_text(encoding='utf-8'))['steps'][0]['tasks']
    idle, owners = dict(robots), {}
    for task in raw:
        pickup = st[task['pickup']][:2]
        robot = min(idle, key=lambda r: (math.dist(idle[r], pickup), r))
        straight = math.dist(idle.pop(robot), pickup) + math.dist(pickup, st[task['dropoff']][:2])
        owners[robot] = (task['id'], straight, task.get('path_m'))
    return owners


ORDER = ['amr_05', 'amr_04', 'amr_03', 'amr_02', 'amr_01']


def _check_first_to_finish(scene, first):
    owners = _busy_owners(scene)
    raw = yaml.safe_load((TASKS / scene).read_text(encoding='utf-8'))['steps'][0]['tasks']
    assert [owners[r][0] for r in ORDER] == [t['id'] for t in raw]          # 낸 순서대로 05, 04, 03, 02, 01
    path = {r: v[2] for r, v in owners.items()}
    straight = {r: v[1] for r, v in owners.items()}
    # A* 경로 길이: 먼저 끝날 로봇이 가장 짧고 다음 로봇과 10 m 넘게 차이 난다
    assert min(v for r, v in path.items() if r != first) >= path[first] + 10.0
    # 직선 거리로도 가장 짧다 (적어 둔 path_m이 엉뚱하지 않은지)
    assert min(straight, key=straight.get) == first
    return path


def test_act1_robot_2_finishes_first_by_more_than_10m():
    # 연우 시나리오: 켬이면 다음에 일을 마친 2호가 회수에 바로 배정된다
    _check_first_to_finish('demo_s2a.yaml', 'amr_02')
    raw = yaml.safe_load((TASKS / 'demo_s2a.yaml').read_text(encoding='utf-8'))['steps'][0]['tasks']
    assert len({t['pickup'] for t in raw}) == 5                             # 픽업 예약으로 기다리지 않는다


def test_act2_robot_4_is_the_first_to_free_up_after_second_zero():
    # 켬이면 0초 직후 다음에 일을 마친 4호가 2인석 배달에 배정된다 (장면 파일 머리말)
    data = yaml.safe_load((TASKS / 'demo_s2b.yaml').read_text(encoding='utf-8'))
    plan, steps = data['plan'], data['steps']
    path = _check_first_to_finish('demo_s2b.yaml', 'amr_04')                       # 4호가 10 m 넘게 짧다
    assert path['amr_04'] == plan['first_m']
    assert sorted(v for r, v in path.items() if r != 'amr_04') == sorted(plan['others_m'])
    # 4호 작업은 관측 평균의 1.5배 속도로도 0초 하한(배달 60 s 대기) 뒤 10 s 넘게 남는다 - 먼저 끝나면 배달을 가져간다
    assert plan['first_m'] / plan['speed_check_mps'] + 2 * plan['dwell_s'] >= plan['t0_min_s'] + 10.0
    scene, _ = run_scene(ds.load_scene(TASKS / 'demo_s2b.yaml'), {3.0: assign_all}, at_station_after('t03', 0.0))
    assert scene.t0 <= plan['t0_min_s']                                            # 0초 하한 계산이 맞다
    # 0초는 4호 작업 하역 2 m 앞 (실제 속도와 상관없이 0초 직후 4호가 빈다)
    core = steps[-1]
    assert core['after']['near'] == {'task': 'b_busy_t03', 'within_m': 2.0} and core.get('t0')
    # 자리: 시작 작업·2인석 배달 12곳이 모두 다르다 (서버 스테이션 예약)
    names = [n for step in steps[:2] for t in step['tasks'] for n in (t['pickup'], t['dropoff'])]
    assert len(names) == len(set(names)) == 12
    # 핵심 자리와 그 1 m 안은 시작 작업에 쓰지 않는다 (buffet_2는 1-2 핵심 자리라 2막은 쓴다 - 매장 규칙 때문에 자리가 모자란다)
    st = stations()
    core_names = ['t11', 'c01', 't01', 't06', 't08', 't12', 't14', 'return_1', 'return_2']
    start = [n for t in steps[0]['tasks'] for n in (t['pickup'], t['dropoff'])]
    assert not [(a, b) for a in start for b in core_names if ds.stations_near(st, a, b)]


# ---------------------------------------------------------------- 1-2 (demo_s12)
S12_ROBOTS = REPO / 'src' / 'bringup' / 'amr_bringup' / 'config' / 'robots_restaurant_s12.yaml'


def test_scene12_start_tasks_go_to_robots_4_and_5_and_the_request_to_robot_1():
    owners = _busy_owners('demo_s12.yaml', S12_ROBOTS)
    # 동쪽 픽업(t15, L03)이라 1호(서쪽 입구 옆)보다 4·5호가 가깝다
    assert [owners[r][0] for r in ('amr_04', 'amr_05')] == ['s12_busy_c01', 's12_busy_L04'] and len(owners) == 2
    # 핵심 요청 픽업(buffet_2)은 남은 로봇 중 1호가 가장 가깝다
    st = stations()
    robots = {k: tuple(v[:2]) for k, v in yaml.safe_load(S12_ROBOTS.read_text(encoding='utf-8'))['robots'].items()}
    idle = {k: v for k, v in robots.items() if k not in owners}
    assert min(idle, key=lambda r: math.dist(idle[r], st['buffet_2'][:2])) == 'amr_01'
    steps = ds.load_scene(TASKS / 'demo_s12.yaml')
    core = [s for s in steps if s.t0]
    assert len(core) == 1
    assert [(t.task_id, t.type, t.pickup, t.dropoff) for t in core[0].tasks] == [
        ('s12_buffet2_t11', 'delivery', 'buffet_2', 't11')]


def test_scene12_request_is_sent_with_t0_after_the_two_start_tasks_are_assigned():
    steps = ds.load_scene(TASKS / 'demo_s12.yaml')
    scene, out = run_scene(steps, {3.0: assign_all})
    # 마지막 시작 작업(0.5 s) 뒤 3 s, 그 전에(3.0 s) 둘 다 할당됐다
    assert (3.5, 't0', 's12_buffet2_t11') in out and (3.5, 'task', 's12_buffet2_t11') in out
    assert [o[0] for o in out if o[2].startswith('s12_busy')] == [0.0, 0.5]


# ---------------------------------------------------------------- 서버 스테이션 예약과 작업표 상태
@pytest.mark.parametrize('scene', sorted(p.name for p in TASKS.glob('demo_*.yaml')))
def test_scene_files_have_no_station_conflicts(scene):
    # 서버는 맡긴 작업의 픽업·하역을 끝날 때까지 잡아 둔다 (station_clearance_m 1 m). 같이 잡힐 수 있는 작업끼리 자리가
    # 겹치면 둘째가 앞 작업이 끝날 때까지 할당되지 않는다 (11/8 녹화: 1-2 request, 2막 more 단계 시간 초과)
    steps = ds.load_scene(TASKS / scene, {'collect_priority': '200'})
    assert ds.station_conflicts(steps, stations()) == []


def test_station_conflicts_catches_shared_pickups_and_skips_the_t0_queue():
    st = stations()
    steps = [ds.Step('busy', [ds.SceneTask('a', 'delivery', 'stall_4', 'L01', 100),
                              ds.SceneTask('b', 'delivery', 'stall_3', 'b02', 100),
                              ds.SceneTask('c', 'delivery', 'stall_4', 'L02', 100)]),     # 11/8 1-2처럼 stall_4를 같이 씀
             ds.Step('core', [ds.SceneTask('x', 'collect', 't01', 'return_1', 200),
                              ds.SceneTask('y', 'collect', 't06', 'return_1', 200)], t0=True)]
    assert ds.station_conflicts(steps, st) == [('a', 'c', 'stall_4', 'stall_4')]   # 0초 단계 회수끼리 퇴식구는 뺀다
    steps[1].tasks.append(ds.SceneTask('z', 'delivery', 'L01', 't05', 100))
    assert ('a', 'z', 'L01', 'L01') in ds.station_conflicts(steps, st)


def _server_table(events):
    """서버 작업표를 흉내 낸다: {시각: {task_id: (assignee, state)}} → run_scene 이벤트 (Scene은 assignee만 본다)."""
    def update(changes):
        def apply(sent, table):
            for task_id, (assignee, _) in changes.items():
                table[task_id] = assignee
        return apply
    return {t: update(c) for t, c in events.items()}


def test_assigned_counts_tasks_already_in_progress_or_completed():
    # 표는 1 s마다라 할당(PENDING + assignee)을 못 보고 처음부터 IN_PROGRESS로, 또는 그새 끝나 COMPLETED로만 보일 수 있다.
    # 서버는 assignee를 끝까지 두므로 둘 다 할당으로 센다
    steps = ds.load_scene(TASKS / 'demo_s12.yaml')
    events = _server_table({1.0: {'s12_busy_c01': ('amr_05', 1)},          # 처음 보인 상태가 IN_PROGRESS
                            2.0: {'s12_busy_L04': ('amr_04', 2)}})         # 처음 보인 상태가 COMPLETED
    scene, out = run_scene(steps, events)
    assert (3.5, 't0', 's12_buffet2_t11') in out                           # 0.5 s + 3 s
    assert not any(e.get('why') == 'timeout' for e in scene.log)


def test_failed_task_back_in_queue_is_not_counted_until_reassigned():
    # 실패 뒤 다시 큐에 들어가면 서버가 assignee를 비운다 → 다시 할당될 때까지 기다린다
    steps = ds.load_scene(TASKS / 'demo_s12.yaml')
    events = _server_table({1.0: {'s12_busy_c01': ('amr_05', 1), 's12_busy_L04': ('amr_04', 1)},
                            1.5: {'s12_busy_L04': ('', 0)},                   # 실패 → PENDING
                            10.0: {'s12_busy_L04': ('amr_03', 1)}})           # 다시 할당
    scene, out = run_scene(steps, events)
    assert next(o[0] for o in out if o[1] == 't0') == 10.0                   # 지연 3 s는 이미 지났다


def test_table_counts_and_record():
    table = {'a': 'amr_01', 'b': '', 'c': 'amr_02'}
    states = {'a': 2, 'b': 0, 'c': 3}
    assert ds.table_counts(['a', 'b', 'c', 'd'], table, states) == {
        'sent': 4, 'assigned': 2, 'pending': 2, 'done': 1, 'failed': 1}
    scene = ds.Scene(ds.load_scene(TASKS / 'demo_s12.yaml'))
    scene.tick(0.0, {})
    scene.record(1.0, {'s12_busy_c01': 'amr_05'}, {'s12_busy_c01': 1})
    assert scene.log[-1] == {'t': 1.0, 'ev': 'count', 'sent': 1, 'assigned': 1, 'pending': 0, 'done': 0, 'failed': 0}


# ---------------------------------------------------------------- after.near (0초를 먼저 비는 로봇의 하역 직전에)
def _near_steps(within=2.0):
    return [ds.Step('busy', [ds.SceneTask('k', 'delivery', 'L01', 'L03', 100),
                             ds.SceneTask('o', 'delivery', 't02', 'L04', 100)], gap_s=0.5),
            ds.Step('core', [ds.SceneTask('c', 'collect', 't03', 'return_1', 200)],
                    after={'assigned': 2, 'near': {'task': 'k', 'within_m': within}, 'timeout_s': 300}, t0=True)]


def test_near_waits_for_the_assigned_robot_to_reach_the_dropoff():
    st = {k: v[:2] for k, v in stations().items()}
    scene = ds.Scene(_near_steps(), st)
    table = {'k': 'amr_04', 'o': 'amr_02'}
    far = {'amr_04': (st['L03'][0] + 5.0, st['L03'][1]), 'amr_02': st['L03']}   # 다른 로봇이 그 자리에 있어도 안 센다
    assert [a[0] for a in scene.tick(0.0, table, far)] == ['task']
    assert [a[0] for a in scene.tick(0.5, table, far)] == ['task']
    assert scene.tick(10.0, table, far) == []
    close = {'amr_04': (st['L03'][0] + 1.5, st['L03'][1])}
    out = scene.tick(20.0, table, close)
    assert [a[0] for a in out] == ['t0', 'task'] and scene.t0 == 20.0


def test_near_needs_the_task_to_be_assigned_and_times_out_otherwise():
    st = {k: v[:2] for k, v in stations().items()}
    scene = ds.Scene(_near_steps(), st)
    scene.tick(0.0, {}, {})
    scene.tick(0.5, {}, {})
    assert scene.tick(100.0, {'o': 'amr_02'}, {'amr_04': st['L03']}) == []         # k가 아직 할당 안 됨
    out = scene.tick(300.5, {'o': 'amr_02'}, {})
    assert out and out[0][0] == 't0'
    assert [e['why'] for e in scene.log if e['ev'] == 'step' and e['step'] == 'core'] == ['timeout']


def test_load_scene_rejects_near_on_an_unknown_task(tmp_path):
    path = tmp_path / 's.yaml'
    path.write_text(yaml.safe_dump({'steps': [
        {'name': 'a', 'tasks': [{'id': 'x', 'type': 'delivery', 'pickup': 'L01', 'dropoff': 'L03'}]},
        {'name': 'b', 'after': {'near': {'task': 'nope', 'within_m': 2}},
         'tasks': [{'id': 'y', 'type': 'collect', 'pickup': 't03', 'dropoff': 'return_1'}]}]}), encoding='utf-8')
    with pytest.raises(ValueError, match='near'):
        ds.load_scene(path)


# ---------------------------------------------------------------- 11/9 녹화: 부스·손님 근처 자리
def _guest_near(guest_file, radius=1.0):
    st = stations()
    out = set()

    def seg(q, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = 0 if dx == dy == 0 else max(0, min(1, ((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / (dx * dx + dy * dy)))
        return math.dist(q, (a[0] + t * dx, a[1] + t * dy))
    for o in yaml.safe_load(guest_file.read_text(encoding='utf-8'))['obstacles']:
        pts = [(w['x'], w['y']) for w in o.get('waypoints', [])]
        for name, v in st.items():
            q = v[:2]
            if o['pattern'] == 'random':
                xs, ys = [a[0] for a in pts], [a[1] for a in pts]
                d = math.hypot(max(min(xs) - q[0], 0, q[0] - max(xs)), max(min(ys) - q[1], 0, q[1] - max(ys)))
            else:
                d = min(seg(q, a, b) for a, b in zip(pts, pts[1:]))
            if d <= radius:
                out.add(name)
    return out


GUESTS = REPO / 'src' / 'simulation' / 'amr_gazebo' / 'config'


@pytest.mark.parametrize('scene, guest_file, allowed', [
    ('demo_s2a.yaml', 'dynamic_obstacles_restaurant.yaml', set()),
    # 2막: 3호·5호 하역을 아래쪽·왼쪽 테이블에 두려면 t07·t13을 써야 한다 (test_act2_only_robot_4_goes_to_the_stalls)
    ('demo_s2b.yaml', 'dynamic_obstacles_restaurant.yaml', {'t07', 't13'}),
    ('demo_s12.yaml', 'dynamic_obstacles_restaurant_peak_scene12.yaml', set())])
def test_start_tasks_avoid_booths_and_guest_paths(scene, guest_file, allowed):
    # 부스 b01~b03: 11/9 녹화에서 도착 실패로 재큐(1막 b02), 로컬 재현(추적·안전 켬)에서 부스 칸에서 못 빠져나옴.
    # 손님 경로 1 m 안 자리: 11/9 녹화에서 5호가 t06 손님 무리에 오래 머묾. 장면이 쓰는 손님 파일 기준
    steps = yaml.safe_load((TASKS / scene).read_text(encoding='utf-8'))['steps']
    start = {n for t in steps[0]['tasks'] for n in (t['pickup'], t['dropoff'])}
    assert not start & {'b01', 'b02', 'b03'}
    near = (start & _guest_near(GUESTS / guest_file)) - allowed
    assert not near, near


def test_act2_keeps_off_the_south_east_corner():
    # 11/11 켬 녹화 on_v3·v5a·v5b: 남동쪽 구석(c01·c02 옆, t10~L03 사이 1 m 통로)에서 셋 다 막혔다.
    # 시작 작업과 2인석 배달은 t10·L03·L04·c02, c01 반경 3 m 안 자리를 쓰지 않고, 배달 픽업은 stall_2
    st = stations()
    steps = yaml.safe_load((TASKS / 'demo_s2b.yaml').read_text(encoding='utf-8'))['steps']
    start = {n for t in steps[0]['tasks'] for n in (t['pickup'], t['dropoff'])}
    assert not start & {'t10', 'L03', 'L04', 'c02'}
    assert not [n for n in start if math.dist(st[n][:2], st['c01'][:2]) <= 3.0]
    delivery = steps[1]['tasks'][0]
    assert (delivery['pickup'], delivery['dropoff']) == ('stall_2', 'c01')
    assert delivery['pickup'] not in start


def test_act2_only_robot_4_goes_to_the_stalls():
    # 11/12 켬 녹화 on_v6b: 시작 뒤 4호(stall_2 픽업)·5호(stall_3 하역)·3호(stall_4 하역)가 매장 앞에 몰려 약 90 s 서로 막혔다.
    # 매장에는 4호만 간다: 다른 시작 작업은 매장을 쓰지 않고 4호 일터(t03 하역, stall_2 픽업) 2.5 m 안에 서지 않는다.
    # 3호·5호 하역은 아래쪽·왼쪽 테이블. 다른 4대는 A* 25 m 이상이다
    st = stations()
    steps = yaml.safe_load((TASKS / 'demo_s2b.yaml').read_text(encoding='utf-8'))['steps']
    owners = _busy_owners('demo_s2b.yaml')
    task = {t['id']: t for t in steps[0]['tasks']}
    stalls = {'stall_1', 'stall_2', 'stall_3', 'stall_4'}
    for robot, (tid, _, path_m) in owners.items():
        t = task[tid]
        if robot == 'amr_04':
            assert (t['pickup'], t['dropoff']) == ('t09', 't03')
            continue
        assert not {t['pickup'], t['dropoff']} & stalls, (robot, t)
        stops = (t['pickup'], t['dropoff'])
        assert all(math.dist(st[n][:2], st[k][:2]) > 2.5 for n in stops for k in ('t03', 'stall_2'))
        assert path_m >= 25.0, (robot, path_m)
    for robot in ('amr_03', 'amr_05'):
        assert task[owners[robot][0]]['dropoff'] in {'t07', 't13', 't15'}
    assert steps[1]['tasks'][0]['pickup'] == 'stall_2'
