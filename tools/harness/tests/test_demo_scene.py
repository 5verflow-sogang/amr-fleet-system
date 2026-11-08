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


def run_scene(steps, events):
    """가짜 시계로 돌린다. events = {시각: 작업표 갱신 함수(sent, table)}. 0.5 s마다 tick, 낸 것을 [(시각, 종류, id)]로."""
    scene = ds.Scene(steps)
    table, out, t = {}, [], 0.0
    while not scene.done and t < 400:
        for when, update in events.items():
            if abs(t - when) < 1e-9:
                update(scene.sent, table)
        for action in scene.tick(t, table):
            out.append((t, action[0], action[1].task_id if action[0] == 'task' else action[2]))
        t = round(t + 0.5, 3)
    return scene, out


def assign_all(sent, table):
    for task_id in sent:
        table[task_id] = 'amr_x'


# ---------------------------------------------------------------- 단계 진행
def test_act1_order_and_times_with_fake_clock():
    steps = ds.load_scene(TASKS / 'demo_s2a.yaml', {'collect_priority': '200'})
    # 4 s에 busy 5건이 모두 할당된다. queue 4건은 계속 대기
    scene, out = run_scene(steps, {4.0: assign_all})
    busy = [o for o in out if o[2].startswith('a_busy')]
    queue = [o for o in out if o[2].startswith('a_wait')]
    assert [o[0] for o in busy] == [0.0, 0.5, 1.0, 1.5, 2.0]        # 0.5 s 간격 (배정 순서를 못 박는다)
    assert [o[0] for o in queue] == [4.0] * 4                      # 할당 확인 즉시
    assert [o[2] for o in queue] == ['a_wait_t09', 'a_wait_t15', 'a_wait_t04', 'a_wait_t02']
    # 대기 4건 확인 + 3 s 뒤 /demo/t0과 회수 호출이 같은 시각, t0이 먼저
    assert out[-2:] == [(7.0, 't0', 'a_collect_t03'), (7.0, 'task', 'a_collect_t03')]
    assert scene.t0 == 7.0
    assert [s.tasks[0].priority for s in steps if s.t0] == [200]


def test_act1_off_fills_collect_priority_100():
    steps = ds.load_scene(TASKS / 'demo_s2a.yaml', {'collect_priority': '100'})
    collect = [t for s in steps for t in s.tasks if t.type == 'collect']
    assert [(t.task_id, t.priority, t.dropoff) for t in collect] == [('a_collect_t03', 100, 'return_1')]
    assert all(t.priority == 100 for s in steps for t in s.tasks if t.type == 'delivery')


def test_act2_second_round_then_delivery_then_collects_60s_later():
    steps = ds.load_scene(TASKS / 'demo_s2b.yaml')
    scene, out = run_scene(steps, {3.0: assign_all})
    # 1바퀴 0~2 s → 할당 확인(3 s) 뒤 2바퀴 5건 3~5 s → 1 s 뒤 2인석 배달(6 s) → 60 s 뒤 회수 5건 2 s 간격
    more = [o for o in out if o[2].startswith('b_more')]
    assert [o[0] for o in more] == [3.0, 3.5, 4.0, 4.5, 5.0] and more[0][2] == 'b_more_t05'
    assert [o for o in out if o[2] == 'b_wait_c01'] == [(6.0, 'task', 'b_wait_c01')]
    collects = [o for o in out if o[1] == 'task' and o[2].startswith('b_collect')]
    assert [o[0] for o in collects] == [66.0, 68.0, 70.0, 72.0, 74.0]
    assert [o[2] for o in collects] == ['b_collect_t01', 'b_collect_t06', 'b_collect_t08', 'b_collect_t12',
                                        'b_collect_t14']
    assert (66.0, 't0', 'b_collect_t01') in out and scene.t0 == 66.0
    assert {t.priority for s in steps for t in s.tasks if t.type == 'collect'} == {200}
    # 2바퀴 작업은 2인석 배달보다 먼저 들어가 같은 우선순위에서 앞선다 (작업표 created 순)
    order = [o[2] for o in out if o[1] == 'task']
    assert all(order.index(m[2]) < order.index('b_wait_c01') for m in more)


def test_act2_does_not_send_collects_if_the_delivery_got_assigned():
    # 배달이 0초 전에 할당되면(로봇이 일찍 빔) 기다리다 시간 초과로 보내고 scene.json에 timeout을 남긴다
    steps = ds.load_scene(TASKS / 'demo_s2b.yaml')
    scene, out = run_scene(steps, {3.0: assign_all, 10.0: assign_all})
    first = next(o[0] for o in out if o[2] == 'b_collect_t01')
    assert first == 6.0 + 90.0
    assert {'t': 96.0, 'ev': 'step', 'step': 'collects', 'why': 'timeout'} in scene.log


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
    # 켬이면 0초 뒤 다음에 일을 마친 4호가 2인석 배달에 배정된다. 시작 작업 두 바퀴로 그 순서를 만든다 (장면 파일 머리말)
    data = yaml.safe_load((TASKS / 'demo_s2b.yaml').read_text(encoding='utf-8'))
    plan, steps = data['plan'], data['steps']
    owners = _busy_owners('demo_s2b.yaml')
    assert [owners[r][0] for r in ORDER] == [t['id'] for t in steps[0]['tasks']]   # 1바퀴: 낸 순서대로 05, 04, 03, 02, 01
    straight = {r: v[1] for r, v in owners.items()}
    assert min(straight, key=straight.get) == 'amr_04'                             # 직선 거리로도 4호 1바퀴가 가장 짧다
    path = {r: v[2] for r, v in owners.items()}
    assert path['amr_04'] == plan['first_round1_m']
    assert sorted(v for r, v in path.items() if r != 'amr_04') == sorted(plan['others_round1_m'])
    # (1) 4호가 1바퀴를 10 m 넘게 먼저 끝내 2바퀴 첫 작업을 가져간다
    assert min(plan['others_round1_m']) >= plan['first_round1_m'] + 10.0
    assert steps[1]['name'] == 'more' and steps[1]['tasks'][0]['id'] == 'b_more_t05'
    # (2) 4호 두 바퀴는 최고 속도로도 0초 뒤 10 s 넘게 남는다 (픽업·하역 4번 머무름)
    v, dwell, t0 = plan['speed_mps'], plan['dwell_s'], plan['t0_s']
    assert plan['first_total_m'] / v + 4 * dwell >= t0 + 10.0
    # (3) 다른 로봇 1바퀴가 4호 두 바퀴보다 짧아 2바퀴 나머지 넷을 다른 로봇이 가져간다
    assert plan['first_total_m'] >= max(plan['others_round1_m']) + 5.0
    # (4) 다른 로봇은 2바퀴 어느 작업을 받아도 두 바퀴 합이 4호보다 5 m 넘게 길다 (자리를 모두 다르게 하면 10 m는 못 맞춘다)
    assert plan['others_min_total_m'] >= plan['first_total_m'] + 5.0
    # 0초 계산: 1바퀴 2 s + 할당 확인 + 2바퀴 2 s + 1 s + 60 s
    scene, _ = run_scene(ds.load_scene(TASKS / 'demo_s2b.yaml'), {3.0: assign_all})
    assert scene.t0 == t0
    # 자리: 1바퀴·2바퀴·2인석 배달 22곳이 모두 다르다 (서버 스테이션 예약, test_scene_files_have_no_station_conflicts)
    names = [n for step in steps[:3] for t in step['tasks'] for n in (t['pickup'], t['dropoff'])]
    assert len(names) == len(set(names)) == 22
    # 1-2·2막 핵심 자리와 그 1 m 안은 시작 작업에 쓰지 않는다
    st = stations()
    core = ['buffet_2', 't11', 'c01', 't01', 't06', 't08', 't12', 't14', 'return_1', 'return_2']
    start = [n for step in steps[:2] for t in step['tasks'] for n in (t['pickup'], t['dropoff'])]
    assert not [(a, b) for a in start for b in core if ds.stations_near(st, a, b)]


# ---------------------------------------------------------------- 1-2 (demo_s12)
S12_ROBOTS = REPO / 'src' / 'bringup' / 'amr_bringup' / 'config' / 'robots_restaurant_s12.yaml'


def test_scene12_start_tasks_go_to_robots_5_and_4_and_the_request_to_robot_1():
    owners = _busy_owners('demo_s12.yaml', S12_ROBOTS)
    # 동쪽 매장 픽업이라 1호(서쪽 입구 옆)보다 5·4호가 가깝다
    assert [owners[r][0] for r in ('amr_05', 'amr_04')] == ['s12_busy_L01', 's12_busy_b02'] and len(owners) == 2
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
    events = _server_table({1.0: {'s12_busy_L01': ('amr_05', 1)},          # 처음 보인 상태가 IN_PROGRESS
                            2.0: {'s12_busy_b02': ('amr_04', 2)}})         # 처음 보인 상태가 COMPLETED
    scene, out = run_scene(steps, events)
    assert (3.5, 't0', 's12_buffet2_t11') in out                           # 0.5 s + 3 s
    assert not any(e.get('why') == 'timeout' for e in scene.log)


def test_failed_task_back_in_queue_is_not_counted_until_reassigned():
    # 실패 뒤 다시 큐에 들어가면 서버가 assignee를 비운다 → 다시 할당될 때까지 기다린다
    steps = ds.load_scene(TASKS / 'demo_s12.yaml')
    events = _server_table({1.0: {'s12_busy_L01': ('amr_05', 1), 's12_busy_b02': ('amr_04', 1)},
                            1.5: {'s12_busy_b02': ('', 0)},                   # 실패 → PENDING
                            10.0: {'s12_busy_b02': ('amr_03', 1)}})           # 다시 할당
    scene, out = run_scene(steps, events)
    assert next(o[0] for o in out if o[1] == 't0') == 10.0                   # 지연 3 s는 이미 지났다


def test_table_counts_and_record():
    table = {'a': 'amr_01', 'b': '', 'c': 'amr_02'}
    states = {'a': 2, 'b': 0, 'c': 3}
    assert ds.table_counts(['a', 'b', 'c', 'd'], table, states) == {
        'sent': 4, 'assigned': 2, 'pending': 2, 'done': 1, 'failed': 1}
    scene = ds.Scene(ds.load_scene(TASKS / 'demo_s12.yaml'))
    scene.tick(0.0, {})
    scene.record(1.0, {'s12_busy_L01': 'amr_05'}, {'s12_busy_L01': 1})
    assert scene.log[-1] == {'t': 1.0, 'ev': 'count', 'sent': 1, 'assigned': 1, 'pending': 0, 'done': 0, 'failed': 0}
