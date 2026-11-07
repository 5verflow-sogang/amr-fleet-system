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
    assert [o[0] for o in busy] == [0.0] * 5
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


def test_act2_collects_60s_after_the_delivery_request_two_seconds_apart():
    steps = ds.load_scene(TASKS / 'demo_s2b.yaml')
    scene, out = run_scene(steps, {3.0: assign_all})
    wait = [o for o in out if o[2] == 'b_wait_c01']
    assert wait == [(3.0, 'task', 'b_wait_c01')]
    collects = [o for o in out if o[1] == 'task' and o[2].startswith('b_collect')]
    # 배달 요청(3 s) + 60 s = 63 s에 첫 회수와 /demo/t0, 그 뒤 2 s 간격
    assert [o[0] for o in collects] == [63.0, 65.0, 67.0, 69.0, 71.0]
    assert [o[2] for o in collects] == ['b_collect_t01', 'b_collect_t06', 'b_collect_t08', 'b_collect_t12',
                                        'b_collect_t14']
    assert (63.0, 't0', 'b_collect_t01') in out and scene.t0 == 63.0
    assert {t.priority for s in steps for t in s.tasks if t.type == 'collect'} == {200}


def test_act2_does_not_send_collects_if_the_delivery_got_assigned():
    # 배달이 0초 전에 할당되면(로봇이 일찍 빔) 기다리다 시간 초과로 보내고 scene.json에 timeout을 남긴다
    steps = ds.load_scene(TASKS / 'demo_s2b.yaml')
    scene, out = run_scene(steps, {3.0: assign_all, 10.0: assign_all})
    first = next(o[0] for o in out if o[2] == 'b_collect_t01')
    assert first == 3.0 + 90.0
    assert {'t': 93.0, 'ev': 'step', 'step': 'collects', 'why': 'timeout'} in scene.log


def test_condition_timeout_sends_anyway_and_logs_it():
    steps = ds.load_scene(TASKS / 'demo_s2a.yaml', {'collect_priority': '200'})
    scene, out = run_scene(steps, {})                 # 아무것도 할당되지 않는다
    queue_at = next(o[0] for o in out if o[2] == 'a_wait_t09')
    assert queue_at == 60.0
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


def _busy_owners(scene):
    """nearest(로봇 → 픽업 직선 거리, 탐욕)로 busy 작업을 낸 순서대로 배정했을 때 {로봇: (작업, 직선 거리 합)}."""
    st = stations()
    robots = {k: tuple(v[:2]) for k, v in yaml.safe_load(ROBOTS.read_text(encoding='utf-8'))['robots'].items()}
    busy = ds.load_scene(TASKS / scene, {'collect_priority': '200'})[0].tasks
    idle, owners = dict(robots), {}
    for task in busy:
        pickup = st[task.pickup][:2]
        robot = min(idle, key=lambda r: (math.dist(idle[r], pickup), r))
        trip = math.dist(idle.pop(robot), pickup) + math.dist(pickup, st[task.dropoff][:2])
        owners[robot] = (task.task_id, trip)
    return owners


def test_act1_robot_2_has_the_shortest_start_trip():
    # 연우 시나리오: 켬이면 다음에 일을 마친 2호가 회수에 바로 배정된다 → 2호 busy 작업이 가장 짧아야 한다
    owners = _busy_owners('demo_s2a.yaml')
    assert [owners[r][0] for r in ('amr_05', 'amr_04', 'amr_03', 'amr_02', 'amr_01')] == \
        [t.task_id for t in ds.load_scene(TASKS / 'demo_s2a.yaml', {'collect_priority': '1'})[0].tasks]
    trips = {r: trip for r, (_, trip) in owners.items()}
    assert min(trips, key=trips.get) == 'amr_02'
    assert min(v for r, v in trips.items() if r != 'amr_02') > 1.3 * trips['amr_02']


def test_act2_robot_4_has_the_shortest_start_trip():
    # 켬이면 0초 뒤 다음에 일을 마친 4호가 2인석 배달에 배정된다 → 4호 busy 작업이 가장 짧되 60 s보다 길어야 한다
    owners = _busy_owners('demo_s2b.yaml')
    trips = {r: trip for r, (_, trip) in owners.items()}
    assert min(trips, key=trips.get) == 'amr_04'
    assert trips['amr_04'] > 22.0          # 서빙 로봇 평균 약 0.3 m/s로 60 s를 넘긴다 (리허설 1-2: 9.75 m / 40 s)
    assert min(v for r, v in trips.items() if r != 'amr_04') > 1.1 * trips['amr_04']
