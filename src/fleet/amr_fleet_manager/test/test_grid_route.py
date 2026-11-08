"""가짜 로봇 지도 경로 (amr_fleet_manager.grid_route)와 경로를 따라가는 TaskRun. ROS 없이 돈다."""

import math
from pathlib import Path

import pytest
import yaml

from amr_fleet_manager.grid_route import GridRoute
from amr_fleet_manager.robot_sim import TaskRun

PKG = Path(__file__).resolve().parents[1]
RESTAURANT_MAP = PKG.parents[1] / 'localization' / 'amr_mapping' / 'maps' / 'restaurant_gt.yaml'
STATIONS = PKG / 'config' / 'stations_restaurant.yaml'


def wall_with_gap():
    """6 x 4 m (0.1 m 칸). x 3.0~3.2 m에 세로 벽, y 3.0~3.8 m만 틈."""
    blocked = [[2.95 < (i + 0.5) * 0.1 < 3.25 and not 3.0 < (j + 0.5) * 0.1 < 3.8 for i in range(60)] for j in range(40)]
    return GridRoute(blocked, 0.1, (0.0, 0.0), clearance_m=0.25, prefer_m=0.4)


def length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def samples(points, step=0.05):
    for a, b in zip(points, points[1:]):
        n = max(1, int(math.dist(a, b) / step))
        for k in range(n + 1):
            yield (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)


def test_route_goes_through_the_gap_and_keeps_clear_of_the_wall():
    g = wall_with_gap()
    start, goal = (1.0, 1.0), (5.0, 1.0)
    path = [start] + g.route(start, goal)
    assert path[-1] == goal
    assert any(3.0 < y < 3.8 for x, y in path if 2.5 < x < 3.7)            # 틈으로 지나간다
    assert all(g.passable(g.cell(p)) for p in samples(path))
    assert length(path) > math.dist(start, goal) + 1.5                     # 직선(4 m)보다 돌아간다


def test_route_without_any_way_falls_back_to_a_straight_line():
    blocked = [[2.95 < (i + 0.5) * 0.1 < 3.25 for i in range(60)] for _ in range(40)]   # 틈 없는 벽
    g = GridRoute(blocked, 0.1, (0.0, 0.0), clearance_m=0.25)
    assert g.route((1.0, 1.0), (5.0, 1.0)) == [(5.0, 1.0)]


def test_task_run_follows_the_route_and_reports_the_new_path():
    g = wall_with_gap()
    run = TaskRun((1.0, 1.0), pickup=None, dropoff=(5.0, 1.0), speed=1.0, dwell_s=0.0, route=g.route)
    run.step(0.1)
    path = run.new_path
    assert path[0] == (1.0, 1.0) and path[-1] == (5.0, 1.0) and len(path) > 2
    for _ in range(400):
        if run.done:
            break
        run.step(0.1)
    assert run.done and run.xy == (5.0, 1.0)
    assert run.distance == pytest.approx(length(path), rel=1e-6)


def test_task_run_without_route_is_still_a_straight_line():
    run = TaskRun((0.0, 0.0), pickup=None, dropoff=(2.0, 0.0), speed=1.0, dwell_s=0.0)
    run.step(0.5)
    assert run.new_path == [(0.0, 0.0), (2.0, 0.0)]
    run.step(2.0)
    assert run.done and run.distance == pytest.approx(2.0)


@pytest.fixture(scope='module')
def restaurant():
    return GridRoute.from_map_yaml(RESTAURANT_MAP)


@pytest.fixture(scope='module')
def stations():
    return {k: (v[0], v[1]) for k, v in yaml.safe_load(STATIONS.read_text(encoding='utf-8'))['stations'].items()}


@pytest.mark.parametrize('a, b', [('stall_4', 'c02'), ('stall_3', 'L03'), ('t13', 'return_1'), ('t15', 'charger_c1'),
                                  ('stall_1', 't14'), ('wait_3', 'stall_2')])
def test_restaurant_routes_stay_off_tables_and_walls(restaurant, stations, a, b):
    # 장면 3 작업 자리끼리: 정차 자리(가구 앞)에서 가장 가까운 빈 칸까지 이어 붙인 첫·끝 구간을 빼면, 경로가 벽·테이블에서
    # 로봇 반폭 넘게 떨어져 있다. 이어 붙인 구간은 짧다
    start, goal = stations[a], stations[b]
    path = [start] + restaurant.route(start, goal)
    assert path[-1] == goal and len(path) >= 3
    assert math.dist(path[0], path[1]) < 0.8 and math.dist(path[-2], path[-1]) < 0.8, (a, b)
    # 경로 점 사이를 0.05 m 간격으로 찍으면 칸 경계에 걸린 점이 있어 칸 반 크기만큼 여유를 둔다
    clear = restaurant.clearance_m - restaurant.cell_m / 2
    assert all(restaurant.dist[c[1]][c[0]] >= clear for c in map(restaurant.cell, samples(path[1:-1]))), (a, b)
