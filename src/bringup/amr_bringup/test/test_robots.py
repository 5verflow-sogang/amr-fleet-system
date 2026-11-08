"""로봇 목록·좌표 변환·seed. ROS 없이 돈다."""

import math
from pathlib import Path

import pytest

import yaml

from amr_bringup.robots import (
    cmd_vel_chain, encoder_seed, load_robots, obstacle_clearance, world_to_map, world_to_map_tf_args)

ROBOTS = str(Path(__file__).resolve().parents[1] / 'config' / 'robots.yaml')
# 같은 저장소의 amr_gazebo 동적 장애물 설정 (src/bringup/amr_bringup/test → src/simulation/amr_gazebo)
OBSTACLES = Path(__file__).resolve().parents[3] / 'simulation' / 'amr_gazebo' / 'config' / 'dynamic_obstacles.yaml'
# 로봇 반 길이 0.3 m + 지게차 반 폭 + 여유
CLEARANCE_M = 1.0


def test_shipped_robots_in_waiting_zone():
    robots = load_robots(ROBOTS, 5)
    assert [r.name for r in robots] == ['amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05']
    # 대기 구역 x 16~26, y −20~−14 (amr_gazebo README)
    assert all(16 < r.x < 26 and -20 < r.y < -14 for r in robots)
    # 로봇 몸체 0.6 m보다 넉넉히 떨어져 있다
    xs = sorted(r.x for r in robots)
    assert min(b - a for a, b in zip(xs, xs[1:])) >= 1.5


def test_restaurant_robots_at_wait_stations():
    # 식당 5대 (robots_restaurant.yaml): Fleet 대기 위치 wait_1~5에서 시작하고, 손님·직원 경로와 떨어져 있다
    robots = load_robots(str(Path(ROBOTS).with_name('robots_restaurant.yaml')), 5)
    root = Path(__file__).resolve().parents[3]
    stations = yaml.safe_load((root / 'fleet' / 'amr_fleet_manager' / 'config' / 'stations_restaurant.yaml')
                              .read_text(encoding='utf-8'))['stations']
    assert [(r.x, r.y, r.yaw) for r in robots] == [tuple(stations[f'wait_{i}']) for i in range(1, 6)]
    config = root / 'simulation' / 'amr_gazebo' / 'config'
    for name in ('dynamic_obstacles_restaurant.yaml', 'dynamic_obstacles_restaurant_peak.yaml',
                 'dynamic_obstacles_restaurant_peak_scene12.yaml', 'dynamic_obstacles_restaurant_staff.yaml'):
        obstacles = yaml.safe_load((config / name).read_text(encoding='utf-8'))['obstacles']
        assert all(obstacle_clearance(r.x, r.y, o) >= CLEARANCE_M for r in robots for o in obstacles), name


def free_on_map(pgm: Path, x: float, y: float, half: float = 0.4) -> bool:
    """식당 정답 지도(restaurant_gt, 0.05 m, 원점 [-12.5, -8.5], map = world)에서 (x, y) 둘레 정사각형이 비었는지."""
    data = pgm.read_bytes()
    width, height, maxval = (int(v) for v in data.split(maxsplit=4)[1:4])
    pixels = data[-width * height:]
    cells = int(half / 0.05)
    col, row = int((x + 12.5) / 0.05), height - 1 - int((y + 8.5) / 0.05)
    return all(pixels[(row + dr) * width + col + dc] > maxval // 2
               for dr in range(-cells, cells + 1) for dc in range(-cells, cells + 1))


@pytest.mark.parametrize('scene', ['s11', 's12'])
def test_restaurant_scene_robots(scene):
    # 식당 시연 장면별 시작 위치: 1호는 입구 옆(서쪽 통로 남쪽 끝), 1-1의 3호는 3번 매장 앞. 나머지는 대기 위치
    robots = {r.name: r for r in load_robots(str(Path(ROBOTS).with_name(f'robots_restaurant_{scene}.yaml')), 5)}
    root = Path(__file__).resolve().parents[3]
    stations = yaml.safe_load((root / 'fleet' / 'amr_fleet_manager' / 'config' / 'stations_restaurant.yaml')
                              .read_text(encoding='utf-8'))['stations']
    first = robots['amr_01']
    assert (first.x, first.y) == (-10.5, -5.6) and first.x < -9.73   # 1-1 성립 조건: 1호가 x -9.73보다 서쪽
    special = {'amr_01': None, 'amr_03': 'stall_3'} if scene == 's11' else {'amr_01': None}
    for name, robot in robots.items():
        if name in special:
            if special[name]:
                assert (robot.x, robot.y, robot.yaw) == tuple(stations[special[name]]), name
            continue
        assert (robot.x, robot.y, robot.yaw) in [tuple(v) for k, v in stations.items() if k.startswith('wait_')], name
    points = list(robots.values())
    assert min(math.hypot(a.x - b.x, a.y - b.y) for i, a in enumerate(points) for b in points[i + 1:]) >= 1.0
    # 1호 자리: 피크 손님·직원 경로와 떨어져 있고 지도(테이블·벽)에서 비어 있다
    config = root / 'simulation' / 'amr_gazebo' / 'config'
    for name in ('dynamic_obstacles_restaurant_peak.yaml', 'dynamic_obstacles_restaurant_staff.yaml'):
        obstacles = yaml.safe_load((config / name).read_text(encoding='utf-8'))['obstacles']
        assert all(obstacle_clearance(first.x, first.y, o) >= CLEARANCE_M for o in obstacles), name
    pgm = root / 'localization' / 'amr_mapping' / 'maps' / 'restaurant_gt.pgm'
    assert all(free_on_map(pgm, r.x, r.y) for r in points)



def test_restaurant_scene3_robots_start_in_open_floor_apart_and_off_guest_paths():
    # 장면 3: 다섯 대가 흩어져 북쪽(홀)을 보고 선다. 대기 줄에서 한꺼번에 나가면 동쪽 통로에서 막혔고, 테이블·부스 정차
    # 자리에서 스폰하면 돌아 나가지 못했다 (11/8 맥 스모크). 지도에서 1.2 m 사각형이 비고, 손님·직원 경로와 1.2 m,
    # 정차 자리와 1 m, 서로 2.5 m 넘게 떨어진다
    robots = list(load_robots(str(Path(ROBOTS).with_name('robots_restaurant_s3.yaml')), 5))
    root = Path(__file__).resolve().parents[3]
    stations = yaml.safe_load((root / 'fleet' / 'amr_fleet_manager' / 'config' / 'stations_restaurant.yaml')
                              .read_text(encoding='utf-8'))['stations']
    assert all(r.yaw == pytest.approx(math.pi / 2) for r in robots)
    assert all(math.hypot(r.x - v[0], r.y - v[1]) >= 1.0 for r in robots for v in stations.values())
    assert min(math.hypot(a.x - b.x, a.y - b.y) for i, a in enumerate(robots) for b in robots[i + 1:]) >= 2.5
    config = root / 'simulation' / 'amr_gazebo' / 'config'
    for name in ('dynamic_obstacles_restaurant.yaml', 'dynamic_obstacles_restaurant_peak.yaml',
                 'dynamic_obstacles_restaurant_staff.yaml'):
        obstacles = yaml.safe_load((config / name).read_text(encoding='utf-8'))['obstacles']
        assert all(obstacle_clearance(r.x, r.y, o) >= 1.2 for r in robots for o in obstacles), name
    pgm = root / 'localization' / 'amr_mapping' / 'maps' / 'restaurant_gt.pgm'
    assert all(free_on_map(pgm, r.x, r.y, half=0.6) for r in robots)


def test_load_robots_count():
    assert len(load_robots(ROBOTS, 3)) == 3
    with pytest.raises(ValueError):
        load_robots(ROBOTS, 6)
    with pytest.raises(ValueError):
        load_robots(ROBOTS, 0)


def test_world_to_map():
    # 지도 v0: world (0, 1.5)가 map (0, 0)
    assert world_to_map(17.0, -17.0, math.pi / 2, [0.0, 1.5, 0.0]) == pytest.approx((17.0, -18.5, math.pi / 2))
    x, y, yaw = world_to_map(1.0, 0.0, 0.0, [0.0, 0.0, math.pi / 2])
    assert (x, y, yaw) == pytest.approx((0.0, -1.0, -math.pi / 2))


def test_encoder_seed():
    # 0은 amr_odometry에서 "매번 무작위"라 그대로 넘긴다
    assert encoder_seed(0, 3) == 0
    seeds = [encoder_seed(s, i) for s in (1, 2) for i in range(5)]
    assert len(set(seeds)) == len(seeds)
    assert all(v > 0 for v in seeds)


def test_obstacle_clearance():
    random_area = {'pattern': 'random', 'waypoints': [{'x': 5.0, 'y': -13.0}, {'x': 18.0, 'y': -17.0}]}
    assert obstacle_clearance(17.0, -15.0, random_area) == 0.0          # 범위 안
    assert obstacle_clearance(17.0, -19.0, random_area) == pytest.approx(2.0)
    path = {'pattern': 'straight', 'waypoints': [{'x': 0.0, 'y': 0.0}, {'x': 10.0, 'y': 0.0}]}
    assert obstacle_clearance(5.0, 3.0, path) == pytest.approx(3.0)
    assert obstacle_clearance(-4.0, 3.0, path) == pytest.approx(5.0)    # 끝점까지


def test_spawns_clear_of_dynamic_obstacles():
    # dynamic_obstacles:=true일 때 서 있는 로봇을 장애물이 치지 않게 한다 (#95 지수 리뷰: forklift_2와 겹쳤음)
    if not OBSTACLES.exists():
        pytest.skip('amr_gazebo 설정이 없다 (패키지만 따로 받은 경우)')
    obstacles = yaml.safe_load(OBSTACLES.read_text(encoding='utf-8'))['obstacles']
    for robot in load_robots(ROBOTS, 5):
        for obstacle in obstacles:
            clearance = obstacle_clearance(robot.x, robot.y, obstacle)
            assert clearance >= CLEARANCE_M, (robot.name, obstacle['id'], round(clearance, 2))


@pytest.mark.parametrize('zone_client, safety, expected', [
    # 둘 다 없으면 Nav2가 Gazebo로 바로 낸다
    (False, False, ('cmd_vel', 'cmd_vel', 'cmd_vel_nav')),
    # 구역 클라이언트만: 마지막 단계라 cmd_vel을 낸다 (2단계)
    (True, False, ('cmd_vel_nav', 'cmd_vel', 'cmd_vel_zone_limited')),
    # 안전 노드만: Nav2 출력을 안전 노드가 받는다
    (False, True, ('cmd_vel_nav', 'cmd_vel_zone_limited', 'cmd_vel_nav')),
    # 전부 (3단계): 최종 cmd_vel은 안전 노드만 낸다
    (True, True, ('cmd_vel_nav', 'cmd_vel_zone_limited', 'cmd_vel_zone_limited')),
])
def test_cmd_vel_chain(zone_client, safety, expected):
    chain = cmd_vel_chain(zone_client, safety)
    assert (chain['nav_out'], chain['zone_out'], chain['safety_in']) == expected


def test_cmd_vel_chain_has_single_final_publisher():
    # 어떤 조합이든 /cmd_vel을 내는 노드는 하나다 (Nav2 · 구역 클라이언트 · 안전 노드 중 마지막 단계)
    for zone_client in (False, True):
        for safety in (False, True):
            chain = cmd_vel_chain(zone_client, safety)
            outputs = [chain['nav_out']]
            if zone_client:
                outputs.append(chain['zone_out'])
            if safety:
                outputs.append('cmd_vel')
            assert outputs.count('cmd_vel') == 1, (zone_client, safety, outputs)


def test_world_to_map_tf_args():
    assert world_to_map_tf_args([0.0, 1.5, 0.0]) == [
        '--x', '0.0', '--y', '1.5', '--z', '0.0', '--yaw', '0.0', '--frame-id', 'world', '--child-frame-id', 'map']
    # 문자열 원점(launch 인자를 yaml로 읽은 값)도 받는다
    assert world_to_map_tf_args(['-0.0562', '1.5317', '0.00009'])[1:8:2] == ['-0.0562', '1.5317', '0.0', '9e-05']


def test_scene12_first_robot_faces_east_and_scene11_faces_north():
    # 1-2: 혼잡도를 켜면 1호가 남쪽 통로를 따라 동쪽(x -7.275 우회 통로)으로 출발한다 → 동쪽을 보고 선다. 1-1은 그대로 북쪽
    s12 = {r.name: r for r in load_robots(str(Path(ROBOTS).with_name('robots_restaurant_s12.yaml')), 5)}
    s11 = {r.name: r for r in load_robots(str(Path(ROBOTS).with_name('robots_restaurant_s11.yaml')), 5)}
    assert (s12['amr_01'].x, s12['amr_01'].y, s12['amr_01'].yaw) == (-10.5, -5.6, 0.0)
    assert s11['amr_01'].yaw == pytest.approx(math.pi / 2)



def test_scene3_gazebo_starts_clear_of_furniture_and_each_other():
    robots = load_robots(str(Path(ROBOTS).with_name('robots_restaurant_s3_gazebo.yaml')), 5)
    pgm = Path(__file__).resolve().parents[3] / 'localization/amr_mapping/maps/restaurant_gt.pgm'
    assert all(free_on_map(pgm, r.x, r.y, half=0.4) for r in robots)
    assert all(math.hypot(a.x - b.x, a.y - b.y) >= 1.5
               for i, a in enumerate(robots) for b in robots[i + 1:])


def test_scene3_gazebo_service_stops_allow_turning_on_the_map():
    root = Path(__file__).resolve().parents[3]
    stations = yaml.safe_load((root / 'fleet/amr_fleet_manager/config/stations_restaurant_s3.yaml').read_text())['stations']
    pgm = root / 'localization/amr_mapping/maps/restaurant_gt.pgm'
    used = ('c01', 'c02', 'stall_1', 'stall_2', 't02', 't03', 'return_1', 'return_2',
            'charger_c1', 'charger_c2', 'charger_c3')
    assert all(free_on_map(pgm, *stations[name][:2], half=0.35) for name in used)
