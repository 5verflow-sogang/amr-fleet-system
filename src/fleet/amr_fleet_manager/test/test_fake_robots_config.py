"""가짜 로봇 시작 위치 설정 (config/fake_robots.yaml, fake_robots_restaurant.yaml). ROS 없이 돈다."""

import re
from pathlib import Path

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / 'config' / 'fake_robots.yaml'
RESTAURANT = PACKAGE / 'config' / 'fake_robots_restaurant.yaml'
LAUNCH = PACKAGE / 'launch' / 'fake_robots.launch.py'


def load(path):
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def start_positions(path=CONFIG):
    return {key.split('/')[1]: tuple(value['ros__parameters']['start_xy'])
            for key, value in load(path).items() if key.endswith('/fake_robot')}


def test_every_robot_up_to_max_has_a_start_position():
    # launch의 count 상한과 설정의 로봇 수가 같아야 한다 (위치가 없으면 원점에 겹친다)
    max_robots = int(re.search(r'^MAX_ROBOTS = (\d+)$', LAUNCH.read_text(encoding='utf-8'), re.M).group(1))
    assert sorted(start_positions()) == [f'amr_{i:02d}' for i in range(1, max_robots + 1)]


def test_start_positions_do_not_overlap():
    positions = list(start_positions().values())
    assert len(set(positions)) == len(positions)


def test_restaurant_has_five_robots_inside_the_hall():
    # 식당 월드는 x -12~12, y -8~8 (m). 관제 대시보드 시험은 5대
    positions = start_positions(RESTAURANT)
    assert sorted(positions) == [f'amr_{i:02d}' for i in range(1, 6)]
    assert len(set(positions.values())) == len(positions)
    assert all(-12 < x < 12 and -8 < y < 8 for x, y in positions.values())


def test_restaurant_keeps_the_common_parameters():
    # 시작 위치만 다르고 속도·대기 시간 등은 기본 파일과 같아야 시험 조건이 같다
    assert load(RESTAURANT)['/**'] == load(CONFIG)['/**']
