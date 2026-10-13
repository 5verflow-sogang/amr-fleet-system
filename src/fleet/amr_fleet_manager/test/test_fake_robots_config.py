"""가짜 로봇 시작 위치 설정 (config/fake_robots.yaml). ROS 없이 돈다."""

import re
from pathlib import Path

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / 'config' / 'fake_robots.yaml'
LAUNCH = PACKAGE / 'launch' / 'fake_robots.launch.py'


def start_positions():
    data = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    return {key.split('/')[1]: tuple(value['ros__parameters']['start_xy'])
            for key, value in data.items() if key.endswith('/fake_robot')}


def test_every_robot_up_to_max_has_a_start_position():
    # launch의 count 상한과 설정의 로봇 수가 같아야 한다 (위치가 없으면 원점에 겹친다)
    max_robots = int(re.search(r'^MAX_ROBOTS = (\d+)$', LAUNCH.read_text(encoding='utf-8'), re.M).group(1))
    assert sorted(start_positions()) == [f'amr_{i:02d}' for i in range(1, max_robots + 1)]


def test_start_positions_do_not_overlap():
    positions = list(start_positions().values())
    assert len(set(positions)) == len(positions)
