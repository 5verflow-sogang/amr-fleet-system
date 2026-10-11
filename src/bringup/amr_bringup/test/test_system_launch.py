"""system.launch.py를 불러 인자와 로봇별 액션이 만들어지는지 본다. 설치된 ROS 워크스페이스(CI)에서만 돈다."""

import importlib.util
import os

import pytest

# ROS 환경에서만 있는 모듈로 판단한다 (이름이 같은 다른 'launch' 패키지가 있을 수 있다)
pytest.importorskip('launch_ros')
pytest.importorskip('ament_index_python')
import launch  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from launch.actions import TimerAction  # noqa: E402
from launch.utilities import perform_substitutions  # noqa: E402
from launch_ros.actions import Node  # noqa: E402

# 이 launch가 include하는 파일. 이름이 바뀌면 여기서 먼저 깨진다
INCLUDED = [
    ('amr_gazebo', 'world.launch.py'),
    ('amr_gazebo', 'dynamic_obstacles.launch.py'),
    ('amr_gazebo', 'spawn_robot.launch.py'),
    ('amr_odometry', 'odometry.launch.py'),
    ('amr_odometry', 'sensor_filters.launch.py'),
    ('amr_localization', 'localization.launch.py'),
    ('amr_bt', 'amr_bt.launch.py'),
    ('amr_fleet_manager', 'fleet_manager.launch.py'),
]


def load_module():
    path = os.path.join(get_package_share_directory('amr_bringup'), 'launch', 'system.launch.py')
    spec = importlib.util.spec_from_file_location('system_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context_with_defaults(module, **overrides):
    context = launch.LaunchContext()
    for declared in module.generate_launch_description().get_launch_arguments():
        context.launch_configurations[declared.name] = perform_substitutions(context, declared.default_value)
    context.launch_configurations.update({k: str(v) for k, v in overrides.items()})
    return context


def test_included_launch_files_exist():
    for package, name in INCLUDED:
        assert os.path.exists(os.path.join(get_package_share_directory(package), 'launch', name)), (package, name)


@pytest.mark.parametrize('num_robots', [1, 5])
def test_actions_per_robot(num_robots):
    module = load_module()
    actions = module._system(context_with_defaults(module, num_robots=num_robots))
    # 월드 1 + 로봇마다 TimerAction 1 + Fleet 서버 include 1 + 장애 주입기 Node 1
    assert len(actions) == 1 + num_robots + 2
    assert sum(isinstance(a, TimerAction) for a in actions) == num_robots
    assert sum(isinstance(a, Node) for a in actions) == 1


def test_options_off():
    module = load_module()
    actions = module._system(context_with_defaults(module, num_robots=2, fleet='false', dynamic_obstacles='true'))
    # 월드 + 동적 장애물 + 로봇 2대, Fleet 없음
    assert len(actions) == 1 + 1 + 2
    assert not any(isinstance(a, Node) for a in actions)
