"""nav2.launch.py의 _nav2()가 nav2_profile·congestion_filter 조합으로도 그냥 도는지.

ROS 환경(launch_ros)이 설치된 곳(CI 컨테이너)에서만 돈다 - test_system_launch.py와 같은 이유.
오버레이 값 자체(footprint·costmap_filters 블록)의 정확성은 ROS 없이 도는
amr_bringup.nav2_profile 쪽(test_nav2_profile.py)이 본다 - 여기서는 그 값을 실제
launch_ros.actions.Node 구성으로 엮는 배선만 깨지지 않는지 확인한다(Node.parameters는
launch_ros 내부 표현이라 직접 들여다보지 않는다).
"""

import importlib.util
import itertools
import os

import pytest

pytest.importorskip('launch_ros')
pytest.importorskip('ament_index_python')
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from launch import LaunchContext  # noqa: E402
from launch.utilities import perform_substitutions  # noqa: E402
from launch_ros.actions import Node  # noqa: E402

from amr_bringup.nav2_profile import CONGESTION_FILTERS, PROFILES  # noqa: E402


def load_module():
    path = os.path.join(get_package_share_directory('amr_bringup'), 'launch', 'nav2.launch.py')
    spec = importlib.util.spec_from_file_location('nav2_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context_with_defaults(module, **overrides):
    context = LaunchContext()
    for declared in module.generate_launch_description().get_launch_arguments():
        context.launch_configurations[declared.name] = perform_substitutions(context, declared.default_value)
    context.launch_configurations.update({k: str(v) for k, v in overrides.items()})
    return context


@pytest.mark.parametrize('profile,filter_choice', itertools.product(PROFILES, CONGESTION_FILTERS))
def test_nav2_builds_for_every_profile_and_filter_combination(profile, filter_choice):
    module = load_module()
    nodes = module._nav2(context_with_defaults(
        module, nav2_profile=profile, congestion_filter=filter_choice))
    # planner_server, controller_server, behavior_server, bt_navigator, lifecycle_manager
    assert len(nodes) == 5
    assert all(isinstance(n, Node) for n in nodes)


def test_nav2_composition_puts_five_servers_in_one_container():
    from launch_ros.actions import ComposableNodeContainer

    module = load_module()
    actions = module._nav2(context_with_defaults(module, nav2_profile='restaurant', use_composition='true'))
    assert len(actions) == 1 and isinstance(actions[0], ComposableNodeContainer)
    descriptions = actions[0]._ComposableNodeContainer__composable_node_descriptions
    plugins = [perform_substitutions(LaunchContext(), d.node_plugin) for d in descriptions]
    assert plugins == ['nav2_planner::PlannerServer', 'nav2_controller::ControllerServer',
                       'behavior_server::BehaviorServer', 'nav2_bt_navigator::BtNavigator',
                       'nav2_lifecycle_manager::LifecycleManager']
