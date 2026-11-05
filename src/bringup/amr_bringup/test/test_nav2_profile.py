"""식당 프로필(footprint)·혼잡도 costmap 필터 오버레이 (amr_bringup.nav2_profile). ROS 없이 돈다."""

import ast
import os

import pytest
import yaml

from amr_bringup.nav2_profile import build_overrides, congestion_filter_overrides, profile_overrides

PARAMS = os.path.join(os.path.dirname(__file__), '..', 'config', 'nav2_params.yaml')


@pytest.fixture(scope='module')
def nav2_params():
    with open(PARAMS) as f:
        return yaml.safe_load(f)


def _footprint_half_extents(footprint_str):
    points = ast.literal_eval(footprint_str)
    return {abs(p[0]) for p in points}, {abs(p[1]) for p in points}


def test_warehouse_profile_adds_nothing(nav2_params):
    assert profile_overrides(nav2_params, 'warehouse') == {}


def test_restaurant_profile_matches_serving_robot_footprint(nav2_params):
    overrides = profile_overrides(nav2_params, 'restaurant')
    assert overrides['global_costmap']['footprint'] == overrides['local_costmap']['footprint']
    xs, ys = _footprint_half_extents(overrides['global_costmap']['footprint'])
    assert xs == pytest.approx({0.565 / 2})
    assert ys == pytest.approx({0.537 / 2})
    assert overrides['DWA']['footprint_half_length'] == pytest.approx(0.565 / 2)
    assert overrides['DWA']['footprint_half_width'] == pytest.approx(0.537 / 2)


def test_unknown_profile_is_rejected(nav2_params):
    with pytest.raises(ValueError):
        profile_overrides(nav2_params, 'cafeteria')


def test_none_filter_adds_nothing():
    assert congestion_filter_overrides('none') == {}


def test_keepout_filter_appends_to_base_plugins_and_uses_congestion_node_topic():
    overrides = congestion_filter_overrides('keepout')
    assert overrides['plugins'] == ['static_layer', 'obstacle_layer', 'inflation_layer', 'keepout_filter']
    assert overrides['keepout_filter']['plugin'] == 'nav2_costmap_2d::KeepoutFilter'
    # 절대 토픽 - 민지 혼잡도 노드(#296)가 내는 이름 그대로, 네임스페이스와 무관
    assert overrides['keepout_filter']['filter_info_topic'] == '/fleet/congestion_keepout_info'


def test_speed_filter_appends_to_base_plugins_and_uses_congestion_node_topic():
    overrides = congestion_filter_overrides('speed')
    assert overrides['plugins'] == ['static_layer', 'obstacle_layer', 'inflation_layer', 'speed_filter']
    assert overrides['speed_filter']['plugin'] == 'nav2_costmap_2d::SpeedFilter'
    assert overrides['speed_filter']['filter_info_topic'] == '/fleet/congestion_speed_info'


def test_unknown_filter_is_rejected():
    with pytest.raises(ValueError):
        congestion_filter_overrides('block')


def test_build_overrides_warehouse_none_keeps_existing_dwa_config_and_empty_costmaps(nav2_params):
    overrides = build_overrides(nav2_params, 'warehouse', 'none', 'odometry/local')
    dwa = nav2_params['controller_server']['ros__parameters']['DWA']
    assert overrides['controller_server']['ros__parameters']['FollowPath'] == dwa
    assert overrides['controller_server']['ros__parameters']['odom_topic'] == 'odometry/local'
    assert overrides['global_costmap']['global_costmap']['ros__parameters'] == {}
    assert overrides['local_costmap']['local_costmap']['ros__parameters'] == {}


def test_build_overrides_restaurant_keepout_combines_footprint_and_filter(nav2_params):
    overrides = build_overrides(nav2_params, 'restaurant', 'keepout', 'odometry/local')
    gc = overrides['global_costmap']['global_costmap']['ros__parameters']
    lc = overrides['local_costmap']['local_costmap']['ros__parameters']
    dwa = overrides['controller_server']['ros__parameters']['FollowPath']
    assert gc['plugins'][-1] == 'keepout_filter'
    assert 'footprint' in gc and gc['footprint'] == lc['footprint']
    assert 'plugins' not in lc  # 필터는 전역 costmap에만 붙는다
    assert dwa['footprint_half_length'] == pytest.approx(0.565 / 2)
    assert dwa['footprint_half_width'] == pytest.approx(0.537 / 2)
    # DWA 블록의 다른 값(footprint 말고)은 기존 nav2_params.yaml 그대로다
    base_dwa = nav2_params['controller_server']['ros__parameters']['DWA']
    assert dwa['max_v'] == base_dwa['max_v']
