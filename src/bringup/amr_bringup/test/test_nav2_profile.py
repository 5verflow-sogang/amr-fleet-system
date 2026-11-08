"""식당 프로필(footprint·순항 속도)·혼잡도 costmap 필터 오버레이 (amr_bringup.nav2_profile). ROS 없이 돈다."""

import ast
import copy
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
    assert sorted(xs) == pytest.approx([0.565 / 2])
    assert sorted(ys) == pytest.approx([0.537 / 2])
    assert overrides['DWA']['footprint_half_length'] == pytest.approx(0.565 / 2)
    assert overrides['DWA']['footprint_half_width'] == pytest.approx(0.537 / 2)
    # 순항 속도는 식당만 0.8 (물류센터 0.5는 Nav KPI 기준이라 그대로)
    assert overrides['DWA']['max_v'] == pytest.approx(0.8)
    assert nav2_params['controller_server']['ros__parameters']['DWA']['max_v'] == pytest.approx(0.5)


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
    # 지정 안 하면 speed_limit_topic은 그대로 - SpeedFilter 기본값(costmap 네임스페이스 기준)을 쓴다
    assert 'speed_limit_topic' not in overrides['speed_filter']


def test_speed_filter_speed_limit_topic_can_be_pointed_at_controller():
    # controller_server 기본 구독 토픽(<ns>/speed_limit)과 SpeedFilter 기본 발행 토픽
    # (<ns>/global_costmap/speed_limit)이 서로 달라 안 맞으면 평생 제한이 안 걸린다(실측 확인) -
    # nav2.launch.py가 로봇 네임스페이스로 만든 절대 경로를 넘겨줘야 한다
    overrides = congestion_filter_overrides('speed', speed_limit_topic='/amr_01/speed_limit')
    assert overrides['speed_filter']['speed_limit_topic'] == '/amr_01/speed_limit'


def test_keepout_filter_ignores_speed_limit_topic():
    # keepout은 속도 제한과 무관 - speed_limit_topic을 줘도 아무 효과 없어야 한다
    overrides = congestion_filter_overrides('keepout', speed_limit_topic='/amr_01/speed_limit')
    assert 'speed_limit_topic' not in overrides['keepout_filter']


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
    assert dwa['max_v'] == pytest.approx(0.8)
    # DWA 블록의 다른 값(footprint·max_v 말고)은 기존 nav2_params.yaml 그대로다
    base_dwa = nav2_params['controller_server']['ros__parameters']['DWA']
    assert dwa['max_w'] == base_dwa['max_w']


@pytest.mark.parametrize('filter_choice', ['none', 'keepout', 'speed'])
def test_restaurant_scan_height_reaches_both_costmaps_without_changing_base(nav2_params, filter_choice):
    original = copy.deepcopy(nav2_params)
    overrides = build_overrides(nav2_params, 'restaurant', filter_choice, 'odometry/local')
    # 최종 YAML 직렬화 이후에도 관측 소스(scan) 아래에 높이가 전달되어야 한다.
    restored = yaml.safe_load(yaml.safe_dump(overrides))
    for costmap in ('global_costmap', 'local_costmap'):
        params = restored[costmap][costmap]['ros__parameters']
        assert params['obstacle_layer']['scan']['max_obstacle_height'] == 2.0
    assert nav2_params == original
    warehouse = build_overrides(nav2_params, 'warehouse', filter_choice, 'odometry/local')
    for costmap in ('global_costmap', 'local_costmap'):
        assert 'obstacle_layer' not in warehouse[costmap][costmap]['ros__parameters']


def test_build_overrides_speed_filter_threads_speed_limit_topic_through(nav2_params):
    overrides = build_overrides(
        nav2_params, 'warehouse', 'speed', 'odometry/local', speed_limit_topic='/amr_01/speed_limit')
    gc = overrides['global_costmap']['global_costmap']['ros__parameters']
    assert gc['speed_filter']['speed_limit_topic'] == '/amr_01/speed_limit'
