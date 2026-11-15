"""식당 안전 프로필(config/safety_restaurant.yaml, safety.launch.py profile:=restaurant).

설정 값 확인은 파일만 읽어 ROS 없이 돈다. launch 배선(파일 목록)은 패키지가 설치된 곳(CI 컨테이너)에서만 돈다.
"""

import dataclasses
import importlib.util
import os

import pytest
import yaml

from amr_perception_core.safety import SafetyConfig

CONFIG_DIR = os.path.join(os.path.dirname(__file__), '..', 'config')


def params(name):
    with open(os.path.join(CONFIG_DIR, name)) as f:
        return yaml.safe_load(f)['/**/safety']['ros__parameters']


@pytest.fixture(scope='module')
def base():
    return params('safety.yaml')


@pytest.fixture(scope='module')
def restaurant():
    return params('safety_restaurant.yaml')


def test_restaurant_overrides_only_existing_keys(base, restaurant):
    # 오타로 새 키를 만들면 노드가 모르는 파라미터라 조용히 무시된다
    assert set(restaurant) <= set(base)


def test_restaurant_keeps_stop_rules_and_warning_limit(restaurant):
    # 정지(진행 경로 0.3 m·TTC)와 0.6 m/s 상한은 물류센터와 같다
    for key in ('critical_distance', 'ttc_critical', 'ttc_warning', 'warning_speed_limit', 'side_margin'):
        assert key not in restaurant, key


def test_restaurant_body_is_serving_robot(restaurant):
    # amr_bringup nav2_profile 식당 footprint와 같은 서빙 로봇 크기 (#299)
    assert (restaurant['robot_length'], restaurant['robot_width']) == pytest.approx((0.565, 0.537))


def test_restaurant_values_pass_safety_config_checks(base, restaurant):
    merged = {**base, **restaurant}
    fields = {f.name for f in dataclasses.fields(SafetyConfig)}
    config = SafetyConfig(**{k: v for k, v in merged.items() if k in fields})
    assert config.dynamic_slow_distance == pytest.approx(0.4)
    assert config.dynamic_slow_speed_limit == pytest.approx(0.3)
    assert config.dynamic_warning_distance == pytest.approx(1.0)


def load_launch_module():
    pytest.importorskip('launch_ros')
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('amr_safety'), 'launch', 'safety.launch.py')
    spec = importlib.util.spec_from_file_location('safety_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('profile, last', [('warehouse', 'safety.yaml'), ('restaurant', 'safety_restaurant.yaml')])
def test_launch_profile_picks_config_files(profile, last):
    files = load_launch_module().config_files(profile)
    assert os.path.basename(files[0]) == 'safety.yaml'
    assert os.path.basename(files[-1]) == last
    assert all(os.path.exists(f) for f in files)
