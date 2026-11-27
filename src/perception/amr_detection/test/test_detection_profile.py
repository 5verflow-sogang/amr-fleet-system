"""식당 인식 프로필(config/detection_restaurant.yaml, detection.launch.py profile:=restaurant).

설정 값 확인은 파일만 읽어 ROS 없이 돈다. launch 배선(파일 목록)은 패키지가 설치된 곳(CI 컨테이너)에서만 돈다.
"""

import importlib.util
import os

import pytest
import yaml

from amr_detection.detector import build_class_map

CONFIG_DIR = os.path.join(os.path.dirname(__file__), '..', 'config')


def params(name):
    with open(os.path.join(CONFIG_DIR, name)) as f:
        return yaml.safe_load(f)['/**/detection']['ros__parameters']


@pytest.fixture(scope='module')
def base():
    return params('detection.yaml')


@pytest.fixture(scope='module')
def restaurant():
    return params('detection_restaurant.yaml')


def test_restaurant_overrides_only_existing_keys(base, restaurant):
    # 오타로 새 키를 만들면 노드가 모르는 파라미터라 조용히 무시된다
    assert set(restaurant) <= set(base)


def test_restaurant_uses_coco_model_for_people_only(restaurant):
    # Dockerfile이 받는 COCO 원본 (README "모델"). 식당에는 화물·표지판이 없어 사람만 낸다
    assert restaurant['model_path'] == '/opt/models/yolov8n.pt'
    assert build_class_map(restaurant['model_classes'], restaurant['output_classes']) == {'person': 'person'}


def test_restaurant_keeps_thresholds_and_depth(base, restaurant):
    # 바뀌는 것은 모델과 클래스뿐이다. 신뢰도·NMS·깊이·공분산은 물류센터와 같다
    assert set(restaurant) == {'model_path', 'model_classes', 'output_classes'}


def load_launch_module():
    pytest.importorskip('launch_ros')
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('amr_detection'), 'launch', 'detection.launch.py')
    spec = importlib.util.spec_from_file_location('detection_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('profile, last', [('warehouse', 'detection.yaml'), ('restaurant', 'detection_restaurant.yaml')])
def test_launch_profile_picks_config_files(profile, last):
    files = load_launch_module().config_files(profile)
    assert os.path.basename(files[0]) == 'detection.yaml'
    assert os.path.basename(files[-1]) == last
    assert all(os.path.exists(f) for f in files)
