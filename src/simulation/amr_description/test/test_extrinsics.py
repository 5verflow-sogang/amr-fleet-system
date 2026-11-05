"""config/extrinsics.yaml의 센서 위치가 xacro 결과 URDF의 고정 조인트와 같은지 본다.

URDF가 extrinsic 파일을 읽지 않고 값을 따로 갖게 되면(파일만 고치고 TF는 그대로) 여기서 잡는다.
"""

import os
import sys
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PREFIX = 'amr_01/'

sys.path.insert(0, PKG)   # 설치 전 소스의 amr_description/extrinsics.py
from amr_description.extrinsics import BASE_LINK_HEIGHT, lidar_height  # noqa: E402


@pytest.fixture(scope='module')
def extrinsics():
    with open(os.path.join(PKG, 'config', 'extrinsics.yaml'), encoding='utf-8') as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def joints():
    doc = xacro.process_file(os.path.join(PKG, 'urdf', 'amr.urdf.xacro'), mappings={'namespace': 'amr_01'})
    root = ET.fromstring(doc.toxml())
    return {j.find('child').get('link'): j for j in root.iter('joint')}


def _floats(text):
    return [float(v) for v in text.split()]


@pytest.mark.parametrize('sensor', ['lidar', 'camera', 'imu'])
def test_sensor_joint_matches_extrinsics(extrinsics, joints, sensor):
    e = extrinsics[sensor]
    joint = joints[PREFIX + e['child']]
    assert joint.get('type') == 'fixed'
    assert joint.find('parent').get('link') == PREFIX + e['parent']
    origin = joint.find('origin')
    assert _floats(origin.get('xyz')) == pytest.approx(e['xyz'])
    assert _floats(origin.get('rpy')) == pytest.approx(e['rpy'])


def test_every_entry_has_pose(extrinsics):
    for name, e in extrinsics.items():
        assert set(e) == {'parent', 'child', 'xyz', 'rpy'}, name
        assert len(e['xyz']) == 3 and len(e['rpy']) == 3, name


def test_base_link_height_matches_urdf(joints):
    # extrinsics.lidar_height()가 쓰는 base_link 높이 = URDF base_footprint → base_link
    origin = joints[PREFIX + 'base_link'].find('origin')
    assert _floats(origin.get('xyz'))[2] == pytest.approx(BASE_LINK_HEIGHT)


def test_lidar_height_from_file(extrinsics):
    assert lidar_height(extrinsics) == pytest.approx(BASE_LINK_HEIGHT + extrinsics['lidar']['xyz'][2])


def test_lidar_is_level(extrinsics):
    # 인지(군집화·추적)·안전 노드는 스캔을 2D로 옮기며 TF의 yaw·x·y만 쓴다. roll·pitch를 넣으면
    # 틀린 위치를 조용히 낸다 (#266 연우 리뷰)
    roll, pitch, _ = extrinsics['lidar']['rpy']
    assert (roll, pitch) == (0.0, 0.0)


def test_serving_camera_matches_extrinsics(extrinsics):
    """body:=serving이면 카메라 조인트가 camera_serving 값을 쓴다 (serving.xacro 타워 머리)."""
    doc = xacro.process_file(os.path.join(PKG, 'urdf', 'amr.urdf.xacro'),
                             mappings={'namespace': 'amr_01', 'body': 'serving'})
    joints = {j.find('child').get('link'): j for j in ET.fromstring(doc.toxml()).iter('joint')}
    e = extrinsics['camera_serving']
    origin = joints[PREFIX + e['child']].find('origin')
    assert _floats(origin.get('xyz')) == pytest.approx(e['xyz'])
    assert _floats(origin.get('rpy')) == pytest.approx(e['rpy'])
