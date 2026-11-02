"""config/extrinsics.yaml의 센서 위치가 xacro 결과 URDF의 고정 조인트와 같은지 본다.

URDF가 extrinsic 파일을 읽지 않고 값을 따로 갖게 되면(파일만 고치고 TF는 그대로) 여기서 잡는다.
"""

import os
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PREFIX = 'amr_01/'


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
