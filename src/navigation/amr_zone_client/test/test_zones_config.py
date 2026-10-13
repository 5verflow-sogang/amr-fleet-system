"""config/zones.yaml 읽기 (zones_config.py). amr_fleet_manager.records.load_zones와 같은 파일을 읽는다."""

import math
import tempfile

import pytest

from amr_zone_client.core import ZoneConfig
from amr_zone_client.zones_config import load_zone_configs, transform_zones

ZONES_YAML = """
zones:
  - id: narrow_01
    capacity: 1
    area_world: [-5.0, -4.4, -2.6, -1.4]
  - id: narrow_02
    capacity: 1
    area_world: [-4.4, 0.6, -1.4, -0.8]
"""


def test_load_zone_configs():
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
        f.write(ZONES_YAML)
        path = f.name

    zones = load_zone_configs(path)
    assert [z.zone_id for z in zones] == ['narrow_01', 'narrow_02']
    assert zones[0].area == (-5.0, -4.4, -2.6, -1.4)


def test_load_zone_configs_empty_path():
    assert load_zone_configs('') == []


def test_transform_zones_world_to_map_translation():
    # 지도 원점이 world (0, 1.5): map = world - (0, 1.5) → TF map ← world 평행이동 (0, -1.5)
    [narrow_02] = transform_zones([ZoneConfig('narrow_02', (-4.4, 0.6, -1.4, -0.8))], 0.0, -1.5, 0.0)
    assert narrow_02.zone_id == 'narrow_02'
    assert narrow_02.area == pytest.approx((-4.4, 0.6, -2.9, -2.3))


def test_transform_zones_rotation_keeps_bounding_box():
    # 90도 회전: (x, y) → (-y, x). 1×2 사각형이 2×1로
    [z] = transform_zones([ZoneConfig('z', (0.0, 1.0, 0.0, 2.0))], 0.0, 0.0, math.pi / 2)
    assert z.area == pytest.approx((-2.0, 0.0, 0.0, 1.0))
