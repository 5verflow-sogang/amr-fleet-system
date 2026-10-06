"""config/zones.yaml 읽기 (zones_config.py). amr_fleet_manager.records.load_zones와 같은 파일을 읽는다."""

import tempfile

from amr_zone_client.zones_config import load_zone_configs

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
