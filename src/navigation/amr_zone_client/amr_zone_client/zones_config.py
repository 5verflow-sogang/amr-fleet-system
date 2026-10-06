"""config/zones.yaml(amr_fleet_manager와 같은 파일) 읽기. ROS 비의존.

area_world만 쓴다 (좌표계 이름은 노드 파라미터로 따로 둔다. README '좌표계' 참고). capacity 등 나머지는
서버(amr_fleet_manager.records.load_zones) 몫이라 여기서는 읽지 않는다.
"""

from __future__ import annotations

from pathlib import Path
from typing import List

import yaml

from .core import ZoneConfig


def load_zone_configs(path: str) -> List[ZoneConfig]:
    """경로가 비면 빈 목록 (트리거할 구역이 없다는 뜻. amr_fleet_manager.records.load_zones와 같은 규칙)."""
    if not path:
        return []
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
    configs = []
    for zone in data.get('zones') or []:
        x_min, x_max, y_min, y_max = (float(v) for v in zone['area_world'])
        configs.append(ZoneConfig(str(zone['id']), (x_min, x_max, y_min, y_max)))
    return configs
