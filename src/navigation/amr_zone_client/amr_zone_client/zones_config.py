"""config/zones.yaml(amr_fleet_manager와 같은 파일) 읽기. ROS 비의존.

area_world만 쓴다 (좌표계 이름은 노드 파라미터로 따로 둔다. README '좌표계' 참고). capacity 등 나머지는
서버(amr_fleet_manager.records.load_zones) 몫이라 여기서는 읽지 않는다.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import List, Sequence

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


def transform_zones(configs: Sequence[ZoneConfig], tx: float, ty: float, yaw: float) -> List[ZoneConfig]:
    """구역 범위를 다른 좌표계로 옮긴다. p' = R(yaw)·p + (tx, ty). 회전하면 네 모서리를 감싸는 축 정렬 사각형.

    zones.yaml의 area_world는 Gazebo world 좌표라, map 좌표로 판단하는 로봇은 world → map 변환
    (TF map ← world)을 거쳐야 한다. 지도 원점이 world (0, 1.5)라 변환하지 않으면 구역이 1.5 m 어긋난다.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for z in configs:
        x_min, x_max, y_min, y_max = z.area
        corners = [(c * x - s * y + tx, s * x + c * y + ty) for x in (x_min, x_max) for y in (y_min, y_max)]
        xs, ys = [p[0] for p in corners], [p[1] for p in corners]
        out.append(ZoneConfig(z.zone_id, (min(xs), max(xs), min(ys), max(ys))))
    return out
