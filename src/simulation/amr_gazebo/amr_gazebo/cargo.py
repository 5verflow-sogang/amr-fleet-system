"""화물 상자 배치 (config/cargo_boxes.yaml) → Gazebo 모델 SDF 하나.

YOLO 학습·평가용 (#99 연우 요청). 상자는 static이고 위치는 yaml 좌표 그대로라 GT 라벨로 쓸 수 있다.
LiDAR 스캔 높이(지면 0.38 m)에 걸리지 않게 놓는다. 걸리면 지도 v1에 없는 물체가 스캔에 찍혀
AMCL·map_quality가 어긋난다 (check_lidar_clearance).
"""

from dataclasses import dataclass
import math
from typing import List

import yaml

# 명세 "가상 물품" 표 (원 명세서 6장): 길이(x) × 폭(y) × 높이 [m]
SIZES = {
    'small': (0.30, 0.20, 0.15),
    'medium': (0.50, 0.40, 0.30),
    'large': (0.60, 0.50, 0.40),
}
# 2D LiDAR 스캔 평면 높이: base_link 0.18 m + lidar_link 0.2 m (amr_description)
LIDAR_HEIGHT = 0.38
# 스캔 평면 위아래로 이만큼은 비워 둔다 (로봇 기울기·가감속 피칭 여유)
LIDAR_MARGIN = 0.05

BODY_COLOR = (0.50, 0.34, 0.18)    # 진한 골판지. 밝은 바닥·랙(주황 0.85 0.45 0.10)·기둥(회색)과 구분 (밝은 색은 조명에 하얗게 날아감)
TAPE_COLOR = (0.22, 0.15, 0.08)
LABEL_COLOR = (0.95, 0.95, 0.93)
# YOLO 학습 라벨 번호 (Label 플러그인). 1 사람, 2 화물, 3 표지판 (README "라벨 카메라")
YOLO_LABEL = 2


@dataclass(frozen=True)
class CargoBox:
    id: str
    size: str
    x: float
    y: float
    z: float = 0.0       # 상자 바닥 높이 (바닥 0, 도크 플랫폼 위 1.0)
    yaw: float = 0.0

    @property
    def dims(self):
        return SIZES[self.size]

    @property
    def center(self):
        """상자 중심 (world). 라벨링에 쓴다."""
        return (self.x, self.y, self.z + self.dims[2] / 2)


def load_boxes(path: str) -> List[CargoBox]:
    with open(path, encoding='utf-8') as f:
        specs = yaml.safe_load(f)['boxes']
    boxes = [CargoBox(id=s['id'], size=s['size'], x=float(s['x']), y=float(s['y']),
                      z=float(s.get('z', 0.0)), yaw=float(s.get('yaw', 0.0))) for s in specs]
    ids = [b.id for b in boxes]
    if len(set(ids)) != len(ids):
        raise ValueError(f'상자 id가 겹친다: {ids}')
    for b in boxes:
        if b.size not in SIZES:
            raise ValueError(f'{b.id}: size는 {list(SIZES)} 중 하나여야 한다: {b.size}')
    return boxes


def check_lidar_clearance(boxes: List[CargoBox]) -> List[str]:
    """LiDAR 스캔 평면(±여유)에 걸리는 상자 id 목록. 비어 있어야 한다."""
    low, high = LIDAR_HEIGHT - LIDAR_MARGIN, LIDAR_HEIGHT + LIDAR_MARGIN
    return [b.id for b in boxes if b.z < high and b.z + b.dims[2] > low]


def _material(rgb):
    c = ' '.join(f'{v:g}' for v in rgb)
    return f'<material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse><specular>0.05 0.05 0.05 1</specular></material>'


def _box_visual(name, pose, size, rgb):
    p = ' '.join(f'{v:g}' for v in pose)
    s = ' '.join(f'{v:g}' for v in size)
    return (f'<visual name="{name}"><pose>{p}</pose><geometry><box><size>{s}</size></box></geometry>'
            f'{_material(rgb)}</visual>')


def _link(b: CargoBox) -> str:
    L, W, H = b.dims
    cx, cy, cz = b.center
    body = ' '.join(f'{v:g}' for v in (L, W, H))
    parts = [
        f'<link name="{b.id}"><pose>{cx:g} {cy:g} {cz:g} 0 0 {b.yaw:g}</pose>',
        f'<collision name="collision"><geometry><box><size>{body}</size></box></geometry></collision>',
        _box_visual('body', (0, 0, 0, 0, 0, 0), (L, W, H), BODY_COLOR),
        # 윗면 가운데 테이프 (길이 방향)
        _box_visual('tape', (0, 0, H / 2 + 0.001, 0, 0, 0), (L, 0.05, 0.002), TAPE_COLOR),
    ]
    # 양쪽 긴 면(±y)의 송장 라벨
    for sign, name in ((1, 'label_left'), (-1, 'label_right')):
        parts.append(_box_visual(name, (L * 0.15, sign * (W / 2 + 0.001), 0, 0, 0, 0),
                                 (L * 0.35, 0.002, H * 0.45), LABEL_COLOR))
    parts.append('</link>')
    return ''.join(parts)


def cargo_model_sdf(b: CargoBox) -> str:
    """상자 하나를 static 모델 하나로 만든 SDF 문자열 (ros_gz_sim create -string, 모델 이름 = 상자 id).

    상자마다 모델을 따로 둔다. 라벨 카메라는 모델 단위로 상자를 하나 내서, 한 모델에 상자 여러 개를
    링크로 넣으면 떨어진 상자들이 큰 상자 하나로 합쳐진다.
    """
    label = ('<plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">'
             f'<label>{YOLO_LABEL}</label></plugin>')
    return (f'<?xml version="1.0"?><sdf version="1.8"><model name="{b.id}"><static>true</static>'
            f'{_link(b)}{label}</model></sdf>')


def footprint(b: CargoBox):
    """바닥에 투영한 상자 네 꼭짓점 (world)."""
    L, W, _ = b.dims
    c, s = math.cos(b.yaw), math.sin(b.yaw)
    return [(b.x + c * dx - s * dy, b.y + s * dx + c * dy)
            for dx, dy in ((L / 2, W / 2), (-L / 2, W / 2), (-L / 2, -W / 2), (L / 2, -W / 2))]
