"""YOLO 학습 데이터 수집의 순수 함수 (ROS 비의존, 단위 테스트 대상). 노드는 dataset_collector.py.

라벨 번호는 Gazebo Label 플러그인 값이다 (amr_gazebo README "라벨 카메라").
"""

import hashlib
import math
from typing import Iterable, List, Optional, Tuple

CLASSES = ['person', 'cargo', 'sign']
# Gazebo 라벨 번호 → YOLO 클래스 번호
LABEL_TO_CLASS = {1: 0, 2: 1, 3: 2}

# (Gazebo 라벨, 상자 중심 x, 중심 y, 너비, 높이) [px]
Box = Tuple[int, float, float, float, float]


def yolo_line(box: Box, width: int, height: int) -> Optional[str]:
    """상자 하나를 YOLO 라벨 한 줄로 (클래스 cx cy w h, 영상 크기로 정규화). 모르는 라벨은 None."""
    label, cx, cy, w, h = box
    cls = LABEL_TO_CLASS.get(int(label))
    if cls is None:
        return None
    return f'{cls} {cx / width:.6f} {cy / height:.6f} {w / width:.6f} {h / height:.6f}'


def yolo_lines(boxes: Iterable[Box], width: int, height: int, min_px: float) -> Tuple[List[str], int]:
    """YOLO 라벨 줄들과 작아서 뺀 상자 수. 한 변이 min_px 미만이면 뺀다."""
    lines, dropped = [], 0
    for box in boxes:
        if min(box[3], box[4]) < min_px:
            dropped += 1
            continue
        line = yolo_line(box, width, height)
        if line is not None:
            lines.append(line)
    return lines, dropped


def split_of(x: float, y: float, val_ratio: float, cell: float = 5.0) -> str:
    """찍은 자리의 cell [m] 칸으로 학습·검증을 나눈다. 같은 칸은 늘 같은 쪽이다.

    같은 장소의 비슷한 사진이 양쪽에 들어가면 검증 성능이 부풀려진다.
    """
    key = f'{math.floor(x / cell)},{math.floor(y / cell)}'
    return 'val' if int(hashlib.md5(key.encode()).hexdigest(), 16) % 100 < val_ratio * 100 else 'train'
