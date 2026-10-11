"""YOLO 학습 데이터 수집의 순수 함수 (amr_detection.dataset)."""

import pytest

from amr_detection.dataset import CLASSES, split_of, yolo_line, yolo_lines


def test_label_numbers_map_to_classes():
    # Gazebo 라벨 1 사람, 2 화물, 3 표지판 → YOLO 0, 1, 2 (amr_gazebo README "라벨 카메라")
    assert CLASSES == ['person', 'cargo', 'sign']
    for label, cls in ((1, 0), (2, 1), (3, 2)):
        assert yolo_line((label, 320, 240, 64, 48), 640, 480).split()[0] == str(cls)
    assert yolo_line((9, 320, 240, 64, 48), 640, 480) is None


def test_yolo_line_is_normalized_center_and_size():
    values = [float(v) for v in yolo_line((2, 160, 120, 64, 48), 640, 480).split()[1:]]
    assert values == pytest.approx([0.25, 0.25, 0.1, 0.1])


def test_small_boxes_dropped_and_counted():
    boxes = [(1, 100, 100, 3.9, 50), (2, 200, 200, 4.0, 4.0), (3, 300, 300, 30, 30)]
    lines, dropped = yolo_lines(boxes, 640, 480, min_px=4.0)
    assert dropped == 1
    assert [ln.split()[0] for ln in lines] == ['1', '2']


def test_split_is_fixed_per_place():
    # 같은 5 m 칸이면 같은 쪽, 칸 경계를 넘으면 바뀔 수 있다
    assert split_of(1.0, 1.0, 0.15) == split_of(4.9, 4.9, 0.15)
    assert {split_of(0.0, 0.0, 0.0), split_of(0.0, 0.0, 1.0)} == {'train', 'val'}


def test_split_ratio_roughly_matches():
    cells = [split_of(x * 5.0 + 1, y * 5.0 + 1, 0.15) for x in range(-6, 6) for y in range(-4, 4)]
    assert 0.05 < cells.count('val') / len(cells) < 0.3
