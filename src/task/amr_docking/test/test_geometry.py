import math

import pytest

from amr_docking.geometry import DockError, average, dock_error

SPACING = 0.32


def board_seen_from(along, lateral, heading, standoff):
    """목표 자세 기준 로봇 자세(along 남은 거리, lateral 왼쪽 +, heading 왼쪽 +)일 때 로봇 기준 마커 중심."""
    # 목표 좌표계: 원점 = 목표, x = 판 쪽. 판 중심 (standoff, 0), 왼쪽 마커는 +y (로봇이 판을 볼 때 왼쪽)
    rx, ry = -along, lateral
    out = []
    for my in (SPACING / 2, -SPACING / 2):
        dx, dy = standoff - rx, my - ry
        c, s = math.cos(-heading), math.sin(-heading)
        out.append((c * dx - s * dy, s * dx + c * dy))
    return out


@pytest.mark.parametrize('along,lateral,heading', [
    (0.0, 0.0, 0.0), (0.1, 0.0, 0.0), (-0.05, 0.0, 0.0), (0.0, 0.05, 0.0), (0.0, -0.03, 0.0),
    (0.0, 0.0, math.radians(5)), (0.2, -0.08, math.radians(-12)), (0.3, 0.1, math.radians(15)),
])
def test_dock_error_recovers_pose(along, lateral, heading):
    left, right = board_seen_from(along, lateral, heading, 0.795)
    e = dock_error(left, right, 0.795)
    assert e.along == pytest.approx(along, abs=1e-9)
    assert e.lateral == pytest.approx(lateral, abs=1e-9)
    assert e.heading == pytest.approx(heading, abs=1e-9)


def test_at_target_board_straight_ahead():
    e = dock_error((0.795, SPACING / 2), (0.795, -SPACING / 2), 0.795)
    assert e.position == pytest.approx(0.0, abs=1e-12)
    assert e.heading == pytest.approx(0.0, abs=1e-12)


def test_coincident_markers_rejected():
    with pytest.raises(ValueError):
        dock_error((1.0, 0.0), (1.0, 0.0), 0.5)


def test_average_wraps_heading():
    e = average([DockError(0.0, 0.0, math.pi - 0.01), DockError(0.0, 0.0, -math.pi + 0.01)])
    assert abs(abs(e.heading) - math.pi) < 1e-9
