"""노드를 띄우지 않고 검사한다: 쿼터니언 yaw, 공분산 배치, 추적 마커."""

import math

import numpy as np
import pytest
from visualization_msgs.msg import Marker

from amr_msgs.msg import TrackedObstacle, TrackedObstacleArray
from amr_tracking.markers import DYNAMIC_COLOR, STATIC_COLOR, track_markers
from amr_tracking.tracking_node import UNKNOWN_VARIANCE, covariance_6x6, yaw_from_quaternion


@pytest.mark.parametrize('yaw', [0.0, 0.5, -1.2, math.pi - 0.01])
def test_yaw_from_quaternion(yaw):
    assert yaw_from_quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) == pytest.approx(yaw)


def test_covariance_6x6_layout():
    cov = np.array(covariance_6x6(np.array([[0.04, 0.01], [0.01, 0.09]]))).reshape(6, 6)
    np.testing.assert_allclose(cov[:2, :2], [[0.04, 0.01], [0.01, 0.09]])
    np.testing.assert_allclose(np.diag(cov)[2:], [UNKNOWN_VARIANCE] * 4)


def _obstacles():
    out = TrackedObstacleArray()
    out.header.frame_id = 'amr_01/odom'
    for track_id, (x, vx, dynamic, radius) in enumerate([(2.0, 1.0, True, 0.3),
                                                         (5.0, 0.0, False, 0.2)]):
        obs = TrackedObstacle(track_id=track_id, is_dynamic=dynamic, radius=radius)
        obs.pose.pose.position.x = x
        obs.twist.twist.linear.x = vx
        out.obstacles.append(obs)
    return out


def test_track_markers_cylinder_arrow_text_per_track():
    markers = track_markers(_obstacles()).markers
    assert markers[0].action == Marker.DELETEALL
    bodies = [m for m in markers if m.ns == 'obstacles']
    arrows = [m for m in markers if m.ns == 'velocity']
    texts = [m for m in markers if m.ns == 'labels']
    assert len(bodies) == len(arrows) == len(texts) == 2
    assert bodies[0].type == Marker.CYLINDER and bodies[0].scale.x == pytest.approx(0.6)
    assert bodies[0].header.frame_id == 'amr_01/odom'
    # 동적은 빨강, 정적은 회색
    assert (bodies[0].color.r, bodies[0].color.g, bodies[0].color.b) == pytest.approx(DYNAMIC_COLOR)
    assert (bodies[1].color.r, bodies[1].color.g, bodies[1].color.b) == pytest.approx(STATIC_COLOR)
    # 화살표는 1초 뒤 위치까지
    assert arrows[0].points[1].x - arrows[0].points[0].x == pytest.approx(1.0)
    assert texts[0].text == '#0 1.0 m/s'


def test_track_markers_empty_still_clears():
    assert [m.action for m in track_markers(TrackedObstacleArray()).markers] == [Marker.DELETEALL]
