"""노드를 띄우지 않고 검사한다: 쿼터니언 yaw, 공분산 배치, 추적 마커, 충돌 위험(TTC)."""

import math

import numpy as np
import pytest
from visualization_msgs.msg import Marker

from amr_msgs.msg import (
    DetectedObject3D,
    DetectedObject3DArray,
    TrackedObstacle,
    TrackedObstacleArray,
)
from amr_tracking.markers import DYNAMIC_COLOR, STATIC_COLOR, track_markers
from amr_tracking.tracking_node import (
    collision_risks,
    covariance_6x6,
    detections_in_fixed,
    robot_velocity_in_fixed,
    UNKNOWN_VARIANCE,
    yaw_from_quaternion,
)


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


def test_robot_velocity_in_fixed_rotates_body_velocity():
    vx, vy = robot_velocity_in_fixed(1.0, 0.0, math.pi / 2)
    assert (vx, vy) == pytest.approx((0.0, 1.0))


def test_collision_risks_per_track():
    # 트랙 0: x=2에서 +1 m/s로 멀어짐 → inf. 트랙 1: x=5 정지(반경 0.2), 로봇이 +1 m/s로 다가감
    risks = collision_risks(_obstacles(), (0.0, 0.0), 0.0, (1.0, 0.0), 0.6, 0.4, 10.0)
    assert risks.header.frame_id == 'amr_01/odom'
    assert [r.track_id for r in risks.risks] == [0, 1]
    assert risks.risks[0].ttc == math.inf
    assert risks.risks[1].ttc == pytest.approx(5.0 - 0.3 - 0.2, rel=1e-5)   # 앞면 0.3, float32


def test_collision_risks_uses_body_axes():
    # 로봇이 +y를 봄 (yaw 90°): odom x=5 트랙은 로봇 오른쪽 옆. 옆면은 0.2 m라 (5 − 0.2 − 0.2) / 1
    risks = collision_risks(_obstacles(), (0.0, 0.0), math.pi / 2, (1.0, 0.0), 0.6, 0.4, 10.0)
    assert risks.risks[1].ttc == pytest.approx(5.0 - 0.2 - 0.2, rel=1e-5)


def test_collision_risks_empty_still_published():
    empty = collision_risks(TrackedObstacleArray(), (0.0, 0.0), 0.0, (0.0, 0.0), 0.6, 0.4, 5.0)
    assert empty.risks == []


def _detections():
    msg = DetectedObject3DArray()
    msg.header.frame_id = 'map'
    obj = DetectedObject3D(class_name='person', confidence=0.9)
    obj.pose.pose.position.x, obj.pose.pose.position.y = 1.0, 0.0
    cov = np.zeros((6, 6))
    cov[:2, :2] = [[0.04, 0.0], [0.0, 0.01]]   # x 방향 σ 0.2, y 방향 σ 0.1
    obj.pose.covariance = cov.ravel().tolist()
    msg.objects.append(obj)
    return msg


def test_detections_in_fixed_rotates_position_and_covariance():
    # map → odom: 90° 회전 후 (2, 3) 이동
    det = detections_in_fixed(_detections(), math.pi / 2, (2.0, 3.0))[0]
    assert (det.class_name, det.confidence) == ('person', pytest.approx(0.9))
    np.testing.assert_allclose(det.xy, [2.0, 4.0], atol=1e-9)
    # 회전하면 x·y 분산이 서로 바뀐다
    np.testing.assert_allclose(det.cov, [[0.01, 0.0], [0.0, 0.04]], atol=1e-12)


def test_track_marker_shows_class():
    obstacles = _obstacles()
    obstacles.obstacles[0].class_name = 'person'
    texts = [m for m in track_markers(obstacles).markers if m.ns == 'labels']
    assert texts[0].text == '#0 person 1.0 m/s'
    assert texts[1].text == '#1 0.0 m/s'
