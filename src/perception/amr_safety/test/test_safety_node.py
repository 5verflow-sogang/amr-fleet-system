"""노드를 띄우지 않고 검사한다: 장애물 → 판정 입력, SafetyStatus 변환, RViz 마커."""

import math

import pytest

from amr_msgs.msg import SafetyStatus, TrackedObstacle, TrackedObstacleArray
from amr_perception_core.safety import (
    LEVEL_CRITICAL,
    LEVEL_NORMAL,
    LEVEL_WARNING,
    REASON_COLLISION_RISK,
    REASON_NONE,
    SafetyState,
)
from amr_safety.markers import LEVEL_COLORS, offset_rectangle, safety_markers
from amr_safety.safety_node import obstacle_infos, to_status_msg


def _obstacles():
    out = TrackedObstacleArray()
    out.header.frame_id = 'amr_01/odom'
    for track_id, (x, y, dynamic, radius) in enumerate([(3.0, 0.0, True, 0.2),
                                                       (1.0, 1.0, False, 0.3)]):
        obs = TrackedObstacle(track_id=track_id, is_dynamic=dynamic, radius=radius)
        obs.pose.pose.position.x, obs.pose.pose.position.y = x, y
        out.obstacles.append(obs)
    return out


def test_obstacle_infos_clearance_from_body_minus_radius():
    # 로봇이 odom (2, 0)에 있음: odom → base_footprint 변환은 (−2, 0) 평행이동
    infos = obstacle_infos(_obstacles(), {0: 0.8}, 0.0, (-2.0, 0.0), 0.6, 0.4)
    # 트랙 0: 몸체 x = 1.0 → 앞면에서 0.7, 반경 0.2 빼면 0.5
    assert infos[0].clearance == pytest.approx(0.5)
    assert (infos[0].ttc, infos[0].is_dynamic) == (pytest.approx(0.8), True)
    # 트랙 1: 몸체 (−1, 1) → 모서리 (−0.3, 0.2)에서 hypot(0.7, 0.8), 반경 0.3 빼기. TTC 없음 → inf
    assert infos[1].clearance == pytest.approx(math.hypot(0.7, 0.8) - 0.3)
    assert infos[1].ttc == math.inf


def test_obstacle_infos_overlap_clamped_to_zero():
    infos = obstacle_infos(_obstacles(), {}, 0.0, (-3.0, 0.0), 0.6, 0.4)
    assert infos[0].clearance == 0.0


def test_obstacle_infos_rotation():
    # 로봇이 원점에서 왼쪽(+y)을 봄: odom → 몸체는 −90도 회전. 트랙 0 (3, 0)은 몸체 오른쪽 (0, −3)
    infos = obstacle_infos(_obstacles(), {}, -math.pi / 2, (0.0, 0.0), 0.6, 0.4)
    assert infos[0].clearance == pytest.approx(3.0 - 0.2 - 0.2)


def test_obstacle_infos_receding_behind_needs_rear_position_and_moving_away():
    out = TrackedObstacleArray()
    # 로봇은 원점에서 +x를 본다. 차체 뒤 끝 x = -0.3, 작업자 반경 0.25 → 몸 전체가 뒤면 x < -0.55
    for track_id, (x, vx) in enumerate([(-0.8, -1.0), (-0.8, 0.6), (-0.4, -1.0), (0.8, -1.0)]):
        obs = TrackedObstacle(track_id=track_id, is_dynamic=True, radius=0.25)
        obs.pose.pose.position.x, obs.pose.pose.position.y = x, 0.5
        obs.twist.twist.linear.x = vx
        out.obstacles.append(obs)
    infos = obstacle_infos(out, {}, 0.0, (0.0, 0.0), 0.6, 0.4, robot_vx=0.3)
    assert [i.receding_behind for i in infos] == [True, False, False, False]
    assert [i.speed for i in infos] == [pytest.approx(1.0), pytest.approx(0.6), pytest.approx(1.0), pytest.approx(1.0)]
    # 뒤에서 따라오는 쪽(로봇 0.3보다 빠른 0.6): 다가온다. 뒤 끝에 걸친 것(-0.4)·앞(0.8)은 뒤가 아니다
    # 로봇이 왼쪽(+y)을 보면 world -y 이동이 몸체 -x: 몸체 뒤(world (0.0, -0.8))에서 멀어진다
    rot = TrackedObstacleArray()
    obs = TrackedObstacle(track_id=0, is_dynamic=True, radius=0.25)
    obs.pose.pose.position.y = -0.8
    obs.twist.twist.linear.y = -1.0
    rot.obstacles.append(obs)
    assert obstacle_infos(rot, {}, -math.pi / 2, (0.0, 0.0), 0.6, 0.4)[0].receding_behind


def test_obstacle_infos_empty():
    assert obstacle_infos(TrackedObstacleArray(), {}, 0.0, (0.0, 0.0), 0.6, 0.4) == []


def test_status_msg_keeps_inf_and_fields():
    state = SafetyState(LEVEL_NORMAL, REASON_NONE, math.inf, '', -1, math.inf)
    msg = to_status_msg(state, SafetyStatus().header.stamp)
    assert msg.speed_limit == math.inf and msg.min_obstacle_distance == math.inf
    assert msg.related_track_id == -1
    warn = to_status_msg(SafetyState(LEVEL_WARNING, REASON_COLLISION_RISK, 0.6, 'TTC 2.0 s', 4,
                                     1.2), SafetyStatus().header.stamp)
    assert (warn.level, warn.reason, warn.related_track_id) == (
        SafetyStatus.LEVEL_WARNING, SafetyStatus.REASON_COLLISION_RISK, 4)
    assert warn.speed_limit == pytest.approx(0.6)


def test_core_constants_match_message():
    from amr_perception_core import safety
    for name in ('LEVEL_NORMAL', 'LEVEL_WARNING', 'LEVEL_CRITICAL', 'LEVEL_ESTOP', 'REASON_NONE',
                 'REASON_OBSTACLE_PROXIMITY', 'REASON_COLLISION_RISK', 'REASON_ESTOP_REQUEST',
                 'REASON_SENSOR_FAULT'):
        assert getattr(safety, name) == getattr(SafetyStatus, name), name


def test_offset_rectangle_is_closed_and_offset():
    points = offset_rectangle(0.6, 0.4, 0.3)
    assert points[0] == points[-1]
    xs, ys = zip(*points)
    assert max(xs) == pytest.approx(0.6) and max(ys) == pytest.approx(0.5)


def test_safety_markers_color_by_level():
    state = SafetyState(LEVEL_CRITICAL, REASON_COLLISION_RISK, 0.0, 'TTC 0.5 s', 7, 0.8)
    markers = {m.ns: m for m in safety_markers(state, 'amr_01/base_footprint', 0.6, 0.4,
                                               0.3, 1.5).markers}
    assert set(markers) == {'footprint', 'critical_zone', 'warning_zone', 'status'}
    zone = markers['critical_zone']
    rgb = (zone.color.r, zone.color.g, zone.color.b)
    assert rgb == pytest.approx(LEVEL_COLORS[LEVEL_CRITICAL])
    assert zone.header.frame_id == 'amr_01/base_footprint'
    assert markers['status'].text == 'CRITICAL (0.0 m/s)\ncollision risk (TTC) #7'
