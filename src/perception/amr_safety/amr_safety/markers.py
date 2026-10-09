"""안전 상태 → RViz2 마커 (safety/markers, base_footprint 좌표).

- 차체 사각형 (흰색)
- 긴급정지 경계: 차체에서 critical_distance만큼 바깥 (단계 색: 초록 NORMAL·노랑 WARNING·빨강 CRITICAL·보라 ESTOP)
- 저속 경계: 차체에서 dynamic_warning_distance만큼 바깥 (옅은 노랑, 움직이는 장애물만 해당)
- 글자: 단계, speed_limit, 원인 (RViz 글꼴에 한글이 없어 영어로 쓴다. 자세한 설명은 safety/status detail)
"""

import math
from typing import List, Tuple

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from amr_perception_core.safety import (
    LEVEL_CRITICAL,
    LEVEL_ESTOP,
    LEVEL_NORMAL,
    LEVEL_WARNING,
    REASON_COLLISION_RISK,
    REASON_ESTOP_REQUEST,
    REASON_NONE,
    REASON_OBSTACLE_PROXIMITY,
    REASON_SENSOR_FAULT,
    SafetyState,
)

LEVEL_COLORS = {
    LEVEL_NORMAL: (0.2, 0.85, 0.3),
    LEVEL_WARNING: (1.0, 0.8, 0.1),
    LEVEL_CRITICAL: (0.95, 0.15, 0.15),
    LEVEL_ESTOP: (0.7, 0.2, 0.9),
}
REASON_LABELS = {
    REASON_NONE: '',
    REASON_OBSTACLE_PROXIMITY: 'obstacle near',
    REASON_COLLISION_RISK: 'collision risk (TTC)',
    REASON_ESTOP_REQUEST: 'E-Stop',
    REASON_SENSOR_FAULT: 'sensor fault',
}
WARNING_ZONE_COLOR = (1.0, 0.85, 0.3)
MARKER_Z = 0.05   # m. 바닥에서 살짝 띄운다


def offset_rectangle(length: float, width: float, offset: float,
                     arc_points: int = 6) -> List[Tuple[float, float]]:
    """차체 사각형에서 offset만큼 떨어진 경계 (모서리는 원호). 닫힌 선이라 첫 점을 끝에 다시 넣는다."""
    hx, hy = 0.5 * length, 0.5 * width
    if offset <= 0.0:
        corners = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]
        return corners + corners[:1]
    out = []
    # 반시계: 왼쪽 앞 모서리부터. 각 모서리 중심과 원호 시작 각
    for (cx, cy), start in (((hx, hy), 0.0), ((-hx, hy), 0.5 * math.pi),
                            ((-hx, -hy), math.pi), ((hx, -hy), 1.5 * math.pi)):
        for i in range(arc_points + 1):
            a = start + 0.5 * math.pi * i / arc_points
            out.append((cx + offset * math.cos(a), cy + offset * math.sin(a)))
    return out + out[:1]


def _line(frame_id: str, ns: str, points, rgb, alpha: float, thickness: float) -> Marker:
    marker = Marker(ns=ns, id=0, type=Marker.LINE_STRIP, action=Marker.ADD)
    marker.header.frame_id = frame_id
    marker.pose.orientation.w = 1.0
    marker.scale.x = thickness
    marker.color.r, marker.color.g, marker.color.b = rgb
    marker.color.a = alpha
    marker.points = [Point(x=x, y=y, z=MARKER_Z) for x, y in points]
    return marker


def safety_markers(state: SafetyState, frame_id: str, length: float, width: float,
                   critical_distance: float, warning_distance: float) -> MarkerArray:
    rgb = LEVEL_COLORS[state.level]
    body = _line(frame_id, 'footprint', offset_rectangle(length, width, 0.0),
                 (1.0, 1.0, 1.0), 1.0, 0.02)
    stop = _line(frame_id, 'critical_zone', offset_rectangle(length, width, critical_distance),
                 rgb, 1.0, 0.04)
    slow = _line(frame_id, 'warning_zone', offset_rectangle(length, width, warning_distance),
                 WARNING_ZONE_COLOR, 0.35, 0.02)

    text = Marker(ns='status', id=0, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
    text.header.frame_id = frame_id
    text.pose.position.z = 1.0
    text.pose.orientation.w = 1.0
    text.scale.z = 0.25
    text.color.r, text.color.g, text.color.b = rgb
    text.color.a = 1.0
    limit = 'no limit' if math.isinf(state.speed_limit) else f'{state.speed_limit:.1f} m/s'
    text.text = f'{state.level_name} ({limit})'
    if state.reason != REASON_NONE:
        text.text += f'\n{REASON_LABELS[state.reason]}'
        if state.related_track_id >= 0:
            text.text += f' #{state.related_track_id}'
    return MarkerArray(markers=[body, stop, slow, text])
