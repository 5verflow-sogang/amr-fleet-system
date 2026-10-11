"""추적 결과(TrackedObstacleArray) → RViz2 마커. TrackedObstacleArray는 RViz가 직접 그리지 못한다.

트랙마다 원기둥(반경 = 트랙 반경, 동적은 빨강·정적은 회색), 속도 화살표(1초 뒤 위치까지), 글자(ID·속력).
"""

import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from amr_msgs.msg import TrackedObstacleArray

DYNAMIC_COLOR = (0.95, 0.2, 0.15)
STATIC_COLOR = (0.6, 0.6, 0.6)
CYLINDER_HEIGHT = 0.5   # m. 사람 다리 높이 정도. 지도 위에서 보기 좋은 높이


def _color(marker: Marker, rgb, alpha: float) -> None:
    marker.color.r, marker.color.g, marker.color.b = rgb
    marker.color.a = alpha


def track_markers(obstacles: TrackedObstacleArray, lifetime_s: float = 0.5) -> MarkerArray:
    """맨 앞 DELETEALL로 지난 프레임 마커를 지운다. lifetime은 발행이 끊겼을 때 남지 않게 하는 안전장치."""
    markers = [Marker(action=Marker.DELETEALL)]
    life = Duration(sec=int(lifetime_s), nanosec=int((lifetime_s % 1.0) * 1e9))
    for i, obs in enumerate(obstacles.obstacles):
        p = obs.pose.pose.position
        v = obs.twist.twist.linear
        rgb = DYNAMIC_COLOR if obs.is_dynamic else STATIC_COLOR

        body = Marker(header=obstacles.header, ns='obstacles', id=i, type=Marker.CYLINDER,
                      action=Marker.ADD, lifetime=life)
        body.pose.position.x, body.pose.position.y = p.x, p.y
        body.pose.position.z = CYLINDER_HEIGHT / 2.0
        body.pose.orientation.w = 1.0
        body.scale.x = body.scale.y = 2.0 * max(float(obs.radius), 0.05)
        body.scale.z = CYLINDER_HEIGHT
        _color(body, rgb, 0.6)

        arrow = Marker(header=obstacles.header, ns='velocity', id=i, type=Marker.ARROW,
                       action=Marker.ADD, lifetime=life)
        arrow.points = [Point(x=p.x, y=p.y, z=CYLINDER_HEIGHT),
                        Point(x=p.x + v.x, y=p.y + v.y, z=CYLINDER_HEIGHT)]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.05, 0.12, 0.12   # 축 지름, 머리 지름·길이
        _color(arrow, rgb, 1.0)

        text = Marker(header=obstacles.header, ns='labels', id=i, type=Marker.TEXT_VIEW_FACING,
                      action=Marker.ADD, lifetime=life)
        text.pose.position.x, text.pose.position.y = p.x, p.y
        text.pose.position.z = CYLINDER_HEIGHT + 0.3
        text.pose.orientation.w = 1.0
        text.scale.z = 0.25
        _color(text, (1.0, 1.0, 1.0), 1.0)
        text.text = f'#{obs.track_id} {math.hypot(v.x, v.y):.1f} m/s'
        markers += [body, arrow, text]
    return MarkerArray(markers=markers)
