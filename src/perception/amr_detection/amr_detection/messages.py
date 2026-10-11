"""검출 결과 → DetectedObject3D 메시지와 RViz2 마커.

원 명세 6장: 인식 결과를 커스텀 메시지로 발행하고, 3D 위치를 지도 위 마커로 표시한다.
마커 텍스트는 원 명세 결과 예시 형식 "Class: <class>, Conf: <conf>, Dist: <dist>m"을 따른다.
"""

from typing import Dict, Tuple

from builtin_interfaces.msg import Duration
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray

from amr_msgs.msg import DetectedObject3D, DetectedObject3DArray

from .detector import Detection

# 추정하지 않는 성분(회전)의 분산. docs/api.md 4-3절
UNKNOWN_VARIANCE = 1e6

CLASS_COLORS: Dict[str, Tuple[float, float, float]] = {
    'cargo': (0.9, 0.2, 0.1),
    'person': (1.0, 0.6, 0.0),
    'sign': (0.1, 0.4, 1.0),
    'forklift': (0.9, 0.9, 0.1),
    'robot': (0.3, 0.8, 0.3),
}
DEFAULT_COLOR = (0.7, 0.7, 0.7)


def covariance_6x6(position_covariance: np.ndarray) -> list:
    """3x3 위치 공분산을 ROS 6x6 공분산(행 우선 36개)에 넣는다. 회전 대각은 UNKNOWN_VARIANCE."""
    cov = np.diag([0.0, 0.0, 0.0] + [UNKNOWN_VARIANCE] * 3)
    cov[:3, :3] = position_covariance
    return cov.ravel().tolist()


def to_object(det: Detection, position: np.ndarray, position_covariance: np.ndarray,
              distance: float) -> DetectedObject3D:
    """position·covariance는 DetectedObject3DArray.header.frame_id 좌표계, distance는 카메라 기준."""
    obj = DetectedObject3D()
    obj.class_name = det.class_name
    obj.class_id = det.class_id
    obj.confidence = float(det.confidence)
    x1, y1, x2, y2 = (float(c) for c in det.xyxy)   # 메시지 필드가 float32라 int를 받지 않는다
    obj.bbox_center_x = (x1 + x2) / 2.0
    obj.bbox_center_y = (y1 + y2) / 2.0
    obj.bbox_width = x2 - x1
    obj.bbox_height = y2 - y1
    pos = obj.pose.pose.position
    pos.x, pos.y, pos.z = (float(c) for c in position)
    obj.pose.pose.orientation.w = 1.0   # 방향은 추정하지 않는다 (api.md 4-3절)
    obj.pose.covariance = covariance_6x6(position_covariance)
    obj.distance = float(distance)
    return obj


def label_text(obj: DetectedObject3D) -> str:
    return f'Class: {obj.class_name}, Conf: {obj.confidence:.2f}, Dist: {obj.distance:.1f}m'


def to_markers(objects: DetectedObject3DArray, box_size: float = 0.4,
               lifetime_s: float = 1.0) -> MarkerArray:
    """객체마다 색깔 상자 + 그 위 텍스트. 맨 앞 DELETEALL로 지난 프레임 마커를 지운다.

    lifetime은 노드가 발행을 멈췄을 때(TF 끊김 등) 마커가 화면에 남지 않게 하는 안전장치다.
    """
    markers = [Marker(action=Marker.DELETEALL)]
    life = Duration(sec=int(lifetime_s), nanosec=int((lifetime_s % 1.0) * 1e9))
    for i, obj in enumerate(objects.objects):
        r, g, b = CLASS_COLORS.get(obj.class_name, DEFAULT_COLOR)
        box = Marker(header=objects.header, ns='objects', id=i, type=Marker.CUBE,
                     action=Marker.ADD, lifetime=life)
        box.pose = obj.pose.pose
        box.scale.x = box.scale.y = box.scale.z = box_size
        box.color.r, box.color.g, box.color.b, box.color.a = r, g, b, 0.8

        text = Marker(header=objects.header, ns='labels', id=i, type=Marker.TEXT_VIEW_FACING,
                      action=Marker.ADD, lifetime=life)
        text.pose.position.x = obj.pose.pose.position.x
        text.pose.position.y = obj.pose.pose.position.y
        text.pose.position.z = obj.pose.pose.position.z + box_size
        text.pose.orientation.w = 1.0
        text.scale.z = 0.2   # 글자 높이 [m]
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = label_text(obj)
        markers += [box, text]
    return MarkerArray(markers=markers)
