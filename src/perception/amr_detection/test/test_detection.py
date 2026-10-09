"""모델·카메라 없이 검사한다: 클래스 매핑, 메시지 필드 배치, 마커 (원 명세 6장)."""

import numpy as np
import pytest
from visualization_msgs.msg import Marker

from amr_detection.detector import Detection, build_class_map, to_detections
from amr_detection.messages import (UNKNOWN_VARIANCE, covariance_6x6, label_text, to_markers,
                                    to_object)
from amr_msgs.msg import DetectedObject3DArray

COCO_NAMES = {0: 'person', 11: 'stop sign', 28: 'suitcase'}
CLASS_MAP = build_class_map(['person', 'stop sign'], ['person', 'sign'])


def test_class_map_pairs_in_order():
    assert CLASS_MAP == {'person': 'person', 'stop sign': 'sign'}


@pytest.mark.parametrize('model, output', [(['person'], ['person', 'sign']), ([], []),
                                           (['person', 'person'], ['person', 'sign'])])
def test_class_map_rejects_bad_lists(model, output):
    with pytest.raises(ValueError):
        build_class_map(model, output)


def test_to_detections_renames_and_drops_unmapped():
    dets = to_detections(COCO_NAMES, [0.0, 28.0, 11.0], [0.9, 0.8, 0.7],
                         [[10, 20, 110, 220], [0, 0, 5, 5], [300, 100, 340, 160]], CLASS_MAP)
    assert [d.class_name for d in dets] == ['person', 'sign']   # suitcase는 매핑 밖이라 버림
    assert dets[0] == Detection('person', 0, pytest.approx(0.9), (10.0, 20.0, 110.0, 220.0))
    assert dets[1].class_id == 11                               # 모델 클래스 번호를 유지


def test_covariance_6x6_layout():
    block = np.array([[0.04, 0.01, 0.0], [0.01, 0.09, 0.0], [0.0, 0.0, 0.0004]])
    cov = np.array(covariance_6x6(block)).reshape(6, 6)
    np.testing.assert_allclose(cov[:3, :3], block)
    np.testing.assert_allclose(np.diag(cov)[3:], [UNKNOWN_VARIANCE] * 3)
    assert not cov[:3, 3:].any() and not cov[3:, :3].any()


def test_to_object_fills_message():
    det = Detection('person', 0, 0.88, (100.0, 50.0, 180.0, 450.0))
    obj = to_object(det, np.array([12.5, 3.2, 0.9]), np.eye(3) * 0.01, 3.5)
    assert (obj.class_name, obj.class_id) == ('person', 0)
    assert obj.confidence == pytest.approx(0.88)
    # bbox는 중심·크기 (DetectedObject3D.msg)
    assert (obj.bbox_center_x, obj.bbox_center_y) == (140.0, 250.0)
    assert (obj.bbox_width, obj.bbox_height) == (80.0, 400.0)
    p = obj.pose.pose
    assert (p.position.x, p.position.y, p.position.z) == pytest.approx((12.5, 3.2, 0.9))
    # 방향은 추정하지 않는다: (0, 0, 0, 1), 회전 분산 1e6 (api.md 4-3절)
    assert (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w) == (0, 0, 0, 1)
    assert obj.pose.covariance[0] == pytest.approx(0.01)
    assert obj.pose.covariance[35] == UNKNOWN_VARIANCE
    assert obj.distance == pytest.approx(3.5)


def test_label_text_matches_spec_example():
    obj = to_object(Detection('cargo', 3, 0.92, (0, 0, 10, 10)), np.zeros(3), np.eye(3), 1.5)
    assert label_text(obj) == 'Class: cargo, Conf: 0.92, Dist: 1.5m'


def _array(n):
    out = DetectedObject3DArray()
    out.header.frame_id = 'map'
    out.header.stamp.sec = 42
    out.objects = [to_object(Detection('person', 0, 0.9, (0, 0, 10, 10)),
                             np.array([float(i), 1.0, 0.5]), np.eye(3), 2.0) for i in range(n)]
    return out


def test_markers_box_and_text_per_object():
    markers = to_markers(_array(2), box_size=0.4).markers
    assert markers[0].action == Marker.DELETEALL      # 지난 프레임 마커를 먼저 지운다
    boxes = [m for m in markers if m.ns == 'objects']
    texts = [m for m in markers if m.ns == 'labels']
    assert [m.type for m in boxes] == [Marker.CUBE] * 2
    assert [m.type for m in texts] == [Marker.TEXT_VIEW_FACING] * 2
    assert len({(m.ns, m.id) for m in markers[1:]}) == 4   # 겹치지 않는 id
    assert boxes[1].header.frame_id == 'map' and boxes[1].header.stamp.sec == 42
    assert boxes[1].pose.position.x == pytest.approx(1.0)
    assert texts[1].pose.position.z == pytest.approx(0.5 + 0.4)   # 상자 위
    assert texts[0].text.startswith('Class: person')


def test_empty_detection_still_clears_markers():
    markers = to_markers(_array(0)).markers
    assert [m.action for m in markers] == [Marker.DELETEALL]
