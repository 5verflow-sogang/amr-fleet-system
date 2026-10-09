import math

import numpy as np
import pytest

from amr_perception_core.association import (
    associate,
    AssociationConfig,
    ClassLabeler,
    FixedDetection,
    TrackSnapshot,
    UNKNOWN_CLASS,
)

COV = np.eye(2) * 0.05 ** 2          # 트랙 위치 σ 5 cm
DET_COV = np.eye(2) * 0.1 ** 2       # 검출 위치 σ 10 cm


def track(track_id, xy, velocity=(0.0, 0.0), stamp=0.0, cov=COV):
    return TrackSnapshot(track_id, np.array(xy, float), np.array(velocity, float), cov,
                         np.eye(2) * 0.05 ** 2, stamp)


def det(xy, class_name='person', confidence=0.8, cov=DET_COV):
    return FixedDetection(class_name, confidence, np.array(xy, float), cov)


# --- 연관 ---

def test_close_detection_is_paired():
    assert associate([track(7, (2.0, 0.0))], [det((2.1, 0.05))], 0.0) == [(0, 0)]


def test_far_detection_outside_gate_is_not_paired():
    assert associate([track(7, (2.0, 0.0))], [det((2.6, 0.0))], 0.0) == []


def test_max_distance_rejects_even_with_large_covariance():
    # 공분산이 커서 게이트 안이어도 1 m 넘게는 짝짓지 않는다
    big = np.eye(2) * 2.0 ** 2
    assert associate([track(7, (2.0, 0.0), cov=big)], [det((3.2, 0.0))], 0.0) == []


def test_one_to_one_closest_pair_first():
    tracks = [track(1, (2.0, 0.0)), track(2, (2.3, 0.0))]
    dets = [det((2.25, 0.0)), det((2.0, 0.02))]
    # 검출 0은 트랙 2와, 검출 1은 트랙 1과 더 가깝다
    assert sorted(associate(tracks, dets, 0.0)) == [(0, 1), (1, 0)]


def test_track_predicted_to_detection_time():
    # 1 m/s로 움직이는 트랙(0.0 s에 x = 2.0). 0.4 s 뒤 검출은 x = 2.4에 있다
    moving = track(3, (2.0, 0.0), velocity=(1.0, 0.0), stamp=0.0)
    assert associate([moving], [det((2.4, 0.0))], 0.4) == [(0, 0)]
    still = track(3, (2.0, 0.0), velocity=(0.0, 0.0), stamp=0.0)
    assert associate([still], [det((2.4, 0.0))], 0.4) == []


def test_low_confidence_detection_is_ignored():
    assert associate([track(1, (2.0, 0.0))], [det((2.0, 0.0), confidence=0.3)], 0.0) == []


def test_empty_inputs():
    assert associate([], [det((1.0, 0.0))], 0.0) == []
    assert associate([track(1, (1.0, 0.0))], [], 0.0) == []


# --- 클래스 누적 ---

def test_single_detection_is_not_enough():
    labeler = ClassLabeler()
    labeler.update(5, 'person', 0.8, 0.0)
    assert labeler.label(5, 0.0).class_name == UNKNOWN_CLASS


def test_two_detections_label_the_track():
    labeler = ClassLabeler()
    labeler.update(5, 'person', 0.8, 0.0)
    labeler.update(5, 'person', 0.6, 0.2)
    label = labeler.label(5, 0.2)
    assert (label.class_name, label.camera_recent) == ('person', True)
    assert label.confidence == pytest.approx(0.7)


def test_label_kept_after_leaving_camera_then_fades():
    labeler = ClassLabeler(AssociationConfig(evidence_tau=5.0, camera_recent_s=1.0))
    for k in range(4):
        labeler.update(5, 'person', 0.8, 0.1 * k)       # 근거 약 3.2
    # 2 s 뒤: 카메라로는 안 보이지만 클래스 유지
    label = labeler.label(5, 2.3)
    assert (label.class_name, label.camera_recent) == ('person', False)
    # 10 s 뒤: 근거 3.1·e^(−2) ≈ 0.42 (≥ 0.3) → 히스테리시스로 유지
    assert labeler.label(5, 10.3).class_name == 'person'
    # 12 s 뒤: 3.1·e^(−2.4) ≈ 0.28 < 0.3 → unknown
    assert labeler.label(5, 12.3).class_name == UNKNOWN_CLASS


def test_sparse_detections_do_not_flicker():
    # 카메라가 3 s에 한 번만 검출(Gazebo CPU 0.3 FPS). 한 번 붙은 뒤로는 검출 사이에도 클래스가 유지된다
    labeler = ClassLabeler()
    names = []
    for k in range(0, 150):
        t = 0.1 * k
        if k % 30 == 0:
            labeler.update(5, 'person', 0.7, t)
        names.append(labeler.label(5, t).class_name)
    first = names.index('person')
    assert first <= 30                                   # 두 번째 검출(3 s)에 붙는다
    assert set(names[first:]) == {'person'}


def test_class_switches_when_other_evidence_dominates():
    labeler = ClassLabeler()
    labeler.update(5, 'cargo', 0.6, 0.0)
    labeler.update(5, 'cargo', 0.6, 0.1)
    assert labeler.label(5, 0.1).class_name == 'cargo'
    for k in range(4):
        labeler.update(5, 'person', 0.9, 0.2 + 0.1 * k)
    assert labeler.label(5, 0.5).class_name == 'person'


def test_unknown_track_and_prune():
    labeler = ClassLabeler()
    assert labeler.label(9, 0.0) == labeler.label(9, 5.0)
    assert labeler.label(9, 0.0).class_name == UNKNOWN_CLASS
    labeler.update(1, 'person', 0.9, 0.0)
    labeler.update(1, 'person', 0.9, 0.1)
    labeler.prune([2, 3])
    assert labeler.label(1, 0.1).class_name == UNKNOWN_CLASS


def test_label_query_out_of_order_time_does_not_break():
    labeler = ClassLabeler()
    labeler.update(1, 'person', 0.9, 1.0)
    labeler.update(1, 'person', 0.9, 1.1)
    assert labeler.label(1, 0.5).class_name == 'person'     # 과거 시각 질의: 근거를 늘리지 않음


@pytest.mark.parametrize('kwargs', [dict(max_distance=0.0), dict(evidence_tau=-1.0),
                                    dict(min_confidence=1.5), dict(gate_mahalanobis_sq=math.inf),
                                    dict(release_evidence=2.0)])
def test_rejects_bad_config(kwargs):
    with pytest.raises(ValueError):
        AssociationConfig(**kwargs)
