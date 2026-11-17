"""num_threads가 실제 torch 스레드 수가 되는지 (detector.YoloDetector).

ultralytics는 첫 추론 때 장치를 고르며 torch 스레드를 min(8, 코어 수 − 1)로 덮는다. 그래서 첫 추론(데움) 뒤에
다시 정한다. 모델·ultralytics가 있는 이미지(docker/Dockerfile)에서만 돈다. 첫 추론이 CPU에서 십수 초 걸린다.
"""

import os

import pytest

COCO_MODEL = '/opt/models/yolov8n.pt'


@pytest.mark.skipif(not os.path.exists(COCO_MODEL), reason='모델이 없는 환경 (Docker 이미지 밖)')
def test_num_threads_survives_ultralytics_setup():
    torch = pytest.importorskip('torch')
    pytest.importorskip('ultralytics')
    from amr_detection.detector import YoloDetector

    previous = torch.get_num_threads()
    try:
        det = YoloDetector(COCO_MODEL, {'person': 'person'}, num_threads=2)
        assert torch.get_num_threads() == 2 and det.num_threads == 2
    finally:
        torch.set_num_threads(previous)
