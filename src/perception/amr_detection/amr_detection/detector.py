"""ultralytics YOLO를 감싸 '영상 → 검출 목록'만 노드에 내보낸다.

모델 클래스 이름을 팀 클래스 이름(docs/api.md 4-3절 class_name 허용 값)으로 바꾸고, 매핑에 없는
클래스는 버린다. 기본 모델(yolov8n_amr_v1)은 person·cargo·sign 3종을 그대로 낸다. COCO 원본(yolov8n)을 쓰면
person·stop sign만 우리 클래스에 대응하고 화물은 없다. 모델을 바꾸면 설정의 매핑만 바꾼다.
"""

from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np

BBox = Tuple[float, float, float, float]


@dataclass(frozen=True)
class Detection:
    class_name: str     # 팀 클래스 이름 (cargo, person, sign, ...)
    class_id: int       # 모델의 클래스 번호
    confidence: float
    xyxy: BBox          # 픽셀 경계 관례 (YOLO 출력 그대로, depth_sampling과 같다)


def build_class_map(model_classes: Sequence[str], output_classes: Sequence[str]) -> Dict[str, str]:
    """모델 클래스 이름 → 팀 클래스 이름. 두 목록은 같은 순서로 짝을 이룬다."""
    if len(model_classes) != len(output_classes):
        raise ValueError(f'model_classes({len(model_classes)}개)와 output_classes'
                         f'({len(output_classes)}개)의 길이가 다르다')
    if not model_classes:
        raise ValueError('클래스 매핑이 비어 있다')
    if len(set(model_classes)) != len(model_classes):
        raise ValueError(f'model_classes에 중복이 있다: {list(model_classes)}')
    return dict(zip(model_classes, output_classes))


def to_detections(names: Dict[int, str], class_ids: Sequence[float], confidences: Sequence[float],
                  boxes_xyxy: Sequence[Sequence[float]], class_map: Dict[str, str]
                  ) -> List[Detection]:
    """YOLO 결과 배열 → Detection 목록. 매핑에 없는 클래스는 버린다."""
    out = []
    for cid, conf, box in zip(class_ids, confidences, boxes_xyxy):
        name = class_map.get(names[int(cid)])
        if name is not None:
            x1, y1, x2, y2 = (float(c) for c in box)
            out.append(Detection(name, int(cid), float(conf), (x1, y1, x2, y2)))
    return out


class YoloDetector:
    """ultralytics YOLO 추론기. bgr 영상(OpenCV 관례)을 받는다."""

    def __init__(self, model_path: str, class_map: Dict[str, str], device: str = 'cpu',
                 image_size: int = 640, confidence: float = 0.5, iou: float = 0.45,
                 num_threads: int = 0):
        # 무거운 import는 실제로 추론기를 만들 때만 한다 (테스트는 to_detections만 쓴다)
        import torch
        from ultralytics import YOLO

        if num_threads > 0:
            # 컨테이너에서 torch 기본 스레드가 1개로 잡혀 CPU 추론이 느렸다 (8코어에서 640 입력 141 ms)
            torch.set_num_threads(num_threads)
        self.model = YOLO(model_path)
        self.names: Dict[int, str] = dict(self.model.names)
        unknown = set(class_map) - set(self.names.values())
        if unknown:
            raise ValueError(f'모델에 없는 클래스: {sorted(unknown)} (모델: {model_path})')
        self.class_map = class_map
        self.kwargs = dict(device=device, imgsz=image_size, conf=confidence, iou=iou,
                           classes=[i for i, n in self.names.items() if n in class_map],
                           verbose=False)
        # 첫 추론은 모델 준비로 십수 초 걸린다 (Docker CPU 실측 약 13 s). 시작할 때 미리 한 번 돌려
        # 첫 카메라 프레임이 그만큼 늦게 처리되지 않게 한다
        self.model.predict(np.zeros((480, 640, 3), dtype=np.uint8), **self.kwargs)

    def detect(self, bgr) -> List[Detection]:
        boxes = self.model.predict(bgr, **self.kwargs)[0].boxes
        return to_detections(self.names, boxes.cls.tolist(), boxes.conf.tolist(),
                             boxes.xyxy.tolist(), self.class_map)
