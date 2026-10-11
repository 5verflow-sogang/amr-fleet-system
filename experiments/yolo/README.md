# yolo

YOLO 인식 3종(사람·화물·표지판, 원 명세 6장)의 학습 데이터셋과 학습 기록. 담당: 연우.

- **데이터 수집**은 ROS 노드라 `src`에 있다: [amr_detection `collect_dataset`](../../src/perception/amr_detection/README.md). 결과를 이 폴더의 `raw/`(git 제외)에 남긴다.
- **학습**은 ROS 없이 학교 GPU 서버에서 ultralytics로 한다 (이 문서).

## 데이터셋 형식

`raw/<시각>/`

| 경로 | 내용 |
| --- | --- |
| `images/{train,val}/*.jpg` | 로봇 RGB 카메라 640×480 |
| `labels/{train,val}/*.txt` | YOLO 형식 (클래스 cx cy w h, 0~1). 클래스 0 `person`, 1 `cargo`, 2 `sign` |
| `data.yaml` | ultralytics 데이터셋 설정 |
| `frames.jsonl` | 사진마다 찍은 자리(x, y, yaw)·시각·상자 수·작아서 뺀 상자 수 |
| `meta.json` | 수집 인자, 저장·시도 수, 클래스별 상자 수 |
| `preview/` | 처음 30장에 상자를 그린 것 (눈으로 확인) |

학습·검증은 찍은 장소(5 m 칸)로 나뉘어 있다.

## 학습 (학교 GPU 서버)

```bash
yolo detect train model=yolov8n.pt data=raw/<시각>/data.yaml imgsz=640 epochs=100 seed=0
```

학습한 가중치는 git에 넣지 않는다(CONTRIBUTING 6절). 결과 버킷에 올리고 `amr_detection/config/detection.yaml`의 `model_path`, `model_classes: ["person", "cargo", "sign"]`, `output_classes`를 바꾼다.

## 기록

| 날짜 | 데이터 | 결과 |
| --- | --- | --- |
| 10/11 | 시험 수집 20장 (seed 1) | 시도 44번 중 20장 저장. 상자 사람 7·화물 11·표지판 8. 미리보기에서 상자가 물체에 맞는 것 확인 (반쯤 가려진 도크 위 상자, T자 팔까지 포함한 사람) |

## 남은 일

- [ ] 본 수집 (2,500장 안팎) → 클래스별 상자 수 확인
- [ ] 학교 GPU 학습, 검증 mAP
- [ ] `detection.yaml` 교체 후 탐지율·FPS KPI 측정 (Gazebo와 함께)
