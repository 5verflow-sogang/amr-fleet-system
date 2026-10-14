# yolo

YOLO 인식 3종(사람·화물·표지판, 원 명세 6장)의 학습 데이터셋과 학습 기록. 담당: 연우.

- **데이터 수집**은 ROS 노드라 `src`에 있다: [amr_detection `collect_dataset`](../../src/perception/amr_detection/README.md). 결과를 이 폴더의 `raw/`(git 제외)에 남긴다.
- **학습**은 ROS 없이 ultralytics로 한다 (이 문서). v1은 Mac(Apple M5, MPS)에서 했다. CUDA GPU에서도 같은 명령에 `device`만 바꾼다.
- 학습한 모델은 공개 레포 [ye0n-w0/yolov8n-fine-tuning](https://github.com/ye0n-w0/yolov8n-fine-tuning)의 Release에 두고, `docker/Dockerfile`이 이미지를 만들 때 받는다 (`/opt/models/`).

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

## 학습

컨테이너와 같은 버전으로 학습한다. 다르면 학습한 가중치를 컨테이너에서 못 열 수 있다 (numpy 2로 저장한 파일은 numpy 1.x에서 `numpy._core` 오류).

```bash
# 저장소 밖 환경 (연구 2 환경 uv.lock과 따로)
uv venv ~/yolo-venv --python 3.11
uv pip install --python ~/yolo-venv ultralytics==8.4.164 "numpy<2" "opencv-python<4.12"   # docker/Dockerfile과 같게

# data.yaml의 path는 수집 컨테이너 경로(/data/...)라, 같은 폴더에 path만 호스트 경로로 바꾼 사본(data_mac.yaml)을 만든다
~/yolo-venv/bin/yolo detect train model=yolov8n.pt data=raw/<시각>/data_mac.yaml imgsz=640 epochs=100 patience=20 \
    batch=16 workers=4 cache=ram seed=0 device=mps project=$PWD/raw/runs name=<이름>
```

- 시작 가중치는 컨테이너와 같은 COCO `yolov8n.pt`(ultralytics assets v8.4.0)다.
- `batch` 32는 epoch당 10% 빨랐지만 업데이트 횟수가 절반이라 1 epoch 성능이 낮았다(mAP50 0.48 vs 0.76). 16을 쓴다.

학습한 가중치는 git에 넣지 않는다(CONTRIBUTING 6절). 새 버전은 이렇게 올린다.

1. 공개 레포 Release에 `yolov8n_amr_v<N>.pt`로 올린다 (README에 데이터·설정·결과, 라이선스 AGPL-3.0)
2. `docker/Dockerfile`에 `ADD --checksum=sha256:<해시> <Release 주소> /opt/models/yolov8n_amr_v<N>.pt`
3. `amr_detection/config/detection.yaml`의 `model_path`(필요하면 `model_classes`·`output_classes`)를 바꾸고 이미지를 다시 빌드한다

## 기록

| 날짜 | 데이터 | 결과 |
| --- | --- | --- |
| 10/11 | 시험 수집 20장 (seed 1) | 시도 44번 중 20장 저장. 상자 사람 7·화물 11·표지판 8. 미리보기에서 상자가 물체에 맞는 것 확인 (반쯤 가려진 도크 위 상자, T자 팔까지 포함한 사람) |
| 10/11 | 본 수집 2,500장 (seed 3, `raw/20261011-144602`) | 시도 16,031번 중 2,500장 저장. 학습 1,975 / 검증 525 (장소로 분할). 상자 사람 563·화물 2,929·표지판 1,169 |
| 10/14 | **v1 학습** (`yolov8n_amr_v1.pt`) | COCO `yolov8n`에서 시작, 위 학습 명령 그대로 100 epoch, Mac M5 MPS 1.8시간. 검증 mAP50 **0.931** / mAP50-95 0.870 (사람 0.972·화물 0.912·표지판 0.908). 클래스끼리 헷갈린 경우는 없고, 틀린 것은 대부분 놓침(사람 5%, 화물·표지판 11%). [Release `yolov8n-amr-v1`](https://github.com/ye0n-w0/yolov8n-fine-tuning/releases/tag/yolov8n-amr-v1), SHA-256 `b623606a…53a0` |
| 10/15 | v1 속도 (컨테이너 CPU, 640 입력, 스레드 4) | Gazebo 없이 약 35 ms. Mac Docker에서 Gazebo가 카메라를 CPU로 렌더링하는 중에는 120~150 ms(약 7 FPS): Gazebo가 CPU를 3코어 넘게 쓴다. 같은 조건에서 스레드를 2개보다 늘려도 빨라지지 않고(1개 205·2개 132·4개 139 ms), 320 입력은 약 82 ms |

## 남은 일

- [x] 본 수집 (2,500장 안팎) → 클래스별 상자 수 확인
- [x] 학습, 검증 mAP (v1)
- [x] `detection.yaml` 교체 (`/opt/models/yolov8n_amr_v1.pt`)
- [ ] 탐지율·FPS KPI 측정 (Gazebo와 함께). Mac Docker는 카메라를 CPU로 렌더링해 추론이 느려지므로, 측정 환경과 추론 시간·RTF를 같이 남긴다
