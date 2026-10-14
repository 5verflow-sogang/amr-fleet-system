# amr_detection

- 담당: 연우
- 빌드 타입: ament_python
- 책임: YOLOv8 객체 인식 노드와 RViz2 마커 발행

## 제공 인터페이스

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Publish | `perception/objects` | `amr_msgs/msg/DetectedObject3DArray` | 카메라 프레임별 인식 결과와 3D 위치·측정 공분산, `target_frame`(기본 `map`) 좌표계. `header.stamp`는 영상 촬영 시각. 검출이 없어도 빈 배열 (TF를 못 구한 프레임은 내지 않는다) |
| Publish | `perception/markers` | `visualization_msgs/msg/MarkerArray` | 객체별 클래스 색 상자와 `Class: <class>, Conf: <conf>, Dist: <dist>m` 텍스트 (RViz 확인용, 수명 1 s) |
| Subscribe | `camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB 영상 (`amr_gazebo`). Depth와 촬영 시각이 같은 짝만 쓴다 |
| Subscribe | `camera/depth/image_raw` | `sensor_msgs/msg/Image` | Depth `32FC1` (m), RGB와 픽셀 1:1. bbox 대표 깊이에 쓴다 |
| Subscribe | `camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 내부 파라미터 K. 처음 받은 값을 쓴다 |

- TF: 영상 촬영 시각의 `camera_optical_frame → target_frame`을 조회한다.
- 처리 과정과 식(YOLO, Pinhole 역투영, 대표 깊이, 공분산 전파, 좌표 변환)은 [docs/algorithms/perception.md 4절](../../../docs/algorithms/perception.md#4-yolo-인식과-3d-변환).

전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

## 파라미터

`config/` 의 YAML로 관리한다. 파라미터 변경에 재빌드가 필요 없어야 한다.

## 실행

```bash
ros2 launch amr_detection detection.launch.py                          # 네임스페이스 amr_01, 출력 map
ros2 launch amr_detection detection.launch.py namespace:=amr_02
ros2 launch amr_detection detection.launch.py target_frame:=amr_01/odom # 위치 추정 없이 시험할 때
ros2 launch amr_detection detection.launch.py device:=cuda:0            # GPU 컨테이너
```

- 로봇 네임스페이스 안에서 띄워야 로봇 카메라(`/<ns>/camera/...`)를 받는다. `amr_bringup`은 `namespace:=<이름>`을 넘긴다.
- 이미지에 `cv_bridge`가 없으면 `apt install ros-humble-cv-bridge`가 필요하다.

## 학습 데이터 수집 (`collect_dataset`, 시뮬 전용)

화물·표지판 클래스를 학습할 데이터를 Gazebo에서 만든다. 라벨은 라벨 카메라(`label_camera:=true`)가 낸 **화면에 보이는 부분의 2D 상자**라 사람이 라벨을 달지 않는다([amr_gazebo README "라벨 카메라"](../../simulation/amr_gazebo/README.md)). 학습(학교 GPU)과 결과 기록은 [experiments/yolo](../../../experiments/yolo/README.md).

```bash
# 결과를 호스트에 남기려고 데이터 폴더만 붙인다 (git 제외 폴더)
docker compose run --rm -v $PWD/experiments/yolo/raw:/data dev bash
# 컨테이너 안
cd /ros2_ws && colcon build --packages-up-to amr_detection && source install/setup.bash
ros2 launch amr_gazebo sim.launch.py gui:=false rviz:=false label_camera:=true cargo:=true dynamic_obstacles:=true obstacle_seed:=1 &
ros2 run amr_detection collect_dataset --out /data --count 2500 --seed 1
```

| Gazebo 라벨 | YOLO 클래스 | 모델 |
| --- | --- | --- |
| 1 | 0 `person` | 작업자 (`dynamic_worker`) |
| 2 | 1 `cargo` | 화물 상자 24개 (`cargo:=true`) |
| 3 | 2 `sign` | 표지판 13개 (월드) |

- 로봇을 지도 v1의 빈 자리(장애물에서 0.6 m 넘게)로 순간이동(`/world/warehouse/set_pose`)하며 찍는다. 70%는 표지판·화물·작업자 하나를 1.5~8 m에서 바라보는 자리, 30%는 아무 데나다.
- 순간이동 뒤 sim 0.3 s가 지난 라벨과 **같은 시각(±5 ms)** RGB 영상을 짝짓는다. 로봇이 요청한 자리에서 0.1 m 넘게 밀려 있으면(충돌) 버린다.
- 물체 없는 사진은 10%만 남긴다. 한 변 4 px 미만 상자만 뺀다. 작은 상자를 빼면 사진에는 물체가 있는데 라벨이 없어 "작은 건 무시"를 배운다 (10 px 기준이면 첫 20장에서 상자 31개 중 5개가 빠졌다).
- 학습·검증은 **장소(5 m 칸)로 나눈다**(검증 약 15%). 같은 장소의 비슷한 사진이 양쪽에 들어가면 검증 성능이 부풀려진다.
- 결과: `<out>/<시각>/` — `images/{train,val}`, `labels/{train,val}` (YOLO 형식), `data.yaml`, `frames.jsonl`(사진마다 자리·상자 수), `meta.json`, `preview/`(처음 30장에 상자를 그린 것).
- 라벨 변환·장소 분할은 `amr_detection/dataset.py`(ROS 비의존, `test/test_dataset.py`), 노드는 `dataset_collector.py`.

## 테스트

```bash
colcon test --packages-select amr_detection
```
