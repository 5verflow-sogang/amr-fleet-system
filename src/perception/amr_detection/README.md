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

## 테스트

```bash
colcon test --packages-select amr_detection
```
