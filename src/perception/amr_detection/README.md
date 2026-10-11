# amr_detection

- 담당: 연우
- 빌드 타입: ament_python
- 책임: YOLOv8 객체 인식 노드와 RViz2 마커 발행

## 제공 인터페이스

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Publish | | | |
| Subscribe | | | |
| Service | | | |
| Action | | | |

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
