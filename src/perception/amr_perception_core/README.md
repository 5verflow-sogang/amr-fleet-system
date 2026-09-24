# amr_perception_core

- 담당: 연우
- 빌드 타입: ament_python
- 책임: Pinhole 3D 변환, KF/AKF, TTC/Probabilistic TTC, VO/ORCA (ROS 비의존 라이브러리)

## 공개 API

ROS 토픽을 직접 다루지 않는다. 공개 함수·클래스 목록을 여기에 적는다.

## 파라미터

ROS 비의존 라이브러리이므로 파라미터는 함수/생성자 인자로 받는다. YAML 로딩은 이 라이브러리를 쓰는 노드 패키지가 담당한다.

## 실행

```bash
# TODO
```

## 테스트

```bash
colcon test --packages-select amr_perception_core
```
