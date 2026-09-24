# amr_fleet_core

- 담당: 하연
- 빌드 타입: ament_python
- 책임: 작업 할당(최소거리/부하균형/Hungarian), 경로 충돌 예측, 교착 탐지 (ROS 비의존 라이브러리)

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
colcon test --packages-select amr_fleet_core
```
