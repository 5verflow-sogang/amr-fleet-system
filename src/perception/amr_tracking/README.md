# amr_tracking

- 담당: 연우
- 빌드 타입: ament_python
- 책임: LiDAR 동적 장애물 분류·추적, 카메라 검출 연관, 충돌 예측 노드

## 제공 인터페이스

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Publish | `perception/obstacles` | `amr_msgs/msg/TrackedObstacleArray` | 확정 트랙의 위치·속도·공분산(KF P). 측정 프레임마다, 장애물이 없어도 빈 배열 |
| Subscribe | `perception/obstacle_measurements` | `geometry_msgs/msg/PoseArray` | 장애물 위치 측정 (인지 도메인 내부). LiDAR 군집화 노드가 발행할 예정 |
| Service | | | |
| Action | | | |

전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

- 측정과 출력은 **`odom` 좌표계**다 (네임스페이스가 붙어 `amr_0N/odom`). 출력의 `header`(시각·좌표계)는 측정을 그대로 따른다.
  - 소비자인 DWA가 odom에서 동작한다 (#47 리뷰, 이진).
  - `map`은 위치 추정이 보정될 때 순간 이동해 KF가 가짜 속도를 추정한다. `odom`은 연속적이다.
  - 로봇 기준 좌표(`lidar_link` 등)를 그대로 넣으면 로봇의 움직임이 장애물 속도로 잡히므로, 측정 쪽에서 `odom`으로 변환해 넣는다.
- 임시 규칙 (LiDAR 동적 장애물 분류 작업에서 교체): `radius`는 고정값, `is_dynamic`은 속력이 임계값을 5프레임 연속 넘으면 true로 바뀌고 유지된다. `class_name`은 카메라 연관 전이라 `unknown`.
- 측정 시각이 거꾸로 가면(시뮬레이션 리셋, rosbag 반복) 경고를 남기고 트랙을 초기화한다. ID는 이어서 매긴다.

### 노드

| 실행 파일 | 노드 이름 | 역할 |
| --- | --- | --- |
| `tracking_node` | `tracking` | 측정 → 다중 장애물 KF 추적 → `perception/obstacles` |
| `synthetic_obstacles_node` | `synthetic_obstacles` | 시험용 측정 발행기. 장애물 2개(sin 왕복 최대 1.0 m/s, 원 궤도 0.8 m/s)에 잡음 σ 0.05 m |

## 파라미터

`config/` 의 YAML로 관리한다. 파라미터 변경에 재빌드가 필요 없어야 한다.
값과 근거는 [config/tracking.yaml](config/tracking.yaml)에 있다. KF·트래커 알고리즘은 [amr_perception_core](../amr_perception_core/README.md)의 `kalman`, `tracker`다.

## 실행

```bash
# 시험용 측정 발행기와 함께 (LiDAR 군집화 노드가 나오기 전)
ros2 launch amr_tracking tracking.launch.py use_synthetic:=true
ros2 topic echo /perception/obstacles
```

## 테스트

```bash
colcon test --packages-select amr_tracking
```
