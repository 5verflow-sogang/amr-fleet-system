# amr_tracking

- 담당: 연우
- 빌드 타입: ament_python
- 책임: LiDAR 동적 장애물 분류·추적, 카메라 검출 연관, 충돌 예측 노드

## 제공 인터페이스

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Publish | `perception/obstacles` | `amr_msgs/msg/TrackedObstacleArray` | 확정 트랙의 위치·속도·공분산(KF P)·반경. 스캔마다, 장애물이 없어도 빈 배열 |
| Publish | `perception/obstacle_markers` | `visualization_msgs/msg/MarkerArray` | 트랙 원기둥(반경, 동적 빨강·정적 회색)·속도 화살표·`#ID 속력` (RViz 확인용) |
| Subscribe | `scan` | `sensor_msgs/msg/LaserScan` | 필터된 LiDAR 스캔 (`amr_odometry` scan_filter). `measurement_source: scan`(기본) |
| Subscribe | `perception/obstacle_measurements` | `geometry_msgs/msg/PoseArray` | 시험용 위치 측정 (`synthetic_obstacles_node`). `measurement_source: poses` |
| Service | | | |
| Action | | | |

전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

- 처리 순서 (스캔마다): 군집화 → R1 크기 규칙(2 m 넘는 군집은 벽·랙 면) → 다리 묶기(가까운 작은 군집 두 개를 한 사람으로) → 스캔 촬영 시각의 `lidar_link → odom` TF로 변환 → KF 추적 → R3 움직임으로 `is_dynamic`. 근거는 [perception.md](../../../docs/algorithms/perception.md) 1절.
- 출력은 **`odom` 좌표계**다 (`fixed_frame`을 비우면 `<네임스페이스>/odom`, 예: `amr_01/odom`). `header.stamp`는 스캔 촬영 시각이다.
  - 소비자인 DWA가 odom에서 동작한다 (#47 리뷰, 이진).
  - `map`은 위치 추정이 보정될 때 순간 이동해 KF가 가짜 속도를 추정한다. `odom`은 연속적이다.
  - 로봇 기준 좌표(`lidar_link` 등)를 그대로 넣으면 로봇의 움직임이 장애물 속도로 잡히므로, 측정 쪽에서 `odom`으로 변환해 넣는다.
- `radius`는 군집 반경의 지수 평활이다. 군집 중심은 센서 쪽 표면으로 치우쳐(반경 0.25 m 원기둥이면 약 0.2 m) 반경이 실제보다 조금 크게 나온다. 충돌 반경으로는 보수적인 쪽이다.
- `is_dynamic`은 속력이 임계값을 5프레임 연속 넘으면 true로 바뀌고 유지된다. `class_name`은 카메라 연관 전이라 `unknown`.
- 지도 대조(R2)가 아직 없어 기둥·랙 끝·다리 사이로 보이는 벽 조각도 **정적 트랙**(`is_dynamic` false)으로 나간다. 소비자는 `is_dynamic`과 관계없이 장애물로 다룬다.
- 측정 시각이 거꾸로 가면(시뮬레이션 리셋, rosbag 반복) 경고를 남기고 트랙을 초기화한다. ID는 이어서 매긴다.

### 노드

| 실행 파일 | 노드 이름 | 역할 |
| --- | --- | --- |
| `tracking_node` | `tracking` | 스캔(또는 위치 측정) → 다중 장애물 KF 추적 → `perception/obstacles`, `perception/obstacle_markers`. 5초마다 프레임당 후보·트랙·동적 수와 처리 시간 로그 |
| `synthetic_obstacles_node` | `synthetic_obstacles` | 시험용 측정 발행기. 장애물 2개(sin 왕복 최대 1.0 m/s, 원 궤도 0.8 m/s)에 잡음 σ 0.05 m |

## 파라미터

`config/` 의 YAML로 관리한다. 파라미터 변경에 재빌드가 필요 없어야 한다.
값과 근거는 [config/tracking.yaml](config/tracking.yaml)에 있다. KF·트래커 알고리즘은 [amr_perception_core](../amr_perception_core/README.md)의 `kalman`, `tracker`다.

## 실행

```bash
# 시뮬레이터 + 센서 필터와 함께 (로봇 네임스페이스 안에서, odom TF 필요)
ros2 launch amr_gazebo sim.launch.py dynamic_obstacles:=true
ros2 launch amr_odometry sensor_filters.launch.py
ros2 run amr_tracking tracking_node --ros-args -r __ns:=/amr_01 \
  --params-file $(ros2 pkg prefix amr_tracking)/share/amr_tracking/config/tracking.yaml -p use_sim_time:=true
# RViz: MarkerArray /amr_01/perception/obstacle_markers (Fixed Frame amr_01/odom)

# 시험용 측정 발행기와 함께 (시뮬레이터 없이)
ros2 launch amr_tracking tracking.launch.py use_synthetic:=true use_sim_time:=false
ros2 topic echo /perception/obstacles
```

## 테스트

```bash
colcon test --packages-select amr_tracking
```
