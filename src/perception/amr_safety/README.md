# amr_safety

- 담당: 연우
- 빌드 타입: ament_python
- 책임: 긴급정지, Safety Zone, E-Stop, 센서 고장 감지. 로봇의 **최종 `cmd_vel`** 을 낸다 (9/28 합의)

판정 규칙은 ROS 비의존 모듈 [`amr_perception_core/safety.py`](../amr_perception_core/amr_perception_core/safety.py)(`SafetyMonitor`)와
[`collision.py`](../amr_perception_core/amr_perception_core/collision.py)(`path_hazard`)에 있고, 이 패키지는 ROS 입출력과 게이트를 맡는다.
알고리즘 근거와 실검증 기록은 [docs/algorithms/perception.md 2절](../../../docs/algorithms/perception.md#2-안전-노드).

## 제공 인터페이스

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Subscribe | `scan` | `sensor_msgs/msg/LaserScan` | 필터 스캔 (`amr_odometry`). 진행 경로 판정은 이 원본 점으로 한다 |
| Subscribe | `perception/obstacles` | `amr_msgs/msg/TrackedObstacleArray` | 움직이는 장애물 근접 판정 |
| Subscribe | `perception/collision_risks` | `amr_msgs/msg/CollisionRiskArray` | 트랙별 TTC (`amr_tracking`) |
| Subscribe | `odom` | `nav_msgs/msg/Odometry` | 끊김 감시 |
| Subscribe | `cmd_vel_zone_limited` (파라미터 `cmd_vel_in_topic`) | `geometry_msgs/msg/Twist` | 게이트 입력 (`amr_zone_client` 출력) |
| Publish | `cmd_vel` (파라미터 `cmd_vel_out_topic`) | `geometry_msgs/msg/Twist` | **최종 속도 명령** |
| Publish | `safety/status` | `amr_msgs/msg/SafetyStatus` | 단계·원인·`speed_limit`. 바뀌면 즉시, 그 외 10 Hz |
| Publish | `safety/markers` | `visualization_msgs/msg/MarkerArray` | 차체, 0.3 m 경계(단계 색), 1.5 m 경계, 상태 글자 (RViz 확인용) |
| Service | `safety/estop` | `std_srvs/srv/SetBool` | `true` 걸기, `false` 해제 요청 |

cmd_vel 체인: `cmd_vel_nav` (Nav2·`amr_control`) → `amr_zone_client` → `cmd_vel_zone_limited` → **`amr_safety`** → `cmd_vel`

## 판정 규칙

| 단계 | `speed_limit` | 조건 (하나라도) |
| --- | --- | --- |
| ESTOP | 0.0 | E-Stop 요청이 걸려 있음 (해제 요청 전까지) |
| CRITICAL | 0.0 | 지금 명령의 **진행 경로**에 LiDAR 점이 걸림 / 움직이는 트랙의 TTC ≤ 1.0 s / 스캔 0.5 s 끊김 |
| WARNING | 0.6 | 진행 경로 밖이지만 LiDAR 점이 차체에서 0.3 m 이내 / 움직이는 트랙의 TTC ≤ 2.0 s / 움직이는 트랙이 1.5 m 이내 / 추적·odom 1.0 s 끊김 |
| NORMAL | inf | 그 밖 |

- **진행 경로 판정 (긴급정지):** 명령 (v, ω)대로 `반응 0.2 s + |v| / 1.0 m/s²`(최소 0.5 s) 동안 움직일 때 차체가 지나갈 자리를 본다. 진행 방향 차체 앞(후진이면 뒤) 0.3 m 띠, 또는 차체 0.05 m 안으로 **가까워지는** 점이 있으면 정지. 멀어지는 명령(벽 앞에서 후진)과 나란히 지나가는 벽(0.6 m 좁은 통로)은 막지 않는다.
- **TTC·근접은 움직이는 트랙만:** 정적 장애물은 진행 경로 판정이 지킨다. 정적 장애물에 TTC를 걸면 벽·랙으로 다가가기만 해도 0.5 m 앞에서 선다.
- 위험해지면 바로 올라가고, 내려갈 때는 조건이 0.5 s 계속 풀려 있어야 내려온다.
- 판정은 명령·스캔이 올 때마다와 20 Hz 타이머로 한다. 명령이 0.5 s 없으면 정지 명령으로 보고 판정한다.

## 게이트 (`gate.limit_velocity`)

- 정지(CRITICAL·ESTOP): 회전도 막는다. 입력이 없어도 10 Hz로 0을 낸다
- WARNING: 선속도 크기를 0.6 m/s 이하로 (후진 포함). 각속도도 같은 비율로 줄여 곡률을 지킨다. 제자리 회전은 그대로
- NaN·inf 명령은 정지로 바꾼다. 입력이 0.5 s 끊기면 0을 한 번 낸다 (Gazebo DiffDrive는 마지막 명령을 따른다). 노드가 내려갈 때도 0을 낸다

## 파라미터

`config/safety.yaml`. 값마다 근거를 주석으로 적었다.

## 실행

```bash
ros2 launch amr_safety safety.launch.py                                   # 네임스페이스 amr_01
ros2 launch amr_safety safety.launch.py cmd_vel_in_topic:=cmd_vel_nav     # 구역 허가 없이 시험
ros2 service call /amr_01/safety/estop std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /amr_01/safety/status
```

## 테스트

```bash
colcon test --packages-select amr_perception_core amr_safety
```
