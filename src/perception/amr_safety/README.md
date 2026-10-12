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

`estop_button` 노드 (RViz2 E-Stop 버튼, 아래 절):

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Subscribe | `safety/status` | `amr_msgs/msg/SafetyStatus` | ESTOP 여부로 다음 요청과 버튼 모양을 정한다 |
| Client | `safety/estop` | `std_srvs/srv/SetBool` | 클릭 시 토글 요청 |
| Interactive marker 서버 | `estop_button` | `estop_button/update`·`feedback` 토픽, `estop_button/get_interactive_markers` 서비스 | RViz2 InteractiveMarkers 디스플레이가 붙는다 |

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

launch는 안전 노드와 함께 E-Stop 버튼 노드(`estop_button`)도 띄운다. RViz를 쓰지 않는 헤드리스 실험에서는 `estop_button:=false`로 끈다.

## E-Stop 버튼 (RViz2)

원 명세 7장 "비상 정지(E-Stop) 버튼을 누르면 로봇이 즉시 물리적으로 정지해야 한다"의 버튼이다. `estop_button` 노드가 로봇 차체(`<ns>/base_footprint`) 위 0.8 m(파라미터 `height`)에 interactive marker 상자를 띄우고, 클릭하면 `safety/estop`을 부른다. 판단 규칙은 [`estop_button.py`](amr_safety/estop_button.py)(ROS 비의존)에 있다.

1. GUI를 띄운다: `docker compose --profile gui up -d` → 브라우저 `http://localhost:6080/vnc.html` (비밀번호 `ros`. [runbook](../../../docs/runbook.md) GUI 절)
2. 화면 안 터미널에서 `rviz2 -d $(ros2 pkg prefix amr_description)/share/amr_description/rviz/amr.rviz`를 실행한다. 이 설정에 `E-Stop button`(InteractiveMarkers, `/amr_01/estop_button`)과 `Safety (amr_safety)`(MarkerArray, `/amr_01/safety/markers`) 디스플레이가 들어 있다
3. 툴바에서 Interact 도구를 고른 상태로 차체 위 상자를 클릭한다

| 버튼 | 뜻 | 클릭하면 |
| --- | --- | --- |
| 빨강 `E-STOP` | 정상 (ESTOP 아님) | `{data: true}` 걸기 |
| 노랑 `E-STOP ON - click to release` | ESTOP 걸림 | `{data: false}` 해제 |
| 회색 `E-STOP (no safety status)` | `safety/status`를 1 s 넘게 못 받음 | `{data: true}` 걸기 (모를 때는 멈추는 쪽) |

글자 끝의 `...`는 응답을 기다리는 중이라는 뜻이고, 그동안의 클릭은 무시한다. 서비스가 없거나 실패·거부하면 노드 로그에 남긴다. 글자는 RViz 글꼴에 한글이 없어 영어로 쓴다.

누르면 일어나는 일:

- 안전 노드는 요청을 받는 즉시 단계를 다시 판정해 ESTOP(`speed_limit` 0.0)으로 올리고 `cmd_vel`에 0을 낸다. 회전도 막고, 입력이 없어도 10 Hz로 0을 계속 낸다 (게이트 절)
- 해제 요청 전까지 ESTOP을 유지한다. 해제하면 0.5 s 뒤 다시 판정해 그때 조건에 맞는 단계로 내려온다 (장애물이 있으면 CRITICAL·WARNING)
- 버튼은 로봇마다 하나다. 여러 대면 로봇마다 launch(`namespace:=amr_0X`)가 버튼 노드를 띄우고, RViz에는 로봇마다 InteractiveMarkers 디스플레이(`/amr_0X/estop_button`)를 추가한다. 기본 설정은 `amr_01`만 담는다

## 테스트

```bash
colcon test --packages-select amr_perception_core amr_safety
```
