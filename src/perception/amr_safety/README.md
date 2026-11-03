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
| WARNING | 0.6 (움직이는 트랙 0.5 m 안이면 0.15) | 진행 경로 밖이지만 LiDAR 점이 차체에서 0.3 m 이내 / 움직이는 트랙의 TTC ≤ 2.0 s / 움직이는 트랙이 1.5 m 이내 / 추적·odom 1.0 s 끊김 |
| NORMAL | inf | 그 밖 |

- **진행 경로 판정 (긴급정지):** 명령 (v, ω)대로 `반응 0.2 s + |v| / 1.0 m/s²`(최소 0.5 s) 동안 움직일 때 차체가 지나갈 자리를 본다. 진행 방향 차체 앞(후진이면 뒤) 0.3 m 띠, 또는 차체 0.05 m 안으로 **가까워지는** 점이 있으면 정지. 멀어지는 명령(벽 앞에서 후진)과 나란히 지나가는 벽(0.6 m 좁은 통로)은 막지 않는다.
- **TTC·근접은 움직이는 트랙만:** 정적 장애물은 진행 경로 판정이 지킨다. 정적 장애물에 TTC를 걸면 벽·랙으로 다가가기만 해도 0.5 m 앞에서 선다.
- **근접 저속:** 움직이는 트랙이 차체 0.5 m(`dynamic_slow_distance`) 안이면 방향과 상관없이 0.15 m/s(`dynamic_slow_speed_limit`)까지. 바로 옆에서 멀어지던 사람이 돌아서면 추적 속도가 따라오는 약 0.5 s 동안 TTC가 무한이라, 그 사이 출발한 로봇에 부딪혔다(I17 시드 1019·1029). 풀릴 때는 0.5 s 뒤에 0.6으로 돌아간다. 근거는 [perception.md](../../../docs/algorithms/perception.md) 2절
- 위험해지면 바로 올라가고, 내려갈 때는 조건이 0.5 s 계속 풀려 있어야 내려온다.
- 판정은 명령·스캔이 올 때마다와 20 Hz 타이머로 한다. 명령이 0.5 s 없으면 정지 명령으로 보고 판정한다.

## 게이트 (`gate.limit_velocity`)

- 정지(CRITICAL·ESTOP): 회전도 막는다. 입력이 없어도 10 Hz로 0을 낸다
- WARNING: 선속도 크기를 `speed_limit`(0.6, 근접 저속이면 0.15 m/s) 이하로 (후진 포함). 각속도도 같은 비율로 줄여 곡률을 지킨다. 제자리 회전은 그대로
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

## 회피 평가 (I17)

원 명세 7장 KPI "동적 장애물 회피 30회 충돌 0건, 경로 이탈 1 m 이내, 원래 경로 복귀 5초 이내"([test-plan I17](../../../docs/test-plan.md))를 Gazebo 정답(GT)으로 잰다. 로봇 판단에는 쓰지 않는 **평가 전용** 도구다. 계산은 ROS 비의존 모듈 `amr_safety/avoidance_eval/`(`metrics.py`·`recorder.py`·`cli.py`)이 하고, `avoidance_eval` 노드는 토픽을 받아 넘기기만 한다(`amr_localization` `eval_node`와 같은 구성).

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Subscribe | `ground_truth/odom` | `nav_msgs/msg/Odometry` | 로봇 GT, world 기준 (`amr_gazebo`) |
| Subscribe | `plan` (`plan_topic`) | `nav_msgs/msg/Path` | Nav2 `planner_server` 전역 경로, map 기준. **처음 받은 경로**를 기준 경로로 고정한다 |
| Subscribe | `/dynamic_obstacles/<id>/ground_truth/odom` | `nav_msgs/msg/Odometry` | 장애물 GT, world 기준. `obstacle_ids`가 비면 1초마다 찾아 붙는다 |
| Service | `~/save` | `std_srvs/srv/Trigger` | 지금까지로 계산해 저장. 종료(SIGINT·SIGTERM) 때도 저장한다. 받은 것이 없어도 무효 사유(`로봇 GT 없음`·`장애물 GT 없음`)를 담아 저장한다 |

**지표 정의**

- **이격 거리** `clearance`: 로봇 차체 사각형(0.6×0.4 m, 중심 = `base_footprint`)과 장애물 충돌체 사이 최소 거리. 장애물 모양은 id 앞부분으로 `amr_gazebo` 모델 충돌체를 쓴다: `worker` 원 r 0.25 m, `forklift` 사각형 1.2×0.9 m, `robot` 사각형 0.6×0.4 m (모르는 종류는 `default_obstacle_radius` 원). 장애물 GT(50 Hz)는 로봇 GT(100 Hz) 시각으로 선형 보간하고, 0.2 s 넘게 끊긴 구간은 모르는 것으로 본다.
- **충돌**: 이격 거리 ≤ `collision_margin`(0.01 m)인 샘플이 이어진 구간 하나 = 1건. Gazebo 충돌체는 서로 파고들지 않고 닿은 채 멈춰 이격이 정확히 0이 되지 않을 수 있어 0보다 조금 크게 뒀다. 충돌 순간 로봇·장애물 속력을 함께 남긴다. 최소 이격이 0.3 m 미만이면 `near_miss`(명세서 연구 2 안전성 지표).
- **경로 이탈** `deviation`: 로봇 위치에서 기준 경로 꺾은선까지 최소 거리. 기준 경로는 회피 전에 받은 첫 전역 경로다. Nav2는 주기적으로 재계획해 회피 중 경로가 장애물을 돌아가게 바뀌므로 그 경로로 재면 "원래 경로" 대비 이탈이 0에 가깝게 나온다. 재계획 경로는 `plans.csv`에만 남긴다. 고정 경로로 재려면 `reference_mode:=fixed`, `fixed_path:=[x0, y0, x1, y1, ...]`(map 기준). 기준 경로를 받기 전 샘플은 보지 않는다.
- **회피 구간과 복귀 시간**: 이탈이 `start_deviation`(0.1 m)을 넘은 시각에 구간이 시작한다(`start_on: deviation`). 그 구간에서 이탈이 `return_deviation`(0.2 m)을 마지막으로 넘은 뒤 0.2 m 이하가 `return_hold_s`(1.0 s) 동안 유지되기 시작한 시각이 복귀 시각이고, **복귀 시간 = 복귀 시각 − 시작 시각**이다. 0.2 m 띠를 벗어나지 않았으면 0, 끝까지 못 돌아오면 미복귀(불합격). 시작 2 s 전(`obstacle_lookback_s`)부터 복귀까지 장애물이 `interaction_distance`(1.5 m, 안전 노드 WARNING 근접 기준과 같음) 안에 있었으면 회피 구간, 아니면 추종 오차 구간으로 보고 복귀 KPI에서 뺀다(이탈 KPI에는 넣는다).
  - `start_on: proximity`면 장애물이 1.5 m 안에 들어온 시각부터 잰다(서서 기다린 시간까지 들어간다), `either`면 둘 중 먼저.
- **유효 실행**: 기준 경로가 있고 장애물이 1.5 m 안에 한 번이라도 들어온 실행. 장애물을 안 만난 실행은 회피를 시험한 것이 아니라 30회에 세지 않는다. 무효 사유에 원인을 나눠 적는다: `목표 도착`(로봇은 갔는데 장애물과 시간이 안 맞음, 시나리오 문제), `로봇이 거의 안 움직임`(로봇 GT 이동 거리 `travel_m` < `stuck_travel_m` 0.5 m, Nav2 실패·정지 등), `목표 전에 멈춤`(그 외).
- **실행 합격**: 유효 + 충돌 0건 + 최대 이탈 ≤ 1.0 m + 모든 회피 구간 복귀 ≤ 5.0 s + 목표(기준 경로 끝점 0.3 m 안) 도착. 목표 도착을 넣은 것은 멈춰 서 있기만 해도 충돌·이탈이 0이 되기 때문이다(`require_goal:=false`로 뺄 수 있다).
- **I17 판정** (`avoidance_report`): 원 명세 KPI가 "30회 모두"라 평균·신뢰구간이 아니라 **최악값**으로 본다. 위에서부터 차례로:
  1. 어느 실행에서든 충돌이 있으면 FAIL. 무효 실행과 하네스 실패 실행(`outcome`이 `completed`가 아님)도 요약이 있으면 센다. 충돌은 기준 경로 없이 셀 수 있다
  2. 유효 실행 중 하나라도 불합격(이탈·복귀·목표)이면 FAIL
  3. 하네스 실패 실행이나 요약이 없는 실행이 있거나, 유효 실행이 30회보다 적으면 INSUFFICIENT(판정 보류, 이유를 `insufficient_reasons`에 남긴다). 재지 못한 실행을 빼고 PASS를 주면 성공한 실행만 고르는 셈이다
  4. 그 외 PASS
- 보고서에는 무효 사유별·하네스 실패 outcome별 실행 수와, 유효 실행 합격률의 Clopper-Pearson 95% 신뢰구간을 함께 낸다(`pass_rate`). 판정에는 쓰지 않는 참고값이다. 30/30이어도 구간은 88.4~100%라 실패율 상한이 약 12%다.
- 하네스 요약(`tools/harness` `avoid.*` 지표)은 판정하지 않고 분포만 보여 준다. `pass`는 유효 실행만 넣고(무효는 `valid` 0), 복귀 못 한 회피 수는 `unreturned`로 늘 넣는다.

기준값은 모두 노드 파라미터(`AvoidanceParams` 필드 이름 그대로, 실수 파라미터는 `5.0`처럼 소수점을 붙인다)이고 `avoidance_summary.json`의 `params`에 남는다. 0.1·0.2 m·1.0 s는 장애물 없는 주행의 추종 오차를 보고 확정했다(11/3): 아래 등록 코드로 돌린 로컬 30회에서 장애물이 1.5 m 밖일 때 이탈은 중앙 0.8 cm, p99 6.5 cm였고, 장애물과 무관한 추종 오차 구간은 0개였다(추종 오차가 0.1 m를 자주 넘으면 회피 구간이 잘게 쪼개진다).

**출력** (`output_dir`, 기본 `/tmp/amr_avoidance/<로봇>_<시각>`)

| 파일 | 내용 |
| --- | --- |
| `avoidance.csv` | 로봇 GT 시점마다 `timestamp, robot_x, robot_y, robot_yaw, deviation, clearance, nearest_obstacle, obstacle_speed, episode, collision, robot` (world, m·rad·s, 시뮬레이션 시간) |
| `avoidance_summary.json` | 충돌 건(시각·장애물·속력), 최소 이격, 최대 이탈, 회피 구간별 시작·최대 이탈·복귀 시간·가장 가까운 장애물과 속력, 목표 도착, 로봇 이동 거리(`travel_m`), KPI 판정, 기준값 |
| `robot_gt.csv`, `obstacles_gt.csv`, `plans.csv` | 원자료. 받는 즉시 이어 써서 강제 종료돼도 남는다 |
| `reference_path.csv`, `avoidance_config.json` | 쓴 기준 경로(world), 기록 설정(`map_origin_in_world`·모양·기준값) |

**실행**

```bash
# 본 측정 (아래 "본 측정 등록"): GCP는 한 번에 15회까지라 시드 2001~2015, 2016~2030 두 번 → 받아서 판정
tools/harness/gcp_run.sh nav_avoidance -n 15 --seed-base 2001 --label main --pr 262 --slot b1
tools/harness/gcp_run.sh nav_avoidance -n 15 --seed-base 2016 --label main --pr 262 --slot b2
ros2 run amr_safety avoidance_report <실행 폴더 30개> --runs 30 --out i17.md --json i17.json
# 로컬 (Mac Docker 등, 튜닝은 연습 시드 1001~)
tools/harness/docker_run.sh nav_avoidance -n 30 --seed-base 1001 --label dev

# 손으로: 주행 스택을 띄운 뒤
ros2 run amr_safety avoidance_eval --ros-args -r __ns:=/amr_01 -p use_sim_time:=true \
    -p "map_origin_in_world:=[0.0, 1.5, 0.0]" -p output_dir:=/tmp/amr_avoidance/run1
ros2 service call /amr_01/avoidance_eval/save std_srvs/srv/Trigger

# 기준값을 바꿔 다시 계산 (ROS 불필요, python3 -m amr_safety.avoidance_eval.cli offline ...과 같다)
ros2 run amr_safety avoidance_eval_offline /tmp/amr_avoidance/run1 --set return_deviation=0.15 --out /tmp/re
```

- `map_origin_in_world`: world에서 본 map 원점 `[x, y, yaw]`. 전역 경로(map)를 GT 좌표계(world)로 옮기는 데만 쓴다. 스택이 쓰는 지도의 정렬값과 같아야 한다. 틀리면 그 차이가 경로 이탈에 그대로 섞인다(v1·v2 0.025 m 정렬값은 약 6 cm 차이로, 회피 구간 시작 기준 0.1 m의 절반이 넘는다). `nav_avoidance`는 지도와 정렬값을 params `map`·`map_origin` 한 곳에 두고 스택과 이 노드에 같이 넘긴다(기본 v2 0.025 m `[-0.0439, 1.4712, -0.00088]`, [amr_mapping README](../../localization/amr_mapping/README.md)). 손으로 `nav2_dwa_demo`(map->odom 항등)에 붙이면 map 원점이 스폰 위치와 같은 `[0.0, 1.5, 0.0]`(노드 기본값)이다.
- `nav_avoidance` 시나리오의 장애물은 실행마다 시드로 만든다(`ros2 run amr_safety avoidance_obstacles --seed <시드> --out <파일>`, [`avoidance_eval/obstacles.py`](amr_safety/avoidance_eval/obstacles.py)). 작업자 1명이 x 2.5~4.0 m 중 한 곳에서 통로를 건너고, 방향·처음 위치(만나는 시점)·속도가 시드마다 다르다. 속도는 시드 3개 중 1개가 원 명세 1.0 m/s, 나머지는 0.5~1.5 m/s다. 같은 시드면 같은 시험이다. 만든 설정은 실행 폴더 `obstacles.yaml`에 남는다.
  - 작업자는 로봇을 피하지 않는다. 다만 로봇이 멈춘 채 앞길을 막으면 약 0.5 m 앞에서 서고, 로봇이 5 s 넘게 계속 멈춰 있으면 돌아선다(`yield_to_robots: [amr_01]`, [amr_gazebo README](../../simulation/amr_gazebo/README.md)). 사람은 멈춘 로봇에 걸어 들어가지 않는다. 로봇을 보지 않던 작업자로 돌린 시험 30회(11/1)에서는 충돌 29건 중 22건이 서 있는 로봇에 작업자가 걸어 들어간 것이었다.
- 손으로 돌릴 때 쓰는 고정 예시는 [`config/avoidance_eval_obstacles.yaml`](config/avoidance_eval_obstacles.yaml)(x = 3에서 1.0 m/s)이다.
- 정적 장애물(벽·랙)과의 충돌은 보지 않는다. GT에 정적 물체 모양이 없어서다.

### 본 측정 등록 (11/3, 결과 전, 이진·연우)

결과를 보기 전에 정한다. 측정 전에 무엇이든 바꾸면 이 절을 먼저 고치고 잰다.

- **코드**: 이 절을 넣은 #262 커밋 하나로 고정한다. #263(작업자 비킴), #268(DWA 회피 모드 대체 탐색), #277(안전 노드 근접 저속, dev에 머지됨)과 dev 3200aeb가 들어 있다. 측정 중에는 push하지 않는다
  - 결과에 측정 커밋 번호를 적는다. #263·#268이 dev에 머지된 뒤, 그 dev 커밋과 측정 커밋의 I17 관련 코드(`src/navigation`, `src/perception`, `src/simulation/amr_gazebo`, `src/bringup`, `tools/harness/scenarios/nav_avoidance.yaml`)가 같은지 `git diff`로 확인해 결과에 적는다
  - 빌드는 RelWithDebInfo(-O2, dev #269)다. 11/1 GCP 시험 30회와 11/2 로컬 확인 일부는 그 전(최적화 없음)이라 본 측정과 조건이 다르다
- **시나리오**: `nav_avoidance` (지도 v2 0.025 m, 원점 `[-0.0439,1.4712,-0.00088]`). Nav2(DWA 회피 모드) + 추적 + 안전 노드, 로봇 world (0, 1.5) → map (6, 0). 작업자 1명, 시드 3개 중 1개는 1.0 m/s, 나머지 0.5~1.5 m/s (위 장애물 설명)
  - **주행 중 장애물 인식은 LiDAR만 쓴다** (지수·이진과 정한 구성): 카메라 끔(`enable_camera:=false`), Nav2 global·local costmap은 `scan`만, Depth 포인트클라우드는 costmap에 붙이지 않는다. 추적의 사람 반경은 `dynamic_min_radius` 0.25 m
- **시드**: 2001~2030 (`gcp_run.sh --seed-base 2001`·`2016`으로 15회씩 두 번, #283). 1~30은 11/1 시험, 1001~은 튜닝에 썼다. 실행마다 `meta.json`의 `seed`·`scene_seed`가 2001~2030인지 본다
- **판정**: `avoidance_report` 최악값(위 "I17 판정"). 30회 모두 충돌 0, 최대 이탈 ≤ 1.0 m, 복귀 ≤ 5.0 s, 목표 도착. 충돌은 로봇이 서 있든 움직이든 모두 센다
  - 시간 지표(복귀 시간 등)는 GT 메시지 시각, 곧 시뮬레이션 시간으로 잰다(`use_sim_time`). RTF와 상관없다(#272는 실제 시간으로 기다리는 시나리오에서 하네스 RTF가 늘 1.0으로 찍히던 표시 문제였다)
- **무효 실행**: `avoidance_eval`이 무효로 본 실행과 하네스 `error`(예: Nav2 시작 단계 통신 문제로 전역 경로가 안 나옴)는 같은 시드로 다시 돌린다. 다시 돌린 시드·횟수·이유를 결과에 적는다
- **러너**: GCP c2d-standard-8 (슬롯 b1~b4, 11/1 시험과 같은 종류). RTF를 결과에 함께 적는다
- **설정**: 바꾸지 않는다
  - DWA 회피 모드: `avoid_ttc_enter_s` 3.0, `avoid_margin_add` 0.4, `avoid_weight_scale` 2.0, `avoid_rollout_time` 3.0, 궤적이 없으면 평소 여유로 다시 탐색(#268)
  - 안전 노드: `dynamic_slow_distance` 0.5 m, `dynamic_slow_speed_limit` 0.15 m/s(#277), 그 밖은 `config/safety.yaml`
  - 평가: `start_deviation` 0.1 m, `return_deviation` 0.2 m, `return_hold_s` 1.0 s, `collision_margin` 0.01 m (기본값)
- **이 코드로 한 로컬 확인** (Mac Docker, RTF 0.96, 연습 시드 1001~1030, 무효는 같은 시드로 다시): `avoidance_report` PASS, 유효 30/30 충돌 0, 최대 이탈 0.147 m, 최대 복귀 0 s, 최소 이격 0.058 m. #268·#277 없이는 27/30, #268만은 28/30이었고 충돌 5건이 모두 "작업자가 로봇 옆 0.2~0.3 m에서 돌아선 직후 로봇 출발"이었다(#277 본문)

### 본 측정 결과 (11/3, GCP)

**I17 PASS** (`avoidance_report`, 최악값): 유효 30/30회, 충돌 0건, 최대 경로 이탈 0.082 m, 복귀 못 한 회피 0, 목표 미도착 0. 최소 이격 0.079 m.

- 조건: 위 등록 그대로다. 커밋 23e461c, GCP c2d-standard-8, 시드 2001~2030(실행 41584943322·41584964100), RTF 0.938~0.955
- 다시 돌린 실행 1회: 시드 2028. bt_navigator가 목표 응답을 보내다 시간 초과로 놓쳐(`Failed to send goal response (timeout)`) 로봇이 출발하지 못했다(하네스 `error`). 등록대로 같은 시드로 다시 돌렸고(실행 41588707469) 통과했다(최소 이격 0.099 m)
- 작업자 속력별: 1.0 m/s(원 명세 조건) 10/10, 1.0 m/s 미만 10/10, 초과 10/10
- 회피 방식 (참고, 판정에 안 씀): 30회 모두 경로 이탈이 0.1 m 안이라 회피 구간이 생기지 않았다. 그래서 복귀 시간은 잰 값이 없다(KPI 조건은 만족). 로봇은 옆으로 비키지 않고(옆 이동 최대 9 cm) 늦추거나 서서 피했다. DWA 회피 모드가 켜진 실행 24회, 근접 저속 28회, 안전 노드 정지(TTC) 23회, 작업자가 1.5 m 안일 때 서 있던 시간 중앙 1.5 s(최대 5.8 s). 작업자가 로봇 길을 가로지르는 장면이라 옆으로 비키면 작업자가 걷는 선 위로 들어가고, 로봇 선이 통로 남쪽 벽에서 1.1 m라 비킬 자리도 좁다. 0.3 m 안까지 스친 실행은 23회다
- 남은 것: #263·#268이 dev에 머지된 뒤 그 dev 커밋과 23e461c의 I17 관련 코드가 같은지 확인해 여기 적는다(등록)
- 원자료: 버킷 `gs://amr-fleet-results/runs/nav_avoidance-main/` (실행 3개 폴더)

## 테스트

```bash
colcon test --packages-select amr_perception_core amr_safety
```

- `test_gate.py`, `test_safety_node.py`: 노드 없이 게이트·판정 입력·메시지 변환
- `test_safety_launch.py` (I14, `launch_testing`): `safety_node`를 띄우고 가짜 입력으로 NORMAL → 진행 경로 장애물 CRITICAL → TTC WARNING(0.6 m/s, 곡률 유지) → 스캔 끊김 CRITICAL을 차례로 걸어 `safety/status`와 최종 `cmd_vel`을 본다
