# amr_bringup

- 담당: 공동 (`system.launch.py` 조립: 하연)
- 빌드 타입: ament_python
- 책임: 전체 시스템 launch 조립과 다중 로봇 네임스페이스 스폰

다른 패키지의 launch를 include할 수 있는 유일한 패키지다(CONTRIBUTING 4-4절). 각 기능의 동작은 그 패키지 README에 있고, 여기서는 묶는 방법만 정한다.

## 실행

```bash
ros2 launch amr_bringup system.launch.py                             # 로봇 5대, 화면 없이, 카메라 끔
ros2 launch amr_bringup system.launch.py num_robots:=3 gui:=true
ros2 launch amr_bringup system.launch.py delay_ms:=200.0 loss_pct:=5.0 seed:=3 dynamic_obstacles:=true
```

## 단계

| 단계 | 포함 | 상태 |
| --- | --- | --- |
| **1단계** | 월드, 로봇 N대 스폰, 오도메트리·센서 필터, 위치추정(AMCL+EKF), BT(작업 수신·Ack·상태 보고), Fleet 서버, 장애 주입기, 동적 장애물(선택) | 이 launch |
| 2단계 | Nav2(경로 계획·주행), BT → Nav2 연결 | Nav2 설정·launch(이진), BT의 이동 노드(지수) 필요 |
| 3단계 | 구역 클라이언트, 추적, 안전 노드(최종 `cmd_vel`) | `amr_safety`(연우), 구역 클라이언트 프레임 접두어(이진) 필요 |

1단계에서는 BT의 이동이 가짜(stub)라 로봇이 실제로 움직이지 않는다. 그래서 **응답 시간 KPI(명령 → 첫 움직임)는 2단계부터** 잴 수 있다. 1단계로 볼 수 있는 것: 5대 동시 위치추정, Fleet 할당 → BT → 완료 보고 흐름, 5대 CPU 사용률(KPI 80%), RTF.

## 로봇마다 뜨는 것 (`<ns>` = `config/robots.yaml`의 이름)

| 패키지 launch | 역할 | bringup이 넘기는 값 |
| --- | --- | --- |
| `amr_gazebo spawn_robot` | 스폰, ROS ↔ Gazebo 브리지 | 스폰 자세, `publish_odom_tf:=false`(EKF가 odom TF를 냄), `enable_camera` |
| `amr_odometry odometry` | 휠 오도메트리 `/<ns>/odom` | `encoder_seed` (아래 seed) |
| `amr_odometry sensor_filters` | `/<ns>/scan`, `/<ns>/imu/data` | — |
| `amr_localization localization` | AMCL + EKF 두 개 | 초기 위치(스폰 자세를 map 좌표로), 지도 서버는 첫 로봇에서 한 번 |
| `amr_bt amr_bt` | Fleet 작업 수신 | `fleet:=true`면 `relayed:=true`(장애 주입기 중계본) |

전역: `world → map` 정적 TF(지도 원점 `map_origin_in_world`를 한 곳에서 발행, 위치추정을 켤 때. world 좌표 설정을 쓰는 노드는 이 TF로 바꾼다), `amr_gazebo world`(`/clock`), `dynamic_obstacles`(선택), Fleet 서버(`use_sim_time:=true`, 구역 lease를 로봇과 같은 시계로), 장애 주입기(로봇 목록을 `robots.yaml`에서 넘김).

로봇은 `spawn_interval_s`(3초) 간격으로 차례로 띄운다. 한꺼번에 스폰하면 Gazebo RTF가 수십 초 동안 크게 떨어진다(명세서 11-3절).

## 인자

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `num_robots` | `5` | `robots_file` 앞에서부터 |
| `robots_file` | `config/robots.yaml` | 로봇 이름과 스폰 자세(world). 대기 구역 남쪽 끝(y −19)에 2 m 간격, 동적 장애물 경로·범위와 1 m 이상 |
| `gui` | `false` | Gazebo 화면 |
| `enable_camera` | `false` | RGBD 카메라. CPU로 5대를 그리면 RTF 0.37 |
| `dynamic_obstacles` | `false` | 동적 장애물 5개 |
| `spawn_interval_s` | `3.0` | 로봇 스폰 시차 |
| `payload_mass` | `0.0` | 모든 로봇의 적재물 질량 [kg] (연구 2 Gazebo 평가) |
| `seed` | `0` | 실행 seed. 엔코더 노이즈(로봇마다 `seed × 100 + 번호`), 동적 장애물, 손실 난수. 하네스는 1부터 준다. 0이면 엔코더 노이즈는 매번 무작위 |
| `localization` | `true` | 끄면 DiffDrive odom TF를 쓴다 |
| `map` | `amr_mapping maps/warehouse_v1_0025.yaml` | 지도 v1 0.025 m (10/13 결정, #157). 정렬 원점과 함께 쓰면 0.05 m보다 정지 오차가 작다 (GCP 10회 2.09 vs 2.84 cm). Nav2 전역 costmap도 이 해상도를 따라가 칸이 4배다. 경로 계획이 느리면 Nav2만 0.05 m 지도로 나눈다 |
| `map_origin_in_world` | `[-0.0562, 1.5317, 0.00009]` | world에서 본 map 원점. 지도↔월드 정렬값(B안, amr_mapping README "지도 정렬"). 지도를 바꾸면 그 지도의 정렬값 |
| `use_imu` | `true` | EKF에 IMU |
| `bt` | `true` | `amr_bt` |
| `fleet` | `true` | Fleet 서버 + 장애 주입기 |
| `algorithm`, `delay_ms`, `loss_pct`, `relay_auction` | `nearest`, `100.0`, `0.0`, `false` | Fleet·장애 조건 ([amr_fleet_manager](../../fleet/amr_fleet_manager/README.md), [amr_fault_injector](../../fleet/amr_fault_injector/README.md)) |
| `assignments_csv`, `epoch_file`, `kill_at_s`, `restart_after_s`, `fault_events_file` | | 하네스용 Fleet 서버 인자 |

## 테스트

```bash
colcon test --packages-select amr_bringup
```

- `test/test_robots.py`: 로봇 목록, world → map 변환, 로봇별 seed, 스폰 위치가 동적 장애물 경로·범위(`amr_gazebo config/dynamic_obstacles.yaml`)와 1 m 이상 떨어졌는지 (ROS 없이)
- `test/test_system_launch.py`: launch를 불러 include하는 파일이 설치돼 있는지, 로봇 수에 맞게 액션이 만들어지는지 (ROS 환경에서만). 실제로 Gazebo를 띄우지는 않는다

## 미결

| 쟁점 | 내용 |
| --- | --- |
| compose `robots` 서비스 | #73(compose `fleet`)과 같은 파일이라 그 뒤에 이 launch로 연결한다 |
| 7·10대 | `robots.yaml`에 자세를 더하면 된다. RTF가 1을 유지하는지 먼저 재야 한다(연구 1 대수 조건) |
| 2·3단계 | 위 "단계" 표의 담당 작업 뒤 인자로 붙인다 |
