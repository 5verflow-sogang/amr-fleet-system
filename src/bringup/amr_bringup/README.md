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

# 2단계: Nav2 + 구역 클라이언트
ros2 launch amr_bringup system.launch.py nav2:=true zone_client:=true
# 3단계: 추적 + 안전 노드까지 (동적 장애물 앞 감속·정지 확인)
ros2 launch amr_bringup system.launch.py nav2:=true zone_client:=true tracking:=true safety:=true dynamic_obstacles:=true

# 로봇 하나에 목표 보내기 (map 기준)
ros2 action send_goal /amr_01/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

## 단계

| 단계 | 포함 | 상태 |
| --- | --- | --- |
| **1단계** | 월드, 로봇 N대 스폰, 오도메트리·센서 필터, 위치추정(AMCL+EKF), BT(작업 수신·Ack·상태 보고), Fleet 서버, 장애 주입기, 동적 장애물(선택) | 이 launch |
| **2단계** | Nav2(경로 계획·주행), 구역 클라이언트 | `nav2:=true zone_client:=true` (`nav2.launch.py`). 목표는 `navigate_to_pose`로 보낸다 |
| **3단계** | 추적, 안전 노드(최종 `cmd_vel`), 인식(카메라) | `tracking:=true safety:=true` (`detection:=true`는 `enable_camera:=true`도 필요) |
| BT → Nav2 | `amr_bt`의 이동이 Nav2를 부르는 것 | `amr_bt`의 `use_nav2`(지수, #141). 이게 켜지면 Fleet 작업이 실제로 로봇을 움직인다 |

세 인자 모두 기본 꺼짐이라 기본 실행은 1단계와 같다. 1단계와 BT가 가짜인 동안에는 로봇이 작업으로는 움직이지 않는다. 그래서 **응답 시간 KPI(명령 → 첫 움직임)는 BT → Nav2가 켜진 뒤부터** 잴 수 있다. 1단계로 볼 수 있는 것: 5대 동시 위치추정, Fleet 할당 → BT → 완료 보고 흐름, 5대 CPU 사용률(KPI 80%), RTF.

## 로봇마다 뜨는 것 (`<ns>` = `config/robots.yaml`의 이름)

| 패키지 launch | 역할 | bringup이 넘기는 값 |
| --- | --- | --- |
| `amr_gazebo spawn_robot` | 스폰, ROS ↔ Gazebo 브리지 | 스폰 자세, `publish_odom_tf:=false`(EKF가 odom TF를 냄), `enable_camera` |
| `amr_odometry odometry` | 휠 오도메트리 `/<ns>/odom` | `encoder_seed` (아래 seed) |
| `amr_odometry sensor_filters` | `/<ns>/scan`, `/<ns>/imu/data` | — |
| `amr_localization localization` | AMCL + EKF 두 개 | 초기 위치(스폰 자세를 map 좌표로), 지도 서버는 첫 로봇에서 한 번 |
| `amr_bt amr_bt` | Fleet 작업 수신 | `fleet:=true`면 `relayed:=true`(장애 주입기 중계본) |
| `amr_bringup nav2` | planner·controller·behavior·bt_navigator | `nav2:=true`. 프레임 접두어, 스캔 절대 이름, 속도 출력 `cmd_vel_nav` |
| `amr_zone_client` | 좁은 통로 진입 허가·속도 제한 | `zone_client:=true`. 서버와 같은 `zones.yaml`, `base_frame=<ns>/base_footprint`, `use_sim_time` |
| `amr_tracking` | LiDAR 장애물 추적, TTC | `tracking:=true` |
| `amr_detection` | YOLO 인식 | `detection:=true` (카메라 필요) |
| `amr_safety` | 최종 `cmd_vel` | `safety:=true` |

### 속도 명령 체인

켜진 단계만 이어서 마지막이 항상 `/<ns>/cmd_vel`(Gazebo 브리지)이 된다 (`robots.cmd_vel_chain`).

| 켠 것 | 체인 |
| --- | --- |
| nav2만 | Nav2 → `cmd_vel` |
| nav2 + zone_client | Nav2 → `cmd_vel_nav` → 구역 클라이언트 → `cmd_vel` (2단계) |
| nav2 + zone_client + safety | Nav2 → `cmd_vel_nav` → 구역 클라이언트 → `cmd_vel_zone_limited` → 안전 노드 → `cmd_vel` (3단계) |

최종 `cmd_vel`을 내는 것은 안전 노드 하나라는 원칙(9/28 회의)은 3단계에서 지켜진다. 안전 노드가 없는 2단계에서는 구역 클라이언트가 임시로 `cmd_vel`을 낸다.

Nav2는 `nav2_params.yaml`의 DWA를 `FollowPath` 이름으로 등록해 쓴다(bt_navigator 기본 트리가 그 이름을 부른다). 컨트롤러가 속도를 읽는 odom은 `odometry/local`(EKF, 50 Hz)이다.

전역: `amr_gazebo world`(`/clock`), `dynamic_obstacles`(선택), Fleet 서버(`use_sim_time:=true`, 구역 lease를 로봇과 같은 시계로), 장애 주입기(로봇 목록을 `robots.yaml`에서 넘김).

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
| `map` | `amr_mapping maps/warehouse_v1.yaml` | 지도 v1 (0.05 m). 0.025 m는 위치 오차에 효과가 없어 쓰지 않는다 |
| `map_origin_in_world` | `[0.0, 1.5, 0.0]` | world에서 본 map 원점. 지도가 바뀌면 그 지도의 매핑 시작 자세 |
| `use_imu` | `true` | EKF에 IMU |
| `bt` | `true` | `amr_bt` |
| `nav2` | `false` | Nav2. `localization:=true`가 필요하다 (map → odom TF) |
| `zone_client` | `false` | 구역 클라이언트 |
| `tracking` | `false` | 장애물 추적·TTC |
| `safety` | `false` | 안전 노드 |
| `detection` | `false` | YOLO 인식. `enable_camera:=true` 필요 |
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
| 실주행 확인 | 2·3단계 launch는 구조 테스트만 했다. Gazebo에서 5대가 목표에 도착하는지, RTF·CPU는 GCP에서 확인한다 (하연 4주차) |
| Nav2 컨트롤러 | DWA 하나만 쓴다. TEB 선택 인자는 DWA·TEB 비교(#145) 뒤 |
