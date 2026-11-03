# amr_nav2_plugins

- 담당: 이진
- 빌드 타입: ament_cmake (C++17)
- 책임: `amr_planning_core`를 Nav2 Controller 플러그인으로 래핑

## 제공 플러그인

| 플러그인 클래스 | base_class | 역할 |
| --- | --- | --- |
| `amr_nav2_plugins::DwaController` | `nav2_core::Controller` | `amr_planning_core::DwaPlanner`(DWA, 동적 장애물 등속 예측 포함)를 감싼다 |

`plugins.xml`으로 내보낸다. `nav2_params.yaml`의 `controller_server.controller_plugins`에
`"DWA"`로 추가하면 로드된다(기존 DWB baseline `"FollowPath"`는 비교용으로 남겨 둔다).

### `DwaController`가 하는 일

1. `configure()`에서 `amr_bringup/config/nav2_params.yaml`의 `DWA:` 섹션을 읽어
   `amr_planning_core::DwaConfig`를 만들고 `perception/obstacles`(상대 이름,
   `amr_msgs/msg/TrackedObstacleArray`)를 구독한다. 움직이는 트랙(`is_dynamic`)만
   동적 장애물로 쓴다 - 정적 트랙은 costmap에 이미 있다(`obstacles.hpp`).
2. `computeVelocityCommands()`에서: local costmap을 `amr_planning_core::Grid`로
   감싸고(costmap origin을 빼 Grid-로컬 좌표로), 전역 경로를 같은 좌표의 점 목록으로
   바꿔 구독해 둔 장애물과 함께 `DwaPlanner`에 위임한다. heading 기준점(경로를 따라
   `max(lookahead_distance, |v|·sim_time)` 앞, `adaptive_lookahead`)과 목표 앞 감속의
   최종 목표(경로 마지막 점)는 코어가 고른다(docs/algorithms/navigation.md 4-7·4-8절).
3. `setPlan()`에서 전역 경로가 costmap 전역 좌표계와 다르면 tf로 변환한다.
4. `setSpeedLimit()`은 Nav2 관례(음수=제한 없음, `percentage`면 `max_v` 대비 비율)를
   그대로 따른다.
5. TTC 회피 모드(`avoid_mode_enabled`, docs/algorithms/navigation.md 4-10절):
   `perception/collision_risks`(`amr_msgs/msg/CollisionRiskArray`)를 구독해 움직이는 트랙의
   최소 TTC로 켜고 끈다(`avoid_mode.hpp`의 `AvoidModeSwitch`, 시간 처리). 바뀌면
   `DwaPlanner::setAvoidMode`로 넘기고 `dwa/avoid_mode`(`std_msgs/msg/Bool`, transient local)를 발행한다.

## 파라미터

`amr_bringup/config/nav2_params.yaml`의 `controller_server.DWA` 섹션. 값은
`amr_planning_core::DwaConfig` 기본값과 같다(각 필드 근거는
[amr_planning_core README](../amr_planning_core/README.md#amr_planning_coredwaplanner-dwahpp)).
단, 좁은 통로(폭 0.6 m) 진입을 위해 `lethal_cost_threshold`는 `254`(기본 `253`),
`lookahead_distance`는 `0.5` m(기본 `1.5` m)로 둔다(이유는 yaml 주석).
추가로:

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `lookahead_distance` | `1.5` m | 전역 경로에서 이 거리만큼 앞의 점을 DWA의 heading 기준점으로 쓴다(`adaptive_lookahead`면 최솟값) |
| `adaptive_lookahead` | `true` | heading 기준점을 궤적 길이(`|v|·sim_time`)만큼 앞으로 늘린다 |
| `use_admissible_velocity` | `true` | 허용 속도 조건 `v <= sqrt(2·acc_v·dist)` (dist: 궤적상 장애물까지) |
| `use_goal_slowdown` | `true` | 목표 앞 감속 `v <= sqrt(2·acc_v·d_goal)` (d_goal: 최종 목표까지 직선 거리) |

## 실행

```bash
ros2 launch amr_bringup nav2_dwa_demo.launch.py gui:=false
```

Gazebo 월드 v0에서 지도 v0(`amr_mapping/maps/warehouse_v0.yaml`)와 map→odom 항등
변환(AMCL 없이)으로 `DwaController` 하나만 올려 목표점 1곳 주행을 확인하는 데모다
(`amr_bringup/launch/nav2_dwa_demo.launch.py`). 로봇 1대(`amr_01`)만 쓰고, `nav2_common.
launch.RewrittenYaml`로 costmap의 `robot_base_frame`·`global_frame`에 네임스페이스를
붙인다. 알려진 상태: 목표가 수락되고 BT가 시작되지만(`Begin navigating from ...`),
그 뒤 `cmd_vel`이 전혀 나오지 않는 지점에서 막혀 있다 — 전역 costmap이 로봇을 범위
밖으로 보는 경고(`Robot is out of bounds of the costmap!`)가 같이 나와, Nav2 bringup
쪽(지도 원점·costmap 크기 정렬 추정) 문제로 보고 있다. `DwaController` 자체는
`pluginlib` 로드 테스트(아래)로 별도 확인했다.

## 테스트

```bash
colcon test --packages-select amr_nav2_plugins
```

- `test/test_waypoint.cpp`: `selectLookaheadWaypoint` 순수 함수 단위 테스트(ROS 비의존).
- `test/test_plugin_loadable.cpp`: `pluginlib::ClassLoader<nav2_core::Controller>`로
  `amr_nav2_plugins::DwaController`를 실제로 로드할 수 있는지 확인한다(`configure()`는
  호출하지 않는다 — 실제 노드·costmap이 필요해서 무겁다).
