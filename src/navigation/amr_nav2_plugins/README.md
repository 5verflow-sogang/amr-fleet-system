# amr_nav2_plugins

- 담당: 이진
- 빌드 타입: ament_cmake (C++17)
- 책임: `amr_planning_core`를 Nav2 Controller·Global Planner 플러그인으로 래핑

## 제공 플러그인

| 플러그인 클래스 | base_class | 역할 |
| --- | --- | --- |
| `amr_nav2_plugins::DwaController` | `nav2_core::Controller` | `amr_planning_core::DwaPlanner`(DWA, 동적 장애물 등속 예측 포함)를 감싼다 |
| `amr_nav2_plugins::AStarGlobalPlanner` | `nav2_core::GlobalPlanner` | `amr_planning_core::AStarPlanner`(8방향 A*) + `smoothPath`(평활화)를 감싼다. 비교용 - 기본 planner는 NavFn |
| `amr_nav2_plugins::FleetPeerLayer` | `nav2_costmap_2d::Layer` | `/fleet/robot_states`의 다른 로봇 위치를 외삽해 차체를 LETHAL로 그린다 (아래) |

`plugins.xml`(Controller·Global Planner)과 `layers.xml`(costmap 레이어)로 내보낸다. `nav2_params.yaml`의 `controller_server.controller_plugins`에
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
6. 정면 비켜 지나가기(`passing_enabled`, docs/algorithms/navigation.md 4-11절): 매 주기 전역 경로를
   `amr_planning_core::PassingManeuver`에 넘겨, 마주 오는 장애물이 막으면 옆으로 옮긴 경로를 `DwaPlanner`에 준다.
   비키는 동안 `DwaPlanner::setPassingTracking`으로 가까운 기준점으로 따라가고, 곧 비킬 예정이거나 비키는 중이면
   회피 모드를 끈다. 바뀌면 `dwa/passing`(`std_msgs/msg/Bool`, transient local)을 발행하고 INFO 로그를 남긴다.

### `AStarGlobalPlanner`가 하는 일

1. `configure()`에서 `planner_server.<이름>` 파라미터(아래)를 읽어 `AStarConfig`·`SmoothingConfig`를 만든다.
2. `createPlan()`에서 전역 costmap을 잠그고 `Grid`로 감싼 뒤, 출발·목표를 셀로 바꿔 `AStarPlanner::plan` →
   `smoothPath`에 맡긴다. 결과(셀 단위)는 `toPlanPath`(`plan_path.hpp`, ROS 노드 없이 테스트)가 `nav_msgs/Path`로 바꾼다
   (점마다 다음 점 방향, 마지막 점은 요청한 목표 자세). 실패하면 빈 경로를 돌려준다(planner_server가 실패 처리).
3. 고르는 법: `ros2 launch amr_bringup nav2.launch.py planner:=astar` - `planner_server.AStar` 블록을 `GridBased`
   이름에 얹는다(BT의 `planner_id`를 그대로 쓰려고, `amr_bringup/planner.py`).

### `FleetPeerLayer`가 하는 일

LiDAR 스캔면(지면 0.38 m = `base_link` 0.18 + extrinsic z 0.2)이 다른 로봇 차체(0.03~0.33 m) 위를 지나가,
`obstacle_layer`에는 다른 로봇이 LiDAR 원통(반경 4 cm) 한 점으로만 찍힌다. 그래서 로봇끼리 서로를 피하지 않고 부딪혀
밀다 섰다(11/13 `fleet_stack` resp50: amr_03·04가 중심 거리 0.55 m로 200 s, 정체 14·24건 대부분 마주침).

1. `/fleet/robot_states`(`amr_bt`가 2 Hz, RELIABLE)를 구독해 나(노드 네임스페이스 첫 칸 또는 `robot_id`)를 뺀
   로봇의 위치를 모은다. 연속 두 상태로 속도를 추정해 지금 시각으로 외삽한다(`max_extrapolation` 상한, `timeout` 넘게 소식이
   없으면 뺀다. `fleet_peers.hpp`, ROS 없이 테스트).
2. `updateBounds`에서 차체 사각형(`peer_half_length`·`peer_half_width` + `padding`)을 costmap 좌표계로 옮기고(TF), 지난
   주기에 그린 영역까지 bounds에 넣는다 - LayeredCostmap이 그 영역을 지운 뒤 새로 그리므로 `obstacle_layer`처럼 표시가 남지
   않는다.
3. `updateCosts`에서 사각형 셀을 LETHAL로 칠하되 **내 차체 안쪽 셀은 비운다**. 이미 붙어 있을 때 내 자리까지 LETHAL이면
   DWA(`lethal_cost_threshold` 254)가 멀어지는 궤적도 고르지 못한다.
4. `nav2_params.yaml`의 전역·지역 costmap에서 `inflation_layer` 앞에 둔다(팽창된다). 식당 프로필은 서빙 로봇(1.16 m)이
   LiDAR로 서로 보이므로 끈다(`amr_bringup/nav2_profile.py`). 파라미터 이름을 `topic`이 아니라 `states_topic`으로 둔 것은
   `nav2.launch.py`의 `RewrittenYaml`이 `topic` 키를 모두 스캔 토픽으로 바꾸기 때문이다.

## 파라미터

`AStarGlobalPlanner`: `nav2_params.yaml`의 `planner_server.AStar` 섹션 (근거는 docs/algorithms/navigation.md 3절).

| 파라미터 | 값 | 설명 |
| --- | --- | --- |
| `heuristic` | `octile` | `octile` \| `euclidean` |
| `lethal_cost_threshold` | `253` | 이 비용 이상인 셀은 지나갈 수 없다 |
| `cost_weight` | `4.0` | 이동 비용 = 거리 × (1 + w × 비용/255). NavFn과 같은 비율 |
| `prevent_corner_cutting` | `true` | 대각선 이동은 양옆 셀이 둘 다 비어야 |
| `smooth_path` | `true` | 평활화 전체 켜기 |
| `smoothing_shortcut` · `smoothing_gradient` | `true` · `true` | 지름길 · 경사 하강 |
| `smoothing_weight_data` · `smoothing_weight_smooth` | `0.1` · `0.3` | 경사 하강 가중치 |
| `smoothing_max_iterations` | `200` | 경사 하강 반복 상한 |
| `smoothing_point_spacing_m` | `0.1` | 경사 하강 점 간격 (m) |

`DwaController`: `amr_bringup/config/nav2_params.yaml`의 `controller_server.DWA` 섹션. 값은
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
| `passing_enabled` 외 `passing_*` | `false` (창고 스택 yaml `true`) | 정면 비켜 지나가기(`PassingConfig` 필드와 같은 이름, 근거는 yaml 주석·navigation.md 4-11절). `passing_heading_time`·`passing_lookahead`는 비키는 동안 DWA 기준점 |

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
- `test/test_plan_path.cpp`: `toPlanPath`(셀 단위 점 → `nav_msgs/Path`) 순수 함수 단위 테스트.
- `test/test_plugin_loadable.cpp`: `pluginlib::ClassLoader`로 `amr_nav2_plugins::DwaController`
  (`nav2_core::Controller`)와 `amr_nav2_plugins::AStarGlobalPlanner`(`nav2_core::GlobalPlanner`)를 실제로 로드할 수
  있는지 확인한다(`configure()`는 호출하지 않는다 — 실제 노드·costmap이 필요해서 무겁다).
