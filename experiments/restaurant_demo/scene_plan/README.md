# scene_plan: 식당 시연 장면 설계 계산 (Gazebo 없이)

장면 파일(tools/harness/tasks/demo_*.yaml)의 시작 자리·시작 작업을 고를 때 쓴 계산이다.
restaurant_gt 지도 + 식당 프로필 전역 costmap(서빙 로봇 0.565 x 0.537 m, footprint_padding 0.01, inflation 0.55 / 3.0)에
amr_planning_core A*(nav2_params.yaml AStar 블록과 같은 octile, lethal 253, cost_weight 4.0, 모서리 자르기 막음)를 돌린다.
경로 길이는 8방향 격자 경로 그대로라(평활화 전) 실제 주행보다 조금 길다.

```bash
# 컨테이너 (amr_planning_core가 빌드된 /ros2_ws)
D=experiments/restaurant_demo/scene_plan
g++ -O2 -std=c++17 -I/ros2_ws/install/amr_planning_core/include $D/astar_cli.cpp \
    /ros2_ws/install/amr_planning_core/lib/libamr_planning_core.a -o /tmp/astar_cli
python3 $D/plan_tools.py trips --ws /ros2_ws/src --astar /tmp/astar_cli --out /tmp/plan   # 로봇 → 픽업 → 하역 길이
python3 $D/plan_tools.py start --ws /ros2_ws/src --astar /tmp/astar_cli --out /tmp/plan   # 1-2 1호 출발 자리 후보
python3 $D/pairs.py /ros2_ws/src /tmp/astar_cli /tmp/plan                          # 정차 자리 쌍 길이 → pairs.json
python3 $D/search_act1.py /tmp/plan/pairs.json src/fleet/amr_fleet_manager/config/stations_restaurant.yaml
python3 $D/search_act2.py /tmp/plan/pairs.json src/fleet/amr_fleet_manager/config/stations_restaurant.yaml
python3 $D/detour.py --map ... --zones ... --stations ... --astar /tmp/astar_cli --out /tmp/plan --start -10.0 -5.8  # 그림
```

- `detour.py`: costmap 만들기(정적 + inflation, 선택으로 손님 시작 자리), keepout(west_aisle 혼잡도 1.0 → 마스크 90 → 비용 229),
  출발 → buffet_2 경로 (a) keepout / (b) 혼잡 없음과 그림
- `plan_tools.py trips`: 대기 줄 5대(robots_restaurant.yaml) × 픽업 6곳 × 하역(테이블·부스·6인석·2인석) A* 길이
- `plan_tools.py start`: 1-2 출발 자리 후보마다 (a)·(b) 경로와 손님 경로(피크·1-2 피크·직원 파일)까지 거리

## 장면 2 시작 작업 (tasks/demo_s2a.yaml·demo_s2b.yaml `path_m`)

11/8 #341(지도 재생성)·#338(식당 속도 0.8 m/s) 뒤 다시 계산했다. 배정: 로봇이 동쪽 벽(x 11.2) 한 줄, 픽업이 모두 북쪽(y 4.15)이라
최소거리 배정은 낸 순서대로 amr_05, 04, 03, 02, 01이다. 길이는 A* (m). 픽업·하역 머무름 2 s씩.

| 막 | 로봇 | 1바퀴 (시작 → 픽업 → 하역) | 길이 | 비고 |
| --- | --- | --- | --- | --- |
| 1막 | amr_05 | stall_3 → t06 | 27.6 | |
| 1막 | amr_04 | stall_1 → t08 | 27.6 | |
| 1막 | amr_03 | buffet_1 → t01 | 27.9 | |
| 1막 | **amr_02** | stall_4 → L01 | **16.6** | 가장 짧다. 다음 로봇과 11.0 m. 0.8 m/s로도 25 s (0초 = 시작 뒤 약 8 s) |
| 1막 | amr_01 | stall_2 → t13 | 27.9 | |
| 2막 | amr_05 | buffet_2 → b01 | 42.6 | |
| 2막 | **amr_04** | stall_4 → t13 | **25.9** | 가장 짧다. 다음 로봇과 15.8 m |
| 2막 | amr_03 | buffet_1 → t10 | 41.8 | |
| 2막 | amr_02 | buffet_1 → t15 | 44.3 | |
| 2막 | amr_01 | buffet_2 → L03 | 44.1 | |

2막은 시작 작업을 두 바퀴 준다. 0.8 m/s에서는 한 작업으로 0초(시작 뒤 약 66 s)까지 5대를 붙잡을 수 없다(가장 긴 작업 약 55 m ≈ 70 s).
2바퀴 5건(stall_2 → b02, buffet_1 → b01, stall_1 → b03, buffet_1 → t05, buffet_2 → L04)을 2인석 배달보다 먼저 넣어 두면, 같은 우선순위에서
먼저 들어온 작업이 앞서므로 1바퀴를 마친 로봇이 차례로 가져가고 2인석 배달은 계속 대기한다.

| 2막 조건 | 값 |
| --- | --- |
| 4호 1바퀴가 가장 짧다 (먼저 끝나 2바퀴 첫 작업 stall_2 → b02를 가져간다) | 25.9 m, 다른 로봇 최소 41.8 m (차이 15.8 m) |
| 4호 두 바퀴 합 | 54.4 m → 0.8 m/s로도 54.4 / 0.8 + 8 = 76 s, 0초(66 s)보다 10 s 늦다 |
| 4호 두 바퀴가 다른 로봇 1바퀴보다 길다 (2바퀴 나머지 넷은 다른 로봇이 가져간다) | 54.4 m ≥ 44.3 + 10.1 m |
| 다른 로봇 두 바퀴 합 (2바퀴 어느 작업을 받아도) | 71.6 m 이상 = 4호 + 17.2 m |

→ 0초 뒤 처음 일을 마치는 로봇은 4호이고, 그때 남은 대기 작업은 2인석 배달과 회수 호출뿐이다(켬: 배달, 끔: 회수).

고른 기준: 핵심 요청 자리(1막 t03·return_1과 대기 배달 하역 t09·t15·t04·t02, 2막 회수 5곳·return_1·2·c01과 c01 픽업 stall_3)와
1 m 안을 쓰지 않는다. 한 바퀴 안에서 한 픽업은 둘까지, 4호 픽업은 혼자. 1막은 픽업이 모두 다르고 다음 로봇과 10 m 넘는 것 중
합이 가장 짧은 것. 2막은 위 표 여유를 크게 하는 조합을 무작위 탐색(150만 번)으로 골랐다.
