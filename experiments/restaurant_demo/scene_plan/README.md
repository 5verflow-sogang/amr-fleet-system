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
python3 $D/detour.py --map ... --zones ... --stations ... --astar /tmp/astar_cli --out /tmp/plan --start -10.0 -5.8  # 그림
```

- `detour.py`: costmap 만들기(정적 + inflation, 선택으로 손님 시작 자리), keepout(west_aisle 혼잡도 1.0 → 마스크 90 → 비용 229),
  출발 → buffet_2 경로 (a) keepout / (b) 혼잡 없음과 그림
- `plan_tools.py trips`: 대기 줄 5대(robots_restaurant.yaml) × 픽업 6곳 × 하역(테이블·부스·6인석·2인석) A* 길이
- `plan_tools.py start`: 1-2 출발 자리 후보마다 (a)·(b) 경로와 손님 경로(피크·1-2 피크·직원 파일)까지 거리

## 장면 2 시작 작업 (tasks/demo_s2a.yaml·demo_s2b.yaml `path_m`)

배정: 로봇이 동쪽 벽(x 11.2) 한 줄, 픽업이 모두 북쪽(y 4.15)이라 최소거리 배정은 낸 순서대로 amr_05, 04, 03, 02, 01이다.
길이 = 시작 자리 → 픽업 → 하역 A* (m). 서빙 로봇 DWA max_v 0.5 m/s, 픽업·하역 머무름 2 s씩.

| 막 | 로봇 | 작업 | 길이 | 비고 |
| --- | --- | --- | --- | --- |
| 1막 | amr_05 | stall_3 → L02 | 26.7 | |
| 1막 | amr_04 | stall_1 → t07 | 27.0 | |
| 1막 | amr_03 | stall_2 → t13 | 26.8 | |
| 1막 | **amr_02** | stall_4 → t05 | **16.7** | 가장 짧다. 다음 로봇과 10.0 m |
| 1막 | amr_01 | buffet_2 → t01 | 27.2 | |
| 2막 | amr_05 | buffet_2 → b01 | 51.4 | |
| 2막 | **amr_04** | stall_1 → L03 | **35.4** | 가장 짧다. 다음 로봇과 15.4 m. 0.5 m/s로도 75 s (0초 = 시작 뒤 약 63 s) |
| 2막 | amr_03 | buffet_1 → b03 | 51.4 | |
| 2막 | amr_02 | buffet_2 → b02 | 50.9 | |
| 2막 | amr_01 | buffet_1 → L02 | 50.8 | |

고른 기준: 핵심 요청 자리(1막 t03·return_1과 대기 배달 하역, 2막 회수 5곳·return_1·2·c01)와 1 m 안을 쓰지 않는다. 1막은
픽업이 모두 다르고 다음 로봇과 10 m 이상인 것 중 합이 가장 짧은 것(끔에서 회수까지 녹화 안에 끝나게). 2막은 4호 35~40 m
중 다음 로봇과 차이가 가장 큰 것. 픽업이 모두 다르면 2막은 조건을 만족하는 조합이 없다(50 m 넘는 작업은 배식대 픽업뿐) -
나머지 넷이 배식대 1·2를 둘씩 쓰게 했다(같은 픽업 둘째 로봇은 예약으로 더 늦어질 뿐이라 4호가 먼저 끝나는 쪽으로만 바뀐다).
