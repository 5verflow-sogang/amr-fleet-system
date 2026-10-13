# BT 작업 한 사이클 (4주차 작업 1·3)

Fleet 서버에 배송 작업을 요청해 로봇이 실제로 움직여 끝내는지 본다. 작업 1은 한 번 COMPLETED를 보는 것, 작업 3은 같은 사이클을 반복해 단일 작업 성공률(명세 KPI 97 %, 판정에는 34회 이상)을 재는 것이다.

흐름: `/fleet/task_requests` → Fleet 서버 할당 → 장애 주입기 중계 → `amr_bt` 수락·Ack → `navigate_to_pose`(Nav2)로 픽업 → `docking/dock`(ArUco) → 싣기 → Nav2로 드롭오프 → 도킹 → 내리기 → `TaskStatus` COMPLETED.

## 실행

```bash
# 터미널 1: 전체 시스템 2단계 + 도킹, 로봇 1대 (카메라가 필요해 1대로 잰다)
ros2 launch amr_bringup system.launch.py num_robots:=1 nav2:=true docking:=true enable_camera:=true
# 터미널 2: 배송 작업을 차례로 요청하고 끝까지 따라감
python3 experiments/bt_cycle/run_cycle.py --pairs rack_c6:rack_b6 rack_b6:rack_c6 --repeat 1 --csv results.csv
```

- `--pairs`: 픽업:드롭오프 스테이션 쌍 (`amr_fleet_manager/config/stations.yaml` 이름). 차례로 돈다
- `--repeat`: 쌍 목록을 몇 바퀴 돌지 (작업 3은 34회 이상이 되게)
- 작업 좌표는 map 기준이다. `--origin`은 system.launch.py `map_origin_in_world`와 같게 둔다 (기본 [0, 1.5, 0])
- 끝 자세 오차는 Gazebo GT(`/amr_01/ground_truth/odom`)와 드롭오프 도킹 자세의 차이다

## 결과: 작업 1 (10/13, #156 브랜치)

bringup 2단계 PR(#156)을 받아 돌렸다 (아직 머지 전). 로봇 1대, 지도 v1, 동적 장애물 없음.

| 회 | 경로 | 결과 | 걸린 시간 (실제) | 이동 거리 | 끝 자세 오차 |
| --- | --- | --- | --- | --- | --- |
| 1 | 대기 구역(스폰) → rack_c6 → rack_b6 | **COMPLETED** | 147 s | 20.7 m | 0.32 cm |
| 2 | rack_b6 → rack_c6 | **COMPLETED** | 86 s | 6.1 m | 0.36 cm |
| 3 | rack_c6 → rack_b6 | **COMPLETED** | 79 s | 6.6 m | 0.28 cm · 0.02° |
| 4 | rack_b6 → rack_c6 | **COMPLETED** | 66 s | 5.8 m | 0.19 cm · 0.16° |

- 4회 모두 단계가 move_to_pickup → docking → loading → move_to_dropoff → docking → unloading → COMPLETED 순서로 나왔다
- 1·2회는 이 스크립트를 정리하기 전 같은 흐름의 임시 스크립트로 쟀다 (방향 오차는 안 남겼다). [results.csv](results.csv)는 3·4회다
- 2회부터는 직전 드롭오프가 다음 픽업이라 픽업 이동이 0 m이고 도킹도 확인만 하고 2 s 만에 끝난다. 작업 3은 픽업·드롭오프가 다른 경로를 섞어서 잰다
- 1회 걸린 시간 중 Fleet 할당 → IN_PROGRESS는 1 s 안이다 (Fleet 응답 시간 KPI는 하연 하네스로 따로 잰다)

## 돌리며 찾은 것

- `amr_bringup` package.xml에 `amr_docking`이 없어 `colcon build --packages-up-to amr_bringup`(#156 안내)으로 빌드하면 `docking:=true`가 시작하자마자 `package 'amr_docking' not found`로 죽는다 (#156 코멘트로 알림). 여기서는 amr_docking을 따로 빌드해 돌렸다

## 한계

- 실제 Nav2 실패(경로 없음, 시간 초과)로 FAILED가 나는 경우는 아직 Gazebo에서 만들어 보지 않았다 (가짜 Nav2 서버로는 amr_bt launch 테스트가 본다)
- 로봇 1대, 동적 장애물 없음, 구역 클라이언트·안전 노드 끔
