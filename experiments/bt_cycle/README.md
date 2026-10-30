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

## 결과: 작업 3 단일 작업 성공률 (10/15, dev 2b5d7f2)

픽업·드롭오프가 다른 6개 경로를 6바퀴, 36회. 창고 서쪽 끝 도크(dock_1·dock_2, 지도 휨이 큰 곳 #183)와 랙·도크를 섞었다. 로봇 1대, 지도 v1, 동적 장애물 없음.

```bash
python3 experiments/bt_cycle/run_cycle.py \
  --pairs rack_c6:rack_b6 rack_b5:dock_b dock_a:rack_a6 rack_a5:dock_1 dock_2:rack_c2 rack_c3:rack_b3 \
  --repeat 6 --timeout 900 --csv results_task3.csv
```

| 항목 | 결과 | 기준 | 판정 |
| --- | --- | --- | --- |
| 단일 작업 성공률 | **36/36 COMPLETED (100 %)** | 97 % (34회 이상, 실패 1회까지) | 통과 |
| 드롭오프 끝 자세 오차 (Gazebo GT, 36회) | 위치 평균 0.40 / 최대 0.75 cm, 방향 평균 0.15 / 최대 0.35° | 도킹 2 cm | 통과 |
| 도킹 오차 (도킹 서버 마커 추정, 픽업·드롭오프 72회) | 위치 평균 0.28 / 최대 0.9 cm, 방향 평균 0.06 / 최대 0.43° | 도킹 2 cm | 통과 |
| 도킹 첫 시도 성공 | 72/74 (97 %), 실패 2회는 BT 재시도로 다음 시도에 성공 | 3회 재시도 | - |
| 도킹 한 번 걸린 시간 (서버 기준) | 평균 30 s, 6.9–75.1 s | - | - |

| 경로 | 성공 | 걸린 시간 평균 | 이동 거리 평균 | 끝 위치 오차 평균 / 최대 | 끝 방향 오차 최대 |
| --- | --- | --- | --- | --- | --- |
| rack_c6 → rack_b6 | 6/6 | 340 s | 29.3 m | 0.28 / 0.34 cm | 0.25° |
| rack_b5 → dock_b | 6/6 | 507 s | 36.8 m | 0.42 / 0.65 cm | 0.12° |
| dock_a → rack_a6 | 6/6 | 330 s | 27.6 m | 0.32 / 0.49 cm | 0.20° |
| rack_a5 → dock_1 | 6/6 | 536 s | 48.0 m | 0.58 / 0.75 cm | 0.30° |
| dock_2 → rack_c2 | 6/6 | 388 s | 32.5 m | 0.51 / 0.71 cm | 0.35° |
| rack_c3 → rack_b3 | 6/6 | 268 s | 16.5 m | 0.31 / 0.38 cm | 0.23° |

- 36회 모두 단계가 move_to_pickup → docking → loading → move_to_dropoff → docking → unloading → COMPLETED 순서
- 도킹 첫 시도 실패 2회는 둘 다 rack_c3 픽업(12·30회)에서 `marker board lost for 3 s in BACKOFF`(뒤로 물러나는 중 마커판을 3 s 놓침, 그때 위치 오차 26–29 cm). `DockWithRetry`의 두 번째 시도에서 0.1·0.3 cm로 성공해 작업은 COMPLETED. 도킹 재시도가 실제로 작업을 살린 경우다
- 지도 휨이 큰 서쪽 끝(dock_1·dock_2)도 끝 위치 오차가 1 cm 안이다. 마지막 정렬을 ArUco 도킹이 해서 지도 오차(14–22 cm, #183)가 스테이션 작업에 남지 않는다
- 걸린 시간 평균 395 s, 이동 거리 평균 31.8 m. 시간은 아래 한계처럼 주행이 느린 값이라 성능 지표로 쓰지 않는다
- 전체 기록은 [results_task3.csv](results_task3.csv). 도킹 서버 값은 같은 실행의 로그(`도킹 성공/실패: ...`)에서 뽑았다

### 이번에 재지 않은 BT 항목

| 항목 | 상태 | 근거 |
| --- | --- | --- |
| BT 노드 15개 이상 | 56개 (직접 만든 종류 11개) | [amr_bt README](../../src/task/amr_bt/README.md), 테스트 `TreeHasAtLeast15Nodes` |
| 서브트리 | 2종 (`VisitStation`, `DockWithRetry`), 각각 2번 재사용 | `behavior_trees/task_cycle.xml` |
| 자동 복구 | 4종: Nav2 재시도(`nav_retries` 2), 도킹 재시도(3회), 구역 대기 멈춤·재개, 위치 복구 멈춤·재개 | amr_bt gtest·launch 테스트. Gazebo에서 실제로 일어난 건 도킹 재시도(위 2회)뿐 |
| Groot 모니터링 | ZMQ 퍼블리셔만 (`groot_port`) | Groot 화면에 붙여 확인한 적은 없음 |
| 노드 추가 확장성 | 10/30: `CheckBattery` 노드를 추가해 기존 코드 3개 파일(11줄 추가·4줄 삭제) 수정, 기준(3개 이내) 충족 | [amr_bt README](../../src/task/amr_bt/README.md) "노드 추가로 확장하기" |

## 돌리며 찾은 것

- `amr_bringup` package.xml에 `amr_docking`이 없어 `colcon build --packages-up-to amr_bringup`(#156 안내)으로 빌드하면 `docking:=true`가 시작하자마자 `package 'amr_docking' not found`로 죽는다 (#156 코멘트로 알림). 여기서는 amr_docking을 따로 빌드해 돌렸다

## 한계

- 실제 Nav2 실패(경로 없음, 시간 초과)로 FAILED가 나는 경우는 아직 Gazebo에서 만들어 보지 않았다 (가짜 Nav2 서버로는 amr_bt launch 테스트가 본다)
- 로봇 1대, 동적 장애물 없음, 구역 클라이언트·안전 노드 끔
- 주행이 느리다: 설정 최대 2.0 m/s인데 실제 cmd_vel은 0.2 m/s 안팎, 이동 구간 평균 0.1 m/s 정도다. 같은 PC에서 카메라를 켠 Gazebo와 Nav2를 같이 돌려 controller_server가 20 Hz를 계속 놓쳤다 (36회 동안 경고 약 7만 번, RTF는 0.98로 정상). DWB(`FollowPath`) 설정 탓인지 CPU 탓인지는 아직 가르지 않았다. 성공률·도킹 오차에는 영향이 없지만 걸린 시간은 실제 주행 속도를 나타내지 않는다
- 5회 드롭오프 이동 중 Windows 절전으로 Docker가 약 3시간 멈췄다가 이어서 끝났다. `wall_s`는 프로세스가 멈춘 시간을 세지 않아 384.5 s로 남았다
- 도킹 첫 시도 실패율(2/74)은 rack_c3 한 곳에 몰려 있다. 횟수가 적어 원인(그 랙의 마커판 시야, BACKOFF 중 3 s 한도)은 따로 보지 않았다
