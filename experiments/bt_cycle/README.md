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

하네스로 N회 반복해 재려면 `task_cycle` 시나리오를 쓴다 (작업 생성기가 `single_cycle` 일정 40건을 넣는다, 아래 10/30 재측정):

```bash
tools/harness/gcp_run.sh task_cycle -n 8 --label kpi       # GCP (RTF 1.0). 1회 25분에 약 5건, 8회면 34건 이상
tools/harness/docker_run.sh task_cycle -n 1 --label smoke  # 로컬 (Windows Docker에서는 아래 한계 참고)
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
| 노드 추가 확장성 | 재지 않음 | - |

## 결과: 단일 작업 성공률 재측정 (10/30, dev 512b6bf, 하네스 `task_cycle`, GCP)

10/15 뒤로 내비(DWA #228·#244, EKF #245 등)가 많이 바뀌어 최신 코드로 다시 쟀다. 10/15와 달리 하네스로 돌렸고, 구역 클라이언트를 켜고 통신 지연 100 ms(원 명세)를 넣었다. 로봇 1대, 지도 v1 0.025 m, 동적 장애물 없음, 실제 ArUco 도킹(카메라 켬).

```bash
tools/harness/gcp_run.sh task_cycle -n 8 --label kpi   # 실행 41069607856, n2d-standard-16
```

| 항목 | 결과 | 기준 | 판정 |
| --- | --- | --- | --- |
| 단일 작업 성공률 | **38/38 COMPLETED (100 %)**, 실패 0 | 97 % (34회 이상, 실패 1회까지) | 통과 |
| 실패 원인별 (이동·도킹·위치·교통·통신·충전·작업 시간 초과) | 모두 0 | - | - |
| 정체(60 s 넘게 멈춤) | 0 | 0 | 통과 |
| 시뮬레이션 속도 | RTF 1.0 (8회 모두) | - | - |

- 8회(seed 1–8, 1회 25분), 회마다 3–6건 완료. 할당 46건 = 완료 38 + 실행이 끝날 때 진행 중이던 8건(회마다 1건)
- 작업당 이동 거리 평균 50.7 m (32.4–72.7), 요청 → 완료 평균 763 s (대기열 대기 포함)
- 도킹 76번(픽업·하역)이 모두 성공했다. 도킹 서버는 멈춰서 잰 평균이 2 cm·1° 안일 때만 성공으로 끝내므로 76번 모두 서버 측정으로 기준 안이다. 오차 수치는 따로 뽑지 않았다 (도킹 KPI 자체는 [amr_docking README](../../src/task/amr_docking/README.md) 10/12 GT 측정)
- 38/38의 95 % 신뢰구간 하한(Clopper–Pearson)은 약 90.7 %다. 판정은 10/15와 같은 "34회 이상, 실패 1회까지"로 했다
- 원자료: `gs://amr-fleet-results/runs/task_cycle-kpi/20261030_41069607856/`

**로컬 Windows Docker에서는 카메라를 켜면 돌지 않았다 (10/28–30).** 같은 시나리오·코드가 로컬에서는 첫 작업도 못 끝냈다 (costmap이 스캔을 받아들이지 못해 로봇 정지 → `NAV_TIMEOUT`). GCP 리눅스에서는 위처럼 정상이다.

| 확인 | 결과 |
| --- | --- |
| 카메라 끔 (도킹 흉내) | 로컬도 3/3 COMPLETED |
| 10/15 코드 (36/36 낸 것) | 로컬에서 같은 증상 → 코드 문제 아님 |
| 하네스 없이 10/15 방식(직접 실행) | 같은 증상 → 하네스 문제 아님 |
| Fast DDS 공유 메모리 끔 (UDP만) | 같은 증상 |
| 스캔 도착 지연 | 평균 0.02 s, 카메라 켬·끔 같음 → 스캔은 제때 옴 |
| RTF (Gazebo만 / 전체 스택) | 카메라 끔 1.0 / 켬 약 0.81 / 전체 스택 약 0.62. 병목은 Gazebo CPU 렌더링 (LiDAR도 `gpu_lidar`라 렌더링한다) |

- 10/15에는 같은 PC에서 카메라를 켜고 RTF 0.98이었다. 그 뒤 바뀐 건 Windows 보안 업데이트(KB5127648, 10/15 22:27)뿐이었다 (WSL 3.0.3·NVIDIA 드라이버는 10/15 측정 전, Docker Desktop은 9/30 뒤 그대로). 원인은 확정하지 못했다
- 그래서 카메라가 필요한 이 측정은 GCP에서 한다. 로컬에서 돌리려면 Gazebo 렌더링을 GPU로 넘겨야 할 것으로 본다

## 돌리며 찾은 것

- `amr_bringup` package.xml에 `amr_docking`이 없어 `colcon build --packages-up-to amr_bringup`(#156 안내)으로 빌드하면 `docking:=true`가 시작하자마자 `package 'amr_docking' not found`로 죽는다 (#156 코멘트로 알림). 여기서는 amr_docking을 따로 빌드해 돌렸다

## 한계

- 실제 Nav2 실패(경로 없음, 시간 초과)로 FAILED가 나는 경우는 아직 Gazebo에서 만들어 보지 않았다 (가짜 Nav2 서버로는 amr_bt launch 테스트가 본다)
- 로봇 1대, 동적 장애물 없음, 구역 클라이언트·안전 노드 끔
- 주행이 느리다: 설정 최대 2.0 m/s인데 실제 cmd_vel은 0.2 m/s 안팎, 이동 구간 평균 0.1 m/s 정도다. 같은 PC에서 카메라를 켠 Gazebo와 Nav2를 같이 돌려 controller_server가 20 Hz를 계속 놓쳤다 (36회 동안 경고 약 7만 번, RTF는 0.98로 정상). DWB(`FollowPath`) 설정 탓인지 CPU 탓인지는 아직 가르지 않았다. 성공률·도킹 오차에는 영향이 없지만 걸린 시간은 실제 주행 속도를 나타내지 않는다
- 5회 드롭오프 이동 중 Windows 절전으로 Docker가 약 3시간 멈췄다가 이어서 끝났다. `wall_s`는 프로세스가 멈춘 시간을 세지 않아 384.5 s로 남았다
- 도킹 첫 시도 실패율(2/74)은 rack_c3 한 곳에 몰려 있다. 횟수가 적어 원인(그 랙의 마커판 시야, BACKOFF 중 3 s 한도)은 따로 보지 않았다
