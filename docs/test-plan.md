# 통합 테스트·커버리지 계획

원 명세서 품질 요구 "통합 테스트 시나리오 최소 10개 작성·자동화", "주요 모듈 커버리지 70% 이상, 모듈별 분리 보고"([명세서 1-2·1-4절](project-spec.md))를 어떤 테스트로 채울지 정한다. 일정은 통합 테스트 10개 P3(11/23~), 커버리지 70% P4(12월)다(9절). 담당: 민지(계획·하네스) + 전원(각자 패키지 테스트).

> 상태: 초안 (10/6). 10/12 회의 뒤 담당과 순서를 확정한다.

## 1. 무엇을 통합 테스트로 세나

**통합 테스트**: 노드 둘 이상이 ROS 토픽·서비스·액션으로 실제로 주고받는 것을 띄워서 자동 판정(합격·불합격)하는 테스트. 로직만 부르는 단위 테스트(pytest)와 구분한다.

| 층 | 어디서 | 시간 | 도구 | 예 |
| --- | --- | --- | --- | --- |
| A. CI 층 | GitHub CI (`ci.yml`, Gazebo 없음) | 1개 2분 이내 | `launch_testing`, 가짜 입력·rosbag | 가짜 로봇 Fleet 왕복, 위치추정 스택 기동 |
| B. 시뮬레이션 층 | GCP 러너 (`fleet-test.yml`, #63) | 1개 수 분~수십 분 | 하네스 시나리오 + KPI 판정 | `loc_basic` 위치 오차 KPI, 응답 시간 KPI |

- B층은 하네스(#77)가 N회 반복하고 KPI를 CI 상한으로 판정한다(PASS / MARGINAL / FAIL). 한 번 돌려 우연히 통과하는 것을 막는다.
- **B층은 원 명세 KPI 하나를 판정하는 단위로 만든다.** 그러면 KPI가 빠짐없이 자동 판정되고, B층 개수는 KPI 수로 정해진다(#86 하연 리뷰).
- 명세 "10개 이상"은 A·B를 합쳐 센다.

## 2. 목록

**이미 있는 것 (A층, `launch_testing`)**

| # | 테스트 | 패키지 | 확인하는 것 | 담당 |
| --- | --- | --- | --- | --- |
| I1 | `test_relay_launch` | amr_fault_injector | 장애주입기가 지연·손실을 걸고 바이트를 그대로 넘김 | 하연 |
| I2 | `test_fake_fleet_launch` | amr_fleet_manager | 서버 + 가짜 로봇: 작업 할당 → Ack → 완료 보고 | 하연 |
| I3 | `test_server_kill_launch` | amr_fleet_manager | 서버 강제종료 → heartbeat 끊김 → epoch 2로 재시작 | 하연 |
| I4 | `test_task_generator_launch` | amr_fleet_manager | 작업 생성기가 서버 구독 확인 뒤 일정대로 발행 | 하연 |
| I5 | `test_zone_client_launch` | amr_zone_client | 로봇 구역 요청 ↔ 서버 허가 | 이진 |

**추가 제안: A층**

| # | 테스트 | 확인하는 것 (판정) | 담당 | 언제 |
| --- | --- | --- | --- | --- |
| I6 | 위치추정 스택 기동 | 가짜 `odom`·`imu/data`·`scan` 입력으로 map_server·AMCL lifecycle active, TF `map→odom→base_footprint` 연결, `odometry/filtered` 50 Hz 이상 | 민지 | 4주차 |
| I7 | 센서 전처리 파이프라인 | `scan_raw → scan`, `imu/data_raw → imu/data` 주기·공분산 채움·바이어스 보정 (인계 계약 항목, #81) | 지수 | 4주차 |
| I13 | 장애물 추적 (`test_tracking_launch`, amr_tracking) | 녹화 rosbag 대신 테스트가 합성 LiDAR 스캔(10 Hz)을 그려 넣는다(결정적, Gazebo 없음): 정지 로봇 앞 2 m를 1.0 m/s로 가로지르는 다리 두 개 + 벽 + 기둥 → `TrackedObstacleArray`에 동적 트랙이 사람 하나뿐이고 위치 0.3 m·속력 0.3 m/s 안, 기둥은 정적, 벽·벽 조각은 동적이 아니다. `collision_risks`에 같은 트랙 | 연우 | P1 |
| I14 | 안전 감속 (`test_safety_launch`, amr_safety) | 가짜 스캔·odom·추적·TTC·명령 입력 → `SafetyStatus`·최종 `cmd_vel`: 빈 장면 NORMAL(명령 그대로), 진행 경로 0.2 m 앞 점 CRITICAL(0), TTC 1.5 s 움직이는 트랙 WARNING(0.6, 곡률 유지), 스캔 끊김 CRITICAL(0) | 연우 | P1 |

**추가 제안: B층 (원 명세 KPI 하나당 하나)**

| # | 원 명세 KPI | 시나리오 · 판정 | 담당 | 언제 |
| --- | --- | --- | --- | --- |
| I8 | 위치 오차 정지 3 cm·직선 5 cm·회전 8 cm (100개 시점) | `loc_basic`, 구간별 RMSE CI 상한 | 민지 | 지도 v1 뒤 |
| I9 | Kidnapped Robot 복구 | 순간 이동 뒤 5초 안 감지, 60초 안 복구(판정 기준 #79), N회 성공률 | 민지 | 작업 8 뒤 |
| I10 | 경로 추종 CTE | Nav2 + 제어기, CTE KPI | 이진 | P1 |
| I16 | 경로 계획 성공률 98% (출발지-목적지 50쌍) | 무작위 50쌍, 성공 수 / 50 | 이진 | P1 |
| I17 | 동적 장애물 회피 30회 충돌 0건, 경로 이탈 1 m 이내, 원래 경로 복귀 5초 이내 | 동적 장애물 월드(#58) 30회(1.0 m/s 장애물 포함), DWA + 추적 + 안전 노드. 충돌 수, 계획 경로 대비 최대 이탈, 회피 시작부터 경로 복귀까지 시간. 측정은 `amr_safety` `avoidance_eval`(GT 기준, 첫 전역 경로를 기준 경로로 고정, 정의·기준값은 [amr_safety README](../src/perception/amr_safety/README.md#회피-평가-i17)), 시나리오는 `nav_avoidance`(초안, DWA 회피 모드 뒤 첫 실행). 30회 모두를 보는 KPI라 CI가 아니라 최악값으로 `avoidance_report`가 판정한다 | 연우·이진 | P1 |
| I18 | 도킹 위치 2 cm·각도 1도 (3회 실패 시 에러 보고) | 스테이션 도킹 N회, 정답 대비 위치·각도 오차 | 지수 | P1 |
| I11 | 단일 작업 성공률 97%, 평균 응답 200 ms(50회 이상, **평균·최대**), 5대 CPU 80% 이하 | 실제 로봇 스택 5대, Fleet 할당 → BT → 이동 → 완료. 성공 비율, `response_time.csv` 평균·최대, CPU 최대 | 지수·하연 | P1 |
| I19 | 5대 동시 운용 교착 없음 | 실제 로봇 스택 5대 Fleet 시나리오, 교착 탐지 0건(또는 탐지 시 해소) | 하연 | P1 |
| I12 | (연구 1) Fleet 장애 조건 | `fleet_i12`(`fleet_fake` + 지연 100 ms·손실 5%). 측정 손실률이 주입값과 1 %p 안, 지연 중앙값이 5 ms 안(CI 상한), 할당 명령 Ack 비율 1 | 하연 | 4주차 |
| I15 | 연속 4시간 운용 | 메모리·지연 추이, 중단 없음 | 전원 | P4 |

- P3 마감(10개)은 A층 I1~I7 + B층 I8·I11·I12처럼 채울 수 있다. 나머지 B층은 해당 KPI를 확정하는 주에 만든다.
- B층 테스트는 하네스 시나리오(`tools/harness/scenarios/*.yaml`)로 만든다. 판정은 요약의 KPI 열을 쓴다.
- A층은 기동 실패(lifecycle 응답 유실, 하네스 실측 약 10%)로 흔들릴 수 있다. 준비 확인(lifecycle active)을 먼저 하고 판정한다(#77의 `ready_cmds`와 같은 방식).
- **I15(4시간)는 Spot이 아닌 일반 VM에서 돌린다.** GCP Spot은 중간에 회수될 수 있고 러너 최대 실행 시간도 5시간이라 빠듯하다(#86 하연).

## 3. 커버리지

**지금 상태 (10/6 `amr_localization`, 단위 테스트만)**

| 모듈 | 커버리지 |
| --- | --- |
| `eval/` (오차 계산) | 96~100% |
| `kidnap/` (감지·복구 로직) | 99~100% |
| `eval_node.py`, `kidnap_recovery_node.py` (ROS 노드) | 0% |
| 패키지 전체 | 62% |

로직은 ROS 없는 모듈로 나눠 단위 테스트하고, 노드 파일은 구독·발행 연결만 한다. 노드 파일은 단위 테스트가 닿지 않아 패키지 전체가 70% 아래로 내려간다. 다른 패키지도 같은 구조라 아래를 정해야 한다.

**보고와 판정을 나눈다** (#86 하연 리뷰)

- **보고**: 원 명세 "주요 모듈별 분리 보고"대로 패키지 전체와 노드 파일까지 모두 표로 보인다. 노드 파일 0%를 빼고 보고하면 숨긴 것처럼 보이기 때문이다.
- **70% 판정**: ROS 없는 로직 모듈(예: `amr_localization.eval`·`kidnap`, `amr_fleet_core`, `amr_planning_core`, `amr_perception_core`) 기준으로 한다. 근거는 원 명세 체크리스트 "핵심 알고리즘(DWA, A*, EKF 등)이 독립 모듈로 구현되어 테스트 가능한가?"다. 노드 연결은 통합 테스트(2절)가 확인한다.
- 노드 파일까지 커버리지에 넣으려면 launch 테스트가 띄우는 하위 프로세스의 커버리지를 모아야 한다(`COVERAGE_PROCESS_START`). 여유가 생기면 추가한다.

측정: Python은 `colcon test --pytest-args --cov=<모듈> --cov-report=xml`, C++는 `--mixin coverage-gcc` + lcov. CI(`ci.yml`)에 모듈별 커버리지 표를 출력하는 단계를 추가하고, 70% 판정(실패)은 P4에 켠다.

## 4. 정해야 할 것 (10/12 회의)

| 항목 | 제안 |
| --- | --- |
| 통합 테스트 정의 | 1절 (노드 둘 이상, 자동 판정, A·B층 합산) |
| B층 = 원 명세 KPI 단위 | I8~I11, I15~I19 담당·시기 |
| P3까지 10개 | A층 I1~I7 + B층 I8·I11·I12 |
| 커버리지 | 보고는 패키지 전체·노드 파일 포함, 70% 판정은 로직 모듈 기준 |
| 4시간 운용 | 일반(비 Spot) VM |
| CI 연결 | A층은 `ci.yml`, B층은 `fleet-test.yml` 야간·라벨 실행 |
