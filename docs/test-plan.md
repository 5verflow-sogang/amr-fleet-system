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
- 명세 "10개 이상"은 A·B를 합쳐 센다. 다만 연구·KPI 증거가 B층에서 나오므로 B층을 최소 4개 둔다.

## 2. 목록

**이미 있는 것 (A층, `launch_testing`)**

| # | 테스트 | 패키지 | 확인하는 것 | 담당 |
| --- | --- | --- | --- | --- |
| I1 | `test_relay_launch` | amr_fault_injector | 장애주입기가 지연·손실을 걸고 바이트를 그대로 넘김 | 하연 |
| I2 | `test_fake_fleet_launch` | amr_fleet_manager | 서버 + 가짜 로봇: 작업 할당 → Ack → 완료 보고 | 하연 |
| I3 | `test_server_kill_launch` | amr_fleet_manager | 서버 강제종료 → heartbeat 끊김 → epoch 2로 재시작 | 하연 |
| I4 | `test_task_generator_launch` | amr_fleet_manager | 작업 생성기가 서버 구독 확인 뒤 일정대로 발행 | 하연 |
| I5 | `test_zone_client_launch` | amr_zone_client | 로봇 구역 요청 ↔ 서버 허가 | 이진 |

**추가 제안**

| # | 테스트 | 층 | 확인하는 것 (판정) | 담당 | 언제 |
| --- | --- | --- | --- | --- | --- |
| I6 | 위치추정 스택 기동 | A | 가짜 `odom`·`imu/data`·`scan` 입력으로 map_server·AMCL lifecycle active, TF `map→odom→base_footprint` 연결, `odometry/filtered` 50 Hz 이상 | 민지 | 4주차 |
| I7 | 센서 전처리 파이프라인 | A | `scan_raw → scan`, `imu/data_raw → imu/data` 주기·공분산 채움·바이어스 보정 (인계 계약 항목, #81) | 지수 | 4주차 |
| I8 | 위치 오차 KPI (`loc_basic`) | B | 정지 3 cm·직진 5 cm·회전 8 cm, N회 CI 상한 | 민지 | 지도 v1 뒤 |
| I9 | Kidnapped 감지·복구 | B | 순간 이동 뒤 5초 안 감지, 60초 안 복구(판정 기준 #79) | 민지 | 작업 8 뒤 |
| I10 | 목표 도달·경로 추종 | B | Nav2 + DWA로 목표 도달, CTE KPI | 이진 | P1 |
| I11 | 작업 사이클 (실제 로봇 스택) | B | Fleet 할당 → BT → 이동 → 완료 보고, 응답 시간 200 ms(50회 이상) | 지수·하연 | P1 |
| I12 | Fleet 장애 조건 (`fleet_fake`) | B | 지연 100 ms·손실 5%에서 Ack 비율·손실률이 주입값과 맞음 | 하연 | 4주차 |
| I13 | 장애물 추적 | A | 녹화 rosbag 재생 → `TrackedObstacleArray` 개수·위치 | 연우 | P1 |
| I14 | 안전 감속 | A | 가짜 근접 장애물 → `SafetyStatus` 감속 한계 | 연우 | P1 |
| I15 | 연속 4시간 운용 | B | 메모리·지연 추이, 중단 없음 | 전원 | P4 |

- I6~I15 중 6개 이상이 끝나면 10개를 넘는다. P3 마감 기준으로 I6·I7·I8·I11·I12·I13을 우선한다.
- B층 테스트는 하네스 시나리오(`tools/harness/scenarios/*.yaml`)로 만든다. 판정은 요약의 KPI 열을 쓴다.
- A층은 기동 실패(lifecycle 응답 유실, 하네스 실측 약 10%)로 흔들릴 수 있다. 준비 확인(lifecycle active)을 먼저 하고 판정한다(#77의 `ready_cmds`와 같은 방식).

## 3. 커버리지

**지금 상태 (10/6 `amr_localization`, 단위 테스트만)**

| 모듈 | 커버리지 |
| --- | --- |
| `eval/` (오차 계산) | 96~100% |
| `kidnap/` (감지·복구 로직) | 99~100% |
| `eval_node.py`, `kidnap_recovery_node.py` (ROS 노드) | 0% |
| 패키지 전체 | 62% |

로직은 ROS 없는 모듈로 나눠 단위 테스트하고, 노드 파일은 구독·발행 연결만 한다. 노드 파일은 단위 테스트가 닿지 않아 패키지 전체가 70% 아래로 내려간다. 다른 패키지도 같은 구조라 아래를 정해야 한다.

| 안 | 내용 | 장단점 |
| --- | --- | --- |
| **A. 로직 모듈 기준 (제안)** | "주요 모듈" = ROS 없는 로직 모듈(예: `amr_localization.eval`·`kidnap`, `amr_fleet_core`, `amr_planning_core`, `amr_perception_core`). 이 모듈들을 70% 이상 | 명세 "주요 모듈" 표현과 맞고 지금 구조로 달성 가능. 노드 연결은 통합 테스트(2절)가 확인 |
| B. 패키지 전체 기준 | 노드 파일까지 포함 | launch 테스트는 노드를 별도 프로세스로 띄워 커버리지가 잡히지 않는다. 하위 프로세스 커버리지(`COVERAGE_PROCESS_START`) 설정이 필요하다 |

측정: Python은 `colcon test --pytest-args --cov=<모듈> --cov-report=xml`, C++는 `--mixin coverage-gcc` + lcov. CI(`ci.yml`)에 모듈별 커버리지 표를 출력하는 단계를 추가하고, 70% 판정(실패)은 P4에 켠다.

## 4. 정해야 할 것 (10/12 회의)

| 항목 | 제안 |
| --- | --- |
| 통합 테스트 정의 | 1절 (노드 둘 이상, 자동 판정, A·B층 합산) |
| 우선 6개와 담당 | I6·I7·I8·I11·I12·I13 |
| 커버리지 기준 | A안 (로직 모듈 70%), 모듈별 분리 보고 |
| CI 연결 | A층은 `ci.yml`, B층은 `fleet-test.yml` 야간·라벨 실행 |
