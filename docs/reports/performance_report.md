# 성능 리포트 (명세서 10장 통합·품질)

명세서 10장이 요구하는 성능·품질 항목의 현재 측정값을 한곳에 모은다. 각 항목의 측정 도구와 원자료는 링크한 문서에 있다.

> 상태: 초안 (11/3, 민지). ⏳ 표시는 재측정 또는 측정 전이다. 재측정이 끝나면 값을 바꾸고 날짜를 적는다.

## 1. 요약

| 요구 (명세서) | 기준 | 현재 값 | 판정 | 근거 |
| --- | --- | --- | --- | --- |
| 평균 응답 시간 | 200 ms 이내, 50회 이상, 평균·최대 (지연 100 ms, 중앙집중, 5대) | 실제 시간 468 ms / 시뮬레이션 시간 211 ms (11/2 `fleet_stack` 1회, RTF 0.42) | ⏳ 재측정 | 2절 |
| 5대 운용 CPU | 80% 이하 | VM 전체 90~97% (16코어 VM 하나에 5대 + Gazebo). 역할별: 로봇 1대 1.76코어 = 4코어 기준 약 44%, Fleet 서버 5% 미만 | ⏳ 판정 기준 합의 | 3절 |
| 연속 4시간 운용 | 중단 없음, 메모리·지연 추이 | 측정 전 | ⏳ | 4절 |
| 커버리지 | 주요 모듈 70% 이상, 모듈별 분리 보고 | 로직 모듈 96.3%, 19개 패키지 모두 70% 이상 (11/3) | ✅ | 5절 |
| 통합 테스트 | 10개 이상 자동화 | A층(CI launch 테스트) 16개 + B층(하네스 KPI 시나리오) 7개 | ✅ | 6절 |
| `colcon build` 경고 | 0 | CI가 PR마다 검사 (경고가 있으면 실패) | ✅ | 7절 |
| 응답 시간 분해 | ros2_tracing으로 로봇 내부 / 로봇↔서버 분해 | 측정 전 | ⏳ | 8절 |

## 2. 응답 시간

- 정의: 작업 명령 발행 → 로봇 첫 움직임 (`response_time.csv` `latency_ms`, 원 명세 측정 표). 수신 기준(`latency_recv_ms`)과
  시뮬레이션 시간 기준(`latency_sim_ms`)도 함께 남긴다 ([하네스 README](../../tools/harness/README.md) "응답 시간")
- 시간 기준: 응답 시간은 실제 시간이 합의다(#5). 다만 11/2 `fleet_stack`에서 Gazebo가 실제의 0.42배로 돌아(VM CPU 포화)
  실제 시간 값이 부풀었다 (#272: 실제 468 ms vs 시뮬레이션 211 ms). 작업 생성기도 실제 시간으로 돌아 작업이 몰려 들어왔다 (#285)
- 재측정: #285(시간 KPI 시뮬레이션 기준, 작업 생성기 `use_sim_time`) 머지 뒤 `fleet_stack` 지연 100 ms, 50회 이상
  (`task_scenario=response_short`), RTF와 함께 보고한다. RTF가 낮으면 VM 크기를 키우거나 대수를 나눠 돌린다

## 3. CPU

- 측정: `cpu_monitor.py`가 컨테이너 cgroup 사용량과 프로세스별 사용량을 1초마다 기록, 역할별 집계(#248)
- 한 VM에서 로봇 5대·Gazebo·Fleet 서버·실험 도구를 함께 돌리므로 VM 전체 사용률은 실제 운용과 다르다. 실제 운용은 서버와 로봇이 따로 컴퓨터를 쓴다
- 판정 기준 (팀 합의 필요, #248): 시뮬레이터·실험 도구를 빼고 **Fleet 서버**와 **로봇 1대**를 각각 기준 컴퓨터(4코어 제안)로 본 사용률

| 역할 (10/14 실행) | 코어 | 4코어 기준 |
| --- | --- | --- |
| 로봇 1대 (가장 많이 쓴 로봇) | 약 1.76 | 약 44% |
| Fleet 서버 | 0.2 미만 | 5% 미만 |
| VM 전체 (16코어) | — | 90~97% |

## 4. 연속 4시간 운용

- 계획: Spot이 아닌 일반 VM에서 `fleet_stack`을 4시간 돌려 메모리(프로세스별 RSS)·지연·작업 완료 추이를 본다 ([test-plan](../test-plan.md) I15)
- GitHub job 제한(240분)과 러너 최대 실행 시간(5시간) 안에 들게 4시간 + 기동·정리로 잡는다
- 상태: 측정 전 (하연과 VM 준비)

## 5. 커버리지

[test-plan.md 3절](../test-plan.md#3-커버리지)에 패키지별 표가 있다. 요약:

- Python은 `colcon test --pytest-with-coverage`, C++는 `--coverage` 빌드 + gcovr. 표는 `tools/dev/coverage_table.py`
- 70% 판정은 **로직 모듈**(ROS 연결 코드가 아닌 파일) 합으로 한다. 노드 파일은 통합 테스트가 확인한다
- 11/3: 로직 모듈 **96.3%** (7253/7532줄), 19개 패키지 모두 70% 이상. 가장 낮은 패키지는 `amr_mapping` 81.8%
- 노드 파일(패키지 전체 기준)은 launch 테스트가 하위 프로세스로 띄워 잡히지 않는다. 패키지 전체로 보면 16.7~99.2%다
- CI `coverage` 작업이 dev push마다 같은 표를 남긴다 (#288). 70% 미만이면 실패하게 하는 것은 P4에 켠다

## 6. 통합 테스트

정의와 목록은 [test-plan.md](../test-plan.md) 1·2절. 노드 둘 이상이 실제로 주고받는 것을 띄워 자동 판정하는 테스트다.

**A층: CI launch 테스트 (PR마다 자동)**

| # | 테스트 | 패키지 | 확인하는 것 |
| --- | --- | --- | --- |
| I1 | `test_relay_launch`, `test_uplink_launch` | amr_fault_injector | 지연·손실 주입, 바이트 그대로 중계 |
| I2 | `test_fake_fleet_launch` | amr_fleet_manager | 할당 → Ack → 완료 보고 |
| I3 | `test_server_kill_launch` | amr_fleet_manager | 서버 강제종료 → epoch 2 재시작 |
| I4 | `test_task_generator_launch` | amr_fleet_manager | 작업 생성기 일정 발행 |
| — | `test_robot_kill_launch`, `test_task_claims_launch` | amr_fleet_manager | 로봇 장애 작업 회수, 작업 점유 |
| I5 | `test_zone_client_launch` | amr_zone_client | 구역 요청 ↔ 허가 |
| I6 | `test_localization_launch` | amr_localization | 위치추정 스택 기동, TF, EKF 50 Hz, 위치 (#287) |
| I13 | `test_tracking_launch`, `test_tracking_eval_launch` | amr_tracking | 합성 스캔 → 동적 트랙 |
| I14 | `test_safety_launch` | amr_safety | 안전 단계·최종 cmd_vel |
| — | `test_auction_fleet_launch`, `test_task_auction_launch` | amr_fleet_decentralized | 분산 경매 |
| — | `test_bt_actions.launch` | amr_bt | BT 액션 |
| — | `test_system_launch` | amr_bringup | 시스템 launch 구성 |

**B층: 하네스 KPI 시나리오 (GCP, N회 반복 + 신뢰구간 판정)**

| # | 원 명세 KPI | 시나리오 | 상태 |
| --- | --- | --- | --- |
| I8 | 위치 오차 3·5·8 cm | `loc_basic` | 측정 구역 안 통과 (지도 v1), v2 재측정 중 (11/3) |
| I9 | Kidnapped 복구 | `kidnap` | 다른 통로 0/10 → ArUco 연결(#240) 뒤 재측정 |
| I10·I16 | CTE, 경로 계획 98% | `nav_kpi` | 측정 구역 기준 충족, v2 재측정 중 (11/3) |
| I11 | 응답 시간·CPU·단일 작업 성공률 | `fleet_stack`, `task_cycle` | 2·3절 재측정 |
| I19 | 5대 교착 없음 | `fleet_stack`, `fleet_deadlock` | 교착 2건 수정 중 (#273, #280) |
| I17 | 동적 장애물 회피 30회 | `nav_avoidance` | 통합 스택 준비 중 (#262) |
| I12 | Fleet 장애 주입값 일치 | `fleet_i12` | #261 리뷰 중 |

남은 계획 항목: I7 센서 전처리 launch 테스트(지수), I18 도킹 하네스 시나리오(지수), I15 4시간(4절).

## 7. 빌드 경고

`ci.yml` "Check build warnings" 단계가 PR마다 `colcon build`의 stderr에서 경고를 찾아 있으면 실패시킨다. dev는 이 검사를 통과한 커밋만 머지된다.

## 8. 응답 시간 분해 (ros2_tracing)

- 계획: 응답 시간을 서버 처리 / 서버→로봇 전송(장애 주입 포함) / 로봇 BT·Nav2 처리 / 첫 움직임으로 나눈다 (명세서 연구 1 측정 도구)
- 상태: 측정 전. 연구 1에서 ros2_tracing 설치와 수신 시각 기록(`latency_recv_ms`)은 되어 있다

## 9. Sim-to-Real Gap 고려사항

시뮬레이션 결과를 실제 로봇에 옮길 때 차이가 날 수 있는 부분이다.

| 항목 | 시뮬레이션 | 실제와의 차이 |
| --- | --- | --- |
| 센서 노이즈 | LiDAR·IMU·엔코더 노이즈 모델 (amr_description `sensors.yaml`, 오도메트리 드리프트 실험으로 기준값 합의) | 반사율·유리·햇빛, 바퀴 미끄러짐은 모델에 없다 |
| 지도 | slam_toolbox로 만든 지도, 지도↔월드 정렬 오차가 위치 KPI에 영향 (localization.md 3절) | 실제는 정렬 기준(GT)이 없어 위치 오차를 같은 방식으로 잴 수 없다 |
| 시간 | Gazebo RTF가 1보다 낮으면 실제 시간 지표가 부푼다 (2절) | 실제는 RTF 1이다. 시간 KPI는 시뮬레이션 시간으로 본다 (#285) |
| 통신 | 장애 주입기로 지연·손실을 넣는다 (연구 1) | 무선 환경의 버스트 손실·대역폭 제한은 단순화했다 |
| 계산 자원 | 한 VM에 여러 대 (3절) | 로봇마다 별도 컴퓨터. CPU는 역할별 값으로 본다 |
| 동적 장애물 | 정해진 경로·무작위 보행 작업자 | 사람의 예측 어려운 행동, 무리 지어 움직임 |
