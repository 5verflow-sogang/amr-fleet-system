# harness

회귀 테스트 하네스 (담당: 민지). 명세서 5-2절.

- 같은 시나리오를 N회 반복해 KPI의 평균·신뢰구간을 산출한다.
- 장애 조건(지연·손실)별 결과와 ros2_tracing 데이터를 수집한다.
- `.github/workflows/fleet-test.yml`이 이 디렉토리의 진입 스크립트를 호출한다.
- 로그 포맷은 명세서 1-4절 "성능 지표 측정 절차" 표를 따른다.

## 로그 포맷 (초안, 하연과 합의 중)

> 상태: 초안. 하연과 합의하면 "초안" 표시를 뗀다. 컬럼 이름과 순서는 명세서 1-4절 표를 그대로 따르고, 이 문서는 단위·파일 배치·부가 컬럼·원자료 보관만 정한다.

### 기본 원칙: 원자료를 남기고 지표는 나중에 계산한다

위치추정·내비게이션 벤치마크들은 계산된 지표보다 **가공하지 않은 원자료**(궤적, 토픽 기록)를 표준 형식으로 남기고, 지표는 오프라인 도구로 계산한다(TUM RGB-D 벤치마크와 evo, Arena-Rosnav). 이렇게 하면 다음이 가능하다.

- 구간 분류 기준이나 지표 정의가 바뀌어도 다시 실행하지 않고 재계산한다.
- 표준 도구(evo)로 ATE·RPE를 뽑아 논문에 쓰고, 직접 만든 계산기의 결과를 교차 검증한다.

그래서 실행마다 **원자료**와 명세서 형식의 **지표 CSV**를 함께 저장한다. 동료 평가용 KPI는 지표 CSV로 제출한다.

### 공통 규칙

- 형식: UTF-8 CSV, 첫 줄은 헤더. 지표 하나당 파일 하나.
- 시각: 초 단위 실수 (예: `123.456789`).
- 시간 기준: 기본은 **시뮬레이션 시간**이다(GT와 추정값을 같은 시계로 맞춰야 하므로). 단, **응답 시간과 연구 1의 지연은 실제 시간**으로 잰다(하연 합의, #5).
- 단위: 거리는 m, 각도는 rad, 지연은 ms. 컬럼 이름에 단위가 없으면 이 규칙을 따른다.
- 명세서 컬럼은 이름·순서를 바꾸지 않는다. 부가 컬럼은 **명세서 컬럼 뒤에만** 추가한다. 앞쪽 열만 잘라내면 명세서 형식이 되도록 하기 위해서다.
- 다중 로봇 시나리오는 부가 컬럼 `robot`(예: `amr_01`)을 붙인다.

### 원자료

| 파일 | 형식 | 내용 | 담당 |
| --- | --- | --- | --- |
| `trajectories/<robot>_gt.tum` | TUM 궤적 형식 | 시뮬레이터 Ground Truth 궤적 | 민지 (GT 토픽은 지수가 `amr_gazebo`에서 제공) |
| `trajectories/<robot>_est.tum` | TUM 궤적 형식 | EKF 추정 궤적 (`odometry/filtered`) | 민지 |
| `rosbag/` | rosbag2 (mcap) | 시나리오에서 정한 토픽 기록 (선택) | 시나리오 작성자 |
| `trace/` | LTTng CTF | ros2_tracing 원본 (UST `ros2:*` 이벤트). `hop_latency.csv`의 원자료 | 하연 (수집 절차는 [runbook](../../docs/runbook.md) "트레이싱") |

- TUM 궤적 형식: 한 줄에 `timestamp x y z qx qy qz qw` (공백 구분, 초·m·쿼터니언). evo가 그대로 읽는다.
- **좌표계 정렬**: GT(Gazebo world)와 추정값(`map`)은 서로 다른 좌표계다. 두 좌표계 사이 변환을 시나리오 설정에 고정값으로 두고, 정렬 방식을 `meta.json`에 기록한다. 평가 도구가 임의로 최적 정렬(Umeyama 등)을 하면 오차가 실제보다 작게 나올 수 있으므로 KPI 산출에는 고정 변환만 쓴다.
- rosbag은 용량이 커서 기본으로 켜지 않는다. 켜는 경우에도 커밋하지 않는다(CONTRIBUTING 6절).

### 지표 CSV

| 파일 | 명세서 컬럼 | 부가 컬럼 | 샘플 수 | 담당 |
| --- | --- | --- | --- | --- |
| `localization.csv` | `timestamp, gt_x, gt_y, est_x, est_y, error` | `segment` (`stationary`·`straight`·`rotation`), `yaw_error`, `robot` | 시나리오당 100개 시점 이상 | 민지 |
| `cte.csv` | `timestamp, planned_x, planned_y, actual_x, actual_y, cte` | `segment` (`straight`·`curve`), `robot` | 구간별 분리 | 이진 |
| `response_time.csv` | `cmd_time, response_time, latency_ms` | `robot`, `task_id`, `latency_sim_ms`, `assignment_id`, `ack_time`, `ack_rtt_ms`, `resends` | 50회 이상 | 하연 |
| `fleet_latency.csv` | — (연구 1, 아래) | — | 메시지 1건 × 로봇 1대마다 1행 | 하연 정의, 하네스 기록기 (민지) |
| `hop_latency.csv` | — (연구 1, 아래) | — | 구독 수신 1건마다 1행 | 하연 (트레이스 변환) |
| `tasks.csv` | 명세서 작업 로그 필드 (아래) | — | 작업마다 1행 | 하네스 기록기 (민지) |
| `assignments.csv` | — (팀 정의, 아래) | — | 할당 호출마다 | 하연 (`amr_fleet_manager`) |
| `coverage/` | `colcon test --coverage` 텍스트·HTML | 모듈별 하위 폴더 | — | 전원 |

- `localization.csv`
  - `error`는 `sqrt((gt_x-est_x)^2 + (gt_y-est_y)^2)`이다. 명세서 KPI(정지 3cm·직선 5cm·회전 8cm)는 `segment`별로 산출한다.
  - `yaw_error`는 방향 오차(rad, -π~π로 정규화)다. 명세서 KPI는 위치만 보지만 회전 구간 분석과 RPE 대조에 쓴다.
  - `segment` 분류 기준(초안, `amr_localization.eval`): 정답 궤적의 각속도가 0.05 rad/s 이상이면 `rotation`, 아니고 선속도가 0.01 m/s 이상이면 `straight`, 둘 다 아니면 `stationary`. 곡선 주행은 `rotation`으로 센다.
  - KPI 판정은 구간별 **RMSE**가 기준 이하이고 샘플이 100개 이상일 때 통과로 본다. Max는 함께 보고한다.
  - 같은 실행의 원자료로 evo ATE를 계산해 `error`의 RMSE와 일치하는지 확인한다.
- `response_time.csv` (초안, #18 리뷰에서 도출)
  - `cmd_time`: 서버가 `/fleet/task_assignments`에 발행한 명령을 하네스 기록기가 받은 시각. 장애 주입기를 거치기 **전** 토픽을 기록하므로 주입된 지연이 응답 시간에 포함된다.
  - `response_time`: 해당 로봇의 `/<robot>/odom` 속도가 임계값(선속도 0.01 m/s 또는 각속도 0.05 rad/s, 초안)을 처음 넘은 메시지를 받은 시각.
  - `RobotState.velocity`(1~5Hz)는 쓰지 않는다. 최대 1초 오차가 생겨 200ms KPI를 잴 수 없다. `odom`은 50Hz라 오차가 20ms 이내다.
  - 두 시각 모두 **실제 시간**이다. 메시지의 `header.stamp`는 시뮬레이션 시간이라 쓰지 않고, 기록기의 수신 시각을 쓴다(rosbag2는 기본으로 실제 시간을 기록한다).
  - `latency_sim_ms`는 같은 구간을 시뮬레이션 시간으로 잰 참고값이다. `real_time_factor`와 함께 보면 시뮬레이터 속도 영향을 가를 수 있다.
  - Ack 왕복시간 (초안, 하연 제안). 응답 시간과 같은 기록기가 같은 실제 시간으로 잰다.
    - `cmd_time`은 그 `assignment_id`의 **첫 발송**을 기록기가 받은 시각이다. 재전송은 `cmd_time`을 바꾸지 않는다.
    - `ack_time`: 같은 `robot`·`assignment_id`의 `TaskAssignmentAck`를 `/fleet/task_assignment_acks`에서 처음 받은 시각. `result`(수락·거부)와 상관없이 첫 Ack.
    - `ack_rtt_ms` = (`ack_time` − `cmd_time`) × 1000. 첫 발송 기준이라 재전송에 걸린 시간이 들어간다.
    - `resends`: 기록기가 같은 `assignment_id`를 받은 횟수 − 1.
    - Ack를 못 받으면 `ack_time`, `ack_rtt_ms`를 비운다.
    - 로봇 → 서버 방향은 지금 장애 주입을 하지 않으므로([api.md 4-4절](../../docs/api.md#4-4-미결-쟁점-928-회의)) `ack_rtt_ms`에는 주입 지연이 한 번만 들어간다.
- `tasks.csv`: `/fleet/task_status`의 마지막 이벤트(COMPLETED·FAILED)를 작업 1건당 1행으로 옮긴다. 명세서가 요구하는 작업 로그 필드와 Fleet 지표(평균 작업 시간·처리량·총 이동 거리)를 여기서 계산한다.
  - 컬럼: `task_id, robot, state, start_time, end_time, duration, distance_traveled, error_code`
  - 로봇 가동률은 `/fleet/robot_states`의 `mode`가 BUSY인 시간 비율로 따로 계산한다.
- `assignments.csv`: 할당 알고리즘을 호출할 때마다 결과를 남긴다. `AssignmentResult`는 토픽으로 나가지 않으므로 `amr_fleet_manager`가 직접 기록해야 한다(2주차 구현 때 반영 요청).
  - 컬럼: `timestamp, call_id, algorithm, cost_fn, task_id, robot_id, cost`
  - 할당 못 한 작업은 `robot_id`를 비운다. 호출별 `total_cost`는 같은 `call_id`의 `cost` 합이다.
  - 알고리즘 3종 비교(연구 1)의 할당 최적성 지표로 쓴다.

### 연구 1 지연 (초안, 하연 제안)

명세서 3-2절의 "두 레이어 P95/P99"를 두 파일로 잰다. 레이어 1은 로봇 내부 노드 간, 레이어 2는 Fleet 서버 ↔ 로봇이다.

| 파일 | 무엇을 재나 | 원자료 | 레이어 |
| --- | --- | --- | --- |
| `fleet_latency.csv` | 서버 발행 → 로봇 쪽 도착 (장애 주입 포함, 끝에서 끝) | 하네스 기록기 수신 시각 | 2 |
| `hop_latency.csv` | 토픽 한 구간씩의 DDS 전달·콜백 시간 (병목 위치) | `trace/` | 1, 2 |

- `fleet_latency.csv`: 메시지 1건 × 로봇 1대마다 1행. 장애주입기 중계 대상 4개 토픽(`task_assignments`, `heartbeat`, `zone_grants`, `task_table`)
  - 컬럼: `topic, robot, key, send_time, recv_time, latency_ms, delivered`
  - `key`: 원본과 중계본을 짝짓는 번호. `task_assignments`는 `assignment_id`, `zone_grants`는 `request_id`, `heartbeat`·`task_table`은 `epoch:seq`. 장애주입기는 이 필드를 바꾸지 않는다([amr_fault_injector](../../src/fleet/amr_fault_injector/README.md))
  - `send_time`: 기록기가 원본 `/fleet/<topic>`을 받은 시각. `recv_time`: 기록기가 중계본 `/<robot>/fleet/<topic>`을 받은 시각. 둘 다 실제 시간
  - `latency_ms` = (`recv_time` − `send_time`) × 1000. 주입 지연 + 중계 1홉 + DDS. 주입 0ms 조건이 중계 오버헤드 기준선이다
  - `delivered`: 중계본이 오면 `1`, 실행이 끝날 때까지 안 오면 `0`(`recv_time`, `latency_ms`는 비움). 손실률을 여기서 센다
  - 같은 `key`가 여러 번 오면(재전송) 첫 도착만 쓴다
- `hop_latency.csv`: 구독 노드가 메시지를 꺼낸 1건(`ros2:rmw_take`, `taken=1`)마다 1행. `trace/`에서 오프라인으로 변환한다
  - 컬럼: `layer, node, topic, source_time, take_time, dds_ms, callback_ms`
  - `layer`: 토픽 이름이 `/fleet/*` 또는 `/<robot>/fleet/*`면 `2`, 나머지는 `1`
  - `node`, `topic`: 구독 핸들을 `rcl_node_init`·`rcl_subscription_init` 이벤트로 노드 이름·토픽에 대응시킨다
  - `source_time`: `rmw_take`의 `source_timestamp` (보낸 쪽이 DDS에 쓴 시각). `take_time`: `rmw_take` 이벤트 시각을 트레이스 시계 오프셋으로 실제 시간으로 바꾼 값
  - `dds_ms` = (`take_time` − `source_time`) × 1000. DDS 전달 + 실행기가 꺼내기까지 대기
  - `callback_ms`: 이어지는 `callback_start` ~ `callback_end`. rclpy 노드처럼 콜백 이벤트가 없으면 비운다
  - Humble의 `rmw_publish` 이벤트에는 시각 필드가 없어 발행 이벤트와 수신 이벤트를 1:1로 짝짓지 않는다. 대신 수신 쪽의 `source_timestamp`를 쓴다 (2주차 트레이스에서 필드 확인, #32)
  - 보낸 쪽과 받은 쪽이 같은 컴퓨터일 때만 맞다. 실물 네트워크 고도화처럼 컴퓨터가 나뉘면 시계 동기화(chrony 등)를 먼저 한다
- 요약 통계
  - 실행마다 그룹별로 P50·P95·P99·최대·N을 낸다. 그룹은 `fleet_latency`는 (`topic`, `robot`), `hop_latency`는 (`layer`, `topic`, `node`)
  - 반복 간에는 위 "요약 통계"와 같이 평균과 부트스트랩 95% 신뢰구간을 낸다
  - P99는 표본이 100개 미만이면 사실상 최대값이다. 실행 1회의 그룹 표본이 100개 미만이면 반복 실행을 합쳐(pool) 계산하고 그렇게 했다고 표시한다
  - 손실된 메시지(`delivered=0`)는 지연 분위수에서 빼고 손실률로 따로 보고한다

### 실행 결과 배치

```
tools/harness/results/<run_id>/
├── meta.json
├── trajectories/
│   ├── amr_01_gt.tum
│   └── amr_01_est.tum
├── localization.csv
├── cte.csv
├── response_time.csv
├── tasks.csv
├── assignments.csv
├── fleet_latency.csv
├── hop_latency.csv
├── trace/
└── coverage/
```

- `run_id`는 `<시나리오>_<YYYYMMDD-HHMMSS>_<반복 번호>` 형식을 쓴다.
- `tools/harness/results/`는 `.gitignore`에 등록돼 있어 커밋되지 않는다. 원자료는 공유 드라이브에 두고 경로만 기록한다(CONTRIBUTING 6절).

`meta.json`에는 결과를 나중에 추적·재현하는 데 필요한 정보를 적는다.

| 필드 | 예 | 이유 |
| --- | --- | --- |
| `git_sha` | `9ef33ea` | 어느 코드로 나온 결과인지 |
| `scenario`, `repeat`, `seed` | `loc_square_10m`, `3`, `42` | 반복·재현 |
| `fault` | `{"delay_ms": 100, "loss_pct": 0}` | 연구 1 조건별 비교 |
| `num_robots` | `5` | 다중 로봇 조건 |
| `assignment_algorithm`, `cost_fn` | `hungarian`, `euclidean` | 할당 알고리즘 비교. 비용 함수가 다르면 `total_cost`끼리 비교할 수 없다 |
| `controller` | `pure_pursuit` · `stanley` | 경로 추종 제어기 비교 (이진 요청, #5) |
| `outcome` | `completed` · `timeout` · `collision` · `error` | 실패한 실행을 통계에서 구분 |
| `frame_alignment` | `fixed` | 좌표계 정렬 방식 |
| `host` | CPU 모델·코어 수, 메모리, GPU 유무 | CPU 80%·응답 시간 KPI는 실행 컴퓨터에 따라 달라짐 |
| `versions` | ROS 배포판, 시뮬레이터, Docker 이미지 태그 | 환경 차이 추적 |
| `use_sim_time`, `real_time_factor` | `true`, `0.87` | 시뮬레이터가 실제보다 느리게 돌았는지 |
| `rmw_implementation` | `rmw_cyclonedds_cpp` | 연구 1 After 측정의 DDS 벤더 비교 (하연 제안) |
| `qos_profile` | `default` | 연구 1 After 측정의 QoS 튜닝 전후 구분. 값은 [api.md 6절](../../docs/api.md#6-qos-기본값) 프로파일 이름 (하연 제안) |

### 요약 통계

- 지표마다 평균·최대·N과 반복 간 **95% 신뢰구간**을 낸다. 명세서가 응답 시간에 평균과 최대를 함께 요구한다.
- 신뢰구간은 분포 가정이 필요 없는 **부트스트랩** 방식을 기본으로 한다. 반복 수가 적은 로봇 실험에서 쓰는 방식이다.
- 반복 횟수 N은 감으로 정하지 않고, 구분하고 싶은 차이 크기에서 **검정력 분석**으로 정한다(하네스 설계 과제, 명세서 8-1절).
- `outcome`이 `completed`가 아닌 실행은 지표 통계에서 빼고 실패율로 따로 보고한다.

### 합의 기록

| 항목 | 결정 | 출처 |
| --- | --- | --- |
| 응답 시간·연구 1 지연의 시간 기준 | 실제 시간 | 하연, #5 |
| 실제 시간 측정 방식 | 하네스 기록기 수신 시각 (노드 `header.stamp`는 시뮬레이션 시간 유지) | 하연, #27 |
| `response_time.csv` 부가 컬럼, `meta.json` 필드 | 설계와 맞으니 우선 진행, 바뀌면 수정 | 하연, #5 |
| `cte.csv` 부가 컬럼 | `segment`, `robot` 확정. `meta.json`에 `controller` 추가 | 이진, #5 |

남은 것: 응답 시간 판정 임계값(0.01 m/s·0.05 rad/s)은 초안이다. 하네스 구현 때 정지 상태 `odom` 노이즈를 보고 확정한다.

리뷰 중 (D5, 하연 제안): `response_time.csv` Ack 컬럼, 연구 1 지연(`fleet_latency.csv`, `hop_latency.csv`), `trace/`, `meta.json`의 `rmw_implementation`·`qos_profile`. 민지가 승인하면 위 표로 옮긴다.

### 참고

- Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS 2012 — TUM 궤적 형식, ATE·RPE
- Kümmerle et al., "On Measuring the Accuracy of SLAM Algorithms", Autonomous Robots 2009 — 상대 변위 기반 평가
- Zhang & Scaramuzza, "A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry", IROS 2018 — 궤적 정렬 방식과 지표의 장단점
- M. Grupp, evo — https://github.com/MichaelGrupp/evo
- Arena-Rosnav / arena-evaluation — 토픽별 CSV 기록 후 오프라인 지표 계산, 에피소드 결과 분류
- NVIDIA ros2_benchmark — 결과와 함께 실행 환경·버전·입력 해시 기록
- Colas et al., "How Many Random Seeds? Statistical Power Analysis in Deep RL Experiments", 2018 — 반복 횟수 결정, 부트스트랩 신뢰구간
