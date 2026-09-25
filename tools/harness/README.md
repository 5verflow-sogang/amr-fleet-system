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
- 시각: 초 단위 실수 (예: `123.456789`). 시간 기준(시뮬레이션 시간 / 실제 시간)은 아래 "합의 필요" 1번.
- 단위: 거리는 m, 각도는 rad, 지연은 ms. 컬럼 이름에 단위가 없으면 이 규칙을 따른다.
- 명세서 컬럼은 이름·순서를 바꾸지 않는다. 부가 컬럼은 **명세서 컬럼 뒤에만** 추가한다. 앞쪽 열만 잘라내면 명세서 형식이 되도록 하기 위해서다.
- 다중 로봇 시나리오는 부가 컬럼 `robot`(예: `amr_01`)을 붙인다.

### 원자료

| 파일 | 형식 | 내용 | 담당 |
| --- | --- | --- | --- |
| `trajectories/<robot>_gt.tum` | TUM 궤적 형식 | 시뮬레이터 Ground Truth 궤적 | 민지 (GT 토픽은 지수가 `amr_gazebo`에서 제공) |
| `trajectories/<robot>_est.tum` | TUM 궤적 형식 | EKF 추정 궤적 (`odometry/filtered`) | 민지 |
| `rosbag/` | rosbag2 (mcap) | 시나리오에서 정한 토픽 기록 (선택) | 시나리오 작성자 |

- TUM 궤적 형식: 한 줄에 `timestamp x y z qx qy qz qw` (공백 구분, 초·m·쿼터니언). evo가 그대로 읽는다.
- **좌표계 정렬**: GT(Gazebo world)와 추정값(`map`)은 서로 다른 좌표계다. 두 좌표계 사이 변환을 시나리오 설정에 고정값으로 두고, 정렬 방식을 `meta.json`에 기록한다. 평가 도구가 임의로 최적 정렬(Umeyama 등)을 하면 오차가 실제보다 작게 나올 수 있으므로 KPI 산출에는 고정 변환만 쓴다.
- rosbag은 용량이 커서 기본으로 켜지 않는다. 켜는 경우에도 커밋하지 않는다(CONTRIBUTING 6절).

### 지표 CSV

| 파일 | 명세서 컬럼 | 부가 컬럼 | 샘플 수 | 담당 |
| --- | --- | --- | --- | --- |
| `localization.csv` | `timestamp, gt_x, gt_y, est_x, est_y, error` | `segment` (`stationary`·`straight`·`rotation`), `yaw_error`, `robot` | 시나리오당 100개 시점 이상 | 민지 |
| `cte.csv` | `timestamp, planned_x, planned_y, actual_x, actual_y, cte` | `segment` (`straight`·`curve`), `robot` | 구간별 분리 | 이진 |
| `response_time.csv` | `cmd_time, response_time, latency_ms` | `robot`, `task_id` | 50회 이상 | 하연 |
| `coverage/` | `colcon test --coverage` 텍스트·HTML | 모듈별 하위 폴더 | — | 전원 |

- `localization.csv`
  - `error`는 `sqrt((gt_x-est_x)^2 + (gt_y-est_y)^2)`이다. 명세서 KPI(정지 3cm·직선 5cm·회전 8cm)는 `segment`별로 산출한다.
  - `yaw_error`는 방향 오차(rad, -π~π로 정규화)다. 명세서 KPI는 위치만 보지만 회전 구간 분석과 RPE 대조에 쓴다.
  - `segment` 분류 기준(GT 속도·각속도 임계값)은 2주차에 RMSE 측정 노드와 함께 정해 이 문서에 추가한다.
  - 같은 실행의 원자료로 evo ATE를 계산해 `error`의 RMSE와 일치하는지 확인한다.
- 연구 1의 P95/P99 지연(ros2_tracing)은 하연이 별도 파일로 정의한다.
- `cte.csv`의 부가 컬럼은 이진 확인이 필요하다.

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
| `outcome` | `completed` · `timeout` · `collision` · `error` | 실패한 실행을 통계에서 구분 |
| `frame_alignment` | `fixed` | 좌표계 정렬 방식 |
| `host` | CPU 모델·코어 수, 메모리, GPU 유무 | CPU 80%·응답 시간 KPI는 실행 컴퓨터에 따라 달라짐 |
| `versions` | ROS 배포판, 시뮬레이터, Docker 이미지 태그 | 환경 차이 추적 |
| `use_sim_time`, `real_time_factor` | `true`, `0.87` | 시뮬레이터가 실제보다 느리게 돌았는지 |

### 요약 통계

- 지표마다 평균·최대·N과 반복 간 **95% 신뢰구간**을 낸다. 명세서가 응답 시간에 평균과 최대를 함께 요구한다.
- 신뢰구간은 분포 가정이 필요 없는 **부트스트랩** 방식을 기본으로 한다. 반복 수가 적은 로봇 실험에서 쓰는 방식이다.
- 반복 횟수 N은 감으로 정하지 않고, 구분하고 싶은 차이 크기에서 **검정력 분석**으로 정한다(하네스 설계 과제, 명세서 8-1절).
- `outcome`이 `completed`가 아닌 실행은 지표 통계에서 빼고 실패율로 따로 보고한다.

### 합의 필요 (하연)

1. **시간 기준**: 위치추정은 GT와 추정값을 맞춰야 해서 시뮬레이션 시간을 쓴다. 응답 시간(200ms KPI)과 연구 1의 지연은 시뮬레이션 시간과 실제 시간 중 무엇으로 잴지 정해야 한다. 시뮬레이터가 실제보다 느리게 돌면 두 값이 달라진다.
2. **`response_time.csv` 부가 컬럼**과 `meta.json` 필드가 하연의 측정 방식과 맞는지.

### 참고

- Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS 2012 — TUM 궤적 형식, ATE·RPE
- Kümmerle et al., "On Measuring the Accuracy of SLAM Algorithms", Autonomous Robots 2009 — 상대 변위 기반 평가
- Zhang & Scaramuzza, "A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry", IROS 2018 — 궤적 정렬 방식과 지표의 장단점
- M. Grupp, evo — https://github.com/MichaelGrupp/evo
- Arena-Rosnav / arena-evaluation — 토픽별 CSV 기록 후 오프라인 지표 계산, 에피소드 결과 분류
- NVIDIA ros2_benchmark — 결과와 함께 실행 환경·버전·입력 해시 기록
- Colas et al., "How Many Random Seeds? Statistical Power Analysis in Deep RL Experiments", 2018 — 반복 횟수 결정, 부트스트랩 신뢰구간
