# harness

회귀 테스트 하네스 (담당: 민지). 명세서 5-2절.

- 같은 시나리오를 N회 반복해 KPI의 평균·신뢰구간을 산출한다.
- 장애 조건(지연·손실)별 결과와 ros2_tracing 데이터를 수집한다.
- `.github/workflows/fleet-test.yml`이 이 디렉토리의 진입 스크립트를 호출한다.
- 로그 포맷은 명세서 1-4절 "성능 지표 측정 절차" 표를 따른다.

## 로그 포맷 (초안, 하연과 합의 중)

> 상태: 초안. 하연과 합의하면 "초안" 표시를 뗀다. 컬럼 이름과 순서는 명세서 1-4절 표를 그대로 따르고, 이 문서는 단위·파일 배치·부가 컬럼만 정한다.

### 공통 규칙

- 형식: UTF-8 CSV, 첫 줄은 헤더. 지표 하나당 파일 하나.
- 시각: 시뮬레이션 시간(`use_sim_time: true`) 기준 초 단위 실수 (예: `123.456789`).
- 단위: 거리는 m, 지연은 ms. 컬럼 이름에 단위가 없으면 이 규칙을 따른다.
- 명세서 컬럼은 이름·순서를 바꾸지 않는다. 부가 컬럼은 **명세서 컬럼 뒤에만** 추가한다.
- 다중 로봇 시나리오는 부가 컬럼 `robot`(예: `amr_01`)을 붙인다.

### 지표별 파일

| 파일 | 명세서 컬럼 | 부가 컬럼 | 샘플 수 | 담당 |
| --- | --- | --- | --- | --- |
| `localization.csv` | `timestamp, gt_x, gt_y, est_x, est_y, error` | `segment` (`stationary`·`straight`·`rotation`), `robot` | 시나리오당 100개 시점 이상 | 민지 |
| `cte.csv` | `timestamp, planned_x, planned_y, actual_x, actual_y, cte` | `segment` (`straight`·`curve`), `robot` | 구간별 분리 | 이진 |
| `response_time.csv` | `cmd_time, response_time, latency_ms` | `robot`, `task_id` | 50회 이상 | 하연 |
| `coverage/` | `colcon test --coverage` 텍스트·HTML | 모듈별 하위 폴더 | — | 전원 |

- `localization.csv`의 `error`는 `sqrt((gt_x-est_x)^2 + (gt_y-est_y)^2)`이다. KPI(정지 3cm·직선 5cm·회전 8cm)는 `segment`별로 산출한다.
- 연구 1의 P95/P99 지연(ros2_tracing)은 하연이 별도 파일로 정의한다.

### 실행 결과 배치

```
tools/harness/results/<run_id>/
├── meta.json              # git SHA, 시나리오 이름, 반복 번호, 난수 시드, 장애 조건(delay_ms, loss_pct), 로봇 대수
├── localization.csv
├── cte.csv
├── response_time.csv
└── coverage/
```

- `run_id`는 `<시나리오>_<YYYYMMDD-HHMMSS>_<반복 번호>` 형식을 쓴다.
- 요약 결과는 지표마다 평균·최대·N과 반복 간 95% 신뢰구간을 낸다.
- `tools/harness/results/`는 `.gitignore`에 등록돼 있어 커밋되지 않는다. 원자료는 공유 드라이브에 두고 경로만 기록한다(CONTRIBUTING 6절).
