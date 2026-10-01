# amr_localization

- 담당: 민지
- 빌드 타입: ament_python
- 책임: AMCL/EKF 설정, Kidnapped Robot 복구, GT 대비 RMSE 측정. 지도는 `amr_mapping`에서 받는다

## 제공 인터페이스

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Publish | | | |
| Subscribe | | | |
| Service | | | |
| Action | | | |

전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

## 정확도 평가 모듈 (`amr_localization.eval`)

ROS에 의존하지 않는 순수 Python 모듈이다. 시뮬레이터 없이 단위 테스트한다. 로그 형식은 [하네스 로그 포맷](../../../tools/harness/README.md)을 따른다.

```python
from amr_localization.eval import read_tum, associate, compute_errors, summarize_by_segment, kpi_check

gt = read_tum('amr_01_gt.tum')
est = read_tum('amr_01_est.tum')
samples = compute_errors(associate(gt, est, max_dt=0.02), gt)
summaries = summarize_by_segment(samples)   # 정지·직선·회전별 n, RMSE, 평균, 최대
print(kpi_check(summaries))                 # 구간별 통과 여부 (정지 3cm·직선 5cm·회전 8cm)
```

| 함수 | 하는 일 |
| --- | --- |
| `read_tum`, `write_tum` | TUM 궤적 파일 입출력 (evo와 같은 형식) |
| `associate` | 추정 시점마다 시각이 가장 가까운 정답 시점을 짝짓기 (기본 허용 0.02초) |
| `compute_errors` | 위치 오차·방향 오차·구간(정답 속도로 분류) |
| `summarize_by_segment`, `kpi_check` | 구간별 RMSE·Max와 KPI 판정 |
| `write_localization_csv` | 하네스 `localization.csv` 쓰기 |

## 파라미터

`config/` 의 YAML로 관리한다. 파라미터 변경에 재빌드가 필요 없어야 한다.

## 실행

```bash
# TODO
```

## 테스트

```bash
colcon test --packages-select amr_localization
```
