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

## 오차 기록 노드 (`eval_node`)

정답(`ground_truth/odom`, world 기준)과 추정(`odometry/filtered`, map 기준)을 받아 위 모듈로 오차를 계산하고 하네스 형식으로 저장한다.

```bash
ros2 run amr_localization eval_node --ros-args -r __ns:=/amr_01 \
  --params-file $(ros2 pkg prefix amr_localization)/share/amr_localization/config/eval.yaml \
  -p map_origin_in_world:="[0.0, 1.5, 0.0]"
ros2 service call /amr_01/localization_eval/save std_srvs/srv/Trigger   # 중간 저장 (종료할 때도 자동 저장)
```

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Subscribe | `ground_truth/odom` | `nav_msgs/msg/Odometry` | 정답, world 기준 (`amr_gazebo`) |
| Subscribe | `odometry/filtered` | `nav_msgs/msg/Odometry` | 추정, map 기준. `estimate_type:=pose`면 `geometry_msgs/msg/PoseWithCovarianceStamped` (예: `amcl_pose`) |
| Service | `~/save` | `std_srvs/srv/Trigger` | 지금까지 기록 저장, 응답에 구간별 요약 |

- `map_origin_in_world`: world에서 본 map 원점 `[x, y, yaw]`. 지도 인계 때 받은 매핑 시작 자세를 넣는다 ([amr_gazebo README](../../simulation/amr_gazebo/README.md) "GT 사용 시 주의"). 틀리면 그만큼 RMSE에 더해진다.
- 정답은 추정 시각으로 **선형 보간**해 짝짓는다. 두 토픽의 발행 시각이 어긋나서 생기는 오차를 없애기 위해서다.
- 저장 위치: `output_dir` (비우면 `/tmp/amr_eval/<robot>_<시각>`). `localization.csv`, `localization_summary.json`, `trajectories/<robot>_gt.tum`·`_est.tum`

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
