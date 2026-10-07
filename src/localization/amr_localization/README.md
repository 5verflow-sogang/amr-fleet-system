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

## 위치추정 스택 (`localization.launch.py`)

지도 서버 + AMCL + EKF 두 개(D4 A안, [api.md 2-1절](../../../docs/api.md#2-1-tf-발행-주체-d4-a안-ekf-두-개))를 띄운다. 설정 근거는 [localization.md](../../../docs/algorithms/localization.md).

```bash
ros2 launch amr_localization localization.launch.py                          # amr_01, 지도 v0
ros2 launch amr_localization localization.launch.py use_imu:=false           # IMU 없이 (센서 플러그인 전)
ros2 launch amr_localization localization.launch.py odom_topic:=wheel/odom   # DiffDrive odom과 겹칠 때 (#35)
```

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Subscribe | `<odom_topic>` (기본 `odom`) | `nav_msgs/msg/Odometry` | 휠 오도메트리 (`amr_odometry`). EKF가 `vx`·`vy`·`vyaw`를 씀 |
| Subscribe | `imu/data` | `sensor_msgs/msg/Imu` | IMU 필터 출력. EKF가 `vyaw`를 씀 (`use_imu:=true`일 때) |
| Subscribe | `scan` | `sensor_msgs/msg/LaserScan` | 스캔 필터 출력 (AMCL) |
| Subscribe | `/map` | `nav_msgs/msg/OccupancyGrid` | 지도. 로봇들이 같이 씀 |
| Publish | `amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | AMCL 추정 (`ekf_global` 입력) |
| Publish | `odometry/local` | `nav_msgs/msg/Odometry` | `ekf_local` 출력, odom 기준 (제어용, 점프 없음) |
| Publish | `odometry/filtered` | `nav_msgs/msg/Odometry` | `ekf_global` 출력, map 기준 (**위치 오차 KPI 측정 대상**) |
| Publish | `/tf` | | `map → <ns>/odom` (`ekf_global`), `<ns>/odom → <ns>/base_footprint` (`ekf_local`) |

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `namespace` | `amr_01` | 로봇. TF 프레임 접두어로도 쓴다 |
| `map` | `amr_mapping/maps/warehouse_v0.yaml` | 지도 |
| `start_map_server` | `true` | 로봇 여러 대면 한 번만 `true` |
| `odom_topic` | `odom` | 휠 오도메트리 토픽 |
| `use_imu` | `true` | IMU가 없는 환경에서는 `false` |
| `initial_x`·`initial_y`·`initial_yaw` | `0.0` | AMCL 초기 위치 (map 기준). 지도 v0에서 기본 스폰 world (0, 1.5) = map (0, 0) |

- 함께 띄워야 하는 것: 시뮬레이터와 로봇(`amr_gazebo`), 휠 오도메트리·센서 필터(`amr_odometry`). D4에 따라 `amr_odometry`는 `publish_tf:=false`(기본), DiffDrive는 `publish_odom_tf:=false`로 띄운다.
- 위치 오차는 `eval_node`(아래)로 잰다: 추정 `odometry/filtered`, `map_origin_in_world: [0.0, 1.5, 0.0]`.

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

- `map_origin_in_world`: world에서 본 map 원점 `[x, y, yaw]`. 지도 인계 때 받은 매핑 시작 자세를 넣는다. 기본값은 창고 지도 v0 기준 `[0.0, 1.5, 0.0]`, sandbox 미로(TurtleBot4)는 `[0.0, 0.0, 0.0]` ([amr_gazebo README](../../simulation/amr_gazebo/README.md) "GT 사용 시 주의"). 틀리면 그만큼 RMSE에 더해진다.
- 정답은 추정 시각으로 **선형 보간**해 짝짓는다. 두 토픽의 발행 시각이 어긋나서 생기는 오차를 없애기 위해서다.
- 저장 위치: `output_dir` (비우면 `/tmp/amr_eval/<robot>_<시각>`). `localization.csv`, `localization_summary.json`, `trajectories/<robot>_gt.tum`·`_est.tum`
- 궤적(원자료)은 받는 즉시 `trajectories/`에 이어 쓴다(약 1초마다 디스크로 내보냄). 오차 CSV·요약은 종료할 때 계산한다. 강제 종료돼 원자료만 남았으면 다시 계산한다:

```bash
ros2 run amr_localization eval_offline <결과 폴더> --robot amr_01   # 구간 기준을 바꿔 다시 계산할 때도
```

## Kidnapped 감지·복구 노드 (`kidnap_recovery`)

> 상태: 착수. 감지 동작, 가까운 이동(약 2 m)은 국소 재초기화로 복구, 먼 이동(다른 통로)은 아직 못 찾는다. 설계·실험·판정 기준은 [localization.md 4절](../../../docs/algorithms/localization.md#4-kidnapped-robot-복구).

```bash
ros2 run amr_localization kidnap_recovery --ros-args -r __ns:=/amr_01 -p use_sim_time:=true \
    -p events_file:=/tmp/kidnap_events.csv
```

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `lost_ratio`, `lost_checks` | 0.5, 3 | 스캔-지도 일치율이 이 값 아래로 연속 몇 번이면 위치 상실 (2 Hz) |
| `ok_ratio`, `ok_std_m`, `ok_checks` | 0.8, 0.15, 3 | 수렴 판정 (일치율, AMCL 위치 표준편차) |
| `local_first`, `local_std_m`, `local_std_yaw` | true, 2.5, 0.8 | 1차 시도는 마지막 확실한 자세 주변 국소 재초기화 |
| `explore_forward`, `explore_speed`, `front_clearance` | false, 0.2, 1.0 | 탐색 단계에서 앞 ±20°가 비어 있으면 회전 대신 전진 (시제품, 기본 끔) |
| `settle_s`, `allow_rotate`, `rotate_s`, `max_attempts` | 10, true, 15, 3 | 시도마다 정지 갱신 → 제자리 회전 |
| `events_file` | `''` | 감지·재초기화·복구 시각 CSV |

- 탐색(회전·전진) 중에만 `cmd_vel_recovery`(`cmd_vel_topic` 파라미터)를 낸다. twist_mux에서 내비(`cmd_vel_nav`)보다 우선하게 넣고, 그 뒤로 구역 허가(`amr_zone_client`)·안전(`amr_safety`)을 거쳐 최종 `cmd_vel`이 된다(#79 이진). 복구 중 내비 목표 중지는 BT가 한다(지수와 합의 예정).
- 상태는 `kidnap_recovery/state`(std_msgs/String)로 낸다.

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
