# amr_gazebo

- 담당: 지수
- 빌드 타입: ament_cmake (월드·모델 자원 + 동적 장애물 Python 노드)
- 책임: 60x40m 물류센터 월드, 동적 장애물, 센서 노이즈 모델

## 제공 인터페이스

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Publish | `/clock` | `rosgraph_msgs/msg/Clock` | 시뮬레이션 시간. 다른 노드는 `use_sim_time:=true`로 쓴다 |
| Publish | `/<ns>/diff_drive/odom` | `nav_msgs/msg/Odometry` | DiffDrive 이상적 휠 오도메트리 (노이즈·공분산 없음), 50 Hz. 로봇 판단용 `/<ns>/odom`은 `amr_odometry`가 낸다 |
| Publish | `/<ns>/joint_states` | `sensor_msgs/msg/JointState` | 바퀴 조인트 각도 |
| Publish | `/<ns>/ground_truth/odom` | `nav_msgs/msg/Odometry` | **GT(진짜 위치·속도)**, 기본 100 Hz (`gt_rate`). `frame_id: world`(Gazebo 월드 원점 = 창고 중심), `child_frame_id: <ns>/base_footprint`. TF로는 내보내지 않음. **평가 전용**(RMSE, 제동 거리) — 로봇 판단에 쓰지 않는다. 아래 "GT 사용 시 주의" 참고 |
| Publish | `/<ns>/scan_raw` | `sensor_msgs/msg/LaserScan` | 2D LiDAR 원본 (노이즈 포함), 10 Hz, frame `<ns>/lidar_link` |
| Publish | `/<ns>/imu/data_raw` | `sensor_msgs/msg/Imu` | IMU 원본 (노이즈·바이어스 포함, 공분산 비어 있음), 100 Hz, frame `<ns>/imu_link` |
| Publish | `/<ns>/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB (`rgb8`, 640×480), 30 Hz, frame `<ns>/camera_optical_frame` |
| Publish | `/<ns>/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Depth (`32FC1`, m, 0.3~10 m), 30 Hz. RGB와 픽셀 1:1 대응 |
| Publish | `/<ns>/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB·Depth 공통 내부 파라미터 (fx = fy = 337.2, cx 320, cy 240) |
| Publish | `/tf` | `tf2_msgs/msg/TFMessage` | `<ns>/odom` → `<ns>/base_footprint` (`publish_odom_tf:=true`, 기본). EKF(`ekf_local`)를 띄우면 `false`로 끈다 (D4) |
| Subscribe | `/<ns>/cmd_vel` | `geometry_msgs/msg/Twist` | 속도 명령. 2.0 m/s, 1.0 m/s², 1.5 rad/s로 제한 |
| Publish | `/dynamic_obstacles/<id>/ground_truth/odom` | `nav_msgs/msg/Odometry` | 동적 장애물 GT (위치·속도), 50 Hz. `frame_id: world`, `child_frame_id: <id>/base_footprint`, twist는 몸체 기준. **평가 전용** (`dynamic_obstacles:=true`일 때) |
| Subscribe | `/dynamic_obstacles/<id>/cmd_vel` | `geometry_msgs/msg/Twist` | 장애물 속도 명령 (몸체 기준, `dynamic_obstacles_node`가 낸다) |
| Service | | | |
| Action | | | |

`<ns>` = 로봇 네임스페이스 (기본 `amr_01`). 로봇 모델은 [amr_description](../amr_description/README.md) 참고.

전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

### GT 사용 시 주의

- **주기**: 기본 100 Hz. 짝짓기 오차를 더 줄여야 하면 `spawn_robot.launch.py gt_rate:=200`처럼 올린다 (시뮬레이션 시간 기준 주기).
- **속도(`twist`) 지연**: 이동 평균이 아니라 직전 샘플과의 차분으로 보인다. 실측(1.0 m/s 가속·제동)에서 위치 중앙차분 속도보다 **항상 한 샘플(100 Hz에서 10 ms) 늦었다.** 가속 시간처럼 타이밍이 중요한 측정은 GT 위치를 미분해서 쓰거나 한 샘플만큼 보정한다.
- **`world`와 `map`은 원점이 다르다**: slam_toolbox는 매핑을 시작한 위치·방향을 `map` 원점으로 잡는다. GT(`world`)와 위치 추정(`map`)을 비교하려면 둘 사이 변환이 필요하고, 이 변환 오차는 RMSE에 그대로 더해진다(정지 KPI 3 cm). 그래서 **지도를 인계할 때 매핑 시작 위치·방향(`world` 기준 x, y, yaw)을 함께 기록한다.** 기본 스폰 위치는 (0, 1.5, 0)이다.

## 월드 레이아웃 (`worlds/warehouse.sdf`)

좌표계: 월드 원점 = 창고 중심, x −30~30 m, y −20~20 m, +y가 북쪽. 벽 안쪽 면 기준 60 × 40 m.
레이아웃은 [원 명세서 8장 결과 예시](../../../docs/project-spec-original.md)를 따른다.

| 구역 | 모델 이름 | 위치 (x, y) | 비고 |
| --- | --- | --- | --- |
| 입고 도크 | `dock_1`, `dock_2` | 서쪽 벽 (−30, 16), (−30, 11) | 플랫폼 1.5 m + 정차 구역(노란 표시) 3 m, +x 방향 |
| 출고 도크 | `dock_a`, `dock_b` | 동쪽 벽 (30, 16), (30, 11) | −x 방향 |
| 메인 통로 | 바닥 표시 | y 8.5 ~ 18.5 | 도크 사이 동서 통로 |
| 선반 A열 | `rack_a1`~`rack_a7` | y = 5, x = −22.5 ~ 22.5 (7.5 m 간격) | 랙 5.0 × 1.2 × 2.5 m |
| 선반 B열 | `rack_b1`~`rack_b7` | y = −2 | `rack_b4`만 x = −1.9 |
| 선반 C열 | `rack_c1`~`rack_c7` | y = −9 | |
| **통로 분리 랙 (2주차)** | `rack_lane_divider` | (−1.9, −0.2) | A–B 통로를 남북으로 나눠 아래 5.0 m 좁은 통로를 만든다. `rack_b4`와 같은 x(수정 기록: 처음엔 x=0이라 어긋나 있었음, 아래 "검증" 절 참고) |
| 열 사이 통로 | | A–B(북쪽 잔여): y 0.4~4.4, B–C: y −8.4~−2.6 | 폭 4.0 m / 5.8 m |
| 교차 통로 | | 같은 열 랙 사이 | 폭 2.5 m |
| **좁은 통로 ①** | `rack_b3`–`rack_b4` 사이 | x −5.0~−4.4, y −2.6~−1.4 | **폭 0.6 m, 길이 1.2 m** (로봇 폭 0.4 m + 0.2 m, Nav2 KPI용) |
| **좁은 통로 ② (2주차)** | `rack_lane_divider`–`rack_b4` 사이 | x −4.4~0.6, y −1.4~−0.8 | **폭 0.6 m, 길이 5.0 m** (①보다 긴 구간에서 지속 주행 테스트용) |
| 기둥 | `pillar_1`~`pillar_6` | (±11.25, 7.0), (±15, 1.5), (±7.5, −5.5) | 0.6 × 0.6 × 6 m, 통로 가운데 |
| 충전 스테이션 | `charger_c1`~`c3` | 남쪽 벽 (−24, −20), (−21, −20), (−18, −20) | 본체 0.3 m + 도킹 패드(초록 표시), +y 방향 |
| 충전 구역 | 바닥 표시 (파랑) | x −26~−16, y −20~−16 | |
| 대기 구역 | 바닥 표시 (회색) | x 16~26, y −20~−14 | |

재사용 모델은 `models/`에 있고 월드에서 `model://<이름>`으로 불러온다.
경로는 `package.xml`의 `gazebo_model_path` export를 통해 `ros_gz_sim`이 자동으로 등록한다.

### 표지판 (YOLO 학습용)

연우 YOLO 인식(#99)이 표지판 클래스를 배울 수 있게 표지판 13개를 랙 끝면·벽에 붙였다(바닥 스탠드 없음). 좌표는 GT 자동 라벨링에 쓰므로 **SDF 고정 좌표**다 (바꾸면 이 표와 라벨링 설정을 같이 고친다).

| 모델 | 종류 | 모양·색 |
| --- | --- | --- |
| `sign_aisle_a` / `_b` / `_c` | 통로 번호판 (A/B/C열) | 보라 판 0.5 × 0.4 m, 흰 7-세그먼트 글자 `A`·`b`·`C` |
| `sign_no_entry` | 진입 금지 | 빨간 원판 지름 0.4 m, 흰 가로 막대 |
| `sign_speed_limit` | 속도 제한 | 빨간 테 지름 0.4 m, 흰 원판, 검은 숫자 `5` |

- 텍스처 없이 visual 색과 단순 도형만 쓴다. 랙(주황)·벽(회백)·기둥(회색)·바닥 표시(노랑·파랑·초록)와 겹치지 않는 보라·빨강·흰색이다.
- 모델 원점 = 판 중심, 앞면이 +x. 판 두께 0.02 m, 붙인 면에서 0.015 m 앞(판 중심)에 둔다. 충돌체 없음(시각 전용).
- **배치 조건** (지수 확인, #92): LiDAR 스캔은 지면 약 0.38 m 높이의 수평면 1장이라 표지판 아래 끝은 0.5 m 이상, 표지판은 바닥에서 0.5~2 m 사이. 지금 배치는 아래 끝 0.7 m 이상, 중심 0.9~1.3 m이고 지지대가 없어 스캔·지도에 안 잡힌다 — 지도(`warehouse_v1`, #96) 재생성이 필요 없다. 도크 플랫폼(높이 1.0 m) 위는 화물 상자 자리라 피했다.
- **카메라**: 지면 0.43 m(`base_link` + 0.25 m), 수평 화각 87°·수직 약 71°. 중심 높이 0.9~1.3 m 표지판은 약 1 m 거리부터 화면에 들어온다.
- 좁은 통로 ①②, 입출고 도크(플랫폼 포함), 충전소·충전 구역, 대기 구역, 동적 장애물 경로, `zones.yaml` 구역(`narrow_01`·`narrow_02`)과 겹치지 않는다.

| 이름 | 종류 | x | y | z (중심) | yaw | 붙인 곳 | 가까운 통로 중심선까지 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `sign_aisle_a_1` | 통로 번호판 A | −4.985 | 5.3 | 1.0 | 0 | `rack_a3` 동쪽 끝면 | A–B 3.8 m |
| `sign_aisle_a_2` | 통로 번호판 A | 12.485 | 4.7 | 1.2 | π | `rack_a6` 서쪽 끝면 | A–B 3.2 m |
| `sign_aisle_b_1` | 통로 번호판 B | −12.485 | −1.7 | 0.9 | 0 | `rack_b2` 동쪽 끝면 | A–B 3.2 m |
| `sign_aisle_b_2` | 통로 번호판 B | 12.485 | −2.3 | 1.1 | π | `rack_b6` 서쪽 끝면 | B–C 3.2 m |
| `sign_aisle_c_1` | 통로 번호판 C | −4.985 | −8.7 | 1.0 | 0 | `rack_c3` 동쪽 끝면 | B–C 3.2 m |
| `sign_aisle_c_2` | 통로 번호판 C | 4.985 | −9.3 | 1.3 | π | `rack_c5` 서쪽 끝면 | B–C 3.8 m |
| `sign_no_entry_1` | 진입 금지 | 0 | −19.985 | 1.2 | π/2 | 남쪽 벽 | 남쪽 5.2 m |
| `sign_no_entry_2` | 진입 금지 | 0 | 19.985 | 1.2 | −π/2 | 북쪽 벽 | 메인 6.5 m |
| `sign_no_entry_3` | 진입 금지 | 25.015 | −9.0 | 1.0 | 0 | `rack_c7` 동쪽 끝면 | B–C 3.5 m |
| `sign_speed_limit_1` | 속도 제한 | −17.515 | 4.65 | 1.0 | π | `rack_a2` 서쪽 끝면 | A–B 3.2 m |
| `sign_speed_limit_2` | 속도 제한 | 29.985 | 7.0 | 1.2 | π | 동쪽 벽 (A열과 메인 통로 사이, `dock_b` 플랫폼 y 9.25~12.75 밖) | A–B 5.5 m (메인 6.5 m) |
| `sign_speed_limit_3` | 속도 제한 | 10.015 | −2.3 | 0.9 | 0 | `rack_b5` 동쪽 끝면 | B–C 3.2 m |
| `sign_speed_limit_4` | 속도 제한 | −29.985 | −4.3 | 1.1 | 0 | 서쪽 벽 | B–C 1.2 m |

yaw는 판 앞면이 향하는 방향(0 = +x 동쪽, π/2 = +y 북쪽). 통로 중심선: 메인 y 13.5 (y 8.5~18.5), A–B y 1.5 (y −1.4~4.4), B–C y −5.5 (y −8.4~−2.6), 남쪽 y −14.8 (C열 남쪽 면 −9.6 ~ 남쪽 벽). 표의 거리는 표지판 y와 가장 가까운 중심선의 차이다.

### 좁은 통로 ② 추가 검증 (2주차)

기존 통로 ①(0.6×1.2 m)은 랙 깊이만큼만 좁아 로봇이 진입각을 바꿔가며 여러 미터를
좁은 폭으로 유지 주행하는 상황을 테스트하기 어려웠다. 기존 랙 위치는 하나도 옮기지
않고 `rack_lane_divider` 1개만 A–B 통로에 추가해 0.6×5.0 m 통로를 만들었다.

**수정 기록**: 처음 추가할 때 `rack_lane_divider`를 창고 중심(x=0)에 둬서
x −2.5~2.5 구간을 막는 랙이 됐는데, 그 아래 `rack_b4`는 x −4.4~0.6 구간이라
둘이 겹치는(=양쪽이 다 막힌 0.6 m 통로) 구간은 x −2.5~0.6, 약 **3.1 m**뿐이었다.
`rack_lane_divider`를 `rack_b4`와 같은 x(−1.9)로 옮겨 x 구간을 완전히 맞췄다 —
이제 통로 전체(5.0 m)가 양쪽 다 막힌 0.6 m 폭이다.

- **남은 메인 통로 폭**: `rack_lane_divider`가 걸쳐 있는 x −4.4~0.6 구간에서만 A–B
  통로 북쪽 잔여 폭이 5.8 m → **4.0 m**로 줄어든다(그 밖의 x 구간은 그대로 5.8 m).
  로봇 2대가 엇갈려 지나가는 데 필요한 최소 폭(1.5 m 이상)을 여유 있게 만족한다.
- **입출고·충전소 경로**: `rack_lane_divider`(x −4.4~0.6, y −1.4~−0.8)는 입출고 도크
  (y 8.5~18.5)나 충전 스테이션(남쪽 벽, y=−20 부근)과 겹치지 않아 그쪽 경로를 막지 않는다.
- **기둥**: `pillar_1`~`pillar_6` 중 어느 것도 x −4.4~0.6, y −0.8~0.4(새 랙이 차지하는
  영역)와 겹치지 않는다(가장 가까운 `pillar_3`/`pillar_4`도 x=∓15, y=1.5로 멀다).
- **다른 랙**: 서쪽의 `rack_b3`(x −10~−5)와는 x −5.0~−4.4(= 좁은 통로 ①의 x 구간)만큼
  떨어져 있어 겹치지 않는다. 동쪽으로는 가장 가까운 `rack_b5`(x 5~10)와도 멀다.
- **좁은 통로 ①의 북쪽 출구**: 통로 ①은 x −5.0~−4.4, y −2.6~−1.4다. `rack_lane_divider`의
  서쪽 끝이 정확히 x=−4.4라서 두 통로는 그 선을 경계로 **맞닿아 있을 뿐 겹치지 않는다**
  (통로 ①은 x<−4.4, 통로 ②는 x≥−4.4) — 통로 ①을 빠져나와 북쪽(y>−1.4)으로 가는
  경로는 여전히 열려 있다.
- **동적 장애물 경로**: `config/dynamic_obstacles.yaml`의 5개 경로(및 그 아래 바닥
  표시) 중 x −4.4~0.6, y −1.4~−0.8을 지나는 것은 없다 — 가장 가까운 `worker_2`도
  y 1.0 이상에서만 움직인다.

**지도에 영향 있음**: `amr_mapping/maps/warehouse_v0.*`(#38)는 이 월드를 수정 전
(`rack_lane_divider` x=0) 상태로 SLAM해서 만든 지도다. 이번 수정으로 그 랙이
x=−1.9로 옮겨졌으니, 지도의 x −2.5~0 부근(랙이 있던 자리와 지금 있는 자리)이
실제 월드와 안 맞게 됐다 — 이 PR은 그 지도 파일을 고치지 않는다. 지수가 다음
매핑 때 다시 떠야 한다.

## 물리 · 조명 (2주차, 이진)

### 바닥 마찰 (`floor` 모델 collision, `<surface><friction><ode>`)

| 파라미터 | 값 | 근거 |
| --- | --- | --- |
| `mu`, `mu2` | 0.7 (등방성) | **[가정]** 실측·인용 자료 없음. 고무 바퀴-에폭시/콘크리트 바닥 접촉의 통상적 마찰계수 범위(약 0.5~0.8)의 중간값을 썼다. 드리프트 실험에서 바닥 슬립이 원인으로 의심되면 실측치로 교체해야 한다 |

### 조명 (`scene`, `light`)

원래 있던 실외용 `directional "sun"` 광원을 지우고 천장 점광원으로 바꿨다.

**v0(점광원 6개, 3열×2행, range 20m, constant/linear/quadratic = 0.3/0.05/0.002)**은
noVNC로 확인해보니 광원 바로 아래만 동그랗게 밝고 사이가 눈에 띄게 어두웠다.
아래 두 가지로 고쳐 **v1**로 반영했다.

| 항목 | v0 → v1 | 근거 |
| --- | --- | --- |
| 조명 개수·배치 | 점광원 6개(3열×2행, 행 간격 20m) → **9개(3열×3행, 행 간격 10m)** | 그리드 간격을 줄여 가장 가까운 광원까지의 최대 거리를 줄임. "너무 많이 늘리지 않는다" 기준으로 50%만 추가 |
| `attenuation` (constant/linear/quadratic) | 0.3 / 0.05 / 0.002 → **0.6 / 0.02 / 0.001** | 감쇠 공식 `1/(constant + linear·d + quadratic·d²)`에서 constant를 올리고 linear·quadratic을 낮춰 거리에 따른 밝기 변화를 완만하게 함(=명암 대비 감소) |
| `attenuation.range` | 20 m → **28 m** | 9개 그리드의 대각선 간격(약 14m)에서 range 컷오프에 걸리지 않게 여유를 둠 |
| `cast_shadows` | `false` (유지) | 점광원 여러 개의 실시간 그림자는 CPU 렌더링 비용이 큼. "그림자는 필요한 것만" 기준으로 계속 0개 유지 |
| scene `ambient` | 0.6 → **0.7** | 광원 사이 바닥의 기본 밝기를 올려 명암 대비를 한 번 더 줄임 |

**[가정]** 정량적 밝기(lux) 실측 검증은 하지 않았다. noVNC 육안 확인 기준으로 조정한
값이며, 연우가 YOLO를 붙일 때 여전히 고르지 않거나 너무 어두우면 다시 조정한다.

## 동적 장애물 자리 (2주차, 이진 — 좌표만, 동작 구현은 지수)

`worlds/warehouse.sdf`의 `dynamic_obstacle_paths` 모델에 바닥 표시(두께 1 mm,
`<collision>` 없음)로 경로·스폰 위치를 표시했다. LiDAR 설치 높이(약 0.2 m)보다
훨씬 얇아 스캔이나 SLAM 맵에 잡히지 않는다. 같은 좌표를
[`config/dynamic_obstacles.yaml`](config/dynamic_obstacles.yaml)에 기계가 읽기 쉬운 형태로도 정리했다.

| id | 종류 | 패턴 | 속도(m/s) | 경로 요약 |
| --- | --- | --- | --- | --- |
| `worker_1` | 작업자 | 직선 | 0.3~0.8 | 메인 통로 동서 왕복, (−24,13)→(24,13) |
| `worker_2` | 작업자 | 곡선 | 0.3~0.8 | A열 남은 통로(y 0.4~4.4) 지그재그, 기둥 3/4 회피 |
| `forklift_1` | 지게차 | 직선 | 0.8~1.5 | B–C 열 사이 통로 동서 왕복, 기둥 5/6과 0.7 m 이상 이격 |
| `forklift_2` | 지게차 | 무작위 | 0.8~1.5 | 대기 구역 북쪽 인근 4점 사이 |
| `robot_1` | 다른 로봇 | 곡선 | 0.5~1.2 | 서쪽 서비스 통로(x=−26) → 충전 구역 방향 |

색상: 작업자=주황, 지게차=빨강, 다른 로봇=보라 (기존 통로선 노랑·충전 파랑·대기 회색과 구분).

### 동적 장애물 이동 (3주차, 지수)

`sim.launch.py dynamic_obstacles:=true`(또는 `dynamic_obstacles.launch.py`)로 위 5개를 스폰하고 움직인다.
기본은 꺼져 있다 (지도 매핑에 장애물이 섞이지 않게).

| 종류 | 모델 | 크기 | 질량 |
| --- | --- | --- | --- |
| 작업자 | `models/dynamic_worker` | 겉모습 사람 메시(키 1.77 m), 충돌체 원기둥 지름 0.5 m | 70 kg |
| 지게차 | `models/dynamic_forklift` | 1.2 × 0.9 × 2.0 m | 1500 kg |
| 다른 로봇 | `models/dynamic_robot` | 0.6 × 0.4 × 0.3 m (우리 로봇과 같은 크기) | 30 kg |

- 이동: `dynamic_obstacles_node`가 장애물 GT로 위치를 보고 20 Hz로 속도를 낸다 (`amr_gazebo/obstacle_motion.py`)
  - `straight`: 첫 점 ↔ 끝 점 직선 왕복
  - `curve`: 경유점을 모두 지나는 Catmull-Rom 곡선을 따라 왕복
  - `random`: 경유점이 이루는 사각 영역 안 무작위 목표점으로 직선 이동 (random waypoint)
  - 속도는 구간(왕복 한 번, 무작위 목표점 하나)마다 `speed_range_mps` 안에서 무작위. `obstacle_seed`로 재현한다 (0이면 실행마다 무작위)
  - 몸체는 진행 방향을 보도록 돈다 (최대 1.5 rad/s). 이동 자체는 방향과 상관없이 경로를 따른다
- Gazebo 쪽: `VelocityControl`(속도 명령) + `OdometryPublisher`(GT). 중력을 끄고 충돌체를 바닥에서 2.5 cm 띄웠다.
  바닥에 닿으면 마찰이 발밑을 잡아 키 큰 작업자가 넘어졌다. 로봇과는 충돌한다
- `VelocityControl`은 마지막 명령을 유지한다. 노드만 죽으면 장애물이 계속 같은 방향으로 가니 sim도 같이 다시 띄운다

sim 확인 (로봇 (0, 10.5), seed 1, 150 s):

| id | 이동 | 측정 속도 (GT) | 범위 |
| --- | --- | --- | --- |
| `worker_1` | 81 m, x −24.0\~23.9 | 0.38\~0.75 m/s | 직선 위 (y 13.0) |
| `worker_2` | 53 m, x −20.0\~19.9 | 0.25\~0.37 m/s | y 1.0\~3.8 |
| `forklift_1` | 182 m, x −24.0\~19.9 | 0.98\~1.46 m/s | y −6.7\~−6.5 |
| `forklift_2` | 168 m | 0.60\~1.49 m/s | x 5.0\~17.8, y −16.9\~−13.0 |
| `robot_1` | 125 m | 0.55\~1.11 m/s | x −26.3\~−22.1, y −17.2\~13.0 |

- 측정 속도 최솟값이 범위보다 약간 낮은 것은 방향이 바뀌는 순간 `OdometryPublisher`가 위치 차분을 평균 내기 때문이다. 명령 속도가 범위 안인지는 단위 테스트로 확인한다
- LiDAR: 로봇 12 m 안에 들어온 `worker_1`은 858회 중 856회 사람 폭(±0.45 m) 안 최근접 스캔이 GT 거리 ±0.3 m 안. `worker_2`는 A열 랙에 가려 틈으로 보일 때만 잡힌다
- 작업자 겉모습은 사람 메시라 LiDAR 스캔 높이(지면 0.38 m)에서는 원기둥이 아니라 **다리 두 개**로 보인다 (실제 사람과 같다)
- 카메라(YOLO): 사전학습 `yolov8n`으로 정면 3.5 m에서 `person` 0.88, 옆모습 6 m에서 0.63. 지게차·다른 로봇은 단순 상자라 COCO 클래스로 잡히지 않는다

### 작업자 사람 메시

`models/dynamic_worker/meshes/MaleVisitorWalk.dae` + `materials/textures/MaleVisitor.png`:
OpenRobotics "Male visitor" ([Gazebo Fuel](https://app.gazebosim.org/OpenRobotics/fuel/models/Male%20visitor), 작성 Wan Yi Seow, **CC0 1.0**), 약 0.9 MB.

- 걷기 애니메이션용 메시라 정지 메시로 쓰면 **T자 자세**(팔 폭 1.64 m)로 그려진다. 팔은 1.4 m 높이라 LiDAR·로봇 충돌에는 영향이 없다
- 정면이 +x라 몸체가 진행 방향을 보면 사람도 앞을 보고 걷는다 (다리는 움직이지 않고 미끄러지듯 이동)

### 화물 상자 (YOLO 인식용, 3주차 지수, #99 연우 요청)

`sim.launch.py cargo:=true`(또는 `cargo_boxes.launch.py`)로 화물 상자 24개를 띄운다. **기본은 꺼져 있다.**
원 명세 인식 3종(화물·사람·표지판) 중 화물을 시뮬 영상으로 학습·평가하려고 넣었다.

| 크기 (명세 "가상 물품") | 길이 × 폭 × 높이 | 개수 | 놓인 곳 |
| --- | --- | --- | --- |
| `small` | 30 × 20 × 15 cm | 9 | 바닥 (랙 긴 면 옆) 7, 도크 플랫폼 2 |
| `medium` | 50 × 40 × 30 cm | 11 | 바닥 7, 도크 플랫폼 4 |
| `large` | 60 × 50 × 40 cm | 4 | 도크 플랫폼만 |

- **위치는 `config/cargo_boxes.yaml` 고정 좌표다** (static). 상자 중심 = (x, y, z + 높이/2), world 기준이라 GT 라벨로 바로 쓴다
- 겉모습: 진한 골판지색 몸체 + 윗면 테이프 + 양쪽 긴 면 흰 송장 라벨 (`amr_gazebo/cargo.py`). 밝은 골판지색은 조명에 하얗게 날아가 바닥과 구분이 안 돼서 진하게 했다
- 상자마다 static 모델 하나(이름 = 상자 id)로 스폰한다. 라벨 카메라가 모델 단위로 상자를 내서, 한 모델에 링크로 묶으면 떨어진 상자들이 큰 상자 하나로 합쳐진다(아래 "라벨 카메라"). 충돌체가 있어 로봇과 부딪힌다

**지도·위치추정에 영향을 주지 않게 놓았다.** 지도 v1에 없는 물체가 스캔에 찍히면 AMCL 일치율과 `map_quality`가 어긋나기 때문이다.

- LiDAR 스캔 평면(지면 0.38 m) ± 5 cm에 걸리는 상자가 없다. 바닥에는 높이 15·30 cm인 small·medium만, large는 도크 플랫폼(높이 1.0 m, 이미 지도에 있음) 위에만 둔다
- 같은 자세(0, 1.5)에서 상자 없음/있음 스캔(빔별 30회 평균)을 비교하면 평균 차이 0.6 cm(노이즈 수준), 10 cm 넘게 달라진 빔 0개
- 단, LiDAR가 못 보는 높이라 **Nav2 costmap(LiDAR 기반)에도 안 잡힌다.** 그래서 바닥 상자는 랙 면에 2 cm 띄워 붙이고 랙 끝 쪽에 뒀다 (매핑·주행 경로는 랙에서 1 m 넘게 떨어짐)
- 동적 장애물 경로(곡선·직선)에서 0.5 m 넘게, 좁은 통로 ①② 밖에 둔다

카메라 확인 (지면 0.43 m = base_link 0.18 + camera 0.25, `cargo:=true`): 출발점 (0, 1.5)에서 A–B 통로 양쪽 상자, B–C 통로 (−1, −4)에서 B·C열 상자,
도크 정차 구역 (−25, 13.5) 서쪽 방향에서 플랫폼 위 상자가 보인다. 바닥 상자는 통로 중앙에서 약 1.3~8 m, 도크 상자는 정차 구역에서 1.5~4.5 m다.

### 라벨 카메라 (YOLO 학습 데이터 수집, 연우)

`label_camera:=true`면 로봇의 RGBD 카메라와 같은 자리·화각(87°)·해상도(640×480)에 Gazebo `boundingbox_camera`를 하나 더 단다. Label 플러그인이 붙은 모델마다 **화면에 보이는 부분의 2D 상자**를 `/<ns>/camera/labels`(`vision_msgs/Detection2DArray`, 10 Hz)로 낸다. 렌더링 부담이 있어 기본은 꺼져 있고, 수집 노드는 [amr_detection `collect_dataset`](../../perception/amr_detection/README.md)이다.

| 라벨 번호 | YOLO 클래스 | 붙인 모델 |
| --- | --- | --- |
| 1 | person | `dynamic_worker` |
| 2 | cargo | 화물 상자 (`cargo.py`, 상자마다 모델) |
| 3 | sign | `sign_*` 5종 |

- 지게차·다른 로봇·랙·벽은 라벨이 없다. 상자는 안 나오지만 앞에 있으면 뒤 물체를 가린다.
- 상자는 **모델 단위로 하나**다. 링크·visual에 따로 라벨을 붙여도 한 모델이면 합쳐진다 (10/11 시험).
- 가려진 부분은 상자에서 빠진다(보이는 픽셀 기준). 완전히 가려지면 상자가 없다.
- 확인 (10/11, 시험 월드): 표지판·화물 상자가 각각 나오고, 라벨 없는 물체는 나오지 않는다. 뒤 상자가 앞 상자에 일부 가려지면 보이는 부분만 상자가 된다.

센서 시스템(`Sensors`, `Imu`)은 월드에 있다. `Sensors`는 LiDAR·카메라를 렌더링(ogre2)으로 만든다.
화면(DISPLAY)이 없는 dev 컨테이너에서도 ogre2가 headless 모드로 바꿔 동작한다
(`Unable to open display: . Trying to run in headless mode.` 경고는 무시해도 된다).

카메라는 CPU로 렌더링해서 시뮬레이션이 실시간보다 느려진다. 로봇 1대 기준 실시간 대비 약 0.77배
(실제 시간으로 재면 LiDAR 7.6 Hz, IMU 79 Hz, 카메라 23 Hz). 시뮬레이션 시간 기준 주기는 설정값 그대로이므로
노드는 `use_sim_time:=true`로 쓴다. 로봇 대수를 늘릴 때는 카메라 해상도·주기 조정이 필요할 수 있다.

## 파라미터

`config/` 의 YAML로 관리한다. 파라미터 변경에 재빌드가 필요 없어야 한다.

- `config/dynamic_obstacles.yaml`: 동적 장애물 좌표·경로·패턴·속도 범위 (위 표 참고)
- `config/cargo_boxes.yaml`: 화물 상자 id·크기·위치 (위 "화물 상자" 참고)

launch 파일과 인자:

| launch | 인자 (기본값) | 설명 |
| --- | --- | --- |
| `world.launch.py` | `world` (`warehouse.sdf`), `gui` (`true`), `verbosity` (`2`) | 월드만. `gui:=false`면 서버만 |
| `spawn_robot.launch.py` | `namespace` (`amr_01`), `x` (`0.0`), `y` (`1.5`), `z` (`0.01`), `yaw` (`0.0`), `payload_mass` (`0.0`), `gt_rate` (`100`), `publish_odom_tf` (`true`), `enable_camera` (`true`), `label_camera` (`false`) | 실행 중인 Gazebo에 로봇 1대 스폰 + 브리지 |
| `sim.launch.py` | `gui` (`true`), `rviz` (`true`), `namespace`, `x`, `y`, `yaw`, `payload_mass` (`0.0`), `publish_odom_tf` (`true`), `enable_camera` (`true`), `label_camera` (`false`), `dynamic_obstacles` (`false`), `obstacle_seed` (`0`), `cargo` (`false`) | 월드 + 로봇 1대 + RViz (+ 동적 장애물, 화물 상자) |
| `dynamic_obstacles.launch.py` | `config` (`config/dynamic_obstacles.yaml`), `only` (전부, 쉼표로 id), `seed` (`0`), `use_sim_time` (`true`) | 실행 중인 Gazebo에 동적 장애물 스폰 + 브리지 + 이동 노드 |
| `cargo_boxes.launch.py` | `config` (`config/cargo_boxes.yaml`) | 실행 중인 Gazebo에 화물 상자 스폰 (LiDAR 높이에 걸리면 실패) |

## 실행

```bash
# 컨테이너 안에서
colcon build --packages-select amr_description amr_gazebo && source install/setup.bash
ros2 launch amr_gazebo sim.launch.py                          # 월드 + 로봇 + RViz (gui 프로필의 noVNC 화면에서)
ros2 launch amr_gazebo sim.launch.py gui:=false rviz:=false   # 서버만 (dev 컨테이너)
ros2 launch amr_gazebo world.launch.py                        # 월드만
ros2 launch amr_gazebo sim.launch.py dynamic_obstacles:=true  # + 동적 장애물 5개
ros2 launch amr_gazebo dynamic_obstacles.launch.py only:=forklift_1 seed:=42  # 실행 중인 sim에 1개만
ros2 launch amr_gazebo sim.launch.py cargo:=true             # + 화물 상자 24개 (YOLO 인식용)
```

## 테스트

```bash
colcon test --packages-select amr_gazebo
```

`test/test_obstacle_motion.py`: 실제 `config/dynamic_obstacles.yaml`로 장애물을 굴려 명세(5개 이상, 0.3~1.5 m/s, 직선·곡선·무작위),
속도 범위, 직선 왕복, 곡선이 경유점을 지나는지, 무작위가 영역 안에 머무는지, seed 재현을 확인한다.

`test/test_cargo.py`: 실제 `config/cargo_boxes.yaml`과 월드 SDF로 상자가 LiDAR 스캔 높이에 안 걸리는지, 카메라 높이 범위(1.5 m 이하),
바닥 상자가 랙·기둥과 겹치지 않고 랙 면에 붙어 있는지, 도크 상자가 플랫폼 위에 있는지, 좁은 통로·동적 장애물 경로를 피하는지, SDF 링크 위치를 확인한다.
