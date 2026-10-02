# amr_gazebo

- 담당: 지수
- 빌드 타입: ament_cmake (자원 전용, C++ 코드 없음)
- 책임: 60x40m 물류센터 월드, 동적 장애물, 센서 노이즈 모델

## 제공 인터페이스

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Publish | `/clock` | `rosgraph_msgs/msg/Clock` | 시뮬레이션 시간. 다른 노드는 `use_sim_time:=true`로 쓴다 |
| Publish | `/<ns>/odom` | `nav_msgs/msg/Odometry` | DiffDrive 휠 오도메트리, 50 Hz (`spawn_robot.launch.py`) |
| Publish | `/<ns>/joint_states` | `sensor_msgs/msg/JointState` | 바퀴 조인트 각도 |
| Publish | `/<ns>/ground_truth/odom` | `nav_msgs/msg/Odometry` | **GT(진짜 위치·속도)**, 기본 100 Hz (`gt_rate`). `frame_id: world`(Gazebo 월드 원점 = 창고 중심), `child_frame_id: <ns>/base_footprint`. TF로는 내보내지 않음. **평가 전용**(RMSE, 제동 거리) — 로봇 판단에 쓰지 않는다. 아래 "GT 사용 시 주의" 참고 |
| Publish | `/<ns>/scan_raw` | `sensor_msgs/msg/LaserScan` | 2D LiDAR 원본 (노이즈 포함), 10 Hz, frame `<ns>/lidar_link` |
| Publish | `/<ns>/imu/data_raw` | `sensor_msgs/msg/Imu` | IMU 원본 (노이즈·바이어스 포함, 공분산 비어 있음), 100 Hz, frame `<ns>/imu_link` |
| Publish | `/<ns>/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB (`rgb8`, 640×480), 30 Hz, frame `<ns>/camera_optical_frame` |
| Publish | `/<ns>/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Depth (`32FC1`, m, 0.3~10 m), 30 Hz. RGB와 픽셀 1:1 대응 |
| Publish | `/<ns>/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB·Depth 공통 내부 파라미터 (fx = fy = 337.2, cx 320, cy 240) |
| Publish | `/tf` | `tf2_msgs/msg/TFMessage` | `<ns>/odom` → `<ns>/base_footprint` (`publish_odom_tf:=true`) |
| Subscribe | `/<ns>/cmd_vel` | `geometry_msgs/msg/Twist` | 속도 명령. 2.0 m/s, 1.0 m/s², 1.5 rad/s로 제한 |
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
| **통로 분리 랙 (2주차)** | `rack_lane_divider` | (0, −0.2) | A–B 통로를 남북으로 나눠 아래 5.0 m 좁은 통로를 만든다 |
| 열 사이 통로 | | A–B(북쪽 잔여): y 0.4~4.4, B–C: y −8.4~−2.6 | 폭 4.0 m / 5.8 m |
| 교차 통로 | | 같은 열 랙 사이 | 폭 2.5 m |
| **좁은 통로 ①** | `rack_b3`–`rack_b4` 사이 | x −5.0~−4.4, y −2.6~−1.4 | **폭 0.6 m, 길이 1.2 m** (로봇 폭 0.4 m + 0.2 m, Nav2 KPI용) |
| **좁은 통로 ② (2주차)** | `rack_lane_divider`–B열 사이 | x −2.5~2.5, y −1.4~−0.8 | **폭 0.6 m, 길이 5.0 m** (①보다 긴 구간에서 지속 주행 테스트용) |
| 기둥 | `pillar_1`~`pillar_6` | (±11.25, 7.0), (±15, 1.5), (±7.5, −5.5) | 0.6 × 0.6 × 6 m, 통로 가운데 |
| 충전 스테이션 | `charger_c1`~`c3` | 남쪽 벽 (−24, −20), (−21, −20), (−18, −20) | 본체 0.3 m + 도킹 패드(초록 표시), +y 방향 |
| 충전 구역 | 바닥 표시 (파랑) | x −26~−16, y −20~−16 | |
| 대기 구역 | 바닥 표시 (회색) | x 16~26, y −20~−14 | |

재사용 모델은 `models/`에 있고 월드에서 `model://<이름>`으로 불러온다.
경로는 `package.xml`의 `gazebo_model_path` export를 통해 `ros_gz_sim`이 자동으로 등록한다.

### 좁은 통로 ② 추가 검증 (2주차)

기존 통로 ①(0.6×1.2 m)은 랙 깊이만큼만 좁아 로봇이 진입각을 바꿔가며 여러 미터를
좁은 폭으로 유지 주행하는 상황을 테스트하기 어려웠다. 기존 랙 위치는 하나도 옮기지
않고 `rack_lane_divider` 1개만 A–B 통로에 추가해 0.6×5.0 m 통로를 만들었다.

- **남은 메인 통로 폭**: `rack_lane_divider`가 걸쳐 있는 x −2.5~2.5 구간에서만 A–B
  통로 북쪽 잔여 폭이 5.8 m → **4.0 m**로 줄어든다(그 밖의 x 구간은 그대로 5.8 m).
  로봇 2대가 엇갈려 지나가는 데 필요한 최소 폭(1.5 m 이상)을 여유 있게 만족한다.
- **입출고·충전소 경로**: `rack_lane_divider`(x −2.5~2.5, y −1.4~−0.8)는 입출고 도크
  (y 8.5~18.5)나 충전 스테이션(남쪽 벽, y=−20 부근)과 겹치지 않아 그쪽 경로를 막지 않는다.

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

아직 없는 것 (지수 담당): 실제 스폰·이동 동작, 충돌체.

센서 시스템(`Sensors`, `Imu`)은 월드에 있다. `Sensors`는 LiDAR·카메라를 렌더링(ogre2)으로 만든다.
화면(DISPLAY)이 없는 dev 컨테이너에서도 ogre2가 headless 모드로 바꿔 동작한다
(`Unable to open display: . Trying to run in headless mode.` 경고는 무시해도 된다).

카메라는 CPU로 렌더링해서 시뮬레이션이 실시간보다 느려진다. 로봇 1대 기준 실시간 대비 약 0.77배
(실제 시간으로 재면 LiDAR 7.6 Hz, IMU 79 Hz, 카메라 23 Hz). 시뮬레이션 시간 기준 주기는 설정값 그대로이므로
노드는 `use_sim_time:=true`로 쓴다. 로봇 대수를 늘릴 때는 카메라 해상도·주기 조정이 필요할 수 있다.

## 파라미터

`config/` 의 YAML로 관리한다. 파라미터 변경에 재빌드가 필요 없어야 한다.

- `config/dynamic_obstacles.yaml`: 동적 장애물 좌표·경로·패턴·속도 범위 (위 표 참고)

launch 파일과 인자:

| launch | 인자 (기본값) | 설명 |
| --- | --- | --- |
| `world.launch.py` | `world` (`warehouse.sdf`), `gui` (`true`), `verbosity` (`2`) | 월드만. `gui:=false`면 서버만 |
| `spawn_robot.launch.py` | `namespace` (`amr_01`), `x` (`0.0`), `y` (`1.5`), `z` (`0.01`), `yaw` (`0.0`), `payload_mass` (`0.0`), `gt_rate` (`100`), `publish_odom_tf` (`true`) | 실행 중인 Gazebo에 로봇 1대 스폰 + 브리지 |
| `sim.launch.py` | `gui` (`true`), `rviz` (`true`), `namespace`, `x`, `y`, `yaw` | 월드 + 로봇 1대 + RViz |

## 실행

```bash
# 컨테이너 안에서
colcon build --packages-select amr_description amr_gazebo && source install/setup.bash
ros2 launch amr_gazebo sim.launch.py                          # 월드 + 로봇 + RViz (gui 프로필의 noVNC 화면에서)
ros2 launch amr_gazebo sim.launch.py gui:=false rviz:=false   # 서버만 (dev 컨테이너)
ros2 launch amr_gazebo world.launch.py                        # 월드만
```

## 테스트

```bash
colcon test --packages-select amr_gazebo
```
