# amr_docking

- 담당: 지수
- 빌드 타입: ament_python
- 책임: ArUco 마커 기반 정밀 도킹 (Dock.action 서버). 3회 재시도는 BT(`amr_bt` `DockWithRetry`)가 한다

## 제공 인터페이스

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Action | `docking/dock` | `amr_msgs/action/Dock` | 스테이션 하나에 한 번 도킹. 결과에 성공·오류 코드(900번대)·최종 오차 |
| Subscribe | `camera/color/image_raw`, `camera/camera_info` | `sensor_msgs/msg/Image`, `CameraInfo` | 도킹 중에만 처리 (sim `enable_camera:=true` 필요) |
| Publish | `cmd_vel_nav` (파라미터 `cmd_vel_topic`) | `geometry_msgs/msg/Twist` | Nav2 컨트롤러와 같은 자리. 구역·안전 노드를 그대로 거친다 |
| TF | 카메라 광학 프레임 → `<ns>/base_footprint` | | URDF 고정 변환 |

모두 네임스페이스 안 상대 이름이다. 전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

## 마커판 (`config/dock_stations.yaml`, `amr_docking/markers.py`)

- 스테이션마다 판 하나에 ArUco 마커 두 장 (DICT_4X4_100). 판 k → 마커 2k(왼쪽)·2k+1(오른쪽, 로봇이 볼 때)
- 도킹 자세는 `amr_fleet_manager` `config/stations.yaml`과 같은 좌표다 (그 파일: "도킹 자세는 ArUco 마커 위치에 맞춰 지수가 확정"). 테스트가 두 파일이 같은지 본다
- 판은 도킹 자세 앞 `standoff`에, 면에서 5 mm 앞에 붙는다

| 스테이션 | 판을 붙인 면 | standoff | 판 (마커 한 변 / 간격 / 중심 높이) |
| --- | --- | --- | --- |
| 랙 21곳 | 랙 면 | 0.795 m | small: 0.14 / 0.32 / 0.22 m |
| 충전소 3곳 | 충전기 본체 앞면 | 0.695 m | small |
| 도크 4곳 | 도크 플랫폼 앞면 | 1.495 m | large: 0.20 / 0.80 / 0.19 m |

- 판(흰 여백 포함)은 바닥 0.06~0.32 m로, 2D LiDAR 스캔 높이(0.38 m ± 0.05)를 피한다. 지도 v1·AMCL에 영향이 없다. 충돌도 없다(시각만)
- 도크는 1.5 m에서 보기 때문에 방향 오차가 옆 오차로 크게 번진다 (1.5 m × 1° ≈ 2.6 cm). 방향 정확도는 마커 간격·크기에 비례해서 큰 판을 쓴다
- Gazebo에는 텍스처 대신 검은 칸을 얇은 상자로 그린다 (텍스처는 면마다 방향이 달라 마커가 뒤집혀 보일 수 있다)

## 도킹 과정

1. **인식** (`detector.py`): 영상에서 마커 두 장을 찾고, 꼭짓점 8개로 판 하나의 PnP(SQPNP)를 풀어 두 마커 중심 자리를 낸다. 마커마다 따로 푸는 것보다 비스듬히 볼 때 방향이 정확하다 (Gazebo, 랙 앞 6° 돌아 있을 때 5.7° vs 4.4°)
2. **오차** (`geometry.py`): 두 중심을 이은 선이 판 면의 방향이다. 도킹 자세 = 판 중심에서 판 바깥으로 `standoff`. 로봇 기준 남은 거리(along)·옆(lateral)·방향(heading)
3. **제어** (`controller.py`, 차동 구동이라 옆으로 못 가서 단계를 나눔)

| 단계 | 하는 일 | 끝 |
| --- | --- | --- |
| SEARCH | 판이 보일 때까지 2 s 기다리고, 안 보이면 제자리에서 ±0.6 rad 돌며 찾는다. 그래도 없으면 천천히 0.3 m 뒤로 빠져 한 번 더 돈다 (목표를 지나쳐 판이 카메라 최소 거리 0.3 m·시야 밖인 경우) | 보이면 다음 / 두 번 다 없으면 `FAILED_TO_DETECT_DOCK`(904) |
| BACKOFF | 목표에 너무 가깝거나(0.25 m 안) 지나쳤는데 옆 오차가 크면 뒤로 (0.4 m + 옆 오차 × 2, 최대 0.8 m) | 접근 거리를 벌면 APPROACH |
| APPROACH | 진행선을 따라 들어간다 (옆 오차 → 방향 보정, Stanley 방식). 지나쳤으면 뒤로. 속도 0.12 m/s, 끝에서 줄이고 마지막 5 cm는 옆 보정을 줄여 크게 꺾지 않는다 | 남은 거리 3 mm 안 → ROTATE |
| ROTATE | 제자리 회전 | 방향 0.25° 안 |
| VERIFY | 멈추고 0.5 s 기다린 뒤 15번 잰 평균 | 옆 오차가 5 mm보다 크면 BACKOFF부터 다시 접근 (3번까지). 방향만 0.5°보다 크면 ROTATE 다시 (3번까지). 2 cm·1° 안이면 성공, 아니면 처음부터 (2번까지) 그래도 안 되면 `FAILED_TO_CONTROL`(905) |

- 접근 중 판이 3 s 넘게 안 보이면 904, 한 번 시도가 90 s를 넘으면 905, 모르는 스테이션은 `DOCK_NOT_IN_DB`(901)
- 측정은 영상 시각(`header.stamp`) 기준 0.3 s보다 오래되면 안 쓴다 (안 보이는 것으로 보고 멈춘다). Gazebo에서 영상 지연은 약 32 ms
- 다시 접근할지는 멈춘 채 잰 평균(VERIFY)으로만 정한다. 1.5 m(도크)에서는 움직이는 동안 옆 오차 측정이 ±2~3 cm 흔들려서, 움직이며 잰 값 하나로 정하면 필요 없이 뒤로 빠지기를 되풀이했다
- 목표 하나를 받으면 수락하는 순간 잡아서, 동시에 온 두 번째 목표는 거부한다

## 파라미터

`config/docking.yaml`:

| 이름 | 기본값 | 설명 |
| --- | --- | --- |
| `stations_file` | `""` (= `config/dock_stations.yaml`) | 스테이션·판 |
| `base_frame` | `""` (= `<ns>/base_footprint`) | |
| `image_topic` / `camera_info_topic` | `camera/color/image_raw` / `camera/camera_info` | |
| `cmd_vel_topic` | `cmd_vel_nav` | 단독 시험은 `cmd_vel` |
| `control_hz` | `20.0` | |
| `max_age_s` | `0.3` | 측정 유효 시간 |
| `v_max` / `w_max` | `0.12` / `0.4` | m/s, rad/s |
| `position_tol` / `yaw_tol_deg` | `0.02` / `1.0` | 성공 판정 (명세 KPI 2 cm·1도) |
| `verify_samples` | `15` | |
| `timeout_s` | `90.0` | 한 번 시도 (시뮬레이션 시간) |
| `log_period_s` | `0.0` | 0보다 크면 단계·오차를 이 주기로 로그 (조정용) |

## 실행

```bash
# Gazebo에 마커판 띄우기 (sim 실행 중)
ros2 launch amr_docking aruco_markers.launch.py
# 도킹 서버 (로봇마다)
ros2 launch amr_docking docking.launch.py namespace:=amr_01
ros2 launch amr_docking docking.launch.py namespace:=amr_01 cmd_vel_topic:=cmd_vel   # Nav2·구역·안전 체인 없이
# 도킹 한 번
ros2 action send_goal /amr_01/docking/dock amr_msgs/action/Dock "{station_id: rack_a4}" --feedback
```

BT에서는 `ros2 launch amr_bt amr_bt.launch.py use_docking:=true`.

## KPI 측정 (Gazebo)

[experiments/docking_kpi](../../../experiments/docking_kpi/README.md): 로봇을 스테이션 근처에 흩어 놓고(Nav2 도착 오차 흉내) 도킹한 뒤 Gazebo GT로 오차를 잰다.

10/12 결과: 랙·충전소·도크, 보통 시작 24회 + Nav2 허용치 끝(반경 25 cm·±14°) 시작 24회 **모두 성공, 모두 KPI(2 cm·1°) 안**. 위치 오차 중앙값 0.32~0.35 cm(최대 1.31 cm), 방향 0.08~0.09°(최대 0.38°).

## 테스트

```bash
colcon test --packages-select amr_docking
```

- `test_geometry`: 마커 중심 → 도킹 오차 (위치·방향을 정확히 되찾는지)
- `test_controller`: 차동 구동 운동학 + 측정 노이즈로 닫힌 고리. 여러 시작 자세에서 실제 오차 1 cm·0.5° 안, 무작위 40회, 못 찾음·잃음·시간 초과, 회전 관성 보정, 지나쳐서 안 보일 때 뒤로 빠져 찾기, 움직이는 동안 옆 오차가 흔들려도 되풀이 안 함
- `test_detector`: 합성 카메라 영상(Gazebo와 같은 칸 배치)으로 인식 → 오차. 랙·충전소·도크 거리에서 목표 근처 8 mm·0.3° 안
- `test_markers`: 스테이션 목록, 마커 id 겹침 없음, LiDAR 높이 회피, 판 위치(랙·충전기·플랫폼 면), `stations.yaml`과 같은 좌표, SDF
