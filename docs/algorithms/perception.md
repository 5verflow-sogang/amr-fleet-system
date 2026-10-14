# 인지 알고리즘 (perception)

> 상태: 초안 (3주차). 담당: 연우. 구현하면서 절을 채운다.

## 1. LiDAR 동적 장애물 분류

원 명세 7장 "LiDAR 데이터에서 동적 장애물을 분류하는 알고리즘을 구현한다"에 해당한다.
스캔을 군집으로 나누고, 정적 구조물(벽·랙·기둥)을 걸러 낸 나머지를 KF 추적에 넘긴 뒤, 움직임으로 동적 여부를 확정한다.

```
scan (amr_odometry scan_filter, lidar_link, REP 117)
  → 군집화 (adaptive breakpoint)                       amr_perception_core/clustering.py  [구현]
  → R1 크기 규칙: 큰 군집(벽·랙 면) 제외                 clustering.is_obstacle_candidate   [구현]
  → 다리 묶기: 가까운 작은 군집 두 개를 한 사람으로       clustering.merge_leg_pairs         [구현]
  → 고정 좌표계로 변환 (lidar_link → odom)               tracking_node + transform_points_2d [구현]
  → R2 정적 지도 대조: 지도 막힌 칸 위의 빔을 군집화 전에 뺌 amr_perception_core/static_map.py [구현, 선택]
  → KF 다중 추적 → R3 속력 + R4 빈 공간 침입으로 is_dynamic  tracker.py + free_space.py        [구현]
  ← 카메라 연관: YOLO 검출(perception/objects)로 트랙 클래스  association.py (1-6절)          [구현]
  → perception/obstacles (TrackedObstacleArray), perception/collision_risks (3절 TTC)
```

### 1-1. 군집화

스캔의 유효 빔을 각도 순서대로 보며, 이웃한 두 점 사이 거리가 임계값 $D_{max}$를 넘으면 군집을 끊는다.
임계값은 거리에 비례해 늘리는 adaptive breakpoint(Borges & Aldon, 2004)다.

```math
D_{max} = r_{prev} \cdot \frac{\sin \Delta\varphi}{\sin(\lambda - \Delta\varphi)} + 3\sigma_r
```

멀수록 이웃 빔 사이 간격이 벌어진다(10 m에서 0.5°는 약 9 cm). 고정 임계값이면 먼 물체가 쪼개지거나 가까운 물체끼리 붙는다.

| 파라미터 | 값 | 근거 |
| --- | --- | --- |
| `breakpoint_lambda_deg` (λ) | 10° | 빔과 표면이 이루는 가장 얕은 각도. 논문 권장 범위(10~15°)의 하한으로 비스듬한 벽도 이어 붙인다 |
| `range_noise_std` (σ_r) | 0.03 m | `amr_description/config/sensors.yaml` LiDAR 가우시안 잡음 |
| `max_gap_beams` | 2 | 물체 안에서 아웃라이어 필터가 지운 빔 몇 개는 건너뛴다. 빠진 각도만큼 $D_{max}$를 늘린다 |
| `min_points` | 3 | 이보다 작은 군집은 잡음. 12 m 앞 0.5 m 물체에도 빔이 5개 맞는다 |

- 무효 빔(NaN, ±inf)은 점으로 쓰지 않는다. `scan_filter`가 REP 117로 표시한다.
- **튄 점 하나 건너뛰기** (`bridge_single_outlier`): 점 i와 i+1이 끊겨도 한 점 건너 이웃(i−1과 i+1, 또는 i와 i+2)이 한 칸 한계 안이면 잇는다.
  위 식의 3σ_r은 점 **하나**의 잡음인데 실제로 비교하는 것은 잡음 낀 거리 **둘의 차이**(σ√2)다. 그래서 기하 항이 작은 가까운 벽에서
  이웃 점 쌍이 약 1% 확률로 끊겨, Gazebo에서 벽 0.6 m 앞에 서 있기만 해도 벽이 프레임마다 다른 곳에서 조각났다(프레임당 1.4번).
  조각 중심이 벽을 따라 뛰어 동적 트랙이 됐다. 고친 뒤 0번(같은 스캔 30장). 한계를 두 칸 각도로 새로 구하면 기하 항이 두 배가 되어
  벽 0.2 m 앞 사람이 벽에 붙으므로 한 칸 한계를 쓴다(단위 테스트로 확인).
- 360° 스캔이면 마지막 빔과 첫 빔도 이웃으로 보고, 로봇 정후방(±180°)에 걸친 물체를 한 군집으로 합친다.

**군집 특징값과 알려진 치우침** (단위 테스트로 확인)

| 값 | 정의 | 치우침 |
| --- | --- | --- |
| `centroid` | 점들의 평균 | LiDAR는 센서 쪽 표면만 봐서 실제 중심보다 **센서 쪽으로** 치우친다 (3 m 앞 반경 0.25 m 원기둥이면 약 0.2 m). KF 관측 잡음 σ 0.05 m에 일부만 반영되어 있다 |
| `radius` | 중심에서 가장 먼 점까지 | 중심이 치우쳐 실제 반경보다 **조금 클 수 있다** (0.3 m 원기둥에서 0.33 m). 충돌 반경으로는 보수적인 쪽 |
| `extent` | 두 점 사이 최대 거리 | 빔이 끝점에 정확히 맞지 않아 실제보다 **빔 간격만큼 짧다** (6 m 벽에서 5.93 m) |

### 1-2. 정적/동적 구분 규칙

| 규칙 | 내용 | 필요 조건 | 상태 |
| --- | --- | --- | --- |
| **R1 크기** | `extent` > 2.0 m면 구조물(벽, 랙 긴 면)로 보고 제외. 점 < 3개는 잡음 | 없음 | 구현 |
| **R2 정적 지도 대조** | 지도 막힌 칸(부풀림 0.15 m) 위의 **빔**을 군집화 전에 뺀다 | 정적 지도 + `map → odom` | 구현 (`static_map.py`), 기본 꺼짐 (1-2-1) |
| **R3 움직임** | 추적 속력 > 0.25 m/s가 5프레임 연속 | KF 추적 | 구현 (`tracker.py`) |
| **R4 빈 공간 침입** | 군집 점이 "최근 빔이 지나간(비어 있던) 칸"에 2개 이상 들어온 프레임이 3번 이상 | odom만 | 구현 (`free_space.py`) |

**결합 순서:** R1 → (R2) → 추적 → R3 **그리고** R4이면 `is_dynamic = true`, 이후 유지(멈춘 사람도 동적 장애물).
R1·R2를 통과한 군집만 KF 추적에 넘기고(정적 구조물이 트랙을 차지하지 않게), 동적 여부는 R3·R4로 확정한다.

**R2가 필요한 근거 (지도 v0 실험).** `warehouse_v0.pgm`에서 가상 스캔(0.5°, 잡음 σ 0.03 m)을 만들어 동적 장애물이 **하나도 없는** 상태로 돌렸다.

| 로봇 위치 (map) | 군집 | R1 통과 후보 | 후보 크기 [m] |
| --- | --- | --- | --- |
| (0, 0) 출발점, A–B 통로 | 17 | 7 | 0.36, 0.44, 0.5, 0.71, 0.74, 1.5, 1.72 |
| (−10, 11.5) 메인 통로 | 12 | 3 | 0.42, 0.8, 1.93 |
| (5, −7) B–C 통로 | 18 | 7 | 0.4, 0.53, 0.61, 0.61, 0.68, 0.93, 1.66 |

모두 정적 물체다. 기둥(0.6 m), 랙의 짧은 면(1.2 m)이나 모서리 일부, 도크·충전기처럼 사람·지게차와 크기가 겹치는 구조물은
**크기만으로는 거를 수 없다.** R3만 쓰면 정적 판정은 맞게 나오지만, 매 스캔 수 개의 가짜 트랙이 생겨 연관 오류와 계산량이 늘어난다.

**R2 세부 (구현)**

| 파라미터 | 값 | 근거 |
| --- | --- | --- |
| `map_inflation` | 0.15 m (3칸) | 지도 오차(물체별 중앙값 2.5 cm, 최대 12.5 cm, amr_mapping README) + 위치 추정 오차(정지 3 cm, 회전 8 cm KPI). 이보다 벽에 가까이 붙은 사람은 추적에서 빠진다 |
| `map_align_max_shift` | 0.3 m | 대조 전에 스캔을 ±0.3 m 밀어 지도와 가장 잘 맞는 평행이동을 쓴다 (위치추정 어긋남 보정) |

- 군집이 아니라 **빔** 단위로 뺀다. 벽 앞에 선 사람은 벽 빔만 빠지고 사람 빔은 남는다.
- `map → odom`은 가장 최근 값, `odom → lidar`는 스캔 촬영 시각 값을 합성한다. 위치추정의 `map → odom`이 스캔보다 늦게 나와 촬영 시각 값을 자주 못 구했다.
- 지도에 없던 물체(새로 놓인 팔레트 등)는 남는다. Gazebo에 기둥을 새로 놓아 트랙 1개로 잡히는 것을 확인했다.
- **한계:** 위치추정이 지도와 어긋나면(Gazebo에서 회전 뒤 0.3 m·6° 관측) 벽 빔이 부풀림 밖으로 새어 나와 효과가 없거나 나빠진다.
  평행이동 보정은 회전 오차를 못 고친다. 그래서 기본은 끄고(`use_static_map: false`) R4로 동적 판정을 지킨다.

**R4 세부 (빈 공간 침입, `free_space.py`)**

로봇이 움직이면 LiDAR(회전 중심 앞 0.15 m)의 시점이 바뀌어, 가까운 랙이 먼 벽을 가리는 경계와 보이는 벽 조각이 달라진다.
조각 중심이 미끄러져 R3 속력 조건을 넘었다. 진짜 사람·지게차는 **방금 비어 있던 공간으로 들어오고**, 벽 조각은 빔이 맞는 칸에만 있다.

- odom 격자(0.1 m 칸, 로봇 주변 30 m)에 칸마다 "빔이 지나간 마지막 시각"과 "반사점이 떨어진 마지막 시각"을 적는다.
- 군집 점이 놓인 칸이 3 s 안에 비어 있었고 3 s 동안 맞은 적이 없으면 증거다 (이번 스캔을 넣기 전 상태로 판정).
- 빔 끝 0.2 m 앞과, 이번 스캔 반사점에서 0.2 m 안의 칸은 비어 있다고 적지 않는다 (거리 잡음, 물체 옆을 스치는 빔).
- 12 m보다 먼 구간은 적지 않는다(빔 간격 > 칸). 그보다 먼 이동체는 가까워진 뒤 동적이 된다.
- odom만 쓰므로 위치추정 오차의 영향을 받지 않는다. 스캔당 약 6 ms.

### 1-2-1. 가짜 동적 트랙 측정 (녹화 재생, 10/10)

로봇이 회전·직진·후진·곡선 주행을 섞은 44초(시뮬레이션) 녹화를 설정마다 같은 속도로 재생했다.
GT 장애물(사람 1 m, 지게차·로봇 2.5 m 안)과 짝이 없는 동적 트랙을 가짜로 셌다.

| 설정 | 가짜 동적 트랙 / 프레임 | 가장 가까운 가짜 | 처리 시간 중앙 (최대) |
| --- | --- | --- | --- |
| R1 + R3 (이전) | 10.07 | 0.1 m | 24 ms (36) |
| R1 + R2 + R3 | 10.76 | 0.7 m | 23 ms (36) |
| R1 + R3 + R4 (스치는 빔 처리 전) | 2.64 | 0.3 m | 30 ms (43) |
| R1 + R2 + R3 + R4 (스치는 빔 처리 전) | 2.08 | 1.7 m | 26 ms (50) |
| R1 + R3 + R4 (+ 스치는 빔 처리) | 2.57 | 0.3 m | 29 ms (38) |
| R1 + R2 + R3 + R4 (+ 스치는 빔 처리) | 1.98 | 1.7 m | 27 ms (40) |
| 위 R1 + R3 + R4에서 `hit_quiet` 1 → 3 s | 1.66 | 0.3 m | |
| 위 R1 + R3 + R4에서 증거 2 → 3프레임 | 1.63 | 0.3 m | |
| **R1 + R3 + R4, `hit_quiet` 3 s + 증거 3프레임 (기본)** | **1.13 (−89%)** | 0.3 m | 28 ms (39) |

- 녹화 당시 위치추정이 어긋나 있어 R2는 효과가 없었다(위 한계).
- 처리 시간은 Gazebo 없이 잰 값이다. Gazebo와 함께 돌리면 CPU를 나눠 써 5초 평균 170 ms까지 늘었다.
- `hit_quiet` 3 s: 빔이 맞았다 놓쳤다 하는 얇은 랙 다리가 1초 넘게 안 맞다가 다시 맞을 때 증거가 되던 것을 막는다.
- 남은 가짜는 대부분 반경 0.6~1.3 m·속력 1.6~3.5 m/s의 구조물 조각이다. 스치는 빔 처리는 거의 효과가 없었다(2.64 → 2.57).
- 이 녹화에는 12 m 안에 실제 이동체가 없어 검출 지연은 Gazebo에서 따로 쟀다 (기본 설정):

| 작업자가 정지한 로봇 쪽으로 | 동적 판정 | 비고 |
| --- | --- | --- |
| 1.0 m/s, 정면 5 m에서 | 걷기 시작 1.2 s 뒤 (4.0 m) | R4 넣기 전 1.1 s. 3.5 m에서 WARNING(당시 `ttc_warning` 3.0 s) |
| 0.5 m/s, 정면 4 m에서 | 1.1 s 뒤 (3.5 m) | TTC가 3 s를 넘어 1.5 m 근접 규칙으로 WARNING |

### 1-3. 군집 반경 전달 (결정: C안)

추적 노드 입력이 `geometry_msgs/PoseArray`라 군집 반경을 담을 자리가 없다. 지금은 `obstacle_radius` 0.3 m 고정이다.
`TrackedObstacle.radius`는 ORCA 충돌 반경으로 쓰이므로 실제 크기가 필요하다.

| 안 | 방법 | 장점 | 단점 |
| --- | --- | --- | --- |
| A | `amr_msgs`에 내부용 군집 메시지 추가 | 노드 분리 유지 | 인터페이스 PR과 검토 필요 (CONTRIBUTING 5절) |
| B | `Pose.position.z`에 반경을 넣음 | 변경 최소 | 필드 의미를 바꾸는 편법. 다른 사람이 읽으면 오해한다 |
| **C (채택)** | 군집화를 `tracking_node` 안에서 호출 (`scan` 직접 구독) | 메시지·노드 추가 없음. 반경을 그대로 트랙에 붙임 | 노드 하나가 군집화·추적을 함께 함. 알고리즘은 core에 분리되어 있어 테스트에는 영향 없음 |

**C안으로 정했다 (10/8).** `tracking_node`가 `scan`을 구독해 군집화하고(`measurement_source: scan`, 기본), 군집 반경을 트랙 반경으로 쓴다(지수 평활 `radius_smoothing` 0.3). `perception/obstacle_measurements`(PoseArray) 입력은 시험용 발행기용으로 남긴다(`measurement_source: poses`, `use_synthetic:=true`).

### 1-3-1. 다리 묶기

LiDAR 높이(약 0.2 m)에서 사람은 다리 두 개로 보인다(amr_gazebo README). 그대로 추적하면 사람 하나가 반경 0.1 m짜리 트랙 두 개가 되어, ORCA·안전 노드가 사람 크기를 작게 본다.

- 보이는 크기(`extent`)가 `leg_max_extent`(0.35 m) 이하인 군집 두 개의 중심이 `leg_merge_distance`(0.6 m, 어깨너비 수준) 안이면 한 군집으로 합친다. 가까운 쌍부터 묶고, 한 군집은 한 번만 묶는다.
- 합친 군집의 중심은 두 다리 사이(몸 중심에 가까움), 반경은 두 다리를 감싸는 크기가 된다.
- 0.9 m 떨어진 작은 물체 둘(사람 둘)은 묶지 않는다. 몸통 크기 물체 둘도 다리가 아니라 묶지 않는다(단위 테스트).
- **알려진 한계:** 다리 사이 틈으로 보이는 뒤쪽 벽 조각은 짧아서 R1을 통과한다(단위 테스트로 확인). R2(지도 대조)가 거른다.

### 1-3-2. 가려짐과 트랙 유지 시간 (Gazebo 녹화, 10/8)

로봇을 출발 구간에 세우고 `worker_2`가 A열 랙 옆 통로를 오가는 동안 스캔·TF·GT를 녹화해(`/tmp/bags/track1`, 시뮬레이션 약 7분) 추적을 다시 돌렸다.

| 항목 | 결과 |
|---|---|
| 작업자가 보이는 프레임 중 한 물체로 묶인 비율 | 99.7% (312프레임 중 311) |
| 다리 두 군집 간격 | 중앙 0.18 m, 최대 0.22 m (`leg_merge_distance` 0.6 m 안) |
| 작업자가 랙에 가려 군집이 없는 프레임 | 35% |
| 가려진 구간 길이 | 중앙 0.4 s, 90% 1.9 s, 최대 5.3 s |
| 위치 오차 / 속도 오차 (짝지어진 프레임) | 평균 4.6 cm / 0.06 m/s |

| `max_coast_time` | 0.5 s | 1.0 s | **2.0 s** | 2.5 s | 3.0 s |
|---|---|---|---|---|---|
| ID 전환 | 4 | 4 | **2** | 1 | 1 |

- **2.0 s로 정했다.** 가려진 구간의 90%를 덮는다. 더 길게 하면 ID 전환은 줄지만, 사라진 사람이 등속 예측 위치로 더 오래 남아(1.5 m/s면 2 s에 3 m) 불필요한 감속을 부를 수 있다. 그동안 KF 공분산이 커지므로 소비자(안전 노드, DWA)는 불확실한 트랙을 구분할 수 있다.
- 5 s 넘게 가려지면 어떤 값으로도 같은 ID를 이어 가지 못한다. 재식별(외형·카메라 연관)은 범위 밖이다.
- 0.6 m는 가정이다. Gazebo 작업자의 다리 군집 간격을 재서 확정한다.

### 1-4. 좌표계

KF는 고정 좌표계(odom 또는 map)의 측정을 전제한다. 로봇 기준(`lidar_link`) 좌표를 그대로 넣으면 로봇이 1 m/s로 갈 때
정지한 랙도 −1 m/s로 움직이는 것으로 추정된다. 군집 점은 스캔 촬영 시각의 `lidar_link → odom` TF로 옮긴다.

### 1-5. 남은 일

| 할 일 | 필요한 것 | 일정 |
| --- | --- | --- |
| 실제 스캔으로 군집화 확인 (벽·랙·기둥 분리, 후보 수) | 지수의 매핑 rosbag (`/amr_01/scan`, `/tf`, `/tf_static`) | 3차 전 |
| ~~군집화 노드 (또는 1-3 C안)~~ | C안으로 구현 (10/8) | 완료 |
| ~~다리 묶기 거리 확정, 작업자 추적 정확도(위치·속도 오차, ID 전환)~~ | Gazebo 녹화로 측정 (10/8, 1-3-2) | 완료 |
| 작업자가 가끔 트랙 두 개로 갈라짐 (카메라 연관 확인 중 Gazebo에서 관측, 10/10) | 녹화로 다리 묶기 실패 구간 분석 | P2 |
| ~~카메라 연관~~ | 구현·Gazebo 확인 (10/10, 1-6) | 완료 |
| ~~R2 구현과 오검출 감소 측정~~ | 구현·측정 (10/10, 1-2-1). 위치추정이 어긋나면 효과 없어 기본 끔 | 완료 |
| 남은 가짜 동적 트랙(프레임당 1.1개, 구조물 조각) 줄이기 | 녹화 재생으로 원인 분석 | P2 |
| R2 회전 보정 또는 위치추정 품질 확인 후 R2 다시 켜기 | 위치추정 오차 추가 측정 (민지·지수와 공유) | P2 |
| 중심 치우침(약 0.2 m) 보정 검토: 센서→중심 방향으로 보이는 폭의 절반만큼 밀기 | GT 대비 위치 오차 측정 (1-7 "센서 방향 편향") | 3차 |
| ~~지게차 크기 확인 → `max_obstacle_extent` 조정~~ | `models/dynamic_forklift` 충돌체 1.2 × 0.9 m(높이 약 2 m, 바닥 0.025 m부터라 LiDAR 높이에 걸린다). LiDAR는 많아야 두 면을 봐 `extent` ≤ 대각선 1.50 m < 2.0 m라 **R1을 통과한다. 조정 불필요** (여유 0.5 m). 다른 로봇 0.6 × 0.4 m는 0.72 m. 단 먼 거리(12 m에서 $D_{max}$ 약 0.7 m)에서 0.7 m 옆 기둥과 한 군집으로 붙으면 2 m를 넘어 그 프레임만 빠질 수 있다 (트랙 유지 2 s가 덮는다) | 완료 (10/12) |
| 동적 판정 지연·정확도 (0.3~1.5 m/s) | 평가 노드 구현 (1-7). Gazebo 측정 | 측정 예정 |

### 1-6. 카메라 연관 (LiDAR 트랙 ↔ YOLO 검출, 10/10)

LiDAR는 위치·속도가 정확하지만 무엇인지 모르고, 카메라(YOLO)는 무엇인지 알지만 시야(87°)가 좁고 프레임이 적다.
`perception/objects` 검출을 트랙에 짝지어 `class_name`·`confidence`·`source`를 채운다 (`amr_perception_core/association.py`).

| 단계 | 방법 | 이유 |
| --- | --- | --- |
| 좌표 맞추기 | 검출(map)을 **같은 촬영 시각의 map → odom**으로 트랙 좌표(odom)에 되돌린다. 공분산은 R C Rᵀ | 검출 노드가 쓴 map → odom을 그대로 되돌리므로 위치추정 오차가 상쇄된다 (1-2 R2에서 본 0.3 m 어긋남의 영향을 받지 않음) |
| 시각 맞추기 | 트랙을 검출 시각으로 등속 예측 (위치 p + vΔt, 공분산 P + Δt²·P_v) | 1 m/s 작업자면 0.2 s 차이로 0.2 m 어긋난다 |
| 짝짓기 | 마할라노비스 거리² ≤ 9.21 (2자유도 χ² 99%) **그리고** 1.0 m 이내, 가까운 쌍부터 1:1 | 추적기 연관과 같은 방식. 공분산이 커도 1 m 넘게는 짝짓지 않는다 |
| 클래스 붙이기 | 클래스별 근거(confidence 합)를 시정수 5 s로 줄이며 쌓아, 1.0 이상이면 붙이고 0.3 아래로 줄면 뗀다 | 한 번 오검출로 붙지 않고, 시야를 벗어나도 약 6 s 유지. Gazebo에서 YOLO가 0.3 FPS일 때 히스테리시스 없이는 검출 사이에 깜빡였다 |
| source | 최근 1 s 안에 짝지어졌으면 `LIDAR_CAMERA`, 아니면 `LIDAR` | 지금 카메라로 보이는지 구분 |
| 반경 | `person`이면 최소 0.25 m | LiDAR 높이에서 다리만 보여 사람 반경이 0.12 m 정도로 작게 잡혔다 (TTC·ORCA 반경) |

- 짝이 없는 검출(표지판 등)은 무시한다. 표지판 기둥은 LiDAR 점이 적어 트랙이 되지 않고, 카메라만으로 트랙을 만드는 것은 범위 밖이다.
- **Gazebo 확인** (작업자가 정지한 로봇 쪽으로 0.3 m/s, YOLO가 CPU를 나눠 0.3 FPS): 검출 10개 중 9개 연관.
  두 번째 검출(걷기 시작 4 s, 3.8 m)에 `person`이 붙고 끝까지 유지, 반경 0.25 m, `source`는 검출 직후 `LIDAR_CAMERA`.

### 1-7. 추적 정확도 평가 (GT 대비)

원 명세 점검 항목 "칼만 필터 기반 객체 추적이 구현되고, 예측 정확도가 적절한가?"와 1-5의 "동적 판정 지연·정확도"를
Gazebo 정답(GT)과 비교해 숫자로 답한다. 계산은 `amr_perception_core/tracking_eval.py`(단위 테스트), 기록은
`amr_tracking`의 `tracking_eval` 노드가 한다.

**좌표 맞추기.** 트랙은 `odom`, GT(`/dynamic_obstacles/<id>/ground_truth/odom`, 로봇 `ground_truth/odom`)는 `world` 기준이다.
트랙 프레임마다 같은 시각의 로봇 GT 자세(world → base)와 TF(odom → base)로 $T_{world,odom} = T_{world,base} \cdot T_{odom,base}^{-1}$을 구해
트랙의 위치·속도·공분산(R P Rᵀ)을 world로 옮긴다. 프레임마다 다시 구하므로 휠 오도메트리가 미끄러져도 상쇄된다.
장애물 GT 속도는 몸체 기준이라 GT yaw로 world 축으로 돌린다.

**짝짓기.** 프레임마다 확정 트랙과 GT 물체를 가장 가까운 쌍부터 1:1로 짝짓는다 (게이트 `gate` 1.0 m).
군집 중심이 센서 쪽 면으로 치우치므로(사람 약 0.2 m, 지게차는 면 절반 0.45 m까지) 마할라노비스가 아니라 거리로 짝짓는다.

| 지표 | 정의 |
| --- | --- |
| 범위 안 짝 비율 | 로봇 12 m(`sensor_range`) 안 GT 프레임 중 트랙과 짝지어진 비율. 랙 가려짐도 놓침으로 센다 |
| 위치 RMSE | 짝지어진 쌍의 트랙 위치 − GT 위치 거리의 RMSE (중앙·95%·최대도 기록) |
| 센서 방향 편향 | 위치 오차를 로봇 → 물체 방향에 투영한 평균. 음수면 센서 쪽 치우침 (1-1 `centroid`, 1-5 중심 보정 검토 근거) |
| 속도 RMSE | 속도 벡터 오차 \|v̂ − v\|의 RMSE. 속력 오차(부호 있는 평균)는 따로 기록 |
| 방향 오차 | GT 속력 ≥ 0.1 m/s이고 트랙 속력 ≥ 0.2 m/s일 때 속도 방향 차이 [°] |
| +1 s·+2 s 예측 오차 | 움직이는 GT에 대해 등속 예측 p̂ + v̂·h와 h초 뒤 실제 GT 위치의 거리. 비교 기준으로 "정지 가정"(p̂ 그대로) 오차도 함께 낸다. DWA·TTC가 쓰는 예측의 정확도다 |
| NEES | 위치 eᵀP⁻¹e (P: 메시지의 KF 공분산 xy 블록), 속도도 같은 방식. 일관된 필터면 평균 2(2자유도), 95%가 5.99 안. 군집 중심 치우침은 KF 잡음 모델에 없어 2보다 크게 나올 것으로 본다 |
| ID 전환 | GT 물체마다 짝지어진 트랙 ID가 바뀐 횟수 (놓친 프레임은 건너뛰고 직전 짝과 비교). 가려진 뒤 새 ID로 다시 잡혀도 1번 |
| 동적 판정 지연 | GT 물체가 "움직이며(≥ 0.1 m/s) 센서 범위 안"인 구간(1 s 이하 끊김은 잇는다)마다, 처음 짝지어진 시각(또는 구간 시작)부터 짝 트랙이 `is_dynamic`이 된 시각까지. 기록 시작 전부터 구간이던 것과, 구간 전부터 추적해 이미 동적이던 트랙은 뺀다. GT 평균 속력 <0.6 / 0.6~1.0 / ≥1.0 m/s로 나눈다 |
| 가짜 동적 트랙 | 프레임당, GT와 짝이 없고 게이트 안에 GT도 없는 동적 트랙 수 (1-2-1과 같은 뜻). 게이트 안에 GT가 있으면 "중복"(사람이 트랙 둘로 갈라짐)으로 따로 센다 |

**실행** (로봇을 세워 두고 장애물이 지나가게 한다. 로봇이 움직이면 가짜 동적 트랙도 함께 잴 수 있다):

```bash
# 1) 시뮬레이터 + 동적 장애물 (seed 고정으로 재현)
ros2 launch amr_gazebo sim.launch.py dynamic_obstacles:=true obstacle_seed:=1 x:=0.0 y:=10.5 gui:=false rviz:=false
# 2) 센서 필터 (/amr_01/scan) + 추적
ros2 launch amr_odometry sensor_filters.launch.py
ros2 launch amr_tracking tracking.launch.py
# 3) 평가 (300 s 기록 후 저장하고 끝난다. duration_s 0이면 Ctrl+C 때 저장)
ros2 run amr_tracking tracking_eval --ros-args -r __ns:=/amr_01 -p use_sim_time:=true \
  -p duration_s:=300.0 -p out_dir:=/tmp/amr_eval/tracking_main_aisle
```

- 결과: `frames.csv`(프레임별 짝·놓침·짝 없는 트랙), `episodes.csv`(동적 판정 구간), `summary.json`, 터미널에 요약 표.
- 장애물 id는 `amr_gazebo/config/dynamic_obstacles.yaml`에서 읽는다 (`-p obstacle_ids:=worker_1,forklift_1`로 좁힐 수 있다).
- 메인 통로(0, 10.5)에서는 `worker_1`(0.3~0.8 m/s)만 12 m 안을 지난다. 지게차(0.8~1.5 m/s)는 B–C 통로(예: `x:=-3.0 y:=-4.0`, `forklift_1`이 2.5 m 옆을 지난다)에서 따로 잰다.

**결과** (측정 예정)

| 장면 | 물체 | 범위 안 짝 비율 | 위치 RMSE [m] | 센서 방향 편향 [m] | 속도 RMSE [m/s] | 방향 오차 [°] | +1 s / +2 s 예측 RMSE [m] (정지 가정) | NEES 위치 평균 | ID 전환 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 메인 통로, 로봇 정지 | `worker_1` | (측정 예정) | (측정 예정) | (측정 예정) | (측정 예정) | (측정 예정) | (측정 예정) | (측정 예정) | (측정 예정) |
| B–C 통로, 로봇 정지 | `forklift_1` | (측정 예정) | (측정 예정) | (측정 예정) | (측정 예정) | (측정 예정) | (측정 예정) | (측정 예정) | (측정 예정) |

| 속력 | 동적 판정 / 관측 구간 | 지연 평균 (최대) [s] | 판정 시 거리 중앙 [m] |
| --- | --- | --- | --- |
| < 0.6 m/s | (측정 예정) | (측정 예정) | (측정 예정) |
| 0.6~1.0 m/s | (측정 예정) | (측정 예정) | (측정 예정) |
| ≥ 1.0 m/s | (측정 예정) | (측정 예정) | (측정 예정) |

| 가짜 동적 트랙 / 프레임 | GT 옆 중복 동적 / 프레임 |
| --- | --- |
| (측정 예정) | (측정 예정) |

## 2. 안전 노드

원 명세 7장 안전 시스템(긴급정지 0.3 m, Safety Zone Warning/Critical, E-Stop, 센서 고장 → 안전 정지·저속)에 해당한다.
`amr_safety`가 로봇의 최종 `cmd_vel`을 낸다(9/28 합의). 판정은 `amr_perception_core/safety.py`(`SafetyMonitor`),
진행 경로 판정은 `collision.path_hazard`, 게이트는 `amr_safety/gate.py`. 토픽·파라미터는 [amr_safety README](../../src/perception/amr_safety/README.md).

| 단계 | `speed_limit` | 조건 (하나라도) |
| --- | --- | --- |
| ESTOP | 0.0 | E-Stop 요청 (해제 요청 전까지) |
| CRITICAL | 0.0 | 지금 명령의 **진행 경로**에 LiDAR 점 / 움직이는 트랙 TTC ≤ 1.0 s / 스캔 0.5 s 끊김 |
| WARNING | 0.6 | 진행 경로 밖 LiDAR 점이 차체에서 0.3 m 이내 / 움직이는 트랙 TTC ≤ 2.0 s 또는 1.5 m 이내 / 추적·odom 1.0 s 끊김 |
| NORMAL | inf | 그 밖 |

### 2-1. 긴급정지: 진행 경로 판정

처음에는 "차체에서 사방 0.3 m 안에 LiDAR 점이 있으면 정지"로 했다. Gazebo에서 두 문제가 나왔다.

- **교착:** 벽 앞 0.19 m에서 선 뒤 후진 명령도 0으로 막혀 빠져나오지 못했다.
- **좁은 통로:** 폭 0.4 m 로봇이 0.6 m 통로에 들어가면 옆 여유가 0.1 m라 늘 정지한다. 명세의 0.3 m 긴급정지와 0.6 m 통로 통과를 함께 만족할 수 없다.

그래서 명령 (v, ω)대로 움직일 때 차체가 지나갈 자리를 본다(Nav2 collision_monitor의 approach, 속도별 안전 스캐너 영역과 같은 방식).

- 보는 시간: 반응 0.2 s + |v| / 1.0 m/s² (최소 0.5 s, 제자리 회전용). 등속 원호로 0.05 s마다 차체 위치를 놓는다.
- 그 시점 차체 좌표에서 점이 **진행 방향 앞(후진이면 뒤) 0.3 m 띠** 안이거나 **차체 0.05 m 안**이면서, 지금보다 **가까워지면** 위험이다.
- 멀어지는 명령(벽 앞 후진)과 나란히 지나가는 벽(옆 여유 0.1 m 통로)은 막지 않는다. 진행 경로가 아닌 0.3 m 안 점은 WARNING(저속)으로 둔다.
- 경로 판정은 명령에 따라 바뀌므로, 정지 뒤 멀어지는 명령이 오면 0.5 s 기다리지 않고 바로 내린다. 다시 다가가는 명령이면 즉시 정지한다.
- 추적이 아니라 LiDAR 원본 점으로 본다. 트랙은 확정까지 몇 프레임 걸리고 동적 판정이 틀릴 수 있다(1-2-1).

| Gazebo 확인 | 결과 |
| --- | --- |
| 기둥(반경 0.15 m)으로 0.5 m/s 직진 | 0.63 m에서 경로 위험 → 정지, 기둥까지 0.45 m 남김 |
| 그 자리에서 후진 −0.3 m/s | 바로 통과 (교착 없음) |

### 2-2. TTC와 근접은 움직이는 트랙만

정적 장애물에 TTC를 걸면 벽·랙을 향해 가기만 해도 TTC 1 s 앞(0.5 m/s면 0.5 m)에서 서서 작업 위치에 다가갈 수 없다.
정적 장애물은 2-1이 지키고, TTC(3절)와 "1.5 m 근접 저속"은 움직이는 트랙에만 건다.

| Gazebo 확인 (작업자가 정지한 로봇 쪽으로 1.0 m/s, `ttc_warning` 3.0 s일 때, 10/9) | 결과 |
| --- | --- |
| 3.0 m | WARNING (TTC 2.7~2.9 s) |
| TTC 0.9 s | CRITICAL |
| 작업자가 1.4 m에서 멈춤 | TTC inf, "움직이는 장애물 1.5 m 이내"로 WARNING 유지 |

### 2-3. 그 밖의 규칙

- 단계는 바로 올리고, 내릴 때는 조건이 0.5 s 계속 풀려야 내린다(깜빡임 방지, 2-1 예외).
- 센서 끊김: 스캔(10 Hz) 0.5 s → 정지, 추적·충돌 위험·odom 1.0 s → 저속. 한 번도 받지 못한 것도 끊김으로 본다(시작하면 스캔이 올 때까지 정지).
- 게이트: 정지는 회전도 막고 입력이 없어도 10 Hz로 0을 낸다. WARNING은 선속도 크기를 0.6 m/s로 줄이며 각속도도 같은 비율로 줄여 곡률을 지킨다. 입력이 0.5 s 끊기면 0을 한 번 낸다(Gazebo DiffDrive는 마지막 명령을 따른다).
- 알려진 한계: 0.3 m 안에 벽이 있으면 진행 방향이 아니어도 WARNING(0.6 m/s)이라, 좁은 통로·랙 앞 작업에서 늘 저속이다.

### 2-4. DWA 회피와의 역할 분담 (10/11 이진·연우)

원 명세의 "TTC 임계값 이하 시 회피 기동"은 플래너가 맡고, 안전 노드는 그 안쪽의 최후 방어선이다. TTC 계산은 `amr_tracking` 한 곳에서만 한다.

| 움직이는 장애물의 TTC | 담당 | 동작 |
| --- | --- | --- |
| 3 s 미만 | DWA | 회피 모드 (동적 가중치·여유 거리 키움) |
| 2 s 미만 | 안전 노드 WARNING | 0.6 m/s 제한 |
| 1 s 미만 | 안전 노드 CRITICAL | 정지 |

- `ttc_warning`을 3.0에서 2.0 s로 내렸다. DWA 회피 모드와 같은 3 s에 감속까지 걸면 비켜 가는 순간 속도가 잘려 회피 궤적이 달라지고, 결과가 어느 쪽 덕분인지 구분되지 않는다.
- 2.0 s도 최고속 2.0 m/s에서 0.6 m/s까지 줄이는 시간(반응 0.2 s + 1.4 s ≈ 1.6 s)보다 길다. 1.5 m 근접 규칙은 그대로라, TTC가 길게 나오는 느린 장애물에는 그대로 저속이 걸린다.
- DWA 궤적 시뮬레이션 시간(`sim_time` 2.0 s)이 회피 모드 기준(3 s)보다 짧아, TTC 2~3 s 구간의 충돌은 DWA 예측 범위 밖이다. 맞출지는 이진과 정한다.

## 3. 충돌 위험 (TTC)

`amr_tracking`이 트랙마다 이 로봇 차체에 닿기까지 남은 시간을 `perception/collision_risks`로 낸다(`collision.footprint_time_to_collision`).

- 상대 위치·속도를 로봇 몸체 축으로 돌리고, 0.05 s마다 장애물 원(트랙 반경)과 **실제 차체 사각형**(0.6 × 0.4 m) 사이 거리를 따라가 처음 닿는 시각을 보간한다. 5 s 안에 닿지 않으면 inf.
- 로봇 회전은 무시한다(등속 직선 상대 운동). 로봇 위치는 스캔 촬영 시각 TF, 속도는 `odom`(0.5 s보다 오래되면 0으로 보고 경고).
- 이미 겹쳐 있으면 **가까워지는 중일 때만** 0, 아니면 inf.
- 처음에는 로봇을 외접원(반지름 0.36 m)으로 본 닫힌 식을 썼다. 원은 옆쪽이 실제보다 0.16 m 커서 0.6 m 통로의 벽과 늘 겹쳤고, 겹치면 멀어져도 TTC가 0이라 CRITICAL이 풀리지 않았다(Gazebo, 벽 조각 트랙).

## 4. YOLO 인식과 3D 변환

원 명세 6장 "YOLOv8 객체 인식, 커스텀 메시지 + RViz2 마커, Pinhole 2D→3D(map frame)"에 해당한다.
학습 목표 "Pinhole Camera Model 기반 2D→3D 변환 원리"의 설명 절이다 ([project-spec.md](../project-spec.md) 학습 목표 대응 문서).
노드는 `amr_detection/detection_node.py`, 변환 식은 ROS 비의존 모듈 `amr_perception_core`(`camera_model.py`, `depth_sampling.py`, `projection.py`)에 있다.
파라미터 값은 `amr_detection/config/detection.yaml`.

```
camera/color/image_raw + camera/depth/image_raw (같은 촬영 시각, 픽셀 1:1)
  → 최신 짝 하나만 보관, 타이머가 처리                         detection_node.on_images / process_latest
  → YOLO 검출 + 클래스 이름 바꾸기 (model → 팀 클래스)          detector.py
  → bbox 중심 ROI에서 대표 깊이 z                               depth_sampling.sample_bbox_depth
  → bbox 중심 (u, v) + z → optical frame 위치·공분산 (Pinhole)  projection.backproject_with_covariance
  → 촬영 시각 TF로 target_frame(기본 map)에 옮김                 projection.transform_with_covariance
  → perception/objects (DetectedObject3DArray), perception/markers (MarkerArray)  messages.py
```

### 4-1. 입력과 YOLO 검출

| 입력 | 내용 |
| --- | --- |
| `camera/color/image_raw` | RGB 640×480, 30 Hz, frame `<ns>/camera_optical_frame` (docs/api.md 5절 amr_gazebo) |
| `camera/depth/image_raw` | Depth `32FC1`(m), RGB와 같은 `rgbd_camera` 센서라 촬영 시각이 같고 픽셀이 1:1이다 (`amr_description/config/sensors.yaml`) |
| `camera/camera_info` | 내부 파라미터 K. 처음 받은 값 하나로 `PinholeCamera.from_k`를 만든다 |

- RGB와 Depth는 `message_filters.TimeSynchronizer`(큐 5)로 촬영 시각이 같은 짝만 받는다.
- 추론이 카메라 주기(30 Hz)보다 느리므로 최신 짝 하나만 보관한다. 처리하지 못한 이전 짝은 버린다. 밀린 프레임을 차례로 처리하면 결과가 점점 과거 장면이 된다.
- 타이머(`max_rate` 30 Hz)가 새 짝이 있는지 보고 처리한다. 실제 FPS는 추론 속도가 정한다.
- K를 받기 전에는 처리하지 않는다.

검출은 ultralytics YOLO(`model_path`, 기본 COCO 사전학습 `yolov8n.pt`)로 한다 (`detector.YoloDetector`).

| 단계 | 방법 | 파라미터 |
| --- | --- | --- |
| 입력 | RGB를 `bgr8`로 바꿔 넘긴다 (ultralytics는 numpy 영상을 BGR로 본다) | `image_size` 640 px |
| 클래스 거르기 | 매핑에 있는 모델 클래스 번호만 추론에 넘긴다 (`classes` 인자) | `model_classes` |
| confidence | 이보다 낮은 검출은 버린다 | `confidence_threshold` 0.5 |
| NMS | 겹친 상자 제거 | `iou_threshold` 0.45 |
| 클래스 이름 바꾸기 | 모델 클래스 이름 → 팀 클래스 이름(docs/api.md 4-3절 `class_name`). 매핑에 없으면 버린다 | `model_classes` → `output_classes` |

- 매핑은 두 목록을 같은 순서로 짝짓는다 (`build_class_map`). 길이가 다르거나, 비었거나, `model_classes`에 중복이 있으면 시작할 때 `ValueError`를 낸다. 모델에 없는 클래스 이름이 있어도 `ValueError`다.
- 기본 매핑은 `person → person`, `stop sign → sign`이다. COCO에는 화물이 없어 지금 모델로는 `cargo`가 나오지 않는다. 화물·표지판을 학습한 모델로 바꾸면 이 두 목록만 바꾼다.
- 검출 하나는 `Detection(class_name, class_id, confidence, xyxy)`다. `xyxy`는 YOLO 출력 그대로의 픽셀 경계 관례 bbox다 (4-2절).
- 시작할 때 빈 영상으로 한 번 추론해 둔다. 첫 추론은 모델 준비로 오래 걸려(`detector.py` 주석: Docker CPU 약 13 s) 첫 카메라 프레임이 그만큼 늦어지는 것을 막는다.
- `num_threads` 4: 컨테이너에서 torch 기본 스레드가 1개로 잡혀 CPU 추론이 느렸다(`detection.yaml` 주석: 640 입력 141 ms → 스레드 4에서 약 99 ms, Gazebo 없이).

### 4-2. Pinhole 카메라 모델

좌표는 ROS camera optical frame 관례를 따른다 (`camera_model.py`).

| 좌표 | 원점 | 축 |
| --- | --- | --- |
| optical frame (X, Y, Z) [m] | 카메라 광학 중심 | Z 전방(광축), X 오른쪽, Y 아래 |
| 픽셀 (u, v) [px] | 영상 왼쪽 위 | u 오른쪽, v 아래 |

내부 파라미터는 CameraInfo K에서 읽는다 (`PinholeCamera.from_k`, 행 우선 9개 또는 3x3).

```math
K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}
```

$f_x, f_y$는 픽셀 단위 초점거리, $(c_x, c_y)$는 주점(광축이 영상과 만나는 픽셀)이다. 초점거리가 0 이하이거나 값이 유한하지 않으면 `ValueError`다.

**투영 (3D → 2D, `PinholeCamera.project`).** 카메라 앞($Z > 0$)의 점을 닮은 삼각형으로 영상면에 옮긴다.

```math
u = f_x \frac{X}{Z} + c_x, \qquad v = f_y \frac{Y}{Z} + c_y
```

**역투영 (2D → 3D, `PinholeCamera.backproject`).** 픽셀 하나는 광학 중심에서 나가는 반직선 하나에 대응해, 영상만으로는 거리를 알 수 없다.
깊이 $z$(광축 방향 거리)를 depth 영상에서 얻어 반직선 위의 한 점을 고른다. 위 식을 $X, Y$에 대해 푼 것이다.

```math
X = \frac{(u - c_x)\, z}{f_x}, \qquad Y = \frac{(v - c_y)\, z}{f_y}, \qquad Z = z
```

- $z$는 광축 방향 거리(depth 영상 값)이고 직선거리가 아니다. 직선거리는 $\lVert (X, Y, Z) \rVert$이고 메시지의 `distance`에 넣는다 (4-6절).
- $z \le 0$이면 `ValueError`다. 왜곡이 보정된(rectified) 영상을 가정하고 왜곡 계수 D는 쓰지 않는다.
- 640×480, 수평 FOV 87°(`sensors.yaml`)면 $f_x = 320 / \tan 43.5^\circ \approx 337$ px다. 단위 테스트가 이 값으로 영상 오른쪽 끝(u = 640)이 43.5° 방향으로 역투영되는지, 투영과 역투영이 서로 되돌리는지 확인한다 (`test_camera_model.py`).

**픽셀 중심 관례.** YOLO bbox는 픽셀 경계 관례로 픽셀 i가 [i, i+1)을 덮고 중심이 i + 0.5다. K는 픽셀 i의 중심을 i로 본다.
그래서 bbox 중심을 역투영할 때는 0.5 px를 뺀다 (`depth_sampling.bbox_center_to_pixel`). 빼지 않으면 u, v가 0.5 px씩 밀린다.

```math
u = \frac{x_1 + x_2}{2} - 0.5, \qquad v = \frac{y_1 + y_2}{2} - 0.5
```

### 4-3. bbox 대표 깊이

bbox 안의 depth 값으로 객체 깊이 $z$ 하나를 정한다 (`depth_sampling.sample_bbox_depth`).

1. **ROI 줄이기:** bbox 중심은 그대로 두고 폭·높이를 `roi_scale`(0.5)배로 줄인다. bbox 가장자리에는 배경이 섞인다.
2. **픽셀 고르기:** 줄인 ROI에 걸친 픽셀을 모두 포함하고(floor·ceil) 영상 밖 부분은 잘라낸다.
3. **유효 깊이:** NaN·inf·0 이하 값과 [`min_depth` 0.3, `max_depth` 10.0] m 밖의 값은 뺀다. 0.3 m는 `sensors.yaml` `range_min`, 10 m는 명세 depth 최대 거리다.
4. **유효 비율:** 유효 픽셀 / ROI 픽셀이 `min_valid_ratio`(0.1)보다 낮으면 실패로 본다.
5. **대표값 (`nearest_cluster`):** 유효 깊이의 하위 10백분위를 전경 기준으로 잡고, 기준 + 0.3 m 안의 픽셀(전경)의 중앙값을 쓴다.

| 방식 | 대표 깊이 | 쓰는 이유 / 한계 |
| --- | --- | --- |
| **`nearest_cluster` (기본)** | 하위 `near_percentile`(10) 백분위 + `foreground_band`(0.3 m) 안 픽셀의 중앙값 | 기둥·표지판처럼 가는 물체는 ROI 안에서도 배경이 더 많다. 전체 중앙값은 배경 깊이를 고른다. 최솟값 대신 백분위를 써 깊이 잡음으로 튄 픽셀 하나에 좌우되지 않는다 |
| `median` | 유효 깊이 전체의 중앙값 | 물체가 ROI의 절반 이상일 때만 물체 깊이가 나온다 |

- 노드가 YAML에서 읽는 값은 `roi_scale`, `min_depth`, `max_depth`, `min_valid_ratio`다. `method`, `near_percentile`, `foreground_band`, `depth_scale`(1.0, `32FC1` m)은 `DepthSamplingConfig` 기본값을 쓴다.
- 실패(`EMPTY_ROI` ROI가 영상 밖, `NO_VALID_DEPTH` 유효 깊이 없음, `LOW_VALID_RATIO` 비율 부족)는 예외가 아니라 상태로 알린다. 노드는 그 검출을 빼고 "깊이 없어 뺀 검출" 수로 센다. 깊이가 없으면 3D 위치를 낼 수 없다.
- RGB와 depth 영상 크기가 다르면 `ValueError`다. 두 영상은 픽셀이 1:1이어야 한다.
- 단위 테스트(`test_depth_sampling.py`)로 확인한 동작: 느슨한 bbox 안 배경 무시, 가는 물체에서 `median`은 배경을, `nearest_cluster`는 물체를 고름, 소수의 가까운 튀는 픽셀 무시, 영상 밖 bbox 잘라내기.

### 4-4. 측정 공분산

같은 픽셀 오차도 멀수록 큰 위치 오차가 된다. bbox 중심 오차 $\sigma_{px}$와 깊이 오차 $\sigma_z$를 야코비안으로 위치에 전파한다 (`projection.backproject_with_covariance`).
$u, v, z$의 오차는 서로 독립이라고 가정한다.

```math
J = \frac{\partial (X, Y, Z)}{\partial (u, v, z)} =
\begin{bmatrix}
z / f_x & 0 & (u - c_x) / f_x \\
0 & z / f_y & (v - c_y) / f_y \\
0 & 0 & 1
\end{bmatrix}
```

```math
\Sigma_{cam} = J \, \mathrm{diag}(\sigma_{px}^2,\ \sigma_{px}^2,\ \sigma_z^2) \, J^\top
```

| 파라미터 | 값 | 근거 |
| --- | --- | --- |
| `pixel_noise_std` ($\sigma_{px}$) | 5.0 px | bbox 중심 오차 가정. GT 비교(NEES)로 다시 정한다 (`detection.yaml`) |
| `depth_noise_std` ($\sigma_z$) | 0.02 m | `sensors.yaml` `depth_noise` stddev |

- 광축 위($u = c_x$)에서는 옆 방향 $\sigma_X = \sigma_{px} \cdot z / f_x$이고 깊이 오차가 옆으로 새지 않는다. 5 px, 5 m면 약 7.4 cm다 (`test_projection.py`, $f_x \approx 337$).
- 영상 가장자리에서는 $(u - c_x)/f_x$ 항 때문에 깊이 오차가 옆 방향으로도 퍼져 X·Z가 상관을 갖는다. 단위 테스트가 표본 2만 개 몬테카를로 공분산과 비교해 대각 5%, X·Z 공분산 10% 안으로 맞는지 확인한다.
- 이 공분산이 `DetectedObject3D.pose.covariance`가 되고, 카메라 연관(1-6절)에서 마할라노비스 거리에 쓰인다.

### 4-5. 목표 좌표계로 변환

optical frame 위치를 `target_frame`(기본 `map`)으로 옮긴다.

- TF는 **영상 촬영 시각**(`rgb.header.stamp`)의 `camera_optical_frame → target_frame`을 찾는다 (`tf_timeout` 0.1 s). 추론에 걸린 시간만큼 로봇이 움직였어도 영상을 찍은 순간의 카메라 자세로 옮긴다.
- optical frame과 `camera_link`의 축 차이(rpy (−π/2, 0, −π/2))는 TF 트리가 처리한다. 노드는 영상의 `frame_id`를 그대로 쓴다.
- TF 회전 쿼터니언은 정규화해 회전 행렬 $R$로 바꾼다 (`quaternion_to_matrix`). TF 메시지 값이 정확히 단위 길이가 아닐 수 있다.

```math
p' = R\,p + t, \qquad \Sigma' = R\,\Sigma_{cam}\,R^\top
```

- 이동 $t$는 공분산을 바꾸지 않고 회전은 축만 돌린다 (`transform_with_covariance`).
- TF를 찾지 못하면 그 프레임은 발행하지 않고 "TF 실패" 수로 센다.
- 위치 추정(`map → odom`) 없이 시험할 때는 `target_frame:=amr_01/odom`으로 띄운다.

### 4-6. 발행 메시지와 RViz 마커

`perception/objects` (`amr_msgs/msg/DetectedObject3DArray`)

| 필드 | 값 |
| --- | --- |
| `header.stamp` | 영상 촬영 시각 (추론이 끝난 시각이 아님, docs/api.md 4-3절) |
| `header.frame_id` | `target_frame` |
| `objects[].class_name`, `class_id`, `confidence` | 팀 클래스 이름, 모델 클래스 번호, YOLO confidence 원본 |
| `objects[].bbox_center_x/y`, `bbox_width/height` | 픽셀 경계 관례 bbox의 중심과 크기 (0.5 px 보정 전 값) |
| `objects[].pose.pose.position` | 4-5절 $p'$ |
| `objects[].pose.pose.orientation` | (0, 0, 0, 1). 방향은 추정하지 않는다 |
| `objects[].pose.covariance` | 6x6 행 우선. 위치 3x3 블록은 $\Sigma'$, 회전 대각은 `1e6` (docs/api.md 4-3절) |
| `objects[].distance` | 카메라에서 객체까지 직선거리 $\lVert p \rVert$ (optical frame) |

- 검출이 없어도 빈 배열을 발행한다 (docs/api.md 4-3절).

`perception/markers` (`visualization_msgs/msg/MarkerArray`, `messages.to_markers`)

- 맨 앞 `DELETEALL`로 지난 프레임 마커를 지운다.
- 객체마다 상자(`CUBE`, ns `objects`, 한 변 `marker_size` 0.4 m, 클래스별 색)와 그 위 텍스트(`TEXT_VIEW_FACING`, ns `labels`, 글자 높이 0.2 m)를 둔다.
- 텍스트는 원 명세 결과 예시 형식 `Class: <class>, Conf: <conf>, Dist: <dist>m`이다 (confidence 소수 둘째 자리, 거리 첫째 자리. 예: `Class: cargo, Conf: 0.92, Dist: 1.5m`, `test_detection.py`).
- 마커 수명은 1 s다. 노드가 발행을 멈추면(TF 끊김 등) 마커가 화면에 남지 않는다.

| 클래스 | 상자 색 (r, g, b) |
| --- | --- |
| `cargo` | (0.9, 0.2, 0.1) |
| `person` | (1.0, 0.6, 0.0) |
| `sign` | (0.1, 0.4, 1.0) |
| `forklift` | (0.9, 0.9, 0.1) |
| `robot` | (0.3, 0.8, 0.3) |
| 그 밖 | (0.7, 0.7, 0.7) |

- 노드는 `stats_period`(5 s)마다 처리 FPS, 평균 추론 시간, 촬영→발행 지연, 객체 수, 깊이 없어 뺀 검출 수, TF 실패 프레임 수를 로그로 남긴다. 원 명세 KPI(CPU 10 FPS) 측정용이다.
