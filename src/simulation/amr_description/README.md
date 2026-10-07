# amr_description

- 담당: 지수
- 빌드 타입: ament_cmake (자원 전용, C++ 코드 없음)
- 책임: AMR URDF/xacro 모델, 센서 extrinsic, TF 구조

## 제공 인터페이스

`description.launch.py` (robot_state_publisher) 기준. `<ns>` = 로봇 네임스페이스 (기본 `amr_01`).

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Publish | `/<ns>/robot_description` | `std_msgs/msg/String` | xacro 처리 결과 URDF |
| Publish | `/tf_static` | `tf2_msgs/msg/TFMessage` | `base_footprint` → `base_link` → 센서·캐스터 |
| Publish | `/tf` | `tf2_msgs/msg/TFMessage` | `base_link` → 바퀴 (`joint_states` 기반) |
| Subscribe | `/<ns>/joint_states` | `sensor_msgs/msg/JointState` | 바퀴 각도 (Gazebo에서 브리지) |

Gazebo 쪽 토픽(`cmd_vel`, `odom`, odom TF, 센서 `scan_raw`·`imu/data_raw`·`camera/*`)은 [amr_gazebo](../amr_gazebo/README.md)의 `spawn_robot.launch.py`가 연결한다.
전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

## 로봇 모델 (`urdf/`)

| 파일 | 내용 |
| --- | --- |
| `amr.urdf.xacro` | 조립. 인자 `namespace`(TF 접두어·Gazebo 토픽), `payload_mass`(적재 질량 kg), `gt_rate`(GT 토픽 주기 Hz), `enable_camera`(카메라 센서 생성 여부) |
| `properties.xacro` | 치수·질량·동역학 제한, 관성 매크로. **값은 여기서만 바꾼다** |
| `base.xacro` | `base_footprint`, `base_link` |
| `wheels.xacro` | 좌우 구동 바퀴, 앞뒤 캐스터 |
| `sensors.xacro` | `lidar_link`, `camera_link` → `camera_optical_frame`, `imu_link`. LiDAR(`gpu_lidar`)·RGBD 카메라(`rgbd_camera` 1개)·IMU 센서. 값은 `config/sensors.yaml`에서 읽음 |
| `gazebo.xacro` | Fortress DiffDrive, JointStatePublisher, OdometryPublisher(GT) 시스템 플러그인 |

### 주요 값

| 항목 | 값 | 비고 |
| --- | --- | --- |
| 몸체 | 0.6 × 0.4 × 0.3 m, 25 kg | `base_link`는 몸체 중심, 지면에서 0.18 m. 제동 거리 테스트(#42) 권장값 |
| 구동 바퀴 | 반지름 0.1 m, 폭 0.05 m, 간격 0.34 m | 몸체 폭(0.4 m) 안에 들어감. 충돌 원통은 폭 0.01 m를 바깥으로 5 mm 옮김 (아래 참고) |
| 캐스터 | 앞뒤 x = ±0.22 m, 반지름 0.03 m | 마찰 0인 고정 구로 근사 |
| 바퀴 최대 토크 | 3 N·m (`wheel_max_torque`) | 소형 AGV 허브모터 수준. 제동 거리 테스트(#42) 권장값 |
| 전체 질량 | 약 28.9 kg (+ `payload_mass`) | 몸체 25 + 바퀴 3 + 캐스터 0.6 + 센서 0.3 |
| 최대 선속도 | 2.0 m/s | DiffDrive에서 제한 |
| 최대 선가속도 | 1.0 m/s² | DiffDrive에서 제한 (가속·감속 모두) |
| 최대 각속도 | 1.5 rad/s | |
| 최대 각가속도 | 3.2 rad/s² | 명세에 없는 임시값. Nav2 DWB `acc_lim_theta`와 맞춤 |

**바퀴 충돌 원통 (`wheel_contact_width`)**: Fortress(DART)는 원통-바닥 접촉점을 원통 **안쪽 모서리**에 잡는다.
보이는 폭 그대로(5 cm) 충돌을 두면 제자리 회전에서 유효 바퀴 간격이 0.34 → 0.298 m로 줄어, 로봇이 바퀴로 계산한 것보다
14% 더 돌고 odom은 회전을 12% 작게 잰다. 충돌 원통을 1 cm로 얇게 하고 반 폭만큼 바깥으로 옮겨 안쪽 모서리를 바퀴 중심선에 맞췄다.

| 충돌 원통 | 제자리 회전 odom / 실제 (좌·우) | 직진 odom / 실제 | 유효 바퀴 간격 |
| --- | --- | --- | --- |
| 폭 5 cm, 중심 (이전) | 0.877 | 1.000 | 0.298 m |
| 폭 1 cm, 중심 | 0.976 | 1.000 | 0.332 m |
| **폭 1 cm, 바깥 5 mm (현재)** | **1.004 · 1.004** | **1.000** | **0.341 m** |

실제 = Gazebo 모델 자세 (`ign model -m amr_01 -p`), odom = DiffDrive odom (지금 이름 `/amr_01/diff_drive/odom`). 0.5 rad/s 4초 회전, 0.5 m/s 3초 직진.

**질량·바퀴 토크 (#42 제동 거리 테스트 반영)**: 바퀴 조인트 `<limit effort>`가 DART에서 실제 토크 상한이 된다.
20 N·m면 모터가 한계에 닿지 않아 적재 질량이 드러나지 않아서 3 N·m로 낮췄다. 3 N·m × 2 / 0.1 m = 60 N이라
최대 적재(대형 박스 25 kg, 전체 약 53.9 kg)에서도 1.1 m/s²까지 낼 수 있어 명세 가속 1.0 m/s²를 지킨다.

| 조건 (GT로 측정, 1 m/s) | 0 kg | 2 kg | 10 kg | 25 kg |
| --- | --- | --- | --- | --- |
| 일반 주행 (DiffDrive 가속 제한 ±1.0 m/s², 현재 설정): 0→0.95 m/s · 제동 거리 | 1.06 s · 0.503 m | 1.06 s · 0.503 m | 1.06 s · 0.503 m | 1.06 s · 0.503 m |
| 일반 주행: 0→1.425 rad/s (최대 1.5) | 0.56 s | | 0.56 s | 0.56 s |
| 급정지, 가속 제한 없음, **3 N·m**: 제동 거리 | **0.256 m** | **0.273 m** | **0.340 m** | **0.465 m** |
| └ 이론값 (전체 28.9 kg + 적재, 지연 없음) | 0.241 m | 0.258 m | 0.324 m | 0.449 m |
| 급정지, 가속 제한 없음, 20 N·m (이전) | 0.104 m | | | 0.106 m |

측정 방법: #42 `experiments/payload_braking/scripts/run_experiment.py` 절차를 그대로 쓰고 로봇만 이 URDF(전체 28.9 kg, 3 N·m)로 바꿨다.
1 m/s 도달 후 1초 유지 → 0 명령, GT 위치 미분 속도가 0.01 m/s 미만이 될 때까지 이동 거리. 조건마다 5회 평균, 표준편차 0.001 m 이하(일반 정지 0 kg만 1회 0.534 m, 표는 중앙값),
바퀴 미끄러짐 0회. 급정지는 #42 `accel_limited=false`처럼 DiffDrive 가감속 제한 줄만 지운 URDF로 쟀다. 회전 행은 다시 재지 않았다.

- 현재 설정은 DiffDrive가 가속·감속을 ±1.0 m/s²로 묶어서 일반 주행에서는 적재와 무관하다 (#42와 같은 결론). 3 N·m로 낮춰도 최대 적재에서 명세 가속·회전을 그대로 낸다.
- 적재 질량은 감속 제한을 넘는 제동(급정지)에서 드러난다. 0 → 25 kg에서 제동 거리 1.8배 (#42와 같은 결론).
- #42 실험 모델(28.0 kg: 0.254 / 0.337 / 0.462 m)보다 0.2~0.3 cm 길다 (0.9 kg 더 무거움). 이론값과의 고정 차이는 약 1.5 cm(지연 약 0.015 s)로 #42(약 2.2 cm)보다 작다.
  적재 질량 제동 모델(#55)을 이 URDF에 맞추려면 `t_delay` ≈ 0.015 s.
- **주의**: 안전 노드가 `cmd_vel`을 0으로 보내는 정지도 DiffDrive 감속 제한(1.0 m/s²)을 받아 1 m/s에서 0.5 m, 2 m/s에서 2 m를 간다.

### TF 트리

```
amr_01/odom                         (발행: Gazebo DiffDrive 기본, EKF를 띄우면 ekf_local — D4)
└── amr_01/base_footprint           지면 위 로봇 중심
    └── amr_01/base_link            (0, 0, 0.18)  몸체 중심
        ├── amr_01/left_wheel_link  (0,  0.17, -0.08)
        ├── amr_01/right_wheel_link (0, -0.17, -0.08)
        ├── amr_01/front_caster_link ( 0.22, 0, -0.15)
        ├── amr_01/rear_caster_link  (-0.22, 0, -0.15)
        ├── amr_01/lidar_link       (0.15, 0, 0.2)
        ├── amr_01/camera_link      (0.18, 0, 0.25)
        │   └── amr_01/camera_optical_frame  rpy (-π/2, 0, -π/2)
        └── amr_01/imu_link         (0, 0, 0.1)
```

센서 위치는 원 명세서 TF 예시값이다. 확인: `ros2 run tf2_tools view_frames`

### 옆에서 본 그림 (x–z, 오른쪽이 앞, 높이는 지면 기준)

비율은 대략이며 정확한 축척이 아니다.

```
 z [m]
 0.43 ┤                             ▣ camera_link (x=0.18)
 0.38 ┤                          ▭ lidar_link (x=0.15)
 0.33 ┤ ┌──────────────────────────────────────┐ ← 몸체 윗면
      │ │                                      │
 0.28 ┤ │                ✚ imu_link            │
      │ │                                      │
 0.18 ┤ │                ● base_link           │   몸체 600×400×300 mm
      │ │              ╭───╮                   │
 0.10 ┤ │              │ ◎ │ 구동 바퀴 중심    │   반지름 0.1
 0.03 ┤ └──○───────────╰───╯────────────────○──┘ ← 몸체 바닥 (지상고 0.03)
 0.00 ┼━━━━┷━━━━━━━━━━━━━┷━━━━━━━━━━━━━━━━━┷━━━━  지면
          ↑              ↑                  ↑
     뒤 캐스터       base_footprint     앞 캐스터
     x=-0.22          x=0              x=+0.22
      뒤(-x) ←──────────────────────────→ 앞(+x)
    x=-0.3                                   x=+0.3
```

### 위에서 본 그림 (x–y, 위쪽이 앞)

```
                        앞 (+x)
                          ↑
    y=+0.2 ┌──────────────┼──────────────┐ y=-0.2     x=+0.3
           │              ▣ camera (0.18)│
           │              ▭ lidar  (0.15)│
           │              ○ 앞 캐스터 (0.22)
           │                             │
 왼쪽(+y) ┃█│             ●✚            │█┃ 오른쪽(-y)
  바퀴    ┃█│      base_link / imu       │█┃  바퀴
 y=+0.17  ┃█│                            │█┃ y=-0.17
           │                             │
           │              ○ 뒤 캐스터 (-0.22)
           │                             │
           └─────────────────────────────┘            x=-0.3
                        뒤 (-x)

  ┃█┃ 바퀴: 폭 0.05, 바깥쪽 끝 y=±0.195  →  몸체 폭(±0.2) 안에 들어감
       → 로봇 전체 폭 0.4 m 유지, 0.6 m 좁은 통로 통과 가능
```

### 좌표축 방향

```
 일반 링크 (base_link, lidar, camera_link …)     camera_optical_frame
 ROS 관례 (REP 103)                              이미지 처리 관례

        z (위)                                         z (앞, 카메라가 보는 방향)
        │                                             ╱
        │                                            ╱
        └──── x (앞)                                 ●──── x (오른쪽)
       ╱                                             │
      ╱                                              │
     y (왼쪽)                                         y (아래)
```

`camera_optical_frame`은 OpenCV·YOLO 등 이미지 처리가 쓰는 "z = 카메라 정면" 좌표계다. 픽셀 → 3D 변환은 이 프레임 기준으로 한다.

## 파라미터

`config/` 의 YAML로 관리한다. 파라미터 변경에 재빌드가 필요 없어야 한다.

- `config/sensors.yaml`: 센서 사양·노이즈. `sensors.xacro`가 xacro 처리 때 읽으므로 값을 바꾸면 launch만 다시 하면 된다. 바꿀 때는 민지에게 알린다 (명세서 8-2절)

### 센서 검증 (정지 상태, 월드 v0 스폰 위치 (0, 1.5))

| 항목 | 설정 | 측정 |
| --- | --- | --- |
| LiDAR 주기 · 빔 | 10 Hz, 720빔 0.5° | 10.0 Hz, 720빔 0.5000° |
| LiDAR 노이즈 σ | 0.03 m | 0.0299 m (빔별 표준편차 중앙값, 100 스캔) |
| LiDAR 거리 | 남쪽 `rack_lane_divider` 1.1 m, 북쪽 `rack_a4` 2.9 m | 1.100 m, 2.899 m. 최소 거리 1.05 m (몸체 가림 없음) |
| IMU 주기 | 100 Hz | 100.0 Hz |
| 자이로 σ | 2.0e-4 rad/s | 2.0e-4 rad/s (3축) |
| 가속도 σ · 바이어스 | 0.017 m/s² · ±0.1 m/s² | 0.016~0.017 m/s² · x −0.099, y −0.100, z +0.100 (9.900 − 9.8) |
| 카메라 (yaw −90°로 스폰, 정면 `rack_lane_divider`) | 640×480, FOV 87° | RGB `rgb8`, Depth `32FC1`, frame `camera_optical_frame` |
| 카메라 내부 파라미터 | fx = 320 / tan(43.5°) = 337.2 | fx = fy = 337.2, cx 320, cy 240 |
| Depth 거리 · 노이즈 σ | 0.92 m · 0.02 m | 0.919 m · 0.020 m (중앙 80×80 px, 40 프레임) |

- Gazebo IMU는 메시지 공분산을 0으로 보낸다. `/imu/data`(필터 출력)에서 채운다.
- RGB와 Depth는 `rgbd_camera` 센서 하나라 주기·클립 거리(0.3~10 m)를 공유한다. 명세(RGB 30 Hz, Depth 15 Hz 이상)는 30 Hz로 둘 다 만족한다.
- Fortress `rgbd_camera`는 Depth 노이즈를 RGB에도 같이 넣는다. RGB 흔들림은 σ 약 0.5/255(1단계 미만)라 인식에 영향이 없다고 보고 그대로 둔다.
- Gazebo 포인트클라우드(`camera/points`)는 점 좌표가 x 전방인데 frame_id가 광학 프레임이라 쓰지 않는다. 포인트클라우드는 Depth 영상과 `camera_info`로 만든다 (작업: Depth 포인트클라우드 변환·다운샘플링).

launch 인자 (`description.launch.py`): `namespace`(기본 `amr_01`), `payload_mass`(기본 `0.0`), `gt_rate`(기본 `100`), `enable_camera`(기본 `true`), `use_sim_time`(기본 `true`)

## 실행

```bash
# robot_state_publisher만
ros2 launch amr_description description.launch.py namespace:=amr_01

# 월드 + 로봇 + RViz (gui 컨테이너에서)
ros2 launch amr_gazebo sim.launch.py

# 움직여 보기
ros2 topic pub -r 10 /amr_01/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
ros2 topic pub --once /amr_01/cmd_vel geometry_msgs/msg/Twist "{}"   # 정지
```

주의: Fortress DiffDrive는 마지막 `cmd_vel`을 계속 유지한다. 발행을 멈춰도 로봇이 서지 않으니 0 명령을 보내서 세운다.

## 테스트

```bash
colcon test --packages-select amr_description
```
