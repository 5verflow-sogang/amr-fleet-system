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

Gazebo 쪽 토픽(`cmd_vel`, `odom`, odom TF)은 [amr_gazebo](../amr_gazebo/README.md)의 `spawn_robot.launch.py`가 연결한다.
전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

## 로봇 모델 (`urdf/`)

| 파일 | 내용 |
| --- | --- |
| `amr.urdf.xacro` | 조립. 인자 `namespace`(TF 접두어·Gazebo 토픽), `payload_mass`(적재 질량 kg) |
| `properties.xacro` | 치수·질량·동역학 제한, 관성 매크로. **값은 여기서만 바꾼다** |
| `base.xacro` | `base_footprint`, `base_link` |
| `wheels.xacro` | 좌우 구동 바퀴, 앞뒤 캐스터 |
| `sensors.xacro` | `lidar_link`, `camera_link` → `camera_optical_frame`, `imu_link` (링크만, 플러그인은 2주차) |
| `gazebo.xacro` | Fortress DiffDrive, JointStatePublisher, OdometryPublisher(GT) 시스템 플러그인 |

### 주요 값

| 항목 | 값 | 비고 |
| --- | --- | --- |
| 몸체 | 0.6 × 0.4 × 0.3 m, 30 kg | `base_link`는 몸체 중심, 지면에서 0.18 m |
| 구동 바퀴 | 반지름 0.1 m, 폭 0.05 m, 간격 0.34 m | 몸체 폭(0.4 m) 안에 들어감 |
| 캐스터 | 앞뒤 x = ±0.22 m, 반지름 0.03 m | 마찰 0인 고정 구로 근사 |
| 전체 질량 | 약 33.9 kg (+ `payload_mass`) | |
| 최대 선속도 | 2.0 m/s | DiffDrive에서 제한 |
| 최대 선가속도 | 1.0 m/s² | |
| 최대 각속도 | 1.5 rad/s | |
| 최대 각가속도 | 3.2 rad/s² | 명세에 없는 임시값. Nav2 DWB `acc_lim_theta`와 맞춤 |

### TF 트리

```
amr_01/odom                         (DiffDrive, spawn_robot.launch.py publish_odom_tf:=true)
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

launch 인자 (`description.launch.py`): `namespace`(기본 `amr_01`), `payload_mass`(기본 `0.0`), `use_sim_time`(기본 `true`)

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
