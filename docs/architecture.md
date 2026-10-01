# 시스템 아키텍처

> 상태: 골격 단계. 컴포넌트 구성과 협업 규칙은 확정이고, 시퀀스 다이어그램과 토픽 상세는 구현하면서 채운다.

## 1. 아키텍처 전략

### 1-1. 결정: ROS2 패키지 단위 모노레포

정식 MSA(서비스별 레포, REST/gRPC 통신)는 쓰지 않는다. ROS2 자체가 노드들이 DDS로 통신하는 분산 구조라서 MSA의 장점(독립 개발·배포, 느슨한 결합)은 기본으로 따라온다. 대신 MSA의 원칙을 ROS2에 맞게 적용한다.

| MSA 원칙            | 이 프로젝트에서의 대응                                        |
| ------------------- | ------------------------------------------------------------- |
| 서비스 단위 소유    | 패키지마다 담당자 1명(README에 명시), 담당자 리뷰를 거쳐 머지 |
| API 계약 우선       | `amr_msgs`에 msg/srv/action을 먼저 정의하고 동결              |
| 서비스 간 결합 금지 | 다른 도메인과는 토픽·서비스·액션으로만 통신, 코드 import 금지 |
| 독립 테스트         | 알고리즘은 ROS 비의존 `*_core`에 두고 단위 테스트             |
| 독립 배포           | docker-compose 서비스로 역할별 컨테이너 분리                 |

MSA를 채택하지 않은 이유:

- 원 명세서가 "`src` 안에 패키지 단위로, `colcon build` 경고 0"을 요구한다. 단일 colcon 워크스페이스가 전제다.
- 서비스 사이에 REST 계층을 두면 DDS 위에 지연이 한 겹 더 붙어 200ms 응답 예산과 연구 1의 지연 측정을 흐린다.

### 1-2. 병렬 작업 원칙

1. **인터페이스 먼저**: P0(~10/11)에 `amr_msgs`와 [api.md](api.md)의 토픽 규칙을 확정한다.
2. **소유권**: 남의 패키지는 PR로만 수정한다.
3. **코어 분리**: `*_core`는 시뮬레이터 없이 개발·테스트한다.
4. **Mock 우선**: 각자 자기 출력 토픽의 mock 노드를 먼저 제공해 서로 기다리지 않는다.
5. **조립 분리**: 패키지별 launch·config를 두고 `amr_bringup`은 include만 한다.

세부 규칙은 [CONTRIBUTING.md](../.github/CONTRIBUTING.md)에 있다.

## 2. 레포 구조

```
amr-fleet-system/
├── src/                  ROS2 패키지 (도메인/패키지)
├── docker/               Dockerfile, Grafana 대시보드
├── docker-compose.yml    역할별 profile (dev, sim, robots, fleet, perception)
├── tools/harness/        회귀 테스트 하네스 (N회 반복, 신뢰구간)
├── experiments/          research1/, research2/ 실험 설정과 분석
├── dashboard/            웹 프론트엔드 대시보드 (선택, ROS 패키지 아님)
├── docs/                 명세, 아키텍처, API, 알고리즘, 리포트
└── .github/              CI, 템플릿, 기여 가이드
```

## 3. 패키지 구성

| 도메인        | 패키지                    | 빌드           | 담당             | 책임                                                       |
| ------------- | ------------------------- | -------------- | ---------------- | ---------------------------------------------------------- |
| interfaces    | `amr_msgs`                | cmake (rosidl) | 하연 (전원 리뷰) | 커스텀 msg/srv/action                                      |
| simulation    | `amr_description`         | cmake          | 지수             | URDF/xacro, 센서 extrinsic, TF                             |
| simulation    | `amr_gazebo`              | cmake          | 이진, 지수       | 물류센터 정적 월드(이진), 동적 장애물·센서 노이즈·GT(지수) |
| localization  | `amr_odometry`            | python         | 지수             | 순기구학, 휠 오도메트리, 센서 전처리 필터                  |
| localization  | `amr_mapping`             | python         | 지수             | slam_toolbox 설정, 지도 저장, 맵 품질 평가                 |
| localization  | `amr_localization`        | python         | 민지             | AMCL·EKF 설정, Kidnapped 복구, GT 대비 RMSE 측정           |
| navigation    | `amr_planning_core`       | cmake          | 이진             | A\*, 평활화, DWA (ROS 비의존)                              |
| navigation    | `amr_nav2_plugins`        | cmake          | 이진             | Nav2 Planner/Controller 플러그인 래퍼                      |
| navigation    | `amr_control`             | cmake          | 이진             | Pure Pursuit/Stanley, PID, 속도 프로파일                   |
| perception    | `amr_detection`           | python         | 연우             | YOLOv8 인식, RViz2 마커                                    |
| perception    | `amr_perception_core`     | python         | 연우             | Pinhole 3D, KF, TTC, ORCA (ROS 비의존)                     |
| perception    | `amr_tracking`            | python         | 연우             | 동적 장애물 분류·추적, 카메라 연관, 충돌 예측              |
| perception    | `amr_safety`              | python         | 연우             | 긴급정지, Safety Zone, E-Stop, 센서 고장 감지              |
| task          | `amr_bt`                  | cmake          | 지수             | BehaviorTree.CPP 작업 수행, 작업 관리                      |
| task          | `amr_docking`             | python         | 지수             | ArUco 정밀 도킹                                            |
| fleet         | `amr_fleet_core`          | python         | 하연             | 할당, 경로 충돌 예측, 교착 탐지 (ROS 비의존)               |
| fleet         | `amr_fleet_manager`       | python         | 하연             | 중앙집중 Fleet 서버, Traffic Management                    |
| fleet         | `amr_fleet_decentralized` | python         | 하연, 이진       | 분산 협상, 서버 장애 폴백 (연구 1 비교군)                  |
| fleet         | `amr_fault_injector`      | python         | 하연             | 지연·손실·서버 강제종료 주입                               |
| observability | `amr_metrics_exporter`    | python         | 지수             | Prometheus exporter, 사고 전후 rosbag 스냅샷               |
| bringup       | `amr_bringup`             | python         | 공동             | 전체 launch 조립, 다중 로봇 스폰                           |

빌드 타입 기준: Nav2 플러그인과 BehaviorTree.CPP 노드는 C++이 필수라 navigation·task(`amr_bt`)는 C++로 둔다. 나머지는 개발 속도를 위해 Python으로 시작한다. 구현 착수 전이라면 담당자가 PR로 변경할 수 있다.

## 4. 컴포넌트 다이어그램

로봇 1대의 스택(×5, 네임스페이스 `amr_01`~`amr_05`)과 Fleet 레이어.

```mermaid
flowchart TB
  subgraph SIM[Gazebo Fortress · amr_gazebo]
    WORLD[물류센터 월드<br/>동적 장애물]
    SENS[LiDAR · Depth · RGB · IMU · Encoder]
  end

  subgraph ROBOT["로봇 스택 (amr_0N 네임스페이스)"]
    ODOM[amr_odometry]
    MAP[amr_mapping<br/>SLAM · 지도 저장]
    LOC[amr_localization<br/>AMCL · EKF]
    DET[amr_detection<br/>YOLOv8]
    TRK[amr_tracking<br/>KF · TTC]
    SAFE[amr_safety]
    NAV[Nav2 + amr_nav2_plugins<br/>A* · DWA]
    CTRL[amr_control]
    BT[amr_bt + amr_docking]
  end

  subgraph FLEET[Fleet 레이어]
    FM[amr_fleet_manager<br/>중앙집중]
    FD[amr_fleet_decentralized<br/>분산 비교군]
    FI[amr_fault_injector]
  end

  subgraph OBS[관측]
    MET[amr_metrics_exporter]
    PROM[Prometheus · Grafana]
    DASH[dashboard / Foxglove]
  end

  SENS --> ODOM --> LOC
  ODOM --> MAP -->|map.pgm · map.yaml| LOC
  SENS --> DET --> TRK
  SENS --> TRK
  LOC --> NAV
  TRK --> NAV
  TRK --> SAFE
  NAV --> CTRL --> SIM
  SAFE --> CTRL
  BT --> NAV
  FM <-->|작업 할당 · 교통 제어| FI
  FI <-->|지연 · 손실 주입| BT
  FD <-.-> BT
  ROBOT --> MET --> PROM
  ROBOT --> DASH
```

## 5. 시퀀스 다이어그램

### 5-1. 작업 요청 → 할당 → 이동 → 완료 보고 (중앙집중 모드)

메시지 필드는 [api.md 4절](api.md#4-amr_msgs-인터페이스-초안) 초안을 따른다. 서버 ↔ 로봇은 모두 `/fleet/` 토픽으로 주고받고, 로봇 안에서 이동·도킹은 액션을 쓴다.

```mermaid
sequenceDiagram
  autonumber
  participant GEN as 작업 생성기<br/>대시보드
  participant FM as amr_fleet_manager
  participant CORE as amr_fleet_core<br/>(할당 알고리즘)
  participant FI as amr_fault_injector
  participant BT as amr_bt<br/>(amr_0N)
  participant NAV as Nav2<br/>(amr_0N)
  participant DOCK as amr_docking<br/>(amr_0N)

  loop 1~5Hz
    BT-)FM: /fleet/robot_states (RobotState: pose, mode, battery)
  end

  GEN-)FM: /fleet/task_requests (Task)
  FM->>FM: 작업 큐에 적재, IDLE 로봇 추림
  FM->>CORE: assign_{algorithm}(tasks, robots)
  CORE-->>FM: AssignmentResult {task_id: robot_id}
  FM-)FI: /fleet/task_assignments (ASSIGN, header.stamp = cmd_time)
  FI-)BT: 중계 (지연·손실 주입)
  Note over BT: robot_id가 자기 것이 아니면 무시
  BT-)FM: /fleet/task_status (PENDING)
  BT-)FM: /fleet/task_status (IN_PROGRESS, phase=move_to_pickup)

  BT->>NAV: navigate_to_pose (pickup_pose)
  NAV-->>BT: 도착
  BT->>DOCK: docking/dock (pickup_station_id)
  DOCK-->>BT: success, position_error
  BT-)FM: /fleet/task_status (phase=loading)

  BT-)FM: /fleet/task_status (phase=move_to_dropoff)
  BT->>NAV: navigate_to_pose (dropoff_pose)
  NAV-->>BT: 도착
  BT->>DOCK: docking/dock (dropoff_station_id)
  DOCK-->>BT: success
  BT-)FM: /fleet/task_status (phase=unloading)

  BT-)FM: /fleet/task_status (COMPLETED, start_time, end_time, distance_traveled)
  FM->>FM: 로봇을 IDLE로 되돌리고 다음 할당
  Note over FM,BT: 실패 시 FAILED + failure_reason, 서버가 재할당 여부 판단
```

- 비동기 화살표(`-)`)는 토픽, 동기 화살표(`->>`)는 액션·함수 호출이다.
- 작업 전달을 토픽으로 한 이유: 장애 주입기가 중간에서 중계할 수 있고, 서버가 죽어도 분산 모드가 같은 토픽을 이어받을 수 있다. 액션은 서버-로봇 1:1 연결이라 둘 다 어렵다 ([api.md 4-4절](api.md#4-4-쟁점-정리) Fleet 쟁점).
- 응답 시간 KPI는 명령 발행부터 로봇이 처음 움직일 때까지를 **실제 시간**으로 잰다(#5). 시작·끝 시각을 어디서 얻는지는 [하네스 로그 포맷](../tools/harness/README.md#지표-csv)의 `response_time.csv`를 따른다.

### 5-2. 이후 추가할 시나리오

TODO: 구현 진행에 맞춰 아래 시나리오를 추가한다 (원 명세서 문서화 요구사항).

- 도킹 실패 → 재시도 → 복귀
- 동적 장애물 감지 → 추적 → TTC 경보 → 회피 → 경로 복귀
- 교착 탐지 → 해소
- 중앙 서버 장애 → 분산 폴백

## 6. 배포 구성

`docker compose up`은 시스템 서비스만 띄우고, 개발 도구는 profile로 켠다. 실행 방법은 [runbook.md](runbook.md)에 있다.

| 서비스 | 실행 | 내용 |
| --- | --- | --- |
| `sim` | 기본 | Gazebo 월드 |
| `robots` | 기본 | 로봇 5대 스택 (`amr_bringup`) |
| `fleet` | 기본 | Fleet 서버, 장애 주입기 |
| `perception` | 기본 | YOLO 추론. GPU는 `docker-compose.gpu.yml`을 겹쳐 쓴다 |
| `dev` | `--profile dev` 또는 `run dev` | 개발 셸 컨테이너 |
| `gui`, `gui-proxy` | `--profile gui` | 브라우저 GUI (Xvfb + noVNC, RViz2). `gui-proxy`가 6080 포트를 호스트에 중계 |
| `sandbox` | `--profile sandbox` | TurtleBot4 + AMCL + Nav2 임시 환경. 우리 로봇으로 교체되면 제거 |
| (예정) `monitoring` | — | Prometheus, Grafana |
| (예정) `dashboard` | — | 웹 대시보드 |
