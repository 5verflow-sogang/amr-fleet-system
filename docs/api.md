# 노드 API 명세

> 원 명세서 요구사항: 각 노드의 토픽·서비스·액션 인터페이스를 명세한다. 인터페이스가 바뀌는 PR은 이 문서를 같은 PR에서 갱신한다.

## 1. 이름 규칙

| 대상 | 규칙 | 예 |
| --- | --- | --- |
| 로봇 네임스페이스 | `amr_<2자리 번호>` | `amr_01` |
| 로봇 토픽 | 노드는 상대 이름을 쓰고, 네임스페이스는 bringup이 부여 | `/amr_01/scan`, `/amr_01/odom` |
| 도메인 토픽 | `<도메인>/<이름>` | `/amr_01/perception/objects` |
| Fleet 전역 토픽 | `/fleet/<이름>` | `/fleet/task_assignments` |
| TF 프레임 | `<네임스페이스>/<프레임>` | `amr_01/base_link` |
| 메시지 타입 | PascalCase, `amr_msgs` | `amr_msgs/msg/DetectedObject3DArray` |

표준 토픽(`scan`, `odom`, `cmd_vel`, `tf`, `map`)은 ROS 관례 이름을 그대로 쓴다.

## 2. TF 트리 (로봇별)

```
map
└── amr_0N/odom            (map→odom: AMCL, 민지)
    └── amr_0N/base_footprint   (odom→base: 지수 책임, 발행 주체는 EKF 구성에 따라 확정)
        └── amr_0N/base_link
            ├── amr_0N/lidar_link
            ├── amr_0N/camera_link
            │   └── amr_0N/camera_optical_frame
            ├── amr_0N/imu_link
            ├── amr_0N/left_wheel_link
            └── amr_0N/right_wheel_link
```

TODO(지수): extrinsic 값 확정 후 좌표 기입, `view_frames` 결과 첨부.

## 3. 합의가 필요한 경계 (P0 확정 대상)

| 경계 | 담당 | 결정할 것 |
| --- | --- | --- |
| 인식 결과 메시지 | 연우 ↔ 이진·지수 | 3D 객체 메시지(class, confidence, PoseWithCovariance) — Costmap·BT가 소비. 초안: [4절](#4-amr_msgs-인터페이스-초안) |
| 중앙 ↔ 분산 Fleet | 하연 ↔ 이진 | 작업 할당·협상 메시지, 서버 heartbeat. 초안: [4절](#4-amr_msgs-인터페이스-초안) |
| 하네스 지표 | 민지 ↔ 하연 | 수집 지표와 로그 포맷 (명세서 1-4절 측정 절차 표 기준, 초안: [tools/harness/README.md](../tools/harness/README.md#로그-포맷-초안-하연과-합의-중)) |
| 매핑 → 위치추정 인계 | 지수 → 민지 | 지도 파일, `/odom`·`/scan`·`/imu/data` 합격 조건, 센서 노이즈 파라미터 변경 통보 ([명세서 8-2절](project-spec.md#8-2-매핑위치추정-분업-지수--민지)) |

## 4. amr_msgs 인터페이스 (초안)

> 상태: 초안. 9/28 회의에서 대주제별 검토자와 확정한다. 필드 정의와 주석은 [src/interfaces/amr_msgs/](../src/interfaces/amr_msgs/)의 `.msg`·`.action` 파일이 기준이다.

### 4-1. 대주제와 검토자

변경 절차는 [CONTRIBUTING 5절](../.github/CONTRIBUTING.md#5-인터페이스-변경-절차-amr_msgs)을 따른다. 발행·구독 담당자 전원의 승인이 필요하다.

| 대주제 | 인터페이스 | 한 줄 설명 | 필수 검토 | 참고 검토 |
| --- | --- | --- | --- | --- |
| 인식 | `DetectedObject3D`, `DetectedObject3DArray` | 카메라로 본 것 (한 장면, ID 없음) | 연우, 이진, 지수 | — |
| 추적 | `TrackedObstacle`, `TrackedObstacleArray` | 시간에 걸쳐 따라가는 장애물 (ID, 속도, 충돌 예측) | 연우, 이진 | — |
| 안전 | `SafetyStatus` | 감속·정지 여부와 원인 | 연우, 이진, 지수 | — |
| Fleet | `Task`, `TaskAssignment`, `TrafficCommand`, `FleetHeartbeat`, `TaskBid` | 서버 → 로봇: 작업 할당, 교통 제어, 서버 생존 신호, 분산 입찰 | 지수, 이진 | 민지 |
| 로봇 | `RobotState`, `TaskStatus`, `Dock` (action) | 로봇 → 서버: 상태·작업 진행 보고, 로봇 내부: 도킹 | 지수 | 민지 |

### 4-2. 토픽·액션 목록

로봇 토픽은 상대 이름으로 쓰고 네임스페이스는 bringup이 붙인다(예: `perception/objects` → `/amr_01/perception/objects`). Fleet 토픽은 `/fleet/`로 시작하는 절대 이름이라 로봇 네임스페이스 안의 노드도 같은 토픽을 쓴다.

| 대주제 | 이름 | 종류 | 타입 | 발행 (서버) | 구독 (클라이언트) | 발행 시점 |
| --- | --- | --- | --- | --- | --- | --- |
| 인식 | `perception/objects` | 토픽 | `amr_msgs/msg/DetectedObject3DArray` | `amr_detection` | `amr_tracking`, Costmap, `amr_bt` | 카메라 프레임마다 |
| 추적 | `perception/obstacles` | 토픽 | `amr_msgs/msg/TrackedObstacleArray` | `amr_tracking` | `amr_safety`, Costmap, DWA | 추적 주기마다 |
| 안전 | `safety/status` | 토픽 | `amr_msgs/msg/SafetyStatus` | `amr_safety` | `amr_control`, `amr_bt`, `amr_fleet_manager`, 대시보드 | 바뀌면 즉시, 그 외 주기 |
| 안전 | `safety/estop` | 서비스 | `std_srvs/srv/SetBool` (안) | `amr_safety` | 대시보드, 운영자 | 요청 시 |
| Fleet | `/fleet/task_requests` | 토픽 | `amr_msgs/msg/Task` | 작업 생성기, 대시보드 | `amr_fleet_manager`, `amr_fleet_decentralized` | 요청 시 |
| Fleet | `/fleet/task_assignments` | 토픽 | `amr_msgs/msg/TaskAssignment` | `amr_fleet_manager` | `amr_bt` | 할당·취소 시 |
| Fleet | `/fleet/traffic_commands` | 토픽 | `amr_msgs/msg/TrafficCommand` | `amr_fleet_manager` | `amr_bt` | 교통 제어 시 |
| Fleet | `/fleet/heartbeat` | 토픽 | `amr_msgs/msg/FleetHeartbeat` | `amr_fleet_manager` | 각 로봇, `amr_fleet_decentralized` | 고정 주기 |
| Fleet | `/fleet/task_bids` | 토픽 | `amr_msgs/msg/TaskBid` | `amr_fleet_decentralized` | `amr_fleet_decentralized` | 분산 모드에서 |
| 로봇 | `/fleet/robot_states` | 토픽 | `amr_msgs/msg/RobotState` | 각 로봇 (`amr_bt`, 안) | `amr_fleet_manager`, `amr_fleet_decentralized`, 대시보드, `amr_metrics_exporter` | 1~5Hz |
| 로봇 | `/fleet/task_status` | 토픽 | `amr_msgs/msg/TaskStatus` | `amr_bt` | `amr_fleet_manager`, 대시보드, 하네스 | 상태·단계가 바뀔 때 |
| 로봇 | `docking/dock` | 액션 | `amr_msgs/action/Dock` | `amr_docking` | `amr_bt` | BT가 요청할 때 |

`/fleet/task_assignments`, `/fleet/traffic_commands`, `/fleet/heartbeat`는 장애 주입기(`amr_fault_injector`)가 중계하며 지연·손실을 건다.

### 4-3. 공통 규칙

| 항목 | 규칙 |
| --- | --- |
| `robot_id` | 로봇 네임스페이스 이름과 같다 (예: `"amr_01"`) |
| 좌표계 | 위치는 `header.frame_id` 기준이며 기본 `"map"` |
| 방향을 모를 때 | `PoseWithCovariance`의 orientation은 (0, 0, 0, 1), 공분산의 회전 성분 대각은 `1e6`. 소비 측은 위치 3×3 블록만 쓴다 |
| 특수 값 | `inf`: 해당 없음 (예: 충돌 경로가 아님, 장애물 없음) / `NaN`: 계산하지 않음 (예: 연구 2 기능을 끈 baseline) |
| 빈 결과 | 검출·장애물이 없어도 빈 배열로 발행한다. 발행이 끊기면 노드 이상으로 본다 |
| 시각 | 인식 결과의 `header.stamp`는 처리 완료 시각이 아니라 **센서 촬영 시각**이다 |
| 응답 시간 KPI | `cmd_time`은 `TaskAssignment.header.stamp`를 쓴다 |
| 작업 로그 | `TaskStatus`의 마지막 이벤트(COMPLETED·FAILED)가 로그 한 줄이 된다. 소요 시간은 `end_time - start_time` |

**`class_name` 허용 값**

| 값 | 뜻 | 구분 |
| --- | --- | --- |
| `cargo` | 화물 | 필수 |
| `person` | 사람 (작업자 포함) | 필수 |
| `sign` | 표지판 | 필수 |
| `forklift` | 지게차 | 선택 (연우 확정) |
| `robot` | 다른 AMR | 선택 (연우 확정) |
| `unknown` | 카메라 검출이 없음 (`TrackedObstacle` 전용) | 고정 |

### 4-4. 미결 쟁점 (9/28 회의)

| 대주제 | 쟁점 | 결정 | 초안 |
| --- | --- | --- | --- |
| 추적 | TTC·충돌확률을 `TrackedObstacle`에 둘지, 별도 메시지로 분리할지 | 연우 | 같이 둔다 |
| 추적 | 장애물 크기를 원(`radius`)으로 충분한지, 사각형(가로·세로)이 필요한지 | 연우, 이진 | 원 |
| 안전 | 감속·정지를 누가 실행하나: 제어가 `speed_limit`을 읽나, 안전 노드가 `cmd_vel`을 직접 막나 | 연우, 이진 | 미정 (후자가 더 안전) |
| 안전 | E-Stop 걸기·해제를 표준 서비스 `std_srvs/SetBool`로 할지 | 연우 | 표준 서비스 |
| Fleet | 작업 전달을 토픽으로 할지 액션으로 할지 | 하연 | 토픽 (장애 주입 중계, 분산 폴백에 유리) |
| Fleet | 할당 토픽을 `/fleet/` 하나로 모을지 로봇별로 나눌지 | 하연 | 하나로 모으고 `robot_id`로 구분 |
| Fleet | heartbeat 주기·타임아웃 | 하연, 이진 | 메시지에 넣지 않고 파라미터로 둔다 |
| 로봇 | `RobotState` 발행 주체와 가상 배터리 모델 담당 | 지수 | `amr_bt` |
| 로봇 | 도킹 3회 재시도를 `Dock` 액션 안에서 할지 BT에서 할지 | 지수 | BT (에러 자동 복구 요구와 맞음) |
| 공통 | 응답 시간을 시뮬레이션 시간으로 잴지 실제 시간으로 잴지 | 민지, 하연 | [하네스 로그 포맷 합의 필요 1번](../tools/harness/README.md#합의-필요-하연)과 함께 결정 |

## 5. 패키지별 인터페이스

각 패키지 README의 표를 여기에 모은다.

### amr_detection

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Subscribe | | | |
| Publish | `perception/objects` | `amr_msgs/msg/DetectedObject3DArray` | 카메라 프레임별 인식 결과와 3D 위치 |

<!-- TODO: 나머지 패키지 섹션을 같은 형식으로 추가 -->

## 6. QoS 기본값

TODO(하연): 센서 스트림(best effort), 명령·작업 상태(reliable), 맵(transient local) 등 기본 프로파일 확정.
