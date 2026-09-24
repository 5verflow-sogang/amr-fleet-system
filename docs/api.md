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
| 인식 결과 메시지 | 연우 ↔ 이진·지수 | 3D 객체 메시지(class, confidence, PoseWithCovariance) — Costmap·BT가 소비 |
| 중앙 ↔ 분산 Fleet | 하연 ↔ 이진 | 작업 할당·협상 메시지, 서버 heartbeat |
| 하네스 지표 | 민지 ↔ 하연 | 수집 지표와 로그 포맷 (명세서 1-4절 측정 절차 표 기준) |
| 매핑 → 위치추정 인계 | 지수 → 민지 | 지도 파일, `/odom`·`/scan`·`/imu/data` 합격 조건, 센서 노이즈 파라미터 변경 통보 ([명세서 8-2절](project-spec.md#8-2-매핑위치추정-분업-지수--민지)) |

## 4. 패키지별 인터페이스

각 패키지 README의 표를 여기에 모은다.

### amr_detection

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Subscribe | | | |
| Publish | | | |

<!-- TODO: 나머지 패키지 섹션을 같은 형식으로 추가 -->

## 5. QoS 기본값

TODO(하연): 센서 스트림(best effort), 명령·작업 상태(reliable), 맵(transient local) 등 기본 프로파일 확정.
