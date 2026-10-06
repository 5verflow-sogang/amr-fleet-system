# amr_zone_client

- 담당: 이진
- 빌드 타입: ament_python
- 책임: 로봇 측 구역 진입 처리 (`ZoneRequest`/`ZoneGrant` 클라이언트). `amr_fleet_manager`의 구역 허가 서버(#57,
  `amr_fleet_core.traffic`)와 짝이다.

## 제공 인터페이스

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Publish | `/fleet/zone_requests` | `amr_msgs/msg/ZoneRequest` | 진입 요청·연장·반납. 중계되지 않는 절대 토픽 (`fleet_manager_node.py`) |
| Subscribe | `fleet/zone_grants` (파라미터 `zone_grants_topic`) | `amr_msgs/msg/ZoneGrant` | 서버 응답. 네임스페이스 안 상대 이름 → `amr_fault_injector`가 중계한 `/<robot_id>/fleet/zone_grants` |
| Subscribe | `plan` (파라미터 `plan_topic`) | `nav_msgs/msg/Path` | Nav2 전역 계획 경로. 로봇 현재 위치 기준으로 다시 계산된 것을 그대로 쓴다 |
| Subscribe | `cmd_vel_nav` (파라미터 `cmd_vel_in_topic`) | `geometry_msgs/msg/Twist` | 게이팅 전 속도 명령 (Nav2 controller_server 출력 등) |
| Publish | `cmd_vel` (파라미터 `cmd_vel_out_topic`) | `geometry_msgs/msg/Twist` | 게이팅 후 속도 명령. 허가 전엔 선속도만 깎는다, 각속도는 그대로 |

전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

### 노드

| 실행 파일 | 노드 이름 | 역할 |
| --- | --- | --- |
| `zone_client_node` | `zone_client` | 경로·위치 → 구역 진입 판단 → `ZoneRequest` 발행, `cmd_vel` 게이팅 |

## 동작 규칙 (상태 흐름)

구역마다 독립된 상태 기계를 둔다 (두 구역을 연달아 지나도 서로 안 엮인다).

```
IDLE ──(경로 기준 경계까지 남은 거리 ≤ trigger_distance_m)──> APPROACHING ──(위치가 구역 안)──> INSIDE ──(위치가 구역 밖)──> IDLE
         │                                                        │
         │                                        grant == REJECTED
         └── (이 전환에서 request_id를 1 늘려 TYPE_ACCESS 발행)      ▼
                                                                 BLOCKED (재시도 안 함, 경고 로그)
```

- **IDLE → APPROACHING**: 계획 경로를 따라 구역 경계까지 남은 거리가 `trigger_distance_m`(기본 3 m) 이하가
  되면 새 `request_id`(1부터 증가)로 `TYPE_ACCESS`를 보낸다.
- **APPROACHING 중**: 같은 `request_id`로 `resend_period_s`(기본 1 s)마다 재전송(연장)한다. 허가
  (`GRANT_GRANTED`)가 없으면 경계 `stop_margin_m`(기본 0.5 m) 앞에서 멈추도록, 기준 감속
  `decel_mps2`(기본 1.0 m/s²)로 선속도 상한을 미리 건다(`geometry.speed_cap_for_stop`). `GRANT_QUEUED`도
  같은 취급 — 서버가 자리가 나면 이 로봇의 마지막 `request_id`로 알아서 `GRANTED`를 보낸다.
  `GRANT_REJECTED`면 `BLOCKED`로 가서 경고를 남기고 더 보내지 않는다 (그래도 정지 상한은 유지한다, 즉
  진입하지 않는다).
- **APPROACHING → INSIDE**: 허가와 상관없이 위치가 실제로 구역 사각형 안에 들어오면 전환한다 (제어가
  정지를 못 지켜도 상태는 현실을 따라간다). 안에서는 1 s마다 같은 `request_id`로 계속 연장하고, 정지
  상한은 걸지 않는다 — **lease가 지나도 멈추지 않고 빠져나간다** (서버도 반납 전까지 구역을 쥐고 있는다).
- **INSIDE → IDLE**: 위치가 구역을 벗어나면 그 즉시 `TYPE_RELEASE`를 보내고 상태를 초기화한다.
- `ZoneGrant`는 전체 로봇에 방송되므로 `robot_id`가 다르면 무시한다. 추가로, 지금 기다리는
  `request_id`와 다른 응답(늦게 온 지난 요청의 응답 등)도 무시한다 — 메시지 규약에는 없는, 장애 주입기의
  지연·재배열을 견디기 위한 구현 선택이다.

## 파라미터

`config/zone_client.yaml`. 값과 근거는 파일 안 주석에 있다.

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `robot_id` | `amr_01` | `ZoneRequest.robot_id`, `ZoneGrant` 필터링 기준 |
| `zones_file` | `''` | 구역 목록 yaml 경로. 서버(`amr_fleet_manager`)와 같은 파일을 가리켜야 한다. 비어 있으면 구역 없음 |
| `map_frame` | `map` | TF 대상 좌표계 이름. `zones.yaml`의 `area_world`도 이 좌표계 기준으로 본다 |
| `base_frame` | `base_footprint` | TF 로봇 좌표계 이름 |
| `plan_topic` | `plan` | Nav2 전역 경로 (상대 이름) |
| `zone_grants_topic` | `fleet/zone_grants` | 상대 이름 (네임스페이스 → 장애 주입기 중계본) |
| `cmd_vel_in_topic` / `cmd_vel_out_topic` | `cmd_vel_nav` / `cmd_vel` | cmd_vel 게이트 입·출력 |
| `update_rate_hz` | `10.0` | TF 조회·구역 판단 주기 |
| `trigger_distance_m` | `3.0` | 경계 앞 요청 거리 (이진, 10/5) |
| `stop_margin_m` | `0.5` | 허가 전 정지 여유 (이진, 10/5) |
| `decel_mps2` | `1.0` | 정지 속도 프로파일 기준 감속 |
| `resend_period_s` | `1.0` | 재전송(연장) 주기 |

`zone_requests_topic`(기본 `/fleet/zone_requests`)는 파라미터로 빼지 않았다 — 로봇 → 서버 토픽은 장애
주입기가 중계하지 않는 절대 토픽이라 네임스페이스와 상관없이 항상 같다 (`fleet_manager_node.py`).

## 좌표계에 대해

`zones.yaml`의 `area_world`는 Gazebo world 좌표다. 지금은 world 원점과 `map` 원점이 같다고 보고 TF
`map_frame`(기본 `map`) 좌표를 그대로 비교한다. 지도 v1(10/9 인계) 뒤 둘이 어긋나면 `map_frame` 파라미터
이름만 바꾸거나, `zones.yaml` 쪽 좌표를 실제 `map` 좌표로 다시 맞추면 된다 — 코드 변경은 필요 없다.

## 실행

```bash
ros2 launch amr_zone_client zone_client.launch.py namespace:=amr_01 robot_id:=amr_01 \
  zones_file:=/ros2_ws/src/fleet/amr_fleet_manager/config/zones.yaml
```

## 테스트

```bash
colcon test --packages-select amr_zone_client
```

- `test/test_geometry.py`, `test/test_core.py`, `test/test_zones_config.py`: ROS 비의존 pytest.
- `test/test_zone_client_launch.py`: 실제 `amr_fleet_manager`(`fleet_manager`)·`amr_fault_injector`
  (`zone_grants`만 중계, `delay_ms:=0`)를 띄우고 TF·`Path`를 직접 흘려서 요청 → 허가 → 진입 → 반납을
  끝까지 확인한다.

## 알려진 한계

- `GRANT_REVOKED`(교착 해소용)는 메시지에는 있지만 서버(`amr_fleet_core.traffic.zones.ZoneManager`)가 아직
  만들어 보내지 않는다. 이 노드도 별도 처리를 하지 않는다 (들어오면 `grant` 상태만 갱신, 구역 안에서 받아도
  대피 로직 없음) — 서버 쪽 구현이 생기면 다시 본다.
- 경로가 그 구역을 더는 지나지 않게 다시 계획됐을 때(예: REJECTED 뒤 우회) `APPROACHING`/`BLOCKED` 상태를
  정리하는 로직은 없다. 우회 계획 자체가 이 패키지 범위 밖이라 범위에서 뺐다.
