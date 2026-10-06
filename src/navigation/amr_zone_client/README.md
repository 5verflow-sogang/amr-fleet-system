# amr_zone_client

- 담당: 이진
- 빌드 타입: ament_python
- 책임: 로봇 측 구역 진입 처리 (`ZoneRequest`/`ZoneGrant` 클라이언트). `amr_fleet_manager`의 구역 허가 서버(#57,
  `amr_fleet_core.traffic`)와 짝이다.

## 제공 인터페이스

| 종류 | 이름 | 타입 | 설명 |
| --- | --- | --- | --- |
| Publish | `/fleet/zone_requests` (파라미터 `zone_requests_topic`) | `amr_msgs/msg/ZoneRequest` | 진입 요청·연장·반납. 중계되지 않는 절대 토픽이라 기본값이 항상 `/fleet/zone_requests`다 (`fleet_manager_node.py`) |
| Subscribe | `fleet/zone_grants` (파라미터 `zone_grants_topic`) | `amr_msgs/msg/ZoneGrant` | 서버 응답. 네임스페이스 안 상대 이름 → `amr_fault_injector`가 중계한 `/<robot_id>/fleet/zone_grants` |
| Subscribe | `plan` (파라미터 `plan_topic`) | `nav_msgs/msg/Path` | Nav2 전역 계획 경로. 로봇 현재 위치 기준으로 다시 계산된 것을 그대로 쓴다 |
| Subscribe | `cmd_vel_nav` (파라미터 `cmd_vel_in_topic`) | `geometry_msgs/msg/Twist` | 게이팅 전 속도 명령 (Nav2 controller_server, 향후 velocity_smoother의 출력) |
| Publish | `cmd_vel_zone_limited` (파라미터 `cmd_vel_out_topic`) | `geometry_msgs/msg/Twist` | 게이팅 후 속도 명령. 허가 전엔 선속도만 깎는다, 각속도는 그대로. **로봇의 최종 `cmd_vel`이 아니다** — 'cmd_vel 체인' 절 참고 |

전체 API 명세는 [docs/api.md](../../../docs/api.md)에 반영한다.

### 노드

| 실행 파일 | 노드 이름 | 역할 |
| --- | --- | --- |
| `zone_client_node` | `zone_client` | 경로·위치 → 구역 진입 판단 → `ZoneRequest` 발행, `cmd_vel` 게이팅 |

## 동작 규칙 (상태 흐름)

구역마다 독립된 상태 기계를 둔다 (두 구역을 연달아 지나도 서로 안 엮인다).

| 지금 상태 | 조건 | 다음 상태 | 보내는 것 |
| --- | --- | --- | --- |
| `IDLE` | 남은 거리 ≤ `trigger_distance_m` | `APPROACHING` | `TYPE_ACCESS` (새 `request_id`) |
| `APPROACHING` | 위치가 구역 안 **and** 유효한 `GRANTED` | `INSIDE` | — |
| `APPROACHING` | `GRANT_REJECTED` 응답 | `BLOCKED` | — |
| `APPROACHING` | 경로에서 구역이 빠짐 **and** 위치가 구역 밖 | `IDLE` | `TYPE_RELEASE` |
| `BLOCKED` | `retry_period_s` 경과, 경로에 구역이 남아 있음 | `APPROACHING` | `TYPE_ACCESS` (새 `request_id`) |
| `BLOCKED` | 경로에서 구역이 빠짐 **and** 위치가 구역 밖 | `IDLE` | (반납 없음 — 서버가 쥔 적이 없다) |
| `INSIDE` | 위치가 구역 밖으로 나감 | `IDLE` | `TYPE_RELEASE` |

표에 없는 조합(예: `APPROACHING`에서 유효한 허가 없이 경계를 넘었거나, 구역 밖에서 lease가 지난
경우)은 상태를 그대로 유지하면서 같은 `request_id`로 계속 요청하고 정지 상한을 다시 건다 — 아래
설명 참고.

- **IDLE → APPROACHING**: 계획 경로를 따라(로봇과 가장 가까운 경로 점부터, 뒤쪽 점은 안 본다) 구역
  경계까지 남은 거리가 `trigger_distance_m`(기본 3 m) 이하가 되면 새 `request_id`(1부터 증가)로
  `TYPE_ACCESS`를 보낸다. 아직 안 건드린 구역은 `lookahead_m`(기본 `trigger_distance_m + 2 m`) 안에서만
  찾는다 — 긴 경로를 매번 끝까지 훑지 않으려고.
- **APPROACHING 중**: 같은 `request_id`로 `resend_period_s`(기본 1 s)마다 재전송(연장)한다. **유효한
  허가**(`GRANT_GRANTED`이고 `lease_expiry`가 아직 안 지남)가 없으면 경계 `stop_margin_m`(기본 0.5 m)
  앞에서 멈추도록, 기준 감속 `decel_mps2`(기본 1.0 m/s²)로 선속도 상한을 미리 건다
  (`geometry.speed_cap_for_stop`). `GRANT_QUEUED`도 같은 취급이다. 허가 없이 경계를 넘어가 버려도
  `INSIDE`로는 안 바뀐다 — 상한이 0으로 묶인 채 같은 `request_id`로 계속 요청한다. 구역 밖에서 허가
  lease가 지나면 허가 없음으로 보고 정지 상한을 다시 건다(같은 `request_id` 유지). `GRANT_REJECTED`면
  `BLOCKED`로 간다.
- **APPROACHING → INSIDE**: 위치가 구역 사각형 안이고 **그 순간 유효한 허가**(`GRANT_GRANTED`이고
  lease 안 지남)가 있을 때만 전환한다. 안에서는 1 s마다 같은 `request_id`로 계속 연장하고, 정지 상한은
  걸지 않는다 — **lease가 지나도 멈추지 않고 빠져나간다** (서버도 반납 전까지 구역을 쥐고 있는다).
- **BLOCKED**: 재시도 없이 경고만 남기던 걸 바꿔서, 경로에 그 구역이 남아 있으면 `retry_period_s`
  (기본 5 s) 뒤 새 `request_id`로 다시 요청하며 `APPROACHING`으로 돌아간다 (영원히 막힌 채로 안 둔다).
  경로가 바뀌어 그 구역을 더는 지나지 않고 로봇도 구역 밖이면 반납 없이 바로 `IDLE`로 돌아간다 — REJECTED는
  서버가 자리를 쥔 적이 없어서 반납할 게 없다.
- **APPROACHING/INSIDE → IDLE (경로 변경)**: 전역 경로가 바뀌어 대기·통과 허가 중인 구역이 경로에서
  빠지고 로봇도 그 구역 밖이면, 서버가 자리를 계속 쥐고 있지 않도록 `TYPE_RELEASE`를 보내고 `IDLE`로
  돌아간다.
- **INSIDE → IDLE**: 위치가 구역을 벗어나면 그 즉시 `TYPE_RELEASE`를 보내고 상태를 초기화한다.
- `ZoneGrant`는 전체 로봇에 방송되므로 `robot_id`가 다르면 무시한다. 추가로, 지금 기다리는
  `request_id`와 다른 응답(늦게 온 지난 요청의 응답 등)도 무시한다 — 메시지 규약에는 없는, 장애 주입기의
  지연·재배열을 견디기 위한 구현 선택이다.
- TF(`map_frame`→`base_frame`) 조회에 실패하면 fail-closed로 속도 상한을 0으로 묶는다(경고 로그는
  5초 throttle).

## 파라미터

`config/zone_client.yaml`. 값과 근거는 파일 안 주석에 있다.

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `robot_id` | 노드 네임스페이스(`/amr_03` → `amr_03`, 없으면 `amr_01`) | `ZoneRequest.robot_id`, `ZoneGrant` 필터링 기준. 파라미터로 주면 그 값이 우선한다 |
| `zones_file` | `''` | 구역 목록 yaml 경로. 서버(`amr_fleet_manager`)와 같은 파일을 가리켜야 한다. 비어 있으면 구역 없음 |
| `map_frame` | `map` | TF 대상 좌표계 이름. `zones.yaml`의 `area_world`도 이 좌표계 기준으로 본다 |
| `base_frame` | `base_footprint` | TF 로봇 좌표계 이름 |
| `plan_topic` | `plan` | Nav2 전역 경로 (상대 이름) |
| `zone_grants_topic` | `fleet/zone_grants` | 상대 이름 (네임스페이스 → 장애 주입기 중계본) |
| `cmd_vel_in_topic` / `cmd_vel_out_topic` | `cmd_vel_nav` / `cmd_vel_zone_limited` | cmd_vel 게이트 입·출력 ('cmd_vel 체인' 절 참고) |
| `update_rate_hz` | `10.0` | TF 조회·구역 판단 주기 |
| `trigger_distance_m` | `3.0` | 경계 앞 요청 거리 (이진, 10/5) |
| `stop_margin_m` | `0.5` | 허가 전 정지 여유 (이진, 10/5) |
| `decel_mps2` | `1.0` | 정지 속도 프로파일 기준 감속 |
| `resend_period_s` | `1.0` | 재전송(연장) 주기 |
| `retry_period_s` | `5.0` | `BLOCKED`(REJECTED) 뒤 재시도까지 기다리는 시간 |
| `lookahead_m` | `0.0` (= `trigger_distance_m + 2.0`) | 아직 안 건드린 구역을 로봇 앞 이 거리까지만 찾는다. `0.0`이면 `trigger_distance_m`에 여유를 더해 자동으로 정한다 |
| `zone_requests_topic` | `/fleet/zone_requests` | 로봇 → 서버 토픽. 다른 파라미터처럼 `declare_parameter`로 선언돼 있지만, 장애 주입기가 중계하지 않는 절대 토픽이라 네임스페이스와 상관없이 항상 같은 기본값을 쓴다(`fleet_manager_node.py`) — 보통 바꿀 일이 없다 |

## cmd_vel 체인

안전 정지는 안전 노드(`amr_safety`, 연우)가 `cmd_vel`을 가로채는 방식으로 두고, 제어 쪽은
`SafetyStatus.speed_limit`을 참고해 부드럽게 감속하는 부가 역할만 하기로 합의했다 (9/28 회의,
[docs/project-spec.md](../../../docs/project-spec.md) '안전 정지 책임' 절, [docs/api.md](../../../docs/api.md)
4-4절 "감속·정지를 누가 실행하나"). 구역 진입 제한도 같은 성격의 하드 제약이라, 이 노드가 최종 `cmd_vel`을
직접 내보내면 안전 노드와 발행자가 둘이 된다.

그래서 이 노드의 출력은 `cmd_vel`이 아니라 `cmd_vel_zone_limited`다. 의도한 체인은:

```
controller_server (Nav2) → cmd_vel_nav → [velocity_smoother, 아직 없음] → amr_zone_client → cmd_vel_zone_limited → amr_safety (아직 없음) → cmd_vel (로봇)
```

`amr_bringup`의 launch 조립과 `amr_safety` 구현이 둘 다 아직 없어서(스텁) 지금은 이 체인을 끝까지
못 맞춰 본다. 그 전까지 시뮬레이션에서 단독으로 쓰려면 `amr_bringup`이 `cmd_vel_zone_limited`를
로봇 `cmd_vel`로 임시 리맵하면 된다 — 다만 `amr_safety`가 들어오면 그 리맵은 지워야 한다.

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
