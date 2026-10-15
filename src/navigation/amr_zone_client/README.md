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
| Subscribe | `safety/status` (파라미터 `safety_status_topic`) | `amr_msgs/msg/SafetyStatus` | `speed_limit`을 구역 상한과 함께 선속도 상한에 반영 ('SafetyStatus 연동' 절 참고) |
| Publish | `cmd_vel_zone_limited` (파라미터 `cmd_vel_out_topic`) | `geometry_msgs/msg/Twist` | 게이팅 후 속도 명령. 허가 전엔 선속도만 깎는다, 각속도는 그대로. `BACKING` 중에는 `cmd_vel_nav`를 무시하고 직접 낸 후진 속도다. **로봇의 최종 `cmd_vel`이 아니다** — 'cmd_vel 체인' 절 참고 |
| Publish | `zone_client/state` | `std_msgs/msg/String` | 요약 상태(`idle`/`approaching`/`waiting`/`blocked`/`inside`/`backing`/`failed`). 바뀌면 즉시, 그 외 1 Hz. 우선순위·`waiting` 판정은 '상태 토픽(zone_client/state)' 절 참고 |

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
| `APPROACHING` | 경로에서 구역이 빠짐 **and** 위치가 구역 밖 **and** (빠진 지 `drop_grace_s` 초과 **or** 직선 거리 > `trigger_distance_m`) | `IDLE` | `TYPE_RELEASE` |
| `BLOCKED` | `retry_period_s` 경과, 경로에 구역이 남아 있음 | `APPROACHING` | `TYPE_ACCESS` (새 `request_id`) |
| `BLOCKED` | 경로에서 구역이 빠짐 **and** 위치가 구역 밖 **and** (빠진 지 `drop_grace_s` 초과 **or** 직선 거리 > `trigger_distance_m`) | `IDLE` | (반납 없음 — 서버가 쥔 적이 없다) |
| `INSIDE` | 위치가 구역 밖으로 나감 | `IDLE` | `TYPE_RELEASE` |
| `APPROACHING` | `GRANT_REVOKED` 응답(교착 해소), 응답 시점에 아직 구역 밖 | `BLOCKED` | `TYPE_RELEASE` (다음 틱) |
| `APPROACHING` | `GRANT_REVOKED` 응답(교착 해소), 응답 시점에 이미 구역 안 | `BACKING` | — |
| `INSIDE` | `GRANT_REVOKED` 응답(교착 해소) | `BACKING` | — |
| `BACKING` | 구역 밖 + `backing_margin_m` 이상 후진 | `IDLE` | `TYPE_RELEASE` |
| `BACKING` | `backing_max_distance_m`·`backing_timeout_s` 중 하나라도 초과 | `FAILED` | (없음 — 반납하지 않는다) |

표에 없는 조합(예: `APPROACHING`에서 유효한 허가 없이 경계를 넘었거나, 구역 밖에서 lease가 지난
경우, 경로에서 빠졌지만 아직 grace 안인 경우)은 상태를 그대로 유지하면서 같은 `request_id`로 계속 요청하고 정지 상한을 다시 건다 — 아래
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
  경로가 바뀌어 그 구역을 더는 지나지 않으면 아래 "경로에서 빠진 구역" 규칙을 따르고, 끝내 `IDLE`로 갈 때는
  반납 없이 간다 — REJECTED는 서버가 자리를 쥔 적이 없어서 반납할 게 없다. 경로에서 빠져 있는 동안은
  재시도하지 않는다.
- **경로에서 빠진 구역 (APPROACHING·BLOCKED, 10/15 하연·이진)**: #66에서 정한 "경로에서 빠지면 즉시 반납"을
  바꿨다. 전역 경로가 바뀌어 아직 들어가지 않은 구역이 경로에서 빠져도, 로봇이 그 구역에서
  `trigger_distance_m`(직선 거리) 안에 있으면 바로 반납하지 않는다 — 같은 `request_id`로 계속 요청해 대기열
  순서를 지키고, 정지 상한은 경로 거리 대신 로봇 → 구역 사각형 **직선 거리**(경로 거리보다 짧아 보수적)로
  건다. 경로가 `drop_grace_s`(기본 3 s) 넘게 계속 빠져 있거나 로봇이 `trigger_distance_m` 밖으로 멀어지면
  그때 `TYPE_RELEASE`(BLOCKED는 반납 없이)를 보내고 `IDLE`로 돌아간다. 그 전에 경로가 다시 그 구역을
  지나면 그대로 이어간다(새 `request_id`를 쓰지 않는다). 교착 중 상대 로봇이 통로를 막자 Nav2가 잠깐 우회
  경로를 냈고, 그 사이 정지 상한이 사라져 허가 없이 구역에 들어간 문제(fleet_deadlock 실험) 때문이다.
  `INSIDE` 구역은 그대로다(아래).
- **INSIDE → IDLE**: 위치가 구역을 벗어나면 그 즉시 `TYPE_RELEASE`를 보내고 상태를 초기화한다.
- **`GRANT_REVOKED`(교착 해소, 10/11 결정)**: 서버가 교착을 풀려고 우선순위 낮은 로봇의 허가를
  회수할 때 쓴다(`REASON_DEADLOCK_RESOLUTION`).
  - `APPROACHING`인데 **응답 시점에 아직 구역 밖**이면 들어가지 않고 `BLOCKED`로 간다 — `REJECTED`와
    달리 이미 허가를 쥐고 있었으므로 다음 틱에 `TYPE_RELEASE`를 한 번 보낸 뒤, `BLOCKED`와 같은
    규칙(`retry_period_s` 뒤 새 `request_id`로 재시도)을 따른다.
  - `APPROACHING`인데 **응답 시점에 이미 구역 안**이면(요청을 보내고 응답이 오기까지 한 틱 사이에
    경계를 넘어선 경우) `BLOCKED`가 아니라 `BACKING`으로 간다(#137 하연 두 번째 리뷰) — 반납하고
    재시도해도 이미 구역 안에 있어서 의미가 없다. "이미 구역 안"인지는 가장 최근 `update()` 틱의
    위치로 판단한다(`on_grant`는 위치를 따로 받지 않는다).
  - `INSIDE`(이미 구역 안)면 `BACKING`으로 가서 **`cmd_vel_nav`를 완전히 무시**하고 `cmd_vel_zone_limited`로
    직접 후진 명령을 낸다 — 아래 'BACKING(교착 해소)' 절 참고.
  - **다른 구역 잠금(#137 하연 두 번째 리뷰)**: 위 세 경우 모두, 그 구역이 복구를 마치고 다시
    `IDLE`로 돌아갈 때까지 **다른 구역**은 전부 건드리지 않는다 — 대기 중이던 요청(`APPROACHING`·
    `BLOCKED`·`INSIDE`)은 그 즉시 반납(`TYPE_RELEASE`)하고 `IDLE`로 돌리며, `IDLE`인 구역은 새로
    트리거하지 않는다. 교착 해소 하나에 집중하고, 복구가 끝나면(그 구역이 `IDLE`로 돌아오면) 평소처럼
    다른 구역도 다시 본다.
- `ZoneGrant`는 전체 로봇에 방송되므로 `robot_id`가 다르면 무시한다. 추가로, 지금 기다리는
  `request_id`와 다른 응답(늦게 온 지난 요청의 응답 등)도 무시한다 — 메시지 규약에는 없는, 장애 주입기의
  지연·재배열을 견디기 위한 구현 선택이다.
- TF(`map_frame`→`base_frame`) 조회에 실패하면 fail-closed로 속도 상한을 0으로 묶는다(경고 로그는
  5초 throttle). `BACKING` 중에도 같은 규칙이다 — 아래 'BACKING(교착 해소)' 절 참고.

### 상태 토픽(zone_client/state)

`zone_client/state`는 구역마다 따로 도는 상태 기계를 로봇 하나의 요약 문자열 하나로 합친 것이다.
`waiting`은 `Phase`가 아니라 `APPROACHING` 중 일부만 가리키는 라벨이다 — **허가가 없고 경계까지
남은 거리가 `stop_margin_m + waiting_margin_m`(기본 0.5 + 0.15 = 0.65 m) 이하**일 때만 `waiting`이고,
그보다 멀면 (속도 상한만 걸린) `approaching`이다(#137 지수 리뷰 — 원래는 `trigger_distance_m`(3 m)
요청을 보내는 순간부터 바로 `waiting`으로 보였는데, 실제로는 아직 멀어서 속도만 깎이는 중이라
"멈춰 기다리는 중"이라는 라벨이 너무 일찍 뜬 것이었다). `waiting_margin_m`은 처음 0.5였는데 지수의
두 번째 리뷰로 0.15까지 더 줄였다 — `waiting`이 뜨는 지점(`speed_cap_for_stop(stop_margin_m +
waiting_margin_m, stop_margin_m, decel_mps2)`)의 속도 상한이 거의 0(≈0.55 m/s, 기존 1.0 m/s보다
훨씬 낮다)이어야 "멈춰 기다리는 중"이라는 라벨에 맞다고 봤다.

구역이 여러 개면(두 구역이 겹치는 구간 등) 가장 "급한"(멈추거나 멈춰 있는) 것 하나로 요약하고,
우선순위는 다음과 같다(`core.ZoneClientCore._STATE_PRIORITY`):

```
failed > backing > blocked > waiting > inside > approaching > idle
```

예: 구역 A에서 `inside`이고 동시에 구역 B에서 `waiting`이면 전체 상태는 `waiting`이다 — 로봇이
실제로 멈춰서 기다리는 쪽을 먼저 보여준다.

### BACKING(교착 해소)

`INSIDE`에서 `GRANT_REVOKED`를 받으면 `cmd_vel_nav`를 무시하고 `backing_speed_mps`(기본 0.15 m/s)로
후진해 구역을 벗어난다. `decel_mps2`로 가속·감속한다(`geometry.ramp_speed`) — 구역 경계를 벗어난 뒤
`backing_margin_m`(기본 0.3 m)만큼 남았을 때부터 같은 `decel_mps2`로 감속해 정확히 그 지점에서 0이
된다(`speed_cap_for_stop`을 "남은 후진 거리"에 재사용). 그 지점에 닿으면 정지 → `TYPE_RELEASE` →
`IDLE`로 돌아가고, 그 뒤로는 평소처럼(새 `request_id`로) 다시 접근한다.

**재전송(#137 하연 두 번째 리뷰)**: `BACKING`·`FAILED` 중에도 `resend_period_s`(기본 1 s)마다 같은
`request_id`로 `TYPE_ACCESS`를 계속 보낸다 — 서버가 `last_seen`을 갱신해 구역을 다른 로봇에게 넘기지
않게 한다. 서버는 어차피 같은 `GRANT_REVOKED`로 답할 뿐이고, 그 응답은 `on_grant`가 `phase`
(`BACKING`·`FAILED`는 `APPROACHING`/`INSIDE`가 아니다)로 걸러 그냥 무시하므로 상태·속도에는
아무 영향이 없다.

안전장치(fail-closed): `backing_max_distance_m`(총 후진 거리, 기본 6 m) 또는 `backing_timeout_s`
(경과 시간, 기본 45 s)를 넘으면 그 순간 속도를 0으로 묶고 ERROR 로그를 한 번 남긴 뒤 `FAILED`로
바뀐다(`zone_client/state`에 `failed`로 드러난다, #137 지수 리뷰). 그 뒤로는 반납·재시도 없이 그대로
멈춰 선다(위 재전송은 계속한다) — 계속 후진하다 더 큰 사고로 이어지는 것보다, 서버가 자리를 계속
쥐고 있더라도 멈춰서 수동 확인을 받는 쪽이 안전하다. 두 기본값은 `zones.yaml`에서 가장 긴 구역
(`narrow_02`, 길이 5.0 m)을 기준으로 뒀다(#137 하연 두 번째 리뷰) — 거리는 그 길이를 다 빠져나오고
`backing_margin_m`(0.3 m)까지 나갈 여유를 더한 6.0 m, 시간은 그 길이를 `backing_speed_mps`
(0.15 m/s)로 다 빠져나오는 데 걸리는 시간(5.0 / 0.15 ≈ 33.3 s)에 여유를 더한 45 s다. 위 재전송
덕분에 더는 서버 `release_timeout_s`(`amr_fleet_manager.yaml` 기본 30 s)보다 작아야 할 필요가
없다 — 재전송이 `last_seen`을 계속 갱신하므로 서버가 "반납이 안 온다"고 구역을 넘겨버릴 일이 없다.

TF(`map_frame`→`base_frame`) 조회 실패는 평소 규칙과 같다 — `BACKING` 중에도 그 순간 0을 낸다(`core`
상태는 그대로 두고 그 틱만 건너뛴다 — TF가 돌아오면 이어서 재개한다).

`safety/status`와의 관계(#137 지수 리뷰): 후진 속도도 `speed_limit`에 걸린다(크기 `|v|` 기준) —
뒤가 막혀 안전 노드가 멈추라고 하면(`CRITICAL`, `speed_limit` 0) 후진도 `decel_mps2`로 부드럽게
멈춘다. **뒤가 막혀 안전 노드가 멈추면 `backing_timeout_s`까지 기다렸다가 `FAILED`가 된다** — 계속
막혀 있으면 영원히 멈춰 있지 않고 결국 `failed`로 드러나 수동 확인을 요청한다. 제한이 풀리면 같은
`decel_mps2`로 다시 `backing_speed_mps`까지 올라간다(급발진하지 않는다).

현재 서버(`amr_fleet_core.traffic.zones.ZoneManager`)는 아직 교착을 **감지**하지 않는다(대기 그래프
순환 탐지는 미구현, `amr_fleet_core/README.md`) — `GRANT_REVOKED`를 실제로 보내는 경로가 생기기 전까지는
이 로직을 켤 상황 자체가 안 생긴다. 서버 쪽이 구현되면 이 클라이언트 처리는 바로 맞물린다.

## 파라미터

`config/zone_client.yaml`. 값과 근거는 파일 안 주석에 있다.

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `robot_id` | 노드 네임스페이스(`/amr_03` → `amr_03`, 없으면 `amr_01`) | `ZoneRequest.robot_id`, `ZoneGrant` 필터링 기준. 파라미터로 주면 그 값이 우선한다 |
| `zones_file` | `''` | 구역 목록 yaml 경로. 서버(`amr_fleet_manager`)와 같은 파일을 가리켜야 한다. 비어 있으면 구역 없음 |
| `map_frame` | `map` | TF 대상 좌표계 이름 |
| `zones_frame` | `''` | `zones.yaml` `area_world`의 좌표계. `world`면 TF `map ← world`로 바꿔 쓴다(시뮬레이션). 비우면 `map_frame` 그대로 (아래 "좌표계에 대해") |
| `base_frame` | `base_footprint` | TF 로봇 좌표계 이름 |
| `plan_topic` | `plan` | Nav2 전역 경로 (상대 이름) |
| `zone_grants_topic` | `fleet/zone_grants` | 상대 이름 (네임스페이스 → 장애 주입기 중계본) |
| `cmd_vel_in_topic` / `cmd_vel_out_topic` | `cmd_vel_nav` / `cmd_vel_zone_limited` | cmd_vel 게이트 입·출력 ('cmd_vel 체인' 절 참고) |
| `update_rate_hz` | `10.0` | TF 조회·구역 판단 주기 |
| `trigger_distance_m` | `3.0` | 경계 앞 요청 거리 (이진, 10/5) |
| `stop_margin_m` | `0.5` | 허가 전 정지 여유 (이진, 10/5) |
| `waiting_margin_m` | `0.15` | `zone_client/state`의 `waiting` 판정 여유. 경계까지 `stop_margin_m + waiting_margin_m`(0.65 m) 이내일 때만 `waiting`, 그 전엔 `approaching` (#137 지수 리뷰, 두 번째 리뷰로 0.5 → 0.15) |
| `decel_mps2` | `1.0` | 정지 속도 프로파일 기준 감속 |
| `resend_period_s` | `1.0` | 재전송(연장) 주기 |
| `retry_period_s` | `5.0` | `BLOCKED`(REJECTED·REVOKED) 뒤 재시도까지 기다리는 시간 |
| `drop_grace_s` | `3.0` | 아직 안 들어간 구역이 경로에서 빠진 채 이 시간이 지나면 반납·`IDLE` (그 전엔 trigger 거리 안이면 요청·정지 상한 유지) |
| `on_path_confirm_s` | `0.3` | 경로에서 빠진 구역이 다시 잡혀도 이 시간 동안 끊기지 않아야 `drop_grace_s` 카운트를 취소 (10/16 fleet_deadlock 재현 - 재계획 주기로 흔들리면 매번 취소돼 무기한 붙들 수 있었다) |
| `backing_speed_mps` | `0.15` | `BACKING` 후진 속도 (10/11 결정) |
| `backing_margin_m` | `0.3` | 구역 경계를 벗어난 뒤 이만큼 더 나가면 정지·반납 |
| `backing_max_distance_m` | `6.0` | `BACKING` 안전장치: 총 후진 거리 한도(fail-closed). 가장 긴 구역(`narrow_02`, 5.0 m) + `backing_margin_m` 여유 (#137 하연 두 번째 리뷰) |
| `backing_timeout_s` | `45.0` | `BACKING` 안전장치: 경과 시간 한도(fail-closed). 가장 긴 구역을 `backing_speed_mps`로 빠져나오는 시간(≈33.3 s) + 여유 (#137 하연 두 번째 리뷰). `BACKING` 중 재전송 덕분에 서버 `release_timeout_s`보다 작아야 할 필요는 없다 |
| `lookahead_m` | `0.0` (= `trigger_distance_m + 2.0`) | 아직 안 건드린 구역을 로봇 앞 이 거리까지만 찾는다. `0.0`이면 `trigger_distance_m`에 여유를 더해 자동으로 정한다 |
| `zone_requests_topic` | `/fleet/zone_requests` | 로봇 → 서버 토픽. 다른 파라미터처럼 `declare_parameter`로 선언돼 있지만, 장애 주입기가 중계하지 않는 절대 토픽이라 네임스페이스와 상관없이 항상 같은 기본값을 쓴다(`fleet_manager_node.py`) — 보통 바꿀 일이 없다 |
| `safety_status_topic` | `safety/status` | `SafetyStatus` 상대 이름 ('SafetyStatus 연동' 절 참고) |
| `safety_status_timeout_s` | `1.0` | 이 시간 동안 메시지를 못 받으면 끊김으로 보고 `speed_limit` 제한을 없앤다 |
| `max_linear_velocity_mps` | `2.0` | `speed_limit`이 낮아질 때 램프의 "제한 없음" 기준(`geometry.ramp_cap_down`). `nav2_params.yaml` `FollowPath.max_vel_x`·URDF `max_linear_velocity`와 동일 |

## cmd_vel 체인

안전 정지는 안전 노드(`amr_safety`, 연우)가 `cmd_vel`을 가로채는 방식으로 두고, 제어 쪽은
`SafetyStatus.speed_limit`을 참고해 부드럽게 감속하는 부가 역할만 하기로 합의했다 (9/28 회의,
[docs/project-spec.md](../../../docs/project-spec.md) '안전 정지 책임' 절, [docs/api.md](../../../docs/api.md)
4-4절 "감속·정지를 누가 실행하나"). 이 노드가 모든 컨트롤러 출력이 거치는 자리라 `speed_limit`
반영도 여기서 한다 (아래 'SafetyStatus 연동' 절, [docs/algorithms/navigation.md](../../../docs/algorithms/navigation.md)
'5-6. speed_limit 연동'). 구역 진입 제한도 같은 성격의 하드 제약이라, 이 노드가 최종 `cmd_vel`을
직접 내보내면 안전 노드와 발행자가 둘이 된다.

그래서 이 노드의 출력은 `cmd_vel`이 아니라 `cmd_vel_zone_limited`다. 의도한 체인은:

```
controller_server (Nav2) → cmd_vel_nav → [velocity_smoother, 아직 없음] → amr_zone_client → cmd_vel_zone_limited → amr_safety (아직 없음) → cmd_vel (로봇)
```

`amr_bringup`의 launch 조립과 `amr_safety` 구현이 둘 다 아직 없어서(스텁) 지금은 이 체인을 끝까지
못 맞춰 본다. 그 전까지 시뮬레이션에서 단독으로 쓰려면 `amr_bringup`이 `cmd_vel_zone_limited`를
로봇 `cmd_vel`로 임시 리맵하면 된다 — 다만 `amr_safety`가 들어오면 그 리맵은 지워야 한다.

## SafetyStatus 연동

구역 상한 외에도 `safety/status`(`amr_msgs/msg/SafetyStatus`)의 `speed_limit`을 받아 **구역
상한과 `speed_limit` 중 더 작은 쪽**을 cmd_vel 선속도 상한으로 쓴다(#69·#70). 적용 위치를
`amr_zone_client`로 둔 이유는 Nav2 컨트롤러가 무엇이든(DWA·Pure Pursuit 등) 결국 여기를
거쳐 `cmd_vel_zone_limited`로 나가기 때문이다 — 컨트롤러마다 따로 `speed_limit`을 챙기지
않아도 된다. 값 정의(`inf`/`0.6`/`0.0` 등)는 [docs/api.md](../../../docs/api.md)를 참고한다.

- `speed_limit`이 **낮아질 때만** `decel_mps2`(구역 진입 감속과 같은 파라미터) 이내로 부드럽게
  따라간다(`geometry.ramp_cap_down`). 높아지거나(완화) 끊김에서 복구될 때는 바로 반영한다 —
  완화는 안전 쪽이라 부드럽게 할 필요가 없다.
- `safety_status_timeout_s`(기본 1.0 s) 동안 메시지를 못 받으면(한 번도 못 받았을 때 포함)
  **제한 없음**으로 본다. 최종 게이트가 `amr_safety`라서다 — 이 노드가 보수적으로 막지 않아도
  안전 노드가 끝에서 한 번 더 막는다.

## 좌표계에 대해

`zones.yaml`의 `area_world`는 Gazebo world 좌표다. 그런데 `map` 원점은 world (0, 1.5)라(지도 매핑 시작 자세),
`area_world`를 `map` 좌표로 그대로 비교하면 구역이 1.5 m 북쪽(통로 옆 랙 위)에 있는 것으로 보여 로봇이 실제 통로
앞에서 허가를 요청하지 않는다 (10/13 발견).

- `zones_frame:=world`이면 시작할 때 TF `map ← world`를 조회해 구역을 `map` 좌표로 바꾼 뒤 판단한다.
  TF는 `amr_bringup system.launch.py`가 지도 원점으로 한 번 발행한다(`world → map` 정적 TF). 원점 값을 이 노드가
  따로 들고 있지 않아 지도를 바꿔도 여기는 고칠 게 없다.
- TF가 오기 전에는 구역이 어디인지 모르므로 위치를 모를 때와 같이 속도 상한 0으로 멈춘다 (fail-closed).
- `zones_frame`을 비우면(기본) `map` 좌표로 그대로 본다. TF 없는 단위·launch 테스트용이다.

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

- `GRANT_REVOKED`(교착 해소) 처리는 이 노드에 구현돼 있다(10/11 결정, 'BACKING(교착 해소)' 절) - 하지만
  서버(`amr_fleet_core.traffic.zones.ZoneManager`)는 아직 교착을 **감지**하지 않아(대기 그래프 순환
  탐지 미구현, `amr_fleet_core/README.md`) 실제로 `GRANT_REVOKED`를 보내는 경로가 없다. `test_core.py`의
  단위 테스트로만 확인했고, 서버가 실제로 보내는 launch 통합 테스트는 서버 쪽 구현이 생긴 뒤에 추가한다.
- BT(`amr_bt`, 지수)가 `BACKING`·`FAILED` 중 자기 상태(이동 중 등)를 어떻게 다룰지는 아직 정해지지
  않았다 - `zone_client/state`가 `backing`이면 로봇이 자기 의지와 무관하게 후진 중이라는 뜻이고,
  `failed`면 안전 한도를 넘어 멈춰 선 채 수동 확인을 기다리는 중(반납·재시도 없음)이라는 뜻이라,
  BT 쪽에서도 참고할 수 있다.
- `backing_margin_m`(0.3 m)은 구역 경계에서 딱 그만큼만 나가고 멈춘다 - 로봇 앞끝(풋프린트)이
  경계에 거의 닿은 채로 서 있을 수 있어서, 양보받은 로봇(`holder_robot_id`)이 지나가는 길을 다시
  막는 경우가 생길 수 있다. margin을 키울지 BT가 옆으로 비켜서게 할지는 10/12 회의에서 정하기로
  했다(#137 하연 두 번째 리뷰 지적 - 아직 `backing_margin_m` 자체는 그대로 뒀다).
