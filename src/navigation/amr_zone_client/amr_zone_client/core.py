"""로봇 측 구역 진입 처리 (ZoneRequest/ZoneGrant). ROS 비의존. rclpy·amr_msgs를 import하지 않는다.

규칙 (이진, 10/5, zones.yaml 주석과 같다. #66 하연 리뷰로 보강):
- 경로 기준 구역 경계 3 m 앞에서 TYPE_ACCESS 요청. 유효한 허가(GRANTED이고 lease가 안 지남) 전에는
  경계 0.5 m 앞에 정지하도록 기준 감속 1.0 m/s²로 미리 감속한다 (speed_cap_for_stop). 허가 없이
  경계를 넘어가 버려도 INSIDE로는 안 바뀐다 - 속도 상한 0을 유지하고 같은 request_id로 계속 요청한다.
- 새 진입마다 request_id를 1 늘린다. 같은 진입의 재전송·연장(대기 중·구역 안)은 같은 request_id를 쓴다.
- 대기(QUEUED) 중에는 계속 기다린다. 서버가 자리가 나면 이 로봇의 마지막 request_id로 GRANTED를 보낸다.
- 구역 밖에서 허가(lease)가 지나면 허가 없음으로 보고 정지선 감속을 다시 건다(같은 request_id로 계속
  요청). 구역 안에서 지나면 지금처럼 멈추지 않고 빠져나간 뒤 반납한다(서버도 반납 전까지 구역을
  쥐고 있는다, amr_fleet_core.traffic.zones).
- REJECTED면 당장은 진입하지 않고 경고를 남긴다. 경로에 그 구역이 남아 있으면 retry_period_s 뒤
  새 request_id로 다시 요청한다(영원히 막힌 채로 두지 않는다). 경로가 바뀌어 그 구역을 더는 지나지
  않으면 IDLE로 돌아간다.
- REVOKED(교착 해소, 10/11 결정)면: APPROACHING(아직 구역 밖)이면 들어가지 않고 그대로 정지점에
  멈춘 채 반납하고, BLOCKED처럼 retry_period_s 뒤 새 request_id로 재시도한다. 단 응답이 오기까지
  한 틱 사이에 이미 경계를 넘어섰으면(APPROACHING인데 실제로는 구역 안) BLOCKED 대신 BACKING으로
  간다(#137 하연 두 번째 리뷰 - 반납하고 재시도해도 이미 안에 있으니 의미가 없다). INSIDE(이미
  구역 안)면 BACKING으로 가서 cmd_vel_nav를 무시하고 직접 후진 명령을 낸다 - 아래 'BACKING' 참고.
  둘 중 어느 쪽이든, 그 구역이 복구를 마치고 IDLE로 돌아갈 때까지 다른 구역의 대기 중 요청은
  반납하고 새 요청도 보내지 않는다(#137 하연 두 번째 리뷰 - 교착 해소에 집중한다).
- 전역 경로가 바뀌어 아직 들어가지 않은 구역(APPROACHING·BLOCKED)이 경로에서 빠져도, 로봇이 그 구역에서
  trigger_distance_m(직선 거리) 안에 있으면 바로 반납하지 않는다(10/15 하연·이진, #66의 "빠지면 즉시
  반납"을 바꿈 - fleet_deadlock에서 교착 상대가 통로를 막자 Nav2가 잠깐 우회 경로를 냈고, 그 사이 정지
  상한이 사라져 허가 없이 구역에 들어갔다). 그동안은 같은 request_id로 계속 요청해 대기열 순서를 지키고,
  정지 상한은 경로 거리 대신 로봇 → 구역 사각형 직선 거리(보수적)로 계산한다. 경로가 drop_grace_s 넘게
  계속 빠져 있거나 로봇이 trigger_distance_m 밖으로 멀어지면 그때 반납(TYPE_RELEASE, BLOCKED는 쥔 자리가
  없으니 반납 없이)하고 IDLE로 돌아간다. 그 전에 경로가 다시 그 구역을 지나면 그대로 이어간다.
  INSIDE 구역은 그대로다(구역을 벗어나면 반납).
- 남은 거리는 경로의 시작점이 아니라 로봇과 가장 가까운 경로 점부터 잰다(경로가 로봇 뒤쪽 점도
  담고 있을 수 있어서 - geometry.remaining_distance_to_rect).
- ZoneGrant는 모든 로봇에 방송되므로 robot_id로 거른다 (on_grant).

속도 상한 병합(#69·#70): 구역 상한과 SafetyStatus.speed_limit 중 더 작은 쪽을 쓴다(둘 다
없으면 제한 없음). speed_limit이 낮아질 때는 decel_mps2 이내로 부드럽게 따라간다(구역
진입 감속과 같은 방식) - update()의 safety_speed_limit_mps 인자, ramp_cap_down 참고.

BACKING(교착 해소, 10/11 결정): 서버가 교착을 풀려고 INSIDE 로봇의 허가를 회수(GRANT_REVOKED,
REASON_DEADLOCK_RESOLUTION)하면 그 구역에서 후진해 나온다.
- cmd_vel_nav는 완전히 무시한다(진행 방향이 틀릴 수 있어서) - update()가 backing_linear_x(음수,
  후진)를 돌려주면 노드는 그 값을 그대로 cmd_vel_zone_limited로 낸다.
- 속도는 backing_speed_mps까지 decel_mps2 이내로 가속하고(ramp_speed), 구역 경계를 벗어난 뒤
  backing_margin_m 남았을 때부터 같은 decel_mps2로 감속해 margin에서 정확히 0이 된다
  (speed_cap_for_stop을 "남은 후진 거리"에 그대로 재사용).
- margin까지 나가면 정지 → RELEASE → IDLE (그 뒤로는 평소처럼 다시 접근하면 새 request_id로 요청).
- safety/status가 CRITICAL(speed_limit 0)이면 후진 목표 속도도 0으로 묶인다(#137 지수 리뷰, 크기
  |v| 기준) - 뒤가 막혀 안전 노드가 멈추라고 하면 후진도 멈춰야 한다. decel_mps2로 부드럽게
  멈추고, 그 상태로 backing_timeout_s를 넘으면 FAILED로 간다(계속 CRITICAL이면 영원히 멈춰
  있지 않고 결국 수동 확인을 요청하게 된다). CRITICAL이 풀리면 같은 decel_mps2로 다시
  backing_speed_mps까지 올라간다(급발진하지 않는다 - target 자체를 낮춰 ramp해서다).
- 안전장치: backing_max_distance_m(총 후진 거리) 또는 backing_timeout_s(경과 시간)를 넘으면
  그 순간 속도를 0으로 묶고 ERROR를 한 번 남긴 뒤 FAILED로 바뀐다(#137 지수 리뷰, 10/11 결정은
  BACKING에 계속 머무는 것이었으나 상태 토픽에 안전 한도 초과가 안 드러나 바꿨다) - fail-closed로,
  그 뒤로는 반납·재시도 없이 그대로 멈춰 서서 수동 확인을 기다린다(계속 후진하다 더 큰 사고로
  이어지는 것보다 안전하다).

상태 토픽(zone_client/state, #137 지수 리뷰): idle / approaching / waiting / blocked / inside /
backing / failed 7가지. waiting은 phase가 아니라 APPROACHING 중 "허가 없음 + 경계까지
stop_margin_m + waiting_margin_m 이내"일 때만의 라벨이다(trigger_distance_m 요청 순간부터
waiting으로 보이면 아직 속도 상한만 걸린 상태인데도 "멈춰 기다리는 중"처럼 보여서 좁혔다) -
그 전에는 approaching으로 보인다. 구역이 여러 개면 가장 "급한" 것 하나로 요약하는데, 우선순위는
failed > backing > blocked > waiting > inside > approaching > idle이다(state_label 참고) -
예를 들어 구역 A에 inside, 구역 B에서 waiting이면 전체 상태는 waiting이다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Sequence

from .geometry import (
    Point, Rect, combine_caps, distance, distance_to_rect, point_in_rect, ramp_cap_down, ramp_speed,
    remaining_distance_to_rect, speed_cap_for_stop,
)

# amr_msgs/msg/ZoneRequest.TYPE_* 와 같은 값
TYPE_ACCESS = 0
TYPE_RELEASE = 1

# amr_msgs/msg/ZoneGrant.GRANT_* 와 같은 값
GRANT_GRANTED = 0
GRANT_QUEUED = 1
GRANT_REJECTED = 2
GRANT_REVOKED = 3


class Phase(Enum):
    IDLE = 'idle'                  # 아직 트리거 거리 밖
    APPROACHING = 'approaching'    # 요청을 보냈고 아직 구역 밖 (대기 또는 통과 허가)
    INSIDE = 'inside'              # 구역 안, 반납 전
    BLOCKED = 'blocked'            # REJECTED·REVOKED(APPROACHING). 재시도만 한다
    BACKING = 'backing'            # REVOKED(INSIDE). 직접 후진해 구역을 벗어나는 중
    FAILED = 'failed'              # BACKING 중 안전 한도(거리·시간) 초과. 멈춰 선 채 수동 확인 대기


@dataclass(frozen=True)
class ZoneConfig:
    zone_id: str
    area: Rect


@dataclass(frozen=True)
class OutgoingRequest:
    zone_id: str
    request_id: int
    type: int


@dataclass
class _ZoneState:
    phase: Phase = Phase.IDLE
    request_id: Optional[int] = None
    grant: Optional[int] = None
    lease_expiry: Optional[float] = None
    last_sent: Optional[float] = None
    # 가장 최근 update() 틱에서 구한 경계까지 남은 거리 (state_label의 waiting 판정용, #137 지수 리뷰) -
    # remaining_distance_to_rect가 매 틱 _update_zone에 넘겨주는 값을 그대로 적어 둔다.
    last_remaining: Optional[float] = None
    # 경로에서 이 구역이 빠지기 시작한 시각 (APPROACHING·BLOCKED만, drop_grace_s 기준). 경로가 다시 이
    # 구역을 지나면 None으로 돌린다.
    off_path_since: Optional[float] = None
    # BLOCKED 전환 뒤 아직 못 보낸 RELEASE가 있는지 (REVOKED면 REJECTED와 달리 반납해야 한다) - 다음
    # update() 틱에 한 번 보내고 지운다.
    pending_release: bool = False
    # BACKING(후진) 진행 상태 - on_grant가 phase만 바꾸고, 실제 값은 _update_zone이 robot_xy를
    # 받는 첫 틱에 채운다(on_grant는 위치를 모른다).
    backing_start_xy: Optional[Point] = None    # 후진 시작 위치 (총 후진 거리 기준)
    backing_since: Optional[float] = None       # 후진 시작 시각 (backing_timeout_s 기준)
    backing_exit_xy: Optional[Point] = None      # 구역을 막 벗어난 위치 (margin 거리 기준)
    backing_speed_effective: float = 0.0        # 지금 내는 후진 속도 크기 (ramp_speed로 가감속)
    backing_last_t: Optional[float] = None      # ramp_speed dt 계산용


# lookahead_m을 안 주면(None) trigger_distance_m + 이 여유만큼만 경로를 훑는다 (#66).
# trigger_distance_m보다는 커야 트리거 자체가 걸린다.
_DEFAULT_LOOKAHEAD_MARGIN_M = 2.0


@dataclass(frozen=True)
class ZoneClientConfig:
    robot_id: str
    zones: Sequence[ZoneConfig] = field(default_factory=list)
    trigger_distance_m: float = 3.0
    stop_margin_m: float = 0.5
    # state_label의 'waiting' 판정 여유(#137 지수 리뷰): 허가 없이 APPROACHING이어도 경계까지
    # stop_margin_m + waiting_margin_m보다 멀면 아직 'approaching'이다(속도 상한만 걸림, 실제로
    # 멈춰 기다리는 중이 아니다) - 요청을 보낸 trigger_distance_m 순간부터 바로 waiting으로 보이던
    # 것을 정지점 근처로 좁힌다. 0.5 → 0.15로 더 좁혔다(#137 지수 두 번째 리뷰) - waiting이 뜰 때는
    # speed_cap_for_stop(remaining, stop_margin_m, decel_mps2) 기준으로 remaining이 stop_margin_m에
    # 거의 다 와야 속도도 거의 0이다 - 여유가 너무 크면 "waiting"인데도 아직 꽤 빠르게 달리는
    # 중일 수 있었다.
    waiting_margin_m: float = 0.15
    decel_mps2: float = 1.0
    resend_period_s: float = 1.0
    # REJECTED 뒤, 경로에 구역이 남아 있으면 이 시간 뒤 새 request_id로 재시도한다 (#66).
    retry_period_s: float = 5.0
    # 아직 들어가지 않은 구역(APPROACHING·BLOCKED)이 경로에서 빠진 채로 이 시간(s)이 지나면 반납·IDLE.
    # 그 전에는 로봇이 trigger_distance_m(직선) 안에 있는 한 요청·정지 상한을 유지한다 (10/15, #66 규칙 변경).
    drop_grace_s: float = 3.0
    # 구역 검사를 로봇 앞 이 거리(m)까지만 한다. None이면 trigger_distance_m + 여유로 정한다 (#66).
    lookahead_m: Optional[float] = None
    # SafetyStatus.speed_limit 램프의 "제한 없음" 기준값(ramp_cap_down). nav2_params.yaml
    # FollowPath.max_vel_x·URDF max_linear_velocity와 동일하게 둔다(#69·#70).
    max_linear_velocity_mps: float = 2.0
    # BACKING(교착 해소, 10/11 결정): INSIDE에서 GRANT_REVOKED를 받으면 이 속도로 후진한다.
    backing_speed_mps: float = 0.15
    # 구역 경계를 벗어난 뒤 이만큼 더 나가면(이 지점에서 속도가 decel_mps2로 0이 된다) 정지·반납한다.
    backing_margin_m: float = 0.3
    # 안전장치(fail-closed): 후진 시작 위치에서 이 거리를 넘으면 멈추고 ERROR. 가장 긴 구역
    # (zones.yaml narrow_02, 길이 5.0 m)을 완전히 빠져나오고 backing_margin_m(0.3 m)까지 나갈
    # 여유를 더해 6.0 m로 둔다(#137 하연 두 번째 리뷰 - zones.yaml 구역 크기에 맞춰 조정한다).
    backing_max_distance_m: float = 6.0
    # 안전장치(fail-closed): 후진을 시작한 뒤 이 시간을 넘으면 멈추고 ERROR. 가장 긴 구역
    # (narrow_02, 5.0 m)을 backing_speed_mps(0.15 m/s)로 다 빠져나오는 데 걸리는 시간
    # (5.0 / 0.15 ≈ 33.3 s)에 여유를 더해 45 s로 둔다(#137 하연 두 번째 리뷰) - BACKING·FAILED
    # 중에도 resend_period_s마다 ACCESS를 보내 서버 last_seen을 갱신하므로(아래 '재전송' 참고),
    # 더는 서버 release_timeout_s(#124 fleet_manager.yaml 기본 30 s)보다 작아야 할 필요가 없다
    # (서버가 "반납이 안 온다"고 구역을 넘겨버릴 일이 없다).
    backing_timeout_s: float = 45.0


class ZoneClientCore:

    def __init__(self, config: ZoneClientConfig):
        self._config = config
        self._zones: Dict[str, ZoneConfig] = {z.zone_id: z for z in config.zones}
        self._state: Dict[str, _ZoneState] = {z.zone_id: _ZoneState() for z in config.zones}
        self._next_request_id = 1
        self._lookahead_m = (
            config.lookahead_m if config.lookahead_m is not None
            else config.trigger_distance_m + _DEFAULT_LOOKAHEAD_MARGIN_M)
        # SafetyStatus.speed_limit 램프 상태 (ramp_cap_down). 노드가 "끊김"을 판단해 None을
        # 넘기면 그 틱은 제한 없음으로 다룬다 - update()의 safety_speed_limit_mps 참고.
        self._safety_cap_effective: Optional[float] = None
        self._safety_cap_last_t: Optional[float] = None
        # 가장 최근 update() 틱의 robot_xy (#137 하연 리뷰) - on_grant는 위치를 안 받는데,
        # APPROACHING 중 REVOKED를 받은 그 틱에 이미 구역 안인지(BLOCKED 대신 BACKING으로 가야
        # 하는지) 판단하려면 필요하다. on_grant가 호출되는 시점은 보통 바로 앞 update() 틱과
        # 거의 같은 시각이라 이 값으로 충분하다.
        self._last_robot_xy: Optional[Point] = None
        # REVOKED를 받은 구역의 id (#137 하연 리뷰) - 복구가 끝나 그 구역이 다시 IDLE이 될 때까지,
        # 다른 구역의 대기 중 요청은 전부 반납하고 새 요청도 안 보낸다(state_label 참고).
        self._revoked_zone_id: Optional[str] = None

    @property
    def zone_ids(self) -> List[str]:
        return list(self._zones)

    @property
    def lookahead_m(self) -> float:
        return self._lookahead_m

    def phase(self, zone_id: str) -> Phase:
        return self._state[zone_id].phase

    def grant(self, zone_id: str) -> Optional[int]:
        return self._state[zone_id].grant

    def request_id(self, zone_id: str) -> Optional[int]:
        return self._state[zone_id].request_id

    # 상태 토픽(zone_client/state)용 - 가장 "급한"(멈추거나 멈춰 있는) 것 하나로 요약한다(한 로봇이
    # 동시에 여러 구역을 다루는 중이면, 실제로 cmd_vel에 영향을 주는 쪽을 우선한다). waiting은 phase가
    # 아니라 APPROACHING 중 일부만 가리키는 라벨이라(_zone_label), 문자열로 우선순위를 둔다
    # (#137 지수 리뷰 - blocked·waiting이 inside보다 먼저: 구역 하나에 멈춰 있으면 다른 구역에
    # 들어가 있어도 전체 상태는 "멈춰 있음"으로 보여야 한다. 예: A inside + B waiting → waiting).
    _STATE_PRIORITY = ('failed', 'backing', 'blocked', 'waiting', 'inside', 'approaching', 'idle')

    def _zone_label(self, state: _ZoneState, now: float) -> str:
        """구역 하나의 상태 라벨. waiting은 APPROACHING 중 허가가 없고 정지점 근처(stop_margin_m +
        waiting_margin_m 안)일 때만이다 - 그 전에는 속도 상한만 걸릴 뿐 아직 'approaching'이다."""
        if state.phase is Phase.APPROACHING and not self._has_valid_grant(state, now) \
                and state.last_remaining is not None \
                and state.last_remaining <= self._config.stop_margin_m + self._config.waiting_margin_m:
            return 'waiting'
        return state.phase.value

    def state_label(self, now: float) -> str:
        best = 'idle'
        for state in self._state.values():
            label = self._zone_label(state, now)
            if self._STATE_PRIORITY.index(label) < self._STATE_PRIORITY.index(best):
                best = label
        return best

    def update(
        self, now: float, path_xy: Sequence[Point], robot_xy: Point,
        safety_speed_limit_mps: Optional[float] = None, current_speed_mps: Optional[float] = None,
    ):
        """한 틱 처리.

        safety_speed_limit_mps: SafetyStatus.speed_limit의 이번 틱 값. 끊김(타임아웃)
        판단은 노드가 하고(README 'SafetyStatus 연동' 절 참고), 끊겼으면 None을 넘긴다 - 메시지
        자체의 inf(LEVEL_NORMAL, "제한 없음")도 None과 같게 다룬다.

        current_speed_mps: 로봇이 지금 실제로 받고 있는 명령 선속도(노드가 마지막으로
        내보낸 cmd_vel.linear.x) - speed_limit이 낮아질 때 램프 시작점을 이 값과
        min()으로 낮춰서, 상한이 실제 속도보다 높아 아직 전혀 효과가 없는 동안에도
        감속이 곧바로 시작되게 한다(#93 연우 리뷰, geometry.ramp_cap_down 참고).

        반환: (보낼 ZoneRequest 목록, cmd_vel 선속도 상한 m/s (제한 없으면 None), BACKING 중이면
        직접 낼 선속도(음수, 후진) 아니면 None, 로그로 남길 ERROR 메시지 목록(보통 빈 목록)).
        상한은 구역 상한과 (부드럽게 따라간) safety_speed_limit_mps 중 더 작은 쪽이다. BACKING
        중인 구역이 있으면 cmd_vel_nav는 무시하고 이 선속도를 그대로 내야 한다(노드 몫) - 이 후진
        속도도 safety_speed_limit_mps에 걸린다(크기 |v| 기준, #137 지수 리뷰 - 뒤가 막혀 안전
        노드가 멈추라고 하면 후진도 멈춰야 하는데, 원래는 safety_cap이 전진 쪽에만 걸리고
        backing_linear_x는 전혀 안 거쳤다).

        safety_cap을 구역 루프보다 먼저 구하는 건 _update_backing이 그 틱의 값을 바로 써야
        해서다(루프 뒤에 구하면 한 틱 늦은 값을 쓰게 된다) - 이 값 자체는 구역 상태와 무관하므로
        순서를 바꿔도 안전하다.

        REVOKED 복구 중(_revoked_zone_id, #137 하연 두 번째 리뷰)이면 그 구역이 아닌 다른 구역은
        전부 건드리지 않는다 - 대기 중이던 요청은 반납하고 IDLE로 되돌리며(최초 1회), IDLE인
        구역은 새로 트리거하지 않는다. 그 구역이 복구를 마치고 다시 IDLE이 되면 풀린다.
        """
        self._last_robot_xy = robot_xy  # on_grant가 다음 REVOKED에서 쓴다 (#137 하연 두 번째 리뷰)
        if safety_speed_limit_mps is not None and math.isinf(safety_speed_limit_mps):
            safety_speed_limit_mps = None  # LEVEL_NORMAL(inf) == 제한 없음
        dt = 0.0 if self._safety_cap_last_t is None else max(0.0, now - self._safety_cap_last_t)
        safety_cap = ramp_cap_down(
            safety_speed_limit_mps, self._safety_cap_effective, dt, self._config.decel_mps2,
            self._config.max_linear_velocity_mps, current_speed_mps)
        self._safety_cap_effective = safety_cap
        self._safety_cap_last_t = now

        outgoing: List[OutgoingRequest] = []
        revoked_zone = self._revoked_zone_id
        if revoked_zone is not None:
            for other_id, other_state in self._state.items():
                if other_id == revoked_zone or other_state.phase is Phase.IDLE:
                    continue
                # 다른 구역에서 REVOKED 복구 중 - 대기 중이던 요청은 반납하고 IDLE로 (#137 하연
                # 두 번째 리뷰). 다음 틱에는 이미 IDLE이라 이 분기를 다시 안 탄다(한 번만 보낸다).
                outgoing.append(OutgoingRequest(other_id, other_state.request_id, TYPE_RELEASE))
                self._state[other_id] = _ZoneState()

        speed_cap: Optional[float] = None
        backing_linear_x: Optional[float] = None
        errors: List[str] = []
        for zone_id, spec in self._zones.items():
            state = self._state[zone_id]
            if revoked_zone is not None and zone_id != revoked_zone and state.phase is Phase.IDLE:
                continue  # REVOKED 복구 중엔 다른 구역에 새로 안 들어간다 (#137 하연 두 번째 리뷰)
            # lookahead_m은 "아직 안 건드린 구역을 얼마나 멀리까지 찾아볼지"만 제한한다(#66) - 이미
            # APPROACHING/BLOCKED/INSIDE로 다루고 있는 구역은 경로가 바뀌어 멀어 보여도(재계획 등)
            # 끝까지 정확히 추적해야 하므로 거리를 안 끊는다.
            max_distance = self._lookahead_m if state.phase is Phase.IDLE else None
            remaining = remaining_distance_to_rect(path_xy, robot_xy, spec.area, max_distance)
            zone_outgoing, zone_cap, zone_backing, zone_errors = self._update_zone(
                zone_id, spec, state, now, remaining, robot_xy, safety_cap)
            outgoing.extend(zone_outgoing)
            errors.extend(zone_errors)
            if zone_cap is not None:
                speed_cap = zone_cap if speed_cap is None else min(speed_cap, zone_cap)
            if zone_backing is not None:
                backing_linear_x = zone_backing  # 동시에 여러 구역이 BACKING일 일은 실제로 없다

        if revoked_zone is not None and self._state[revoked_zone].phase is Phase.IDLE:
            self._revoked_zone_id = None  # 복구가 끝났다 - 평소처럼 다른 구역도 다시 본다

        return outgoing, combine_caps(speed_cap, safety_cap), backing_linear_x, errors

    def _has_valid_grant(self, state: _ZoneState, now: float) -> bool:
        """GRANTED이고 lease_expiry가 아직 안 지났는지. INSIDE 전환·정지 상한 해제 둘 다 이 기준을 쓴다."""
        return (state.grant == GRANT_GRANTED and state.lease_expiry is not None
                and now < state.lease_expiry)

    def _resend_access(self, zone_id: str, state: _ZoneState, now: float) -> List[OutgoingRequest]:
        """BACKING·FAILED 중에도 resend_period_s마다 같은 request_id로 ACCESS를 보낸다(#137 하연
        리뷰) - 서버가 last_seen을 갱신해 구역을 다른 로봇에게 넘기지 않게 한다. 서버는 똑같이
        REVOKED로 답할 뿐이고, 그 응답은 on_grant가 phase(BACKING·FAILED는 APPROACHING/INSIDE가
        아니다)로 걸러 무시하므로 상태는 안 바뀐다."""
        if now - state.last_sent >= self._config.resend_period_s:
            state.last_sent = now
            return [OutgoingRequest(zone_id, state.request_id, TYPE_ACCESS)]
        return []

    def _update_zone(self, zone_id: str, spec: ZoneConfig, state: _ZoneState, now: float,
                      remaining: Optional[float], robot_xy: Point, safety_cap: Optional[float]):
        outgoing: List[OutgoingRequest] = []
        inside = point_in_rect(robot_xy, spec.area)
        dropped_from_path = remaining is None and not inside
        detouring = False
        if state.phase in (Phase.APPROACHING, Phase.BLOCKED):
            if dropped_from_path:
                # 경로에서 빠졌다 - 가까이 있는 동안은 drop_grace_s까지 기다린다(10/15, #66 규칙 변경).
                # 그동안 남은 거리는 직선 거리로 본다(경로 거리보다 짧아 정지 상한이 보수적이다)
                if state.off_path_since is None:
                    state.off_path_since = now
                straight = distance_to_rect(robot_xy, spec.area)
                if (straight <= self._config.trigger_distance_m
                        and now - state.off_path_since <= self._config.drop_grace_s):
                    dropped_from_path = False
                    detouring = True
                    remaining = straight
            else:
                state.off_path_since = None  # 경로가 다시 이 구역을 지난다 - 그대로 이어간다
        state.last_remaining = remaining  # state_label의 waiting 판정용 (#137 지수 리뷰)

        if state.phase is Phase.IDLE:
            if remaining is not None and remaining <= self._config.trigger_distance_m:
                state.phase = Phase.APPROACHING
                state.request_id = self._next_request_id
                self._next_request_id += 1
                state.grant = None
                state.lease_expiry = None
                state.last_sent = now
                outgoing.append(OutgoingRequest(zone_id, state.request_id, TYPE_ACCESS))

        elif state.phase is Phase.APPROACHING:
            if dropped_from_path:
                # 경로가 바뀌어 이 구역을 더는 지나지 않는다 - 서버가 쥐고 있을 자리를 반납한다 (#66)
                outgoing.append(OutgoingRequest(zone_id, state.request_id, TYPE_RELEASE))
                self._state[zone_id] = _ZoneState()
                return outgoing, None, None, []
            if inside and self._has_valid_grant(state, now):
                state.phase = Phase.INSIDE
            elif now - state.last_sent >= self._config.resend_period_s:
                # 허가 없이 경계를 넘었어도(inside인데 유효한 허가가 없어도) APPROACHING에 남아
                # 같은 request_id로 계속 요청한다 - 속도 상한은 아래에서 remaining=0 기준으로 0이 된다
                outgoing.append(OutgoingRequest(zone_id, state.request_id, TYPE_ACCESS))
                state.last_sent = now

        elif state.phase is Phase.BLOCKED:
            if state.pending_release:
                # REVOKED(APPROACHING)로 막 전환됐다 - REJECTED와 달리 서버가 쥐고 있던 허가를
                # 반납해야 한다(10/11 결정). dropped_from_path 여부와 무관하게 한 번 보낸다.
                outgoing.append(OutgoingRequest(zone_id, state.request_id, TYPE_RELEASE))
                state.pending_release = False
            if dropped_from_path:
                self._state[zone_id] = _ZoneState()
                return outgoing, None, None, []
            if not detouring and now - state.last_sent >= self._config.retry_period_s:
                # REJECTED·REVOKED 모두 경로에 구역이 남아 있으면 retry_period_s 뒤 새 request_id로
                # 재시도한다 (#66, 10/11)
                state.request_id = self._next_request_id
                self._next_request_id += 1
                state.grant = None
                state.lease_expiry = None
                state.last_sent = now
                state.phase = Phase.APPROACHING
                outgoing.append(OutgoingRequest(zone_id, state.request_id, TYPE_ACCESS))

        elif state.phase is Phase.INSIDE:
            if not inside:
                outgoing.append(OutgoingRequest(zone_id, state.request_id, TYPE_RELEASE))
                self._state[zone_id] = _ZoneState()
                return outgoing, None, None, []
            if now - state.last_sent >= self._config.resend_period_s:
                outgoing.append(OutgoingRequest(zone_id, state.request_id, TYPE_ACCESS))
                state.last_sent = now

        elif state.phase is Phase.BACKING:
            return self._update_backing(zone_id, state, now, inside, robot_xy, safety_cap)

        elif state.phase is Phase.FAILED:
            # 안전 한도를 넘어 멈춰 세운 뒤 - 반납·재시도는 없지만(fail-closed, 수동 확인 필요),
            # ACCESS는 계속 재전송한다(#137 하연 리뷰 - _resend_access 참고)
            return self._resend_access(zone_id, state, now), None, 0.0, []

        speed_cap = None
        if state.phase in (Phase.APPROACHING, Phase.BLOCKED) and not self._has_valid_grant(state, now) \
                and remaining is not None:
            speed_cap = speed_cap_for_stop(remaining, self._config.stop_margin_m, self._config.decel_mps2)
        return outgoing, speed_cap, None, []

    def _update_backing(self, zone_id: str, state: _ZoneState, now: float, inside: bool,
                         robot_xy: Point, safety_cap: Optional[float]):
        """INSIDE에서 REVOKED를 받아 후진하는 중(10/11 결정) - cmd_vel_nav 대신 이 선속도(음수)를
        직접 낸다. 반환은 _update_zone과 같은 (outgoing, speed_cap=None, backing_linear_x, errors).

        safety_cap(SafetyStatus.speed_limit이 부드럽게 따라간 값, 크기 기준)이 있으면 목표 후진
        속도도 그 안으로 묶는다(#137 지수 리뷰) - ramp_speed의 목표 자체를 낮춰서, backing_speed_effective가
        "실제로 낸 적 없는" 더 높은 값으로 남지 않게 한다(그래야 CRITICAL이 풀렸을 때 거기서부터
        급발진하지 않고 다시 부드럽게 올라간다).
        """
        if state.backing_start_xy is None:
            # on_grant는 위치를 몰라 phase만 바꿔 둔다 - 여기서 첫 틱에 채운다.
            state.backing_start_xy = robot_xy
            state.backing_since = now
            state.backing_last_t = now

        total_backed = distance(robot_xy, state.backing_start_xy)
        elapsed = now - state.backing_since
        if total_backed > self._config.backing_max_distance_m or elapsed > self._config.backing_timeout_s:
            # fail-closed: 반납·재시도 없이 멈춰 세운 채로 둔다 - 계속 후진하다 더 큰 사고로
            # 이어지는 것보다는, 서버가 자리를 계속 쥐고 있더라도 멈춰서 수동 확인을 받는 게 낫다.
            # FAILED로 바뀌면 다음 틱부터 _update_zone의 FAILED 분기가 돌아 이 메시지는 한 번만 난다
            # (#137 지수 리뷰 - state_label에도 'failed'로 드러난다).
            state.phase = Phase.FAILED
            return self._resend_access(zone_id, state, now), None, 0.0, [
                f'구역 {zone_id} 후진 안전 한도 초과(이동 {total_backed:.2f} m, 경과 {elapsed:.1f} s) '
                '- 정지 (fail-closed, 수동 확인 필요)']

        if not inside and state.backing_exit_xy is None:
            state.backing_exit_xy = robot_xy
        if state.backing_exit_xy is not None:
            past = distance(robot_xy, state.backing_exit_xy)
            if past >= self._config.backing_margin_m:
                outgoing = [OutgoingRequest(zone_id, state.request_id, TYPE_RELEASE)]
                self._state[zone_id] = _ZoneState()
                return outgoing, None, 0.0, []
            target = min(
                self._config.backing_speed_mps,
                speed_cap_for_stop(self._config.backing_margin_m - past, 0.0, self._config.decel_mps2))
        else:
            target = self._config.backing_speed_mps
        if safety_cap is not None:
            target = min(target, safety_cap)

        dt = max(0.0, now - state.backing_last_t)
        state.backing_speed_effective = ramp_speed(
            state.backing_speed_effective, target, dt, self._config.decel_mps2)
        state.backing_last_t = now
        return self._resend_access(zone_id, state, now), None, -state.backing_speed_effective, []

    def on_grant(self, now: float, robot_id: str, zone_id: str, request_id: int, grant: int,
                 lease_expiry: Optional[float], reason: int, holder_robot_id: str) -> Optional[str]:
        """ZoneGrant 수신. 우리 로봇 것이 아니거나 지금 기다리는 request_id가 아니면 무시한다.

        REJECTED·REVOKED로 막 전환됐을 때만 경고 메시지를 돌려준다 (노드가 로그로 남기도록).
        그 외는 None. REVOKED(교착 해소)는 위치(robot_xy)가 필요한 처리(반납·후진 시작)가 있는데
        여기선 모른다 - phase만 바꿔 두고 다음 update() 틱(_update_zone/_update_backing)에서 채운다.
        """
        if robot_id != self._config.robot_id:
            return None
        state = self._state.get(zone_id)
        if state is None or state.phase not in (Phase.APPROACHING, Phase.INSIDE):
            return None
        if state.request_id != request_id:
            return None
        state.grant = grant
        state.lease_expiry = lease_expiry
        if grant == GRANT_REJECTED and state.phase is Phase.APPROACHING:
            state.phase = Phase.BLOCKED
            state.last_sent = now  # retry_period_s를 여기서부터 잰다 (#66)
            return (f'구역 {zone_id} 진입 거부 (reason={reason}, '
                    f'holder={holder_robot_id or "-"}) → 진입하지 않음, {self._config.retry_period_s}초 뒤 재시도')
        if grant == GRANT_REVOKED:
            # holder_robot_id는 REVOKED에서 "양보받는 로봇"이다(REJECTED의 "점유 중인 로봇"과 다른
            # 뜻, amr_fleet_core README '구역 허가' 절) - yield_to로 적어 혼동하지 않게 한다.
            # 복구가 끝나 이 구역이 다시 IDLE이 될 때까지 다른 구역은 건드리지 않는다(#137 하연
            # 두 번째 리뷰) - update()가 이 값을 보고 다른 구역의 대기 중 요청을 반납·억제한다.
            self._revoked_zone_id = zone_id
            if state.phase is Phase.APPROACHING:
                # 응답이 오기까지 한 틱 사이에 로봇이 이미 경계를 넘어섰을 수 있다 - 그러면
                # BLOCKED(반납 후 재시도)가 아니라 BACKING으로 가야 한다(#137 하연 두 번째 리뷰).
                # self._last_robot_xy는 update()가 매 틱 채워 두는 가장 최근 위치다.
                already_inside = (self._last_robot_xy is not None
                                   and point_in_rect(self._last_robot_xy, self._zones[zone_id].area))
                if already_inside:
                    state.phase = Phase.BACKING
                    return (f'구역 {zone_id} 허가 회수(교착 해소, reason={reason}, '
                            f'yield_to={holder_robot_id or "-"}) → 응답 전에 이미 구역 안 - 후진 시작')
                state.phase = Phase.BLOCKED
                state.last_sent = now
                state.pending_release = True
                return (f'구역 {zone_id} 허가 회수(교착 해소, reason={reason}, '
                        f'yield_to={holder_robot_id or "-"}) → 진입하지 않고 반납, '
                        f'{self._config.retry_period_s}초 뒤 재시도')
            state.phase = Phase.BACKING  # INSIDE에서 받음 - 다음 틱부터 직접 후진
            return (f'구역 {zone_id} 허가 회수(교착 해소, reason={reason}, '
                    f'yield_to={holder_robot_id or "-"}) → 후진 시작')
        return None
