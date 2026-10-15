"""RViz2 E-Stop 버튼의 판단 규칙 (ROS 비의존). 노드는 estop_button_node.py.

- 클릭하면 지금 상태를 뒤집는 요청을 보낸다: ESTOP이 아니면 걸기(true), ESTOP이면 해제(false)
- safety/status를 아직 못 받았거나 끊겼으면 걸기를 보낸다. 모를 때는 멈추는 쪽이 안전하다
- 글자는 영어로 쓴다 (RViz 글꼴에 한글이 없다. markers.py와 같은 이유)
"""

from dataclasses import dataclass
from typing import Optional, Tuple

from amr_perception_core.safety import LEVEL_ESTOP

# 상태 불명으로 볼 safety/status 공백 [s]. 안전 노드는 최소 10 Hz로 낸다
STATUS_TIMEOUT_S = 1.0

COLOR_ARMED = (0.9, 0.1, 0.1)        # 빨강: 누르면 정지
COLOR_ENGAGED = (1.0, 0.8, 0.1)      # 노랑: 정지 중, 누르면 해제
COLOR_UNKNOWN = (0.5, 0.5, 0.5)      # 회색: 안전 노드 상태 모름


@dataclass(frozen=True)
class ButtonLook:
    rgb: Tuple[float, float, float]
    text: str


def estop_engaged(level: Optional[int]) -> Optional[bool]:
    """SafetyStatus.level → E-Stop이 걸려 있는지. 상태를 모르면 None."""
    return None if level is None else level == LEVEL_ESTOP


def status_level(last_level: Optional[int], age_s: Optional[float],
                 timeout_s: float = STATUS_TIMEOUT_S) -> Optional[int]:
    """마지막 SafetyStatus.level. 받은 적 없거나 timeout_s보다 오래됐으면 None."""
    if last_level is None or age_s is None or age_s > timeout_s:
        return None
    return last_level


def next_request(level: Optional[int]) -> bool:
    """클릭했을 때 safety/estop에 보낼 data. ESTOP이면 해제(False), 그 밖(모름 포함)은 걸기(True)."""
    return not estop_engaged(level)


def button_look(level: Optional[int], pending: bool = False) -> ButtonLook:
    """버튼 색과 글자. pending이면 서비스 응답을 기다리는 중."""
    engaged = estop_engaged(level)
    if engaged is None:
        look = ButtonLook(COLOR_UNKNOWN, 'E-STOP (no safety status)')
    elif engaged:
        look = ButtonLook(COLOR_ENGAGED, 'E-STOP ON - click to release')
    else:
        look = ButtonLook(COLOR_ARMED, 'E-STOP')
    if pending:
        look = ButtonLook(look.rgb, look.text + ' ...')
    return look
