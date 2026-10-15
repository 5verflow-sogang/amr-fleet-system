"""RViz2 E-Stop 버튼 규칙 (ROS 없이): 상태 → 다음 요청, 버튼 모양."""

import pytest

from amr_perception_core.safety import LEVEL_CRITICAL, LEVEL_ESTOP, LEVEL_NORMAL, LEVEL_WARNING
from amr_safety.estop_button import (
    COLOR_ARMED,
    COLOR_ENGAGED,
    COLOR_UNKNOWN,
    STATUS_TIMEOUT_S,
    button_look,
    next_request,
    status_level,
)


@pytest.mark.parametrize('level', [LEVEL_NORMAL, LEVEL_WARNING, LEVEL_CRITICAL])
def test_click_engages_when_not_estop(level):
    assert next_request(level) is True


def test_click_releases_when_estop():
    assert next_request(LEVEL_ESTOP) is False


def test_click_engages_when_status_unknown():
    # 안전 노드 상태를 모르면 멈추는 쪽을 보낸다
    assert next_request(None) is True


def test_status_level_times_out():
    assert status_level(LEVEL_ESTOP, 0.2) == LEVEL_ESTOP
    assert status_level(LEVEL_ESTOP, STATUS_TIMEOUT_S + 0.1) is None
    assert status_level(None, None) is None
    assert status_level(LEVEL_NORMAL, None) is None


def test_look_follows_state():
    assert button_look(LEVEL_NORMAL).rgb == COLOR_ARMED
    assert button_look(LEVEL_CRITICAL).text == 'E-STOP'
    engaged = button_look(LEVEL_ESTOP)
    assert engaged.rgb == COLOR_ENGAGED and 'release' in engaged.text
    assert button_look(None).rgb == COLOR_UNKNOWN


def test_look_pending_keeps_color():
    pending = button_look(LEVEL_NORMAL, pending=True)
    assert pending.rgb == COLOR_ARMED and pending.text.endswith('...')
    assert pending != button_look(LEVEL_NORMAL)


def test_texts_are_ascii():
    # RViz 글꼴에 한글이 없다
    for level in (None, LEVEL_NORMAL, LEVEL_ESTOP):
        for pending in (False, True):
            assert button_look(level, pending).text.isascii()
