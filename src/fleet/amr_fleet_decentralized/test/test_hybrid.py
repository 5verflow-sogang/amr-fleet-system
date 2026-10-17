"""하이브리드 폴백 모드 전환과 작업표 이어받기 (auction/hybrid.py, architecture.md 5-2절). ROS 비의존."""

from types import SimpleNamespace as NS

import pytest

from amr_fleet_decentralized.auction import (
    ERROR_TASK_TAKEN, MODE_CENTRALIZED, MODE_DECENTRALIZED, FallbackMode, FallbackParams, TaskRetryLedger,
    backlog_task_ids, can_retry_failed_task,
)


def beats(fb, start, end, period=0.2, server_mode=MODE_CENTRALIZED):
    """start부터 end 전까지 period마다 heartbeat. 바뀐 모드들."""
    changes = []
    i = 0
    while round(start + i * period, 6) < end:
        t = round(start + i * period, 6)
        changed = fb.on_heartbeat(t, server_mode)
        if changed is not None:
            changes.append((t, changed))
        i += 1
    return changes


def test_no_switch_before_first_heartbeat():
    fb = FallbackMode()
    assert fb.update(100.0) is None
    assert fb.mode == MODE_CENTRALIZED


def test_falls_back_after_two_seconds_silence():
    fb = FallbackMode()
    beats(fb, 0.0, 10.0)                       # 마지막 heartbeat 9.8초
    assert fb.update(11.7) is None             # 1.9초
    assert fb.update(11.8) == MODE_DECENTRALIZED
    assert fb.update(12.0) is None             # 한 번만
    assert fb.transitions == 1


def test_returns_after_five_stable_seconds():
    fb = FallbackMode()
    beats(fb, 0.0, 1.0)
    fb.update(5.0)
    assert fb.mode == MODE_DECENTRALIZED
    # 20초부터 다시 온다: 25초에 돌아온다
    changes = beats(fb, 20.0, 26.0)
    assert changes == [(25.0, MODE_CENTRALIZED)]
    assert fb.transitions == 2


def test_flapping_link_restarts_stable_timer():
    fb = FallbackMode()
    beats(fb, 0.0, 1.0)
    fb.update(5.0)
    beats(fb, 10.0, 13.0)                      # 3초 붙었다가
    assert fb.update(15.2) is None             # 2초 넘게 끊김: 안정 시각을 버린다
    assert beats(fb, 16.0, 20.9) == []         # 다시 붙은 16초부터 5초 전
    assert fb.on_heartbeat(21.0) == MODE_CENTRALIZED


def test_short_gap_does_not_reset_stable_timer():
    fb = FallbackMode()
    beats(fb, 0.0, 1.0)
    fb.update(5.0)
    beats(fb, 10.0, 12.0)
    # 1.5초 끊김(2초 안 됨)은 안정으로 본다
    assert beats(fb, 13.5, 15.2) == [(15.1, MODE_CENTRALIZED)]   # 10초부터 5초 (heartbeat 13.5+0.2k)


def test_server_directed_decentral_option_c():
    """C안 자리: 서버가 MODE_DECENTRALIZED를 보내면 바로 분산, 돌아오라고 한 때부터 5초."""
    fb = FallbackMode()
    beats(fb, 0.0, 1.0)
    assert fb.on_heartbeat(1.0, MODE_DECENTRALIZED) == MODE_DECENTRALIZED
    assert beats(fb, 1.2, 30.0, server_mode=MODE_DECENTRALIZED) == []   # 서버가 시키는 동안은 그대로
    assert beats(fb, 30.0, 35.1) == [(35.0, MODE_CENTRALIZED)]


def test_params_validated():
    with pytest.raises(ValueError):
        FallbackParams(fallback_timeout_s=0.0)
    with pytest.raises(ValueError):
        FallbackParams(return_stable_s=-1.0)


def record(task_id, assignee='', state=0):
    return NS(task=NS(task_id=task_id), assignee_id=assignee, state=state)


def test_backlog_is_unassigned_or_mine_and_pending():
    records = [record('t1'), record('t2', 'amr_03'), record('t3', 'amr_01'),
               record('t4', 'amr_03', state=1), record('t5', '', state=2)]
    assert backlog_task_ids(records, 'amr_03') == ['t1', 't2']


def test_task_taken_failure_not_reauctioned():
    assert ERROR_TASK_TAKEN == 104   # amr_msgs ErrorInfo.ASSIGN_TASK_TAKEN
    assert can_retry_failed_task(failures=0, failed_retries=1, error_code=ERROR_TASK_TAKEN) is False
    ledger = TaskRetryLedger(failed_retries=1)
    assert not ledger.on_failed('t1', ERROR_TASK_TAKEN)
    assert ledger.failures('t1') == 0
