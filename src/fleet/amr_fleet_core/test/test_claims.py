"""먼저 수락한 쪽이 주인 (claims.TaskClaims). rclpy 없이 돈다."""

from amr_fleet_core.claims import (STATE_COMPLETED, STATE_FAILED, STATE_IN_PROGRESS, STATE_PENDING,
                                      TaskClaims)


def test_earliest_start_owns_ties_by_robot_id():
    c = TaskClaims()
    c.observe('t1', 'amr_03', STATE_PENDING, 2.0, 0.0)
    c.observe('t1', 'amr_02', STATE_IN_PROGRESS, 1.0, 0.1)
    c.observe('t1', 'amr_01', STATE_PENDING, 1.0, 0.2)
    assert c.owner('t1', 0.3) == 'amr_01'
    assert c.owner('t1', 0.3, exclude='amr_01') == 'amr_02'
    assert c.owner('t2', 0.3) is None


def test_rival_for_running_and_new_assignment():
    c = TaskClaims()
    c.observe('t1', 'amr_02', STATE_IN_PROGRESS, 5.0, 0.0)
    # 4초에 수락한 amr_01이 먼저다
    assert c.rival('t1', 'amr_01', 4.0, 0.1) is None
    # 6초에 수락한 amr_03은 물러나야 한다
    assert c.rival('t1', 'amr_03', 6.0, 0.1) == 'amr_02'
    # 아직 수락 전이면 맡은 로봇이 있다는 것만으로 거부한다
    assert c.rival('t1', 'amr_01', None, 0.1) == 'amr_02'


def test_completed_wins_over_earlier_start():
    c = TaskClaims()
    c.observe('t1', 'amr_01', STATE_IN_PROGRESS, 1.0, 0.0)
    c.observe('t1', 'amr_02', STATE_COMPLETED, 3.0, 0.1)
    assert c.owner('t1', 0.2) == 'amr_02'
    assert c.rival('t1', 'amr_01', 1.0, 0.2) == 'amr_02'
    assert c.completed('t1', 'amr_02') and not c.completed('t1', 'amr_01')


def test_failed_drops_claim():
    c = TaskClaims()
    c.observe('t1', 'amr_01', STATE_IN_PROGRESS, 1.0, 0.0)
    c.observe('t1', 'amr_02', STATE_IN_PROGRESS, 2.0, 0.0)
    c.observe('t1', 'amr_01', STATE_FAILED, 1.0, 0.5)
    assert c.owner('t1', 0.6) == 'amr_02'


def test_start_kept_from_first_report():
    c = TaskClaims()
    c.observe('t1', 'amr_01', STATE_PENDING, 1.0, 0.0)
    c.observe('t1', 'amr_01', STATE_IN_PROGRESS, 9.0, 0.5)
    c.observe('t1', 'amr_02', STATE_PENDING, 2.0, 0.6)
    assert c.owner('t1', 0.7) == 'amr_01'


def test_silent_robot_claim_expires_but_completed_stays():
    c = TaskClaims(timeout_s=5.0)
    c.robot_state('amr_01', 't1', 0.0)
    c.observe('t1', 'amr_01', STATE_IN_PROGRESS, 1.0, 0.0)
    c.observe('t2', 'amr_02', STATE_COMPLETED, 1.0, 0.0)
    assert c.owner('t1', 4.9) == 'amr_01'
    assert c.owner('t1', 5.0) is None         # RobotState 5초 끊김 (서버가 작업을 회수하는 때와 같다)
    assert c.owner('t2', 100.0) == 'amr_02'
    # 다시 살아나 그 작업을 계속 보고하면 다시 센다
    c.robot_state('amr_01', 't1', 6.0)
    assert c.owner('t1', 6.1) == 'amr_01'


def test_claim_without_robot_state_times_out_from_report():
    c = TaskClaims(timeout_s=5.0)
    c.observe('t1', 'amr_01', STATE_PENDING, 1.0, 0.0)
    assert c.owner('t1', 4.0) == 'amr_01'
    assert c.owner('t1', 5.5) is None


def test_robot_state_on_other_task_drops_claim():
    """재시작했거나 작업을 놓은 로봇: 주장 뒤에 온 RobotState가 다른 작업(또는 IDLE)."""
    c = TaskClaims(timeout_s=5.0)
    c.observe('t1', 'amr_01', STATE_IN_PROGRESS, 1.0, 1.0)
    c.robot_state('amr_01', 't1', 0.5)        # 주장보다 먼저 온 RobotState는 아직 그 작업을 몰라도 된다
    assert c.owner('t1', 1.1) == 'amr_01'
    c.robot_state('amr_01', '', 2.0)
    assert c.owner('t1', 2.1) is None
