from amr_fleet_decentralized.auction import (
    ERROR_CANCELED,
    MODE_BUSY,
    MODE_IDLE,
    AuctionParams,
    Bid,
    RobotInfo,
    TaskInfo,
    TaskRetryLedger,
    award_exhausted,
    can_reclaim,
    can_retry_failed_task,
    can_withdraw,
    compute_bid_cost,
    decide_bid,
    decide_winner,
    register_bid,
    should_resend_award,
    should_resend_bid,
    should_withdraw_award,
    winner_lost,
)


def robot(robot_id, xy=(0.0, 0.0), battery=100.0, mode=MODE_IDLE):
    return RobotInfo(robot_id=robot_id, xy=xy, battery_percent=battery, mode=mode)


def task(task_id='t1', pickup=(3.0, 4.0), dropoff=(3.0, 4.0), deadline=None):
    return TaskInfo(task_id=task_id, pickup_xy=pickup, dropoff_xy=dropoff, deadline=deadline)


def bid(robot_id, cost, bid_time=0.0, task_id='t1'):
    return Bid(robot_id=robot_id, task_id=task_id, cost=cost, estimated_finish_time=0.0, bid_time=bid_time)


# --- 입찰가 계산: 현재 위치 -> 작업 시작점 직선 거리 (중앙 nearest와 같은 규칙, 10/7 결정) ---


def test_compute_bid_cost_is_straight_line_to_pickup():
    r = robot('amr_01', xy=(0.0, 0.0))
    t = task(pickup=(3.0, 4.0), dropoff=(30.0, 40.0))  # dropoff는 멀어도 비용에 안 들어간다
    assert compute_bid_cost(r, t) == 5.0


def test_compute_bid_cost_falls_back_to_dropoff_without_pickup():
    r = robot('amr_01', xy=(0.0, 0.0))
    t = TaskInfo(task_id='t1', dropoff_xy=(3.0, 4.0))  # pickup 없음 (CHARGE·RETURN류)
    assert compute_bid_cost(r, t) == 5.0


# --- 로봇 쪽 입찰 결정 ---


def test_decide_bid_skips_when_not_idle():
    # 10/7 결정: IDLE 로봇만 입찰한다 (명세 3장 "가장 가까운 로봇 선점")
    r = robot('amr_01', mode=MODE_BUSY)
    assert decide_bid(r, task(), now_s=0.0) is None


def test_decide_bid_skips_when_battery_too_low():
    params = AuctionParams(min_battery_percent=20.0)
    r = robot('amr_01', battery=19.9)
    assert decide_bid(r, task(), now_s=0.0, params=params) is None


def test_decide_bid_skips_when_deadline_cannot_be_met():
    params = AuctionParams(robot_speed_mps=1.0)
    r = robot('amr_01', xy=(0.0, 0.0))
    t = task(pickup=(100.0, 0.0), dropoff=(100.0, 0.0), deadline=5.0)  # 100m/1.0mps=100s > 5s
    assert decide_bid(r, t, now_s=0.0, params=params) is None


def test_decide_bid_returns_bid_when_eligible():
    params = AuctionParams(robot_speed_mps=1.0)
    r = robot('amr_01', xy=(0.0, 0.0))
    t = task(pickup=(3.0, 4.0), dropoff=(3.0, 4.0))
    result = decide_bid(r, t, now_s=10.0, params=params)
    assert result is not None
    assert result.cost == 5.0
    assert result.estimated_finish_time == 15.0


# --- 경매인 쪽 낙찰 결정: 최저가 낙찰 / 동점 / 마감 후 입찰 무시 ---
# bid_time은 "경매인이 받은 시각"으로 본다(10/7 결정) - 아래 bid()는 그 수신 시각을 그대로
# 받아 쓰는 테스트 헬퍼다(실제 노드에서는 보낸 로봇의 타임스탬프가 아니라 자기 수신 시각을 넣는다).


def test_decide_winner_picks_lowest_cost():
    bids = [bid('amr_01', cost=5.0), bid('amr_02', cost=3.0), bid('amr_03', cost=4.0)]
    result = decide_winner('t1', bids, announce_time_s=0.0, bid_window_s=2.0)
    assert result.winner_id == 'amr_02'
    assert result.winning_cost == 3.0


def test_decide_winner_breaks_tie_by_robot_id_lexicographic():
    bids = [bid('amr_03', cost=5.0), bid('amr_01', cost=5.0), bid('amr_02', cost=5.0)]
    result = decide_winner('t1', bids, announce_time_s=0.0, bid_window_s=2.0)
    assert result.winner_id == 'amr_01'


def test_decide_winner_ignores_late_bids():
    # amr_01의 낮은 가격 입찰을 경매인이 마감(2.0s)을 넘겨서 받음 - 무효로 치고 뺀다.
    bids = [bid('amr_01', cost=1.0, bid_time=2.1), bid('amr_02', cost=5.0, bid_time=1.5)]
    result = decide_winner('t1', bids, announce_time_s=0.0, bid_window_s=2.0)
    assert result.winner_id == 'amr_02'
    assert result.winning_cost == 5.0


def test_decide_winner_no_bids_at_all():
    result = decide_winner('t1', [], announce_time_s=0.0, bid_window_s=2.0)
    assert result.winner_id is None
    assert result.winning_cost is None


def test_decide_winner_no_valid_bids_when_all_late():
    bids = [bid('amr_01', cost=1.0, bid_time=5.0)]
    result = decide_winner('t1', bids, announce_time_s=0.0, bid_window_s=2.0)
    assert result.winner_id is None


# --- BID 재전송 (#155·#171): 1회성 발행이 디스커버리 레이스로 유실돼도 재전송으로 구제 ---


def test_should_resend_bid_waits_for_period():
    assert should_resend_bid(last_sent_time=0.0, now_s=0.4, resend_period_s=0.5, deadline_s=2.0) is False
    assert should_resend_bid(last_sent_time=0.0, now_s=0.5, resend_period_s=0.5, deadline_s=2.0) is True


def test_should_resend_bid_stops_at_deadline():
    assert should_resend_bid(last_sent_time=1.5, now_s=2.0, resend_period_s=0.5, deadline_s=2.0) is False


def test_register_bid_ignores_duplicate_from_same_robot():
    # 재전송된 BID(같은 로봇, 같은 cost)는 중복으로 세지 않는다 - 먼저 받은 것만 남는다
    bids = register_bid([], bid('amr_02', cost=5.0, bid_time=0.1))
    bids = register_bid(bids, bid('amr_02', cost=5.0, bid_time=0.6))
    assert len(bids) == 1
    assert bids[0].bid_time == 0.1


def test_register_bid_keeps_distinct_robots():
    bids = register_bid([], bid('amr_01', cost=5.0))
    bids = register_bid(bids, bid('amr_02', cost=3.0))
    assert {b.robot_id for b in bids} == {'amr_01', 'amr_02'}


def test_resend_recovers_auction_from_lost_first_bid():
    # 첫 BID는 경매인에 전혀 도착하지 않았다(유실) - register_bid가 호출된 적 없다.
    # bid_resend_period_s 뒤 재전송된 입찰만 도착하는데, 그래도 마감(2.0s) 전이라 낙찰된다.
    bids = register_bid([], bid('amr_02', cost=3.0, bid_time=0.5))
    result = decide_winner('t1', bids, announce_time_s=0.0, bid_window_s=2.0)
    assert result.winner_id == 'amr_02'


# --- 경매 뒷단계: AWARD 재전송 (1초 간격 x 5회) · WITHDRAW 범위 (ACCEPT 전까지만) ---


def test_should_resend_award_waits_for_interval():
    assert should_resend_award(
        attempts_sent=1, last_sent_time=0.0, now_s=0.9, resend_interval_s=1.0, max_attempts=5,
    ) is False
    assert should_resend_award(
        attempts_sent=1, last_sent_time=0.0, now_s=1.0, resend_interval_s=1.0, max_attempts=5,
    ) is True


def test_should_resend_award_stops_after_max_attempts():
    assert should_resend_award(
        attempts_sent=5, last_sent_time=0.0, now_s=100.0, resend_interval_s=1.0, max_attempts=5,
    ) is False


def test_award_exhausted_after_max_attempts():
    assert award_exhausted(attempts_sent=4, max_attempts=5) is False
    assert award_exhausted(attempts_sent=5, max_attempts=5) is True


def test_can_withdraw_only_before_accept():
    assert can_withdraw(accepted=False) is True
    assert can_withdraw(accepted=True) is False


# --- 동시 낙찰 경합: 이미 ACCEPT한 로봇은 새 AWARD를 WITHDRAW, 경매인은 다음 순위에게 ---


def test_should_withdraw_award_when_holding_other_task_or_withdrew_before():
    assert should_withdraw_award(holding_other_task=False, withdrew_before=False) is False
    assert should_withdraw_award(holding_other_task=True, withdrew_before=False) is True
    # 한 번 포기한 경매는 일이 끝난 뒤 재전송 AWARD가 와도 다시 포기한다(다음 순위와 중복 방지)
    assert should_withdraw_award(holding_other_task=False, withdrew_before=True) is True


def test_decide_winner_excluding_withdrawn_picks_next_rank():
    bids = [bid('amr_01', cost=3.0), bid('amr_02', cost=4.0), bid('amr_03', cost=5.0)]
    result = decide_winner('t1', bids, 0.0, 2.0, excluded={'amr_01'})
    assert result.winner_id == 'amr_02'
    assert result.winning_cost == 4.0
    assert decide_winner('t1', bids, 0.0, 2.0, excluded={'amr_01', 'amr_02'}).winner_id == 'amr_03'


def test_decide_winner_excluding_all_bidders_has_no_winner():
    result = decide_winner('t1', [bid('amr_01', cost=3.0)], 0.0, 2.0, excluded={'amr_01'})
    assert result.winner_id is None


# --- ACCEPT 뒤 TaskStatus FAILED: 최대 failed_retries번 재경매 (10/9 결정, #107·#108) ---


def test_auction_params_default_failed_retries_is_one():
    assert AuctionParams().failed_retries == 1


def test_can_retry_failed_task_allows_up_to_limit():
    assert can_retry_failed_task(failures=0, failed_retries=1) is True


def test_can_retry_failed_task_stops_after_limit():
    assert can_retry_failed_task(failures=1, failed_retries=1) is False


def test_can_retry_failed_task_skips_canceled():
    # 취소(ASSIGN_CANCELED)로 온 FAILED는 재경매하지 않는다 (중앙 #107과 같음). 일반 FAILED는 1회 그대로
    from amr_msgs.msg import ErrorInfo
    assert ERROR_CANCELED == ErrorInfo.ASSIGN_CANCELED
    assert can_retry_failed_task(failures=0, failed_retries=1, error_code=ErrorInfo.ASSIGN_CANCELED) is False
    assert can_retry_failed_task(failures=0, failed_retries=1, error_code=ErrorInfo.NONE) is True
    assert can_retry_failed_task(failures=1, failed_retries=1, error_code=ErrorInfo.NONE) is False



# --- 낙찰 로봇 장애 회수 (중앙 #132와 같은 규칙: RobotState 5초 끊김, FAILED와 따로, 상한 없음) ---


def test_auction_params_default_reclaim_matches_central():
    assert AuctionParams().reclaim_timeout_s == 5.0
    assert AuctionParams().max_reclaims < 0


def test_winner_lost_after_state_gap_reclaims():
    # 10초에 수락·마지막 상태, 그 뒤 끊김 -> 5초째에 회수
    assert winner_lost(last_seen=10.0, accepted_at=10.0, now_s=14.9, reclaim_timeout_s=5.0) is False
    assert winner_lost(last_seen=10.0, accepted_at=10.0, now_s=15.0, reclaim_timeout_s=5.0) is True


def test_winner_back_within_timeout_not_reclaimed():
    # 10초부터 끊겼다가 4초 뒤(14초) 상태가 다시 오면 회수하지 않는다 - 기준이 14초로 다시 잡힌다
    assert winner_lost(last_seen=14.0, accepted_at=10.0, now_s=15.0, reclaim_timeout_s=5.0) is False
    assert winner_lost(last_seen=14.0, accepted_at=10.0, now_s=18.9, reclaim_timeout_s=5.0) is False
    assert winner_lost(last_seen=14.0, accepted_at=10.0, now_s=19.0, reclaim_timeout_s=5.0) is True


def test_winner_lost_uses_accept_time_and_can_be_disabled():
    # 상태를 한 번도 못 받은 낙찰 로봇은 ACCEPT 시각부터 잰다 (수락하자마자 회수하지 않음)
    assert winner_lost(last_seen=None, accepted_at=10.0, now_s=12.0, reclaim_timeout_s=5.0) is False
    assert winner_lost(last_seen=None, accepted_at=10.0, now_s=15.0, reclaim_timeout_s=5.0) is True
    # 수락 전 상태는 ACCEPT 시각에 가려진다
    assert winner_lost(last_seen=1.0, accepted_at=10.0, now_s=14.0, reclaim_timeout_s=5.0) is False
    # 0이면 끈다
    assert winner_lost(last_seen=0.0, accepted_at=0.0, now_s=100.0, reclaim_timeout_s=0.0) is False


def test_reclaim_not_counted_in_failures():
    ledger = TaskRetryLedger(failed_retries=1)
    # 회수는 상한 없이(기본) 몇 번이든 재경매하고, FAILED 재시도 횟수를 쓰지 않는다
    assert ledger.on_winner_lost('task_1') is True
    assert ledger.on_winner_lost('task_1') is True
    assert ledger.reclaims('task_1') == 2
    assert ledger.failures('task_1') == 0
    # 그래서 FAILED 1회 재경매는 그대로 남아 있다
    assert ledger.on_failed('task_1') is True
    assert ledger.on_failed('task_1') is False
    assert ledger.reclaims('task_1') == 2


def test_reclaim_limit_parameter():
    assert can_reclaim(reclaims=100, max_reclaims=-1) is True
    assert can_reclaim(reclaims=0, max_reclaims=1) is True
    assert can_reclaim(reclaims=1, max_reclaims=1) is False
    ledger = TaskRetryLedger(failed_retries=1, max_reclaims=1)
    assert ledger.on_winner_lost('task_1') is True
    assert ledger.on_winner_lost('task_1') is False
