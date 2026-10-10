from amr_fleet_decentralized.auction import (
    MODE_BUSY,
    MODE_IDLE,
    AuctionParams,
    Bid,
    RobotInfo,
    TaskInfo,
    award_exhausted,
    can_retry_failed_task,
    can_withdraw,
    compute_bid_cost,
    decide_bid,
    decide_winner,
    should_resend_award,
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


# --- ACCEPT 뒤 TaskStatus FAILED: 최대 failed_retries번 재경매 (10/9 결정, #107·#108) ---


def test_auction_params_default_failed_retries_is_one():
    assert AuctionParams().failed_retries == 1


def test_can_retry_failed_task_allows_up_to_limit():
    assert can_retry_failed_task(failures=0, failed_retries=1) is True


def test_can_retry_failed_task_stops_after_limit():
    assert can_retry_failed_task(failures=1, failed_retries=1) is False
