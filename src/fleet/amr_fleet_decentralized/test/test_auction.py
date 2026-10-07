from amr_fleet_decentralized.auction import (
    AuctionParams,
    Bid,
    RobotInfo,
    TaskInfo,
    compute_bid_cost,
    decide_bid,
    decide_winner,
)


def robot(robot_id, xy=(0.0, 0.0), battery=100.0, task_remaining=0.0, task_dropoff=None):
    return RobotInfo(
        robot_id=robot_id,
        xy=xy,
        battery_percent=battery,
        current_task_dropoff_xy=task_dropoff,
        current_task_remaining_m=task_remaining,
    )


def task(task_id='t1', pickup=(3.0, 4.0), dropoff=(3.0, 4.0), deadline=None):
    return TaskInfo(task_id=task_id, pickup_xy=pickup, dropoff_xy=dropoff, deadline=deadline)


def bid(robot_id, cost, bid_time=0.0, task_id='t1'):
    return Bid(robot_id=robot_id, task_id=task_id, cost=cost, estimated_finish_time=0.0, bid_time=bid_time)


# --- 입찰가 계산 ---


def test_compute_bid_cost_from_idle_position():
    # 유휴 로봇: 현재 위치 -> pickup -> dropoff(픽업과 같은 지점이라 그 구간은 0)
    r = robot('amr_01', xy=(0.0, 0.0))
    t = task(pickup=(3.0, 4.0), dropoff=(3.0, 4.0))
    assert compute_bid_cost(r, t) == 5.0


def test_compute_bid_cost_includes_current_task_remaining_distance():
    # 진행 중인 작업이 있으면 그걸 마칠 때까지 남은 거리부터 더한다.
    r = robot('amr_01', xy=(0.0, 0.0), task_remaining=10.0, task_dropoff=(3.0, 4.0))
    t = task(pickup=(3.0, 4.0), dropoff=(3.0, 4.0))  # 큐 끝(dropoff)이 곧 pickup
    assert compute_bid_cost(r, t) == 10.0


# --- 로봇 쪽 입찰 결정 ---


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
    # amr_01의 낮은 가격 입찰이 마감(2.0s)을 넘겨 도착 - 무효로 치고 뺀다.
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
