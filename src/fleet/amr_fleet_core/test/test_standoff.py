"""구역 밖 대치(서로 막고 멈춘 로봇들) 탐지와 양보 지시."""

import math

from amr_fleet_core.deadlock import BACK, FORWARD, StandoffConfig, StandoffDetector

CFG = StandoffConfig(radius_m=1.2, still_s=10.0, still_m=0.2, cooldown_s=20.0)


def feed(det, robot, x, y, yaw=0.0, busy=True, active=True, t0=0.0, t1=30.0, step=0.5):
    t = t0
    while t <= t1 + 1e-9:
        det.update(robot, t, x, y, yaw, busy, active)
        t += step


def no_priority(_robot):
    return 0


def test_two_busy_robots_face_to_face_lower_priority_backs_off():
    det = StandoffDetector(CFG)
    feed(det, 'amr_01', 0.0, 0.0, yaw=0.0)            # 동쪽을 본다
    feed(det, 'amr_02', 0.65, 0.0, yaw=math.pi)        # 서쪽을 본다, 마주 섰다
    prio = {'amr_01': 5, 'amr_02': 0}
    orders = det.check(30.0, prio.get)
    assert len(orders) == 1
    o = orders[0]
    assert (o.robot_id, o.yield_to, o.group) == ('amr_02', 'amr_01', ('amr_01', 'amr_02'))
    assert o.direction == BACK   # 상대가 앞에 있다


def test_same_priority_later_id_yields_and_moves_forward_when_other_is_behind():
    det = StandoffDetector(CFG)
    feed(det, 'amr_01', 0.0, 0.0, yaw=0.0)
    feed(det, 'amr_03', 0.7, 0.0, yaw=0.0)   # amr_01이 amr_03 뒤에 있다
    [o] = det.check(30.0, no_priority)
    assert o.robot_id == 'amr_03'
    assert o.direction == FORWARD


def test_not_yet_still_long_enough():
    det = StandoffDetector(CFG)
    feed(det, 'amr_01', 0.0, 0.0, t0=25.0)
    feed(det, 'amr_02', 0.6, 0.0, t0=25.0)
    assert det.check(30.0, no_priority) == []


def test_moving_robot_is_not_in_standoff():
    det = StandoffDetector(CFG)
    feed(det, 'amr_01', 0.0, 0.0)
    t = 0.0
    while t <= 30.0:
        # 0.05 m/s로 천천히 지나간다 (10 s에 0.5 m > still_m). 11/2 북쪽 통로처럼 0.02 m/s로 밀리는 건
        # 10 s에 0.2 m라 멈춤으로 잡힌다 - 그건 대치가 맞다
        det.update('amr_02', t, 0.6 + 0.05 * t, 0.0, 0.0, True)
        t += 0.5
    assert det.check(30.0, no_priority) == []


def test_far_apart_or_idle_only_is_not_standoff():
    det = StandoffDetector(CFG)
    feed(det, 'amr_01', 0.0, 0.0)
    feed(det, 'amr_02', 2.0, 0.0)                 # 1.2 m 밖
    feed(det, 'amr_04', 10.0, 0.0, busy=False)
    feed(det, 'amr_05', 10.5, 0.0, busy=False)    # 둘 다 대기 중: 줄 서 있는 것이지 대치가 아니다
    assert det.check(30.0, no_priority) == []


def test_idle_robot_in_the_way_yields_first():
    det = StandoffDetector(CFG)
    feed(det, 'amr_01', 0.0, 0.0, busy=False)
    feed(det, 'amr_02', 0.6, 0.0, yaw=math.pi)
    prio = {'amr_01': 0, 'amr_02': 0}
    [o] = det.check(30.0, prio.get)
    assert o.robot_id == 'amr_01'


def test_inactive_robot_is_ignored():
    det = StandoffDetector(CFG)
    feed(det, 'amr_01', 0.0, 0.0)
    feed(det, 'amr_02', 0.6, 0.0, active=False)   # 충전·오류
    assert det.check(30.0, no_priority) == []


def test_chain_of_three_is_one_group_and_cooldown_blocks_repeat():
    det = StandoffDetector(CFG)
    feed(det, 'amr_01', 0.0, 0.0)
    feed(det, 'amr_02', 1.0, 0.0)
    feed(det, 'amr_03', 2.0, 0.0)     # amr_01과는 2 m지만 amr_02를 거쳐 이어진다
    [o] = det.check(30.0, no_priority)
    assert o.group == ('amr_01', 'amr_02', 'amr_03')
    assert o.robot_id == 'amr_03' and o.yield_to == 'amr_02'
    feed(det, 'amr_01', 0.0, 0.0, t0=30.5, t1=49.0)
    feed(det, 'amr_02', 1.0, 0.0, t0=30.5, t1=49.0)
    feed(det, 'amr_03', 2.0, 0.0, t0=30.5, t1=49.0)
    assert det.check(49.0, no_priority) == []          # cooldown 20 s 안
    feed(det, 'amr_01', 0.0, 0.0, t0=49.5, t1=51.0)
    feed(det, 'amr_02', 1.0, 0.0, t0=49.5, t1=51.0)
    feed(det, 'amr_03', 2.0, 0.0, t0=49.5, t1=51.0)
    assert len(det.check(51.0, no_priority)) == 1      # 그래도 안 풀렸으면 다시 지시


def test_old_or_duplicate_state_is_ignored():
    det = StandoffDetector(CFG)
    feed(det, 'amr_01', 0.0, 0.0)
    det.update('amr_01', 10.0, 5.0, 5.0, 0.0, True)     # 늦게 온 옛 상태
    feed(det, 'amr_02', 0.6, 0.0)
    assert len(det.check(30.0, no_priority)) == 1


def test_direction_flips_when_a_third_robot_blocks_the_way_back():
    det = StandoffDetector(CFG)
    feed(det, 'amr_01', 0.0, 0.0, yaw=0.0)
    feed(det, 'amr_02', 0.2, 0.8, yaw=math.pi)             # 상대, 비스듬히 앞 (옆으로 0.8 > corridor라 앞길은 안 막음)
    feed(det, 'amr_03', -1.0, 0.1, yaw=0.0, active=False)  # 뒤에 서 있는 로봇 (도킹 중이라 묶음엔 안 들어감)
    prio = {'amr_01': 0, 'amr_02': 5, 'amr_03': 0}
    [o] = det.check(30.0, prio.get)
    assert (o.robot_id, o.group) == ('amr_01', ('amr_01', 'amr_02'))
    assert o.direction == FORWARD   # 상대에게서 멀어지는 쪽(뒤)은 amr_03이 막아 앞으로


def test_next_candidate_when_victim_boxed_in():
    det = StandoffDetector(CFG)
    feed(det, 'amr_01', 0.0, 0.0, yaw=0.0)
    feed(det, 'amr_02', 0.65, 0.0, yaw=math.pi)
    feed(det, 'amr_03', -0.9, 0.0, yaw=0.0, active=False)   # amr_01 뒤
    prio = {'amr_01': 0, 'amr_02': 5}
    [o] = det.check(30.0, prio.get)
    # amr_01은 앞(amr_02)·뒤(amr_03) 다 막혔다 → 우선순위가 높아도 amr_02가 비킨다 (amr_02 뒤는 비어 있다)
    assert o.robot_id == 'amr_02' and o.direction == BACK


def test_no_order_when_everyone_boxed_in():
    det = StandoffDetector(CFG)
    feed(det, 'amr_01', 0.0, 0.0, yaw=0.0)
    feed(det, 'amr_02', 0.65, 0.0, yaw=math.pi)
    feed(det, 'amr_03', -0.9, 0.0, active=False)
    feed(det, 'amr_04', 1.55, 0.0, active=False)
    assert det.check(30.0, no_priority) == []
