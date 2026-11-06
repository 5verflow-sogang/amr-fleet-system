"""자동 충전 배정 (charging). ROS 없이 돈다."""

import pytest

from amr_fleet_core.assignment import RobotInfo
from amr_fleet_core.charging import ChargeConfig, ChargeOrder, Station, max_charging, nearest_free, plan_charging

CHARGERS = [Station('c1', (0.0, 0.0)), Station('c2', (2.0, 0.0)), Station('c3', (4.0, 0.0))]


def robot(robot_id, battery, xy=(10.0, 10.0)):
    return RobotInfo(robot_id=robot_id, xy=xy, battery_percent=battery, mode=0)


def ids(robots):
    return {r.robot_id for r in robots}


def test_config_validation():
    for kwargs in ({'low_percent': -1.0}, {'low_percent': 101.0}, {'min_in_service': -1},
                   {'occupied_radius_m': -0.1}):
        with pytest.raises(ValueError):
            ChargeConfig(**kwargs)


def test_max_charging_keeps_min_in_service():
    assert max_charging(5, ChargeConfig(min_in_service=3)) == 2
    assert max_charging(2, ChargeConfig(min_in_service=3)) == 0     # 이미 N대 밑이면 더 보내지 않는다
    assert max_charging(5, ChargeConfig()) == 5                     # 0이면 충전기 수로만 묶는다


def test_five_robots_drop_together_n3_sends_two_lowest_first():
    # 명세 완료 기준: 5대, N=3에서 배터리가 동시에 떨어져도 충전하러 가는 로봇은 2대를 넘지 않는다
    robots = [robot('amr_01', 25.0), robot('amr_02', 12.0), robot('amr_03', 28.0), robot('amr_04', 18.0),
              robot('amr_05', 22.0)]
    plan = plan_charging(robots, ids(robots), {}, CHARGERS, ChargeConfig(low_percent=30.0, min_in_service=3))
    assert [o.robot_id for o in plan.orders] == ['amr_02', 'amr_04']
    assert plan.waiting == ['amr_05', 'amr_01', 'amr_03']


def test_slots_count_robots_already_charging():
    robots = [robot(f'amr_0{i}', 20.0) for i in range(1, 6)]
    plan = plan_charging(robots, ids(robots), {'amr_01': 'c1'}, CHARGERS, ChargeConfig(min_in_service=3))
    assert [o.robot_id for o in plan.orders] == ['amr_02']
    plan = plan_charging(robots, ids(robots), {'amr_01': 'c1', 'amr_02': 'c2'}, CHARGERS,
                         ChargeConfig(min_in_service=3))
    assert plan.orders == [] and plan.waiting == ['amr_03', 'amr_04', 'amr_05']


def test_charger_count_limits_without_min_in_service():
    robots = [robot(f'amr_0{i}', 10.0 + i) for i in range(1, 6)]
    plan = plan_charging(robots, ids(robots), {}, CHARGERS, ChargeConfig())
    assert [o.robot_id for o in plan.orders] == ['amr_01', 'amr_02', 'amr_03']
    assert sorted(o.station for o in plan.orders) == ['c1', 'c2', 'c3']
    assert plan.waiting == ['amr_04', 'amr_05']


def test_nearest_free_charger_and_reserved_skipped():
    robots = [robot('amr_01', 10.0, xy=(2.1, 1.0)), robot('amr_02', 50.0)]
    plan = plan_charging(robots, ids(robots), {}, CHARGERS, ChargeConfig())
    assert plan.orders == [ChargeOrder('amr_01', 'c2')]
    # 가장 가까운 c3는 amr_02가 예약 → 다음으로 가까운 c2
    robots = [robot('amr_01', 10.0, xy=(3.1, 1.0)), robot('amr_02', 50.0)]
    plan = plan_charging(robots, ids(robots), {'amr_02': 'c3'}, CHARGERS, ChargeConfig())
    assert plan.orders == [ChargeOrder('amr_01', 'c2')]


def test_busy_robot_waits_until_free():
    robots = [robot('amr_01', 10.0), robot('amr_02', 15.0)]
    plan = plan_charging(robots, {'amr_02'}, {}, CHARGERS, ChargeConfig())
    assert [o.robot_id for o in plan.orders] == ['amr_02']
    assert plan.waiting == ['amr_01']


def test_robot_standing_on_charger_blocks_it_for_others_only():
    # amr_02가 c1 위에 서 있다 (예약 없이: 충전을 마치고 일을 기다림). amr_01은 c1 옆이지만 c2로
    robots = [robot('amr_01', 10.0, xy=(0.5, 0.5)), robot('amr_02', 90.0, xy=(0.0, 0.0))]
    plan = plan_charging(robots, ids(robots), {}, CHARGERS, ChargeConfig())
    assert plan.orders == [ChargeOrder('amr_01', 'c2')]
    # 서 있는 로봇 자신이 낮아지면 그 자리를 쓴다
    robots = [robot('amr_02', 10.0, xy=(0.0, 0.0))]
    assert plan_charging(robots, ids(robots), {}, CHARGERS, ChargeConfig()).orders == [ChargeOrder('amr_02', 'c1')]


def test_above_low_is_not_sent():
    robots = [robot('amr_01', 30.0), robot('amr_02', 29.9)]
    plan = plan_charging(robots, ids(robots), {}, CHARGERS, ChargeConfig(low_percent=30.0))
    assert [o.robot_id for o in plan.orders] == ['amr_02'] and plan.waiting == []


def test_nearest_free_ties_by_name():
    stations = [Station('b', (1.0, 0.0)), Station('a', (-1.0, 0.0))]
    assert nearest_free((0.0, 0.0), stations, set()).name == 'a'
    assert nearest_free((0.0, 0.0), stations, {'a', 'b'}) is None
