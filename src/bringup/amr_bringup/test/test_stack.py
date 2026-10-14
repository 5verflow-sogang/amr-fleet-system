"""로봇 한 대의 launch 목록 (amr_bringup.stack). ROS 없이 돈다."""

import pytest

from amr_bringup.stack import Options, robot_stack

POSE = (21.0, -19.0, 1.57)
MAP_POSE = (21.0, -20.5, 1.57)


def stack(index=0, **flags):
    return robot_stack(Options(**flags), index, 'amr_02', POSE, MAP_POSE)


def launches(items):
    return [i.launch for i in items]


def args(items, launch):
    found = [i.arguments for i in items if i.launch == launch]
    assert len(found) == 1, (launch, launches(items))
    return found[0]


def test_default_is_stage_one():
    assert launches(stack()) == ['spawn_robot.launch.py', 'odometry.launch.py', 'sensor_filters.launch.py',
                                 'localization.launch.py', 'amr_bt.launch.py']


def test_stage_two():
    assert launches(stack(nav2=True, zone_client=True))[-2:] == ['nav2.launch.py', 'zone_client.launch.py']


def test_stage_three_safety_is_last():
    items = stack(nav2=True, zone_client=True, tracking=True, safety=True)
    assert launches(items)[-4:] == ['nav2.launch.py', 'zone_client.launch.py', 'tracking.launch.py',
                                    'safety.launch.py']


def test_every_include_gets_namespace_and_sim_time():
    items = stack(nav2=True, zone_client=True, tracking=True, safety=True,
                  detection=True, docking=True, enable_camera=True)
    for item in items:
        assert item.arguments['namespace'] == 'amr_02', item
        assert item.arguments['use_sim_time'] == 'true', item


@pytest.mark.parametrize('zone_client, safety, nav_out, zone_out, safety_in', [
    (False, False, 'cmd_vel', None, None),
    (True, False, 'cmd_vel_nav', 'cmd_vel', None),
    (False, True, 'cmd_vel_nav', None, 'cmd_vel_nav'),
    (True, True, 'cmd_vel_nav', 'cmd_vel_zone_limited', 'cmd_vel_zone_limited'),
])
def test_cmd_vel_topics_follow_chain(zone_client, safety, nav_out, zone_out, safety_in):
    items = stack(nav2=True, zone_client=zone_client, safety=safety)
    assert args(items, 'nav2.launch.py')['cmd_vel_topic'] == nav_out
    if zone_out:
        assert args(items, 'zone_client.launch.py')['cmd_vel_out_topic'] == zone_out
    if safety_in:
        assert args(items, 'safety.launch.py')['cmd_vel_in_topic'] == safety_in


def test_zone_client_uses_prefixed_frame_and_server_zones():
    items = robot_stack(Options(nav2=True, zone_client=True, zones_file='/z.yaml'), 0, 'amr_02', POSE, MAP_POSE)
    zone = args(items, 'zone_client.launch.py')
    assert (zone['robot_id'], zone['base_frame'], zone['zones_file']) == ('amr_02', 'amr_02/base_footprint', '/z.yaml')
    # zones.yaml은 world 좌표라 TF map ← world로 바꿔 쓴다
    assert zone['zones_frame'] == 'world'


def test_bt_follows_nav2_and_docking():
    assert (args(stack(), 'amr_bt.launch.py')['use_nav2'], args(stack(), 'amr_bt.launch.py')['use_docking']) \
        == ('false', 'false')
    bt = args(stack(nav2=True, docking=True, enable_camera=True), 'amr_bt.launch.py')
    assert (bt['use_nav2'], bt['use_docking']) == ('true', 'true')


def test_docking_server_drives_through_same_chain():
    items = stack(nav2=True, zone_client=True, safety=True, docking=True, enable_camera=True)
    assert args(items, 'docking.launch.py')['cmd_vel_topic'] == args(items, 'nav2.launch.py')['cmd_vel_topic']


def test_map_server_only_on_first_robot():
    assert args(stack(index=0), 'localization.launch.py')['start_map_server'] == 'true'
    assert args(stack(index=1), 'localization.launch.py')['start_map_server'] == 'false'


def test_localization_off_uses_diffdrive_tf():
    items = stack(localization=False)
    assert 'localization.launch.py' not in launches(items)
    assert args(items, 'spawn_robot.launch.py')['publish_odom_tf'] == 'true'


@pytest.mark.parametrize('flags, message', [
    ({'nav2': True, 'localization': False}, 'localization'),
    ({'detection': True}, 'enable_camera'),
    ({'docking': True, 'enable_camera': True}, 'nav2'),
    ({'docking': True, 'nav2': True}, 'enable_camera'),
])
def test_invalid_options(flags, message):
    with pytest.raises(RuntimeError, match=message):
        stack(**flags)
