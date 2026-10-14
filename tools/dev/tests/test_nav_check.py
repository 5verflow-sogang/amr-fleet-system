"""nav_check.py의 ROS 없이 도는 부분 (목표 해석, 로그 분류, launch 명령)."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nav_check import classify, launch_command, parse_goal  # noqa: E402


def test_goal_with_robot_and_yaw():
    g = parse_goal('amr_02:6,-1.5,1.57')
    assert (g.robot, g.x, g.y, g.yaw) == ('amr_02', 6.0, -1.5, 1.57)


def test_goal_without_robot_is_first_robot():
    g = parse_goal('20,-14')
    assert (g.robot, g.x, g.y, g.yaw) == ('amr_01', 20.0, -14.0, 0.0)


@pytest.mark.parametrize('text', ['amr_01:6', 'amr_01:1,2,3,4', 'robot1:6,0'])
def test_bad_goal(text):
    with pytest.raises(ValueError):
        parse_goal(text)


def test_classify_attributes_robot():
    line = '[controller_server-14] [WARN] [1791905515.9] [amr_02.controller_server]: DWA: 유효한 궤적을 못 찾아 정지한다'
    assert classify(line) == [('amr_02', 'dwa_no_traj')]
    assert classify('[bt_navigator-22] [ERROR] [1.0] [amr_01.bt_navigator]: Goal failed') == [('amr_01', 'goal_failed')]


def test_classify_process_died_has_no_robot():
    assert classify('[ERROR] [controller_server-14]: process has died [pid 812, exit code -11]') == [(None, 'died')]


def test_classify_ignores_other_lines():
    assert classify('[INFO] [amr_01.bt_navigator]: Begin navigating') == []


def test_launch_command_stage_and_extra():
    cmd = launch_command(2, 3, ['dynamic_obstacles:=true', 'nav2:=false'])
    assert cmd[:4] == ['ros2', 'launch', 'amr_bringup', 'system.launch.py']
    assert set(cmd[4:]) == {'num_robots:=2', 'nav2:=false', 'zone_client:=true', 'tracking:=true',
                            'safety:=true', 'dynamic_obstacles:=true'}


def test_launch_command_rejects_bad_extra():
    with pytest.raises(ValueError):
        launch_command(1, 2, ['dynamic_obstacles=true'])
