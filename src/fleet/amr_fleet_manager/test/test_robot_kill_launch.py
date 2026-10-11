"""로봇 장애 → 작업 회수 → 다른 로봇에 다시 할당 (launch_testing).

서버 + 가짜 로봇 2대를 띄우고, 작업을 맡아 이동 중인 amr_01을 fake_robots.launch.py의 kill_robot으로 죽인다.
서버가 RobotState 끊김(reclaim_timeout_s)으로 작업을 회수해 amr_02에게 새 assignment_id로 맡기고, amr_02가 끝내는지 본다.

ROS_DOMAIN_ID를 이 테스트 전용 값으로 둔다. CI에서 다른 패키지의 Fleet 서버가 같은 도메인에서 돌면 작업을 가로챈다 (#109와 같은 방식).
"""

import csv
import os
import tempfile
import time
import unittest
import zlib

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from amr_msgs.msg import Task, TaskAssignment, TaskStatus
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

_TEST_ROS_DOMAIN_ID = str(zlib.crc32(b'amr_fleet_manager/test_robot_kill') % 102)
# 가짜 로봇은 1 m/s. 작업을 받은 amr_01이 픽업(14 m 떨어짐)으로 가는 도중에 죽도록 한다
KILL_AT_S = 10.0
PICKUP_X = -12.0


@pytest.mark.launch_test
def generate_test_description():
    tmp = tempfile.mkdtemp()
    events = os.path.join(tmp, 'fault_events.csv')
    share = get_package_share_directory('amr_fleet_manager')
    fleet = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'fleet_manager.launch.py')),
        launch_arguments={'epoch_file': os.path.join(tmp, 'epoch'), 'zones_file': ''}.items(),
    )
    robots = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'fake_robots.launch.py')),
        launch_arguments={
            'count': '2', 'kill_robot': 'amr_01', 'kill_robot_at_s': str(KILL_AT_S),
            'fault_events_file': events,
        }.items(),
    )
    return launch.LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', _TEST_ROS_DOMAIN_ID),
        fleet, robots, launch_testing.actions.ReadyToTest(),
    ]), {'events': events}


class TestRobotKill(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_robot_kill')

    def tearDown(self):
        self.node.destroy_node()

    def spin_until(self, condition, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not condition():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return condition()

    def test_dead_robot_task_reassigned(self, events):
        assignments, statuses = [], []
        self.node.create_subscription(
            TaskAssignment, '/fleet/task_assignments',
            lambda m: assignments.append(m) if m.task.task_id == 'task_kill' else None, 10)
        self.node.create_subscription(
            TaskStatus, '/fleet/task_status',
            lambda m: statuses.append(m) if m.task_id == 'task_kill' else None, 10)
        publisher = self.node.create_publisher(Task, '/fleet/task_requests', 10)
        self.assertTrue(self.spin_until(
            lambda: publisher.get_subscription_count() >= 1
            and self.node.count_publishers('/fleet/robot_states') >= 2, 15.0))
        self.spin_until(lambda: False, 1.0)

        task = Task(task_id='task_kill', task_type=Task.TYPE_DELIVERY, item_type=Task.ITEM_SMALL)
        task.pickup_pose.position.x = PICKUP_X
        task.pickup_pose.position.y = 2.0
        task.dropoff_pose.position.x = PICKUP_X
        task.dropoff_pose.position.y = 3.0
        publisher.publish(task)

        # 가까운 amr_01(시작 x=2)이 받아 움직이기 시작한다. 죽기 전에 이걸 못 보면 시나리오가 성립하지 않는다
        self.assertTrue(self.spin_until(
            lambda: any(s.robot_id == 'amr_01' and s.state == TaskStatus.STATE_IN_PROGRESS for s in statuses),
            KILL_AT_S - 2.0), f'amr_01이 죽기 전에 작업을 시작하지 않음: {[(a.robot_id) for a in assignments]}')

        done = self.spin_until(
            lambda: any(s.robot_id == 'amr_02' and s.state == TaskStatus.STATE_COMPLETED for s in statuses), 60.0)
        self.assertTrue(done, f'amr_02 완료 보고 없음: {[(a.robot_id, a.assignment_id) for a in assignments]}, '
                              f'{[(s.robot_id, s.state, s.phase) for s in statuses]}')

        robots = [a.robot_id for a in assignments]
        self.assertEqual(robots[0], 'amr_01')
        self.assertIn('amr_02', robots)
        first = assignments[0].assignment_id
        second = next(a for a in assignments if a.robot_id == 'amr_02').assignment_id
        self.assertGreater(second, first)
        self.assertFalse(any(s.robot_id == 'amr_01' and s.state == TaskStatus.STATE_COMPLETED for s in statuses))

        with open(events, newline='') as f:
            rows = list(csv.reader(f))
        self.assertEqual([r[1] for r in rows[1:]], ['robot_kill:amr_01'])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # 죽인 로봇은 SIGKILL(-9)로 끝난다
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -9])
