"""작업 생성기가 JSON Task 일정대로, map 좌표로 바꿔 Task를 발행하는지 확인한다 (launch_testing)."""

import json
import math
import os
import tempfile
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from amr_msgs.msg import Task
from ament_index_python.packages import get_package_share_directory

TASKS = {'tasks': [
    {'at_s': 0.0, 'task_id': 'gen_0001', 'type': 'delivery', 'item': 'large',
     'pickup': 'dock_1', 'dropoff': 'rack_a4', 'priority': 3, 'deadline_s': 60.0},
    {'at_s': 1.0, 'task_id': 'gen_0002', 'type': 'charge', 'item': 'none', 'pickup': '', 'dropoff': 'charger_c1'},
]}


@pytest.mark.launch_test
def generate_test_description():
    tmp = tempfile.mkdtemp()
    tasks_file = os.path.join(tmp, 'tasks.json')
    with open(tasks_file, 'w', encoding='utf-8') as f:
        json.dump(TASKS, f)
    generator = launch_ros.actions.Node(
        package='amr_fleet_manager',
        executable='task_generator',
        parameters=[{
            'tasks_file': tasks_file,
            'stations_file': os.path.join(get_package_share_directory('amr_fleet_manager'), 'config', 'stations.yaml'),
            'map_origin_in_world': [0.0, 1.5, 0.0],
        }],
    )
    return launch.LaunchDescription([generator, launch_testing.actions.ReadyToTest()])


class TestTaskGenerator(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_task_generator')

    def tearDown(self):
        self.node.destroy_node()

    def test_publishes_schedule(self):
        received = []
        # 다른 패키지 테스트와 섞이지 않게 이 일정의 작업만 센다
        self.node.create_subscription(
            Task, '/fleet/task_requests',
            lambda msg: received.append((time.monotonic(), msg)) if msg.task_id.startswith('gen_') else None, 10)

        end = time.monotonic() + 20.0
        while time.monotonic() < end and len(received) < 2:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual([m.task_id for _, m in received], ['gen_0001', 'gen_0002'])

        (t1, first), (t2, second) = received
        # 두 번째는 1초 뒤 (노드 시계 = 실제 시간, 타이머 주기 0.1초)
        self.assertAlmostEqual(t2 - t1, 1.0, delta=0.4)

        self.assertEqual((first.task_type, first.item_type, first.priority), (Task.TYPE_DELIVERY, Task.ITEM_LARGE, 3))
        self.assertEqual((first.pickup_station_id, first.dropoff_station_id), ('dock_1', 'rack_a4'))
        # dock_1 world (−27, 16, π) → map (−27, 14.5, π)
        self.assertAlmostEqual(first.pickup_pose.position.x, -27.0, places=6)
        self.assertAlmostEqual(first.pickup_pose.position.y, 14.5, places=6)
        self.assertAlmostEqual(abs(first.pickup_pose.orientation.z), 1.0, places=6)
        # rack_a4 world (0, 3.6, π/2) → map (0, 2.1, π/2)
        self.assertAlmostEqual(first.dropoff_pose.position.y, 2.1, places=6)
        self.assertAlmostEqual(first.dropoff_pose.orientation.z, math.sin(math.pi / 4), places=6)
        deadline = first.deadline.sec + first.deadline.nanosec * 1e-9
        created = first.created_time.sec + first.created_time.nanosec * 1e-9
        self.assertAlmostEqual(deadline - created, 60.0, places=3)

        self.assertEqual((second.task_type, second.item_type), (Task.TYPE_CHARGE, Task.ITEM_NONE))
        self.assertEqual(second.pickup_station_id, '')
        self.assertEqual((second.deadline.sec, second.deadline.nanosec), (0, 0))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
