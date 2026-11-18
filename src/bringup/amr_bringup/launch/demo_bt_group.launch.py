"""장면 3 배터리 시계를 맞추기 위해 Nav2·카메라 준비 후 작업 노드들을 함께 시작한다."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _group(context):
    count = int(LaunchConfiguration('num_robots').perform(context))
    if not 1 <= count <= 5:
        raise ValueError('num_robots must be between 1 and 5')
    share = get_package_share_directory('amr_bt')
    extra = LaunchConfiguration('params_file').perform(context)
    if not os.path.isabs(extra):
        extra = os.path.join(share, 'config', extra)
    return [Node(package='amr_bt', executable='amr_bt_node', name='amr_bt',
                 namespace=f'amr_{i:02d}', output='screen', parameters=[
                     os.path.join(share, 'config', 'amr_bt.yaml'), extra,
                     {'use_sim_time': True, 'use_nav2': True, 'use_docking': False,
                      'server_topic_prefix': 'fleet'}]) for i in range(1, count + 1)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='5'),
        DeclareLaunchArgument('params_file', default_value='amr_bt_battery_demo_s3_gazebo.yaml'),
        OpaqueFunction(function=_group),
    ])
