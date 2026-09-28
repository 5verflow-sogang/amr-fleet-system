"""물류센터 월드를 Gazebo Fortress로 띄운다.

    ros2 launch amr_gazebo world.launch.py              # GUI 포함
    ros2 launch amr_gazebo world.launch.py gui:=false   # 서버만

models/ 경로는 package.xml의 gazebo_model_path export로 ros_gz_sim이 자동 등록한다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_gazebo(context):
    pkg_share = get_package_share_directory('amr_gazebo')

    world = LaunchConfiguration('world').perform(context)
    if not os.path.isabs(world):
        world = os.path.join(pkg_share, 'worlds', world)

    gz_args = f'{world} -r -v {LaunchConfiguration("verbosity").perform(context)}'
    if LaunchConfiguration('gui').perform(context).lower() != 'true':
        gz_args += ' -s'

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='warehouse.sdf',
            description='worlds/ 안의 파일 이름 또는 절대 경로'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='false면 GUI 없이 서버만 실행'),
        DeclareLaunchArgument(
            'verbosity', default_value='2',
            description='Gazebo 로그 레벨 (0~4)'),
        OpaqueFunction(function=_launch_gazebo),
        # 시뮬레이션 시간을 ROS로 넘긴다. 다른 노드는 use_sim_time:=true로 이 시간을 쓴다.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
            output='screen',
        ),
    ])
