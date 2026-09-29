"""xacro를 처리해 robot_state_publisher를 띄운다.

    ros2 launch amr_description description.launch.py namespace:=amr_01

발행: /<namespace>/robot_description, /tf_static (센서·캐스터), /tf (바퀴, joint_states 필요)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('amr_description'), 'urdf', 'amr.urdf.xacro')

    namespace = LaunchConfiguration('namespace')
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' namespace:=', namespace,
            ' payload_mass:=', LaunchConfiguration('payload_mass'),
        ]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='amr_01',
            description='로봇 네임스페이스. TF 프레임 접두어로도 쓴다'),
        DeclareLaunchArgument(
            'payload_mass', default_value='0.0',
            description='적재물 질량 [kg]'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Gazebo /clock 사용 여부'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen',
        ),
    ])
