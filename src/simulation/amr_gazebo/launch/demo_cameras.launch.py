"""실행 중인 Gazebo에 시연 녹화용 고정 카메라를 스폰하고 이미지를 ROS로 넘긴다 (amr_gazebo.demo_cameras).

    ros2 launch amr_gazebo demo_cameras.launch.py                      # config/demo_cameras_restaurant.yaml 전부
    ros2 launch amr_gazebo demo_cameras.launch.py cameras:=top,west    # 일부만 (CPU 렌더링 부담 줄이기)

토픽: /demo_cam/<이름>/image (sensor_msgs/Image). 녹화는 ros2 bag record로 따로 한다.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_gazebo.demo_cameras import bridge_args, load_cameras, model_sdf


def _spawn(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    path = arg('config') or os.path.join(
        get_package_share_directory('amr_gazebo'), 'config', 'demo_cameras_restaurant.yaml')
    cameras = load_cameras(path)
    chosen = [n.strip() for n in arg('cameras').split(',') if n.strip()] or list(cameras)
    unknown = [n for n in chosen if n not in cameras]
    if unknown:
        raise RuntimeError(f'{path}에 없는 카메라: {unknown} (있는 것: {list(cameras)})')
    selected = [cameras[n] for n in chosen]
    sdf = tempfile.NamedTemporaryFile('w', suffix='.sdf', prefix='demo_cameras_', delete=False)
    sdf.write(model_sdf(selected))
    sdf.close()
    use_sim_time = arg('use_sim_time').lower() == 'true'
    return [
        Node(package='ros_gz_sim', executable='create', arguments=['-file', sdf.name, '-name', 'demo_cameras'],
             output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='demo_cameras_bridge',
             arguments=bridge_args(selected), parameters=[{'use_sim_time': use_sim_time}], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='', description='카메라 yaml (기본 config/demo_cameras_restaurant.yaml)'),
        DeclareLaunchArgument('cameras', default_value='', description='쓸 카메라 이름 목록 (쉼표). 비우면 전부'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_spawn),
    ])
