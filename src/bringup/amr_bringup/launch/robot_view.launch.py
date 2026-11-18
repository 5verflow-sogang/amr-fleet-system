"""로봇 시점 RViz (식당 시연 분할 영상): 지도·LiDAR 점·전역 costmap·혼잡 마스크·전역 경로·DWA 지역 궤적·발자국·사람 추적.

rviz/robot_view.rviz는 amr_01 기준이다. robot:=amr_03처럼 주면 설정 안의 amr_01을 그 이름으로 바꾼 임시 파일로 띄운다 -
1-3 등 다른 장면도 로봇 이름만 바꿔 같은 화면을 쓴다. 고정 프레임은 map, 화면은 그 로봇(base_footprint)을 따라가는 탑다운.

    ros2 launch amr_bringup robot_view.launch.py robot:=amr_01
    # 녹화 bag을 다시 틀 때(시뮬레이션 시계): tools/harness/rviz_video.sh <run_dir>/view_bag amr_01
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_bringup.robot_view import TEMPLATE_ROBOT, robot_config


def _rviz(context):
    robot = LaunchConfiguration('robot').perform(context)
    config = LaunchConfiguration('rviz_config').perform(context) or os.path.join(
        get_package_share_directory('amr_bringup'), 'rviz', 'robot_view.rviz')
    with open(config, encoding='utf-8') as f:
        text = robot_config(f.read(), robot)
    out = tempfile.NamedTemporaryFile('w', suffix=f'_{robot}.rviz', delete=False, encoding='utf-8')
    out.write(text)
    out.close()
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    return [Node(package='rviz2', executable='rviz2', name=f'rviz_{robot}', arguments=['-d', out.name],
                 parameters=[{'use_sim_time': use_sim_time}], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value=TEMPLATE_ROBOT, description='볼 로봇 (네임스페이스)'),
        DeclareLaunchArgument('rviz_config', default_value='', description='비우면 amr_bringup rviz/robot_view.rviz'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_rviz),
    ])
