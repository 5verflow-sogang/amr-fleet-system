"""적재 질량 제동 거리 실험용 빈 평평한 월드만 띄운다 (수동 확인용).

    ros2 launch world.launch.py
    ros2 launch world.launch.py gui:=true   # noVNC로 눈으로 보고 싶을 때

scripts/run_experiment.py는 이 launch 파일을 쓰지 않고 `ign gazebo -s -r`을
직접 subprocess로 띄운다 - 트라이얼마다 모델을 지우고 다시 스폰해야 해서
launch 파일보다 세밀한 제어가 필요하기 때문이다(README "구현 노트" 참고).
이 launch 파일은 사람이 수동으로 월드/모델을 확인할 때 쓴다.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

WORLD_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           'world', 'flat_ground.sdf')


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false', description='Gazebo GUI 실행 여부'),
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', WORLD_PATH],
            output='screen',
            condition=IfCondition(gui),
        ),
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-s', '-r', WORLD_PATH],
            output='screen',
            condition=UnlessCondition(gui),
        ),
    ])
