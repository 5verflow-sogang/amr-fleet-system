"""연구 2 RL 평가 환경: 빈 월드 + 로봇 1대(적재 질량) + 에피소드 실행 노드 (rl_eval_node).

    ros2 launch amr_gazebo rl_eval.launch.py payload_mass:=25.0 episodes:=10 policy:=brake
    ros2 launch amr_gazebo rl_eval.launch.py payload_mass:=10.0 eval_list:=3 scenarios:=S1 csv:=/tmp/rl/list3.csv
    ros2 launch amr_gazebo rl_eval.launch.py policy:=my_pkg.ppo_policy:act   # 학습한 정책 (obs dict → v, ω)
    ros2 launch amr_gazebo rl_eval.launch.py mode:=brake_sweep payload_mass:=10.0 csv:=/tmp/rl/brake.csv   # 속도별 급정지

- 시나리오는 experiments/research2 (연우 2D 환경)와 같다: 같은 eval_list·에피소드 번호면 같은 통로·장애물
- 질량은 스폰할 때 URDF에 들어가므로 질량마다 다시 띄운다
- DiffDrive 가속 상한을 풀어(max_linear_acceleration 10) 바퀴 토크 한계(3 N·m)가 질량에 따라 가감속을 정한다
  (#55 이진 제동 실험: 상한 1.0이면 질량과 상관없이 같다). 2D 동역학(dynamics.py)과 같은 조건이다
- 카메라는 끈다 (정책 관측은 2D처럼 GT). 끝나면 노드가 멈춘다 (Ctrl-C로 끈다)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('amr_gazebo'), 'launch')
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('payload_mass', default_value='0.0', description='적재 질량 [kg]'),
        DeclareLaunchArgument('eval_list', default_value='0', description='평가 목록 k (2D seeds.eval_seed)'),
        DeclareLaunchArgument('episodes', default_value='5', description='목록에서 몇 개'),
        DeclareLaunchArgument('start_episode', default_value='0'),
        DeclareLaunchArgument('scenarios', default_value='S1,S2', description='S1(돌발 장애물), S2(정밀 정지), mixed'),
        DeclareLaunchArgument('policy', default_value='brake', description='cruise, brake 또는 모듈:함수'),
        DeclareLaunchArgument('csv', default_value='/tmp/rl_eval/episodes.csv'),
        DeclareLaunchArgument('research2_dir', default_value='', description='비우면 찾는다 (AMR_RESEARCH2_DIR 등)'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('mode', default_value='episodes', description='episodes 또는 brake_sweep (속도별 급정지)'),
        DeclareLaunchArgument('sweep_speeds', default_value='0.5,1.0,1.5,2.0', description='brake_sweep 속도 [m/s]'),
        DeclareLaunchArgument('sweep_repeats', default_value='3', description='brake_sweep 속도마다 횟수'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'world.launch.py')),
            launch_arguments={'world': 'rl_eval.sdf', 'gui': arg('gui'), 'verbosity': '1'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'spawn_robot.launch.py')),
            launch_arguments={
                'namespace': 'amr_01', 'x': '-10.0', 'y': '0.0', 'payload_mass': arg('payload_mass'),
                'max_linear_acceleration': '10.0', 'enable_camera': 'false', 'publish_odom_tf': 'true',
            }.items(),
        ),
        # 돌발 장애물 (에피소드마다 다시 스폰해도 Gazebo 토픽 이름이 같다)
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='popout_bridge',
            arguments=['/model/popout/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                       '/model/popout/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
            parameters=[{'use_sim_time': True}], output='screen',
        ),
        TimerAction(period=8.0, actions=[Node(
            package='amr_gazebo', executable='rl_eval_node.py', name='rl_eval', output='screen',
            parameters=[{
                'use_sim_time': True, 'payload_kg': arg('payload_mass'), 'eval_list': arg('eval_list'),
                'episodes': arg('episodes'), 'start_episode': arg('start_episode'),
                'scenarios': arg('scenarios'), 'policy': arg('policy'), 'csv': arg('csv'),
                'research2_dir': arg('research2_dir'), 'mode': arg('mode'),
                'sweep_speeds': arg('sweep_speeds'), 'sweep_repeats': arg('sweep_repeats'),
            }],
        )]),
    ])
