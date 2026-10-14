"""로봇 1대의 Nav2 (경로 계획 planner_server, 주행 controller_server, behavior_server, bt_navigator).

system.launch.py가 로봇마다 부른다. 월드·지도·위치추정은 안 띄운다 (map → <ns>/odom TF는 AMCL + EKF가 낸다).

    ros2 launch amr_bringup nav2.launch.py namespace:=amr_02

nav2_params.yaml은 네임스페이스를 가정하지 않는다 (CONTRIBUTING 4-5절). 프레임 이름이 들어가는 값은 RewrittenYaml로 덮어쓴다.
planner_server는 global_costmap만, controller_server는 local_costmap만 읽으므로 둘에 서로 다른 재작성 파일을 준다
(global_frame이 map / <ns>/odom으로 다르다). 자세한 경위는 nav2_dwa_demo.launch.py와 docs/algorithms/navigation.md "DWA·Nav2 연결".

컨트롤러: bt_navigator 기본 트리가 "FollowPath"라는 이름을 하드코딩해서 부르므로, nav2_params.yaml의 DWA 설정을 FollowPath 이름으로 등록한다.
속도 명령은 cmd_vel이 아니라 cmd_vel_nav로 낸다 (cmd_vel 체인: cmd_vel_nav → 구역 클라이언트 → cmd_vel_zone_limited → 안전 노드 → cmd_vel).
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

LIFECYCLE_NODES = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']


def _nav2(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    ns = arg('namespace')
    use_sim_time = {'use_sim_time': arg('use_sim_time').lower() == 'true'}
    odom_topic = arg('odom_topic')
    cmd_vel_topic = arg('cmd_vel_topic')
    params_file = arg('params_file')
    base_footprint = f'{ns}/base_footprint'
    odom_frame = f'{ns}/odom'
    scan_topic = f'/{ns}/scan'

    with open(params_file) as f:
        dwa_config = yaml.safe_load(f)['controller_server']['ros__parameters']['DWA']
    overrides = {ns: {'controller_server': {'ros__parameters': {
        'controller_plugins': ['FollowPath'],
        'FollowPath': dwa_config,
        'odom_topic': odom_topic,
    }}}}
    # 로봇마다 다른 임시 파일이 필요하다 (프로세스가 끝나면 OS가 지운다)
    overrides_file = tempfile.NamedTemporaryFile(
        mode='w', prefix=f'nav2_{ns}_', suffix='.yaml', delete=False)
    yaml.dump(overrides, overrides_file)
    overrides_file.close()

    planner_params = RewrittenYaml(
        source_file=params_file, root_key=ns,
        param_rewrites={'robot_base_frame': base_footprint, 'topic': scan_topic, 'map_topic': '/map'},
        convert_types=True)
    controller_params = RewrittenYaml(
        source_file=params_file, root_key=ns,
        param_rewrites={'robot_base_frame': base_footprint, 'global_frame': odom_frame,
                        'map_topic': '/map', 'topic': scan_topic},
        convert_types=True)

    # 컨트롤러와 복구 행동(spin/backup)이 낸 속도는 모두 cmd_vel_nav로 모은다
    cmd_vel_remap = [('cmd_vel', cmd_vel_topic)]
    return [
        Node(package='nav2_planner', executable='planner_server', name='planner_server', namespace=ns,
             parameters=[planner_params, use_sim_time, {'GridBased.use_astar': True}], output='screen'),
        Node(package='nav2_controller', executable='controller_server', name='controller_server', namespace=ns,
             parameters=[controller_params, use_sim_time, overrides_file.name],
             remappings=cmd_vel_remap, output='screen'),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', namespace=ns,
             parameters=[{'robot_base_frame': base_footprint, 'global_frame': odom_frame, **use_sim_time}],
             remappings=cmd_vel_remap, output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', namespace=ns,
             parameters=[{'global_frame': 'map', 'robot_base_frame': base_footprint,
                          'odom_topic': odom_topic, **use_sim_time}],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', namespace=ns,
             parameters=[{'autostart': True, 'node_names': LIFECYCLE_NODES, **use_sim_time}],
             output='screen'),
    ]


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('amr_bringup'), 'config', 'nav2_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01', description='로봇 네임스페이스'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params, description='Nav2 파라미터'),
        DeclareLaunchArgument(
            'odom_topic', default_value='odometry/local',
            description='컨트롤러가 속도를 읽는 odom (상대 이름). 위치추정 없이는 diff_drive/odom'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='cmd_vel_nav',
            description='Nav2 속도 명령 출력. 구역 클라이언트·안전 노드가 없으면 cmd_vel'),
        OpaqueFunction(function=_nav2),
    ])
