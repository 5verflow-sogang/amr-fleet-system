"""전역 planner 비교용 최소 스택: map_server + planner_server(planner 5개) + 정적 TF. Gazebo·로봇·위치추정은 안 띄운다.

    ros2 launch experiments/planner_compare/planner_compare.launch.py map:=<지도 yaml>

planner_server의 전역 costmap·planner 값은 amr_bringup config/nav2_params.yaml에서 그대로 가져오고 두 가지만 바꾼다:
- 전역 costmap 레이어는 static + inflation만 (센서가 없어 obstacle_layer가 있으면 costmap이 "최신"이 안 돼 planner가 기다린다)
- planner_plugins: NavFn(Dijkstra, yaml 값) · NavFnAStar(스택이 실제로 쓰는 use_astar: true, nav2.launch.py) · Smac2D(기본값)
  · AStarRaw(우리 A*, 평활화 끔) · AStar(우리 A*, nav2_params.yaml AStar 블록 그대로)
compare.py가 ComputePathToPose(use_start, planner_id)로 같은 출발·목표를 planner마다 부른다.
"""

import copy
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PLANNERS = ['NavFn', 'NavFnAStar', 'Smac2D', 'AStarRaw', 'AStar']


def planner_params(nav2_params: dict) -> dict:
    """nav2_params.yaml → 비교용 planner_server·global_costmap 파라미터 (네임스페이스 없음)."""
    costmap = copy.deepcopy(nav2_params['global_costmap']['global_costmap']['ros__parameters'])
    costmap['plugins'] = ['static_layer', 'inflation_layer']
    costmap.pop('obstacle_layer', None)
    costmap['use_sim_time'] = False

    planner = nav2_params['planner_server']['ros__parameters']
    navfn = copy.deepcopy(planner['GridBased'])
    navfn_astar = dict(navfn, use_astar=True)
    astar = copy.deepcopy(planner['AStar'])
    astar_raw = dict(astar, smooth_path=False)
    server = {
        'use_sim_time': False,
        'expected_planner_frequency': planner['expected_planner_frequency'],
        'planner_plugins': PLANNERS,
        'NavFn': navfn,
        'NavFnAStar': navfn_astar,
        'Smac2D': {'plugin': 'nav2_smac_planner/SmacPlanner2D'},
        'AStarRaw': astar_raw,
        'AStar': astar,
    }
    return {'planner_server': {'ros__parameters': server},
            'global_costmap': {'global_costmap': {'ros__parameters': costmap}}}


def _nodes(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    with open(params_file) as f:
        params = planner_params(yaml.safe_load(f))
    out = tempfile.NamedTemporaryFile(mode='w', prefix='planner_compare_', suffix='.yaml', delete=False)
    yaml.dump(params, out)
    out.close()
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': LaunchConfiguration('map').perform(context), 'use_sim_time': False}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[out.name]),
        # 전역 costmap이 robot_base_frame(base_footprint) TF를 찾는다. 출발점은 요청(use_start)으로 준다
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_base', output='log',
             arguments=['--frame-id', 'map', '--child-frame-id', 'base_footprint']),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_planner',
             output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'planner_server'], 'use_sim_time': False}]),
    ]


def generate_launch_description():
    bringup = get_package_share_directory('amr_bringup')
    mapping = get_package_share_directory('amr_mapping')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('map', default_value=os.path.join(mapping, 'maps', 'warehouse_v2_0025.yaml'),
                              description='지도 yaml (기본: 스택 기본 지도 v2 0.025 m)'),
        OpaqueFunction(function=_nodes),
    ])
