"""혼잡도 노드. 손님 정답 위치로 구역별 혼잡도(/fleet/congestion)와 costmap 마스크를 발행한다 (api.md 4-5절).

    ros2 launch amr_fleet_manager congestion.launch.py                                  # 물류센터 가짜 구역
    ros2 launch amr_fleet_manager congestion.launch.py site:=restaurant guests:=peak    # 식당, 피크 몰림 손님
    ros2 launch amr_fleet_manager congestion.launch.py zones_file:=<구역> obstacles_file:=<손님> map_origin_in_world:='[x, y, yaw]'

site가 구역 파일·손님 파일·map 원점의 기본값을 한 번에 고른다. 셋 중 직접 준 것은 그 값을 쓴다.
식당은 map = world([0, 0, 0])라, 물류센터 원점이 그대로 들어가면 손님이 1.47 m 밀려 다른 구역에 세어진다.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# site → (구역 파일, 손님 파일 {guests: 파일}, map_origin_in_world)
# 손님 파일은 amr_gazebo config 아래. 식당 손님 파일에는 손님만 있다(직원은 _staff.yaml, 서 있는 손님은 세지 않는다)
SITES = {
    'warehouse': ('congestion_zones.yaml', {'normal': 'dynamic_obstacles.yaml'}, [-0.0439, 1.4712, -0.00088]),
    'restaurant': ('congestion_zones_restaurant.yaml',
                   {'normal': 'dynamic_obstacles_restaurant.yaml', 'peak': 'dynamic_obstacles_restaurant_peak.yaml',
                    # 시나리오 1-2: 피크에서 우회 통로(x -7.275)를 왕복하는 customer_2를 뺐다
                    'peak_scene12': 'dynamic_obstacles_restaurant_peak_scene12.yaml'},
                   [0.0, 0.0, 0.0]),
}


def _node(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    site, guests = arg('site'), arg('guests')
    if site not in SITES:
        raise ValueError(f'알 수 없는 site: {site} (가능: {sorted(SITES)})')
    zones_name, guest_files, origin = SITES[site]
    if guests not in guest_files:
        raise ValueError(f'{site}에 없는 guests: {guests} (가능: {sorted(guest_files)})')

    zones = arg('zones_file') or os.path.join(get_package_share_directory('amr_fleet_manager'), 'config', zones_name)
    obstacles = arg('obstacles_file')
    if not obstacles:
        # Fleet 컨테이너처럼 amr_gazebo가 없으면 obstacles_file을 직접 준다
        try:
            obstacles = os.path.join(get_package_share_directory('amr_gazebo'), 'config', guest_files[guests])
        except PackageNotFoundError:
            raise ValueError('amr_gazebo가 없다: obstacles_file을 직접 준다')
    origin_arg = arg('map_origin_in_world')
    if origin_arg:
        origin = [float(v) for v in origin_arg.strip('[] ').split(',')]
    return [Node(
        package='amr_fleet_manager',
        executable='congestion',
        parameters=[{
            'zones_file': zones,
            'obstacles_file': obstacles,
            'map_origin_in_world': origin,
            'use_sim_time': arg('use_sim_time').lower() == 'true',
        }],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('site', default_value='warehouse', description='warehouse / restaurant'),
        DeclareLaunchArgument('guests', default_value='normal',
                              description='손님 파일: normal / peak / peak_scene12 (식당만. peak는 서쪽 통로·배식대 앞에 6명 더, '
                                          'peak_scene12는 peak에서 우회 통로의 customer_2를 뺀 1-2용)'),
        DeclareLaunchArgument('zones_file', default_value='', description='혼잡도 구역 파일. 비우면 site 기본'),
        DeclareLaunchArgument(
            'obstacles_file', default_value='',
            description='손님 id를 읽을 동적 장애물 설정 (amr_gazebo dynamic_obstacles.yaml 형식). 비우면 site·guests 기본'),
        DeclareLaunchArgument(
            'map_origin_in_world', default_value='',
            description="world에서 본 map 원점 '[x, y, yaw]'. 비우면 site 기본 (물류센터 지도 v2, 식당 [0, 0, 0])"),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_node),
    ])
