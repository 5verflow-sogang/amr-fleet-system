"""로봇 시점 RViz 설정 (rviz/robot_view.rviz). ROS 없이 돈다."""

from pathlib import Path

import yaml

from amr_bringup.robot_view import robot_config

RVIZ = Path(__file__).resolve().parents[1] / 'rviz' / 'robot_view.rviz'


def _topics(config: dict) -> dict:
    return {d['Name']: d['Topic']['Value'] for d in config['Visualization Manager']['Displays'] if 'Topic' in d}


def test_robot_view_shows_the_scene_layers_for_amr_01():
    config = yaml.safe_load(RVIZ.read_text(encoding='utf-8'))
    topics = _topics(config)
    assert topics == {
        'Map': '/map',
        'Global Costmap': '/amr_01/global_costmap/costmap',
        'Congestion Mask': '/fleet/congestion_mask',
        'LiDAR': '/amr_01/scan',
        'Global Plan': '/amr_01/plan',
        'DWA Local Trajectory': '/amr_01/dwa/local_plan',
        'Footprint': '/amr_01/local_costmap/published_footprint',
        'People Tracking': '/amr_01/perception/obstacle_markers',
    }
    assert config['Visualization Manager']['Global Options']['Fixed Frame'] == 'map'
    assert config['Visualization Manager']['Views']['Current']['Target Frame'] == 'amr_01/base_footprint'


def test_robot_view_for_another_robot_only_changes_the_robot_name():
    # 1-3 등 다른 장면: 로봇 이름만 바꾸면 같은 화면 (지도·혼잡 마스크 같은 공용 토픽은 그대로)
    text = RVIZ.read_text(encoding='utf-8')
    other = yaml.safe_load(robot_config(text, 'amr_03'))
    topics = _topics(other)
    assert topics['Global Plan'] == '/amr_03/plan' and topics['LiDAR'] == '/amr_03/scan'
    assert topics['Map'] == '/map' and topics['Congestion Mask'] == '/fleet/congestion_mask'
    assert other['Visualization Manager']['Views']['Current']['Target Frame'] == 'amr_03/base_footprint'
    assert 'amr_01' not in robot_config(text, 'amr_03')
