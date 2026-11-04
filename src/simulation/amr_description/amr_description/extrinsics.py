"""config/extrinsics.yaml 읽기 (ROS 노드 없이).

센서 위치가 필요한 Python 코드는 숫자를 따로 두지 말고 여기서 읽는다. 그래야 파일만 고쳐도 같이 따라온다
(#266 민지 리뷰: 화물 상자·도킹 마커판의 스캔 평면 비키기 확인이 LiDAR 높이 0.38을 따로 갖고 있었다).
"""

import os

from ament_index_python.packages import get_package_share_directory
import yaml

# base_footprint(지면) → base_link 높이 [m] = ground_clearance 0.03 + base_height 0.3 / 2 (urdf/properties.xacro base_z).
# 로봇 몸체 치수라 extrinsic 파일에 두지 않는다. test_extrinsics.py가 URDF base_joint와 같은지 본다
BASE_LINK_HEIGHT = 0.18


def load(path=None) -> dict:
    """extrinsics.yaml 내용. path를 비우면 설치된 amr_description의 파일."""
    if path is None:
        path = os.path.join(get_package_share_directory('amr_description'), 'config', 'extrinsics.yaml')
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def lidar_height(extrinsics=None) -> float:
    """2D LiDAR 스캔 평면의 지면 높이 [m] = base_link 높이 + lidar z."""
    e = (extrinsics if extrinsics is not None else load())['lidar']
    if e['parent'] != 'base_link':
        raise ValueError(f"lidar parent가 base_link가 아니다: {e['parent']}")
    return BASE_LINK_HEIGHT + e['xyz'][2]
