"""시연 고정 카메라 설정 → SDF·브리지 인자 (amr_gazebo.demo_cameras). ROS 없이 돈다."""

import math
import os
import xml.etree.ElementTree as ET

from amr_gazebo.demo_cameras import bridge_args, load_cameras, model_sdf

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(PKG, 'config', 'demo_cameras_restaurant.yaml')


def test_shipped_cameras_merge_defaults():
    cameras = load_cameras(CONFIG)
    assert {'top', 'west', 'stalls'} <= set(cameras)
    top = cameras['top']
    assert (top.width, top.height, top.rate, top.fov_deg) == (1280, 720, 15.0, 90.0)
    assert cameras['west'].fov_deg == 70.0
    # 탑뷰가 식당 25 x 17 m(지도 restaurant_gt)를 다 담는다: 가로 화각 → 가로 폭, 세로는 화면비로
    half_w = top.pose[2] * math.tan(math.radians(top.fov_deg) / 2)
    assert 2 * half_w >= 25.0 and 2 * half_w * top.height / top.width >= 17.0 - 0.5


def test_sdf_is_static_model_with_one_sensor_per_camera():
    cameras = list(load_cameras(CONFIG).values())
    model = ET.fromstring(model_sdf(cameras)).find('model')
    assert model.findtext('static') == 'true'
    sensors = model.findall('link/sensor')
    assert [s.get('name') for s in sensors] == [c.name for c in cameras]
    for sensor, camera in zip(sensors, cameras):
        assert sensor.get('type') == 'camera'
        assert sensor.findtext('topic') == f'/demo_cam/{camera.name}/image'
        assert sensor.findtext('update_rate') == '15'
        assert sensor.findtext('camera/image/width') == '1280'
    # 충돌·모양이 없어 LiDAR와 로봇에 걸리지 않는다
    assert not model.findall('link/collision') and not model.findall('link/visual')


def test_bridge_args_match_sensor_topics():
    cameras = list(load_cameras(CONFIG).values())
    assert bridge_args(cameras[:1]) == ['/demo_cam/top/image@sensor_msgs/msg/Image[ignition.msgs.Image']
