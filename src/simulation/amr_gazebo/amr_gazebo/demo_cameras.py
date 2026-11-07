"""시연 녹화용 고정 카메라 (config/demo_cameras_restaurant.yaml → Gazebo 모델 SDF·브리지 인자). ROS 없이 돈다.

카메라는 static 모델 하나(demo_cameras)의 링크로 들어간다. 충돌·모양이 없어 LiDAR·로봇·손님과 부딪히지 않는다.
이미지는 Gazebo 토픽 /demo_cam/<이름>/image로 나오고, 브리지가 같은 이름의 ROS 토픽(sensor_msgs/Image)으로 넘긴다.
update_rate는 시뮬레이션 시간 기준이라, RTF가 낮아도 시뮬레이션 1초에 rate장이 나온다 (영상은 header 시각으로 만든다).
"""

import math
from typing import Dict, List, NamedTuple

import yaml

TOPIC = '/demo_cam/{name}/image'


class Camera(NamedTuple):
    name: str
    pose: tuple          # x, y, z, roll, pitch, yaw (m, rad, world)
    width: int
    height: int
    fov_deg: float       # 가로 화각
    rate: float          # Hz (시뮬레이션 시간)


def load_cameras(path: str) -> Dict[str, Camera]:
    """yaml의 defaults(width·height·fov_deg·rate)에 카메라마다 덮어쓴 값을 합친다."""
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    defaults = data.get('defaults', {})
    cameras = {}
    for name, spec in data['cameras'].items():
        merged = {**defaults, **spec}
        pose = tuple(float(v) for v in merged['pose'])
        if len(pose) != 6:
            raise ValueError(f'{name}: pose는 [x, y, z, roll, pitch, yaw] 여섯 값')
        cameras[name] = Camera(name, pose, int(merged['width']), int(merged['height']),
                               float(merged['fov_deg']), float(merged['rate']))
    return cameras


def model_sdf(cameras: List[Camera]) -> str:
    links = []
    for c in cameras:
        links.append(f"""
    <link name="{c.name}">
      <pose>{' '.join(f'{v:g}' for v in c.pose)}</pose>
      <sensor name="{c.name}" type="camera">
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>{c.rate:g}</update_rate>
        <topic>{TOPIC.format(name=c.name)}</topic>
        <camera>
          <horizontal_fov>{math.radians(c.fov_deg):.6f}</horizontal_fov>
          <image><width>{c.width}</width><height>{c.height}</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>60</far></clip>
        </camera>
      </sensor>
    </link>""")
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <model name="demo_cameras">
    <static>true</static>{''.join(links)}
  </model>
</sdf>
"""


def bridge_args(cameras: List[Camera]) -> List[str]:
    return [f'{TOPIC.format(name=c.name)}@sensor_msgs/msg/Image[ignition.msgs.Image' for c in cameras]
