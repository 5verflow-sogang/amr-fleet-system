"""로봇 한 대마다 띄울 launch 목록. ROS 없이 돈다.

system.launch.py가 이 목록을 launch 동작(include)으로 바꾼다. "무엇을 어떤 인자로 띄우는가"를 여기에 모아
launch 라이브러리 객체를 뜯어보지 않고도 테스트할 수 있게 한다.
"""

from dataclasses import dataclass
from typing import List, NamedTuple

from amr_bringup.robots import cmd_vel_chain, encoder_seed


class Include(NamedTuple):
    """다른 패키지의 launch 하나와 넘길 인자."""
    package: str
    launch: str
    arguments: dict


@dataclass
class Options:
    """system.launch.py 인자 중 로봇 스택을 정하는 것."""
    localization: bool = True
    bt: bool = True
    fleet: bool = True
    nav2: bool = False
    zone_client: bool = False
    tracking: bool = False
    safety: bool = False
    detection: bool = False
    docking: bool = False
    enable_camera: bool = False
    payload_mass: str = '0.0'
    seed: int = 0
    map: str = ''
    use_imu: str = 'true'
    zones_file: str = ''

    def validate(self) -> None:
        if self.nav2 and not self.localization:
            raise RuntimeError('nav2:=true는 map → odom TF를 내는 위치추정이 필요하다 (localization:=true)')
        for name in ('detection', 'docking'):
            if getattr(self, name) and not self.enable_camera:
                raise RuntimeError(f'{name}:=true는 카메라가 필요하다 (enable_camera:=true)')


def robot_stack(opt: Options, index: int, name: str, pose, map_pose) -> List[Include]:
    """index번째 로봇(name)의 launch 목록. 순서대로 띄운다.

    pose: 스폰 자세 (x, y, yaw) world, map_pose: 같은 자세를 map 좌표로 바꾼 값 (AMCL 초기 위치).
    """
    opt.validate()
    x, y, yaw = pose
    mx, my, myaw = map_pose
    chain = cmd_vel_chain(opt.zone_client, opt.safety)
    stack = [
        Include('amr_gazebo', 'spawn_robot.launch.py', dict(
            namespace=name, x=x, y=y, yaw=yaw,
            # EKF(ekf_local)가 odom → base_footprint를 내므로 DiffDrive TF는 끈다 (docs/api.md 2-1절)
            publish_odom_tf='false' if opt.localization else 'true',
            enable_camera=str(opt.enable_camera).lower(), payload_mass=opt.payload_mass, use_sim_time='true')),
        Include('amr_odometry', 'odometry.launch.py', dict(
            namespace=name, encoder_seed=encoder_seed(opt.seed, index), use_sim_time='true')),
        Include('amr_odometry', 'sensor_filters.launch.py', dict(namespace=name, use_sim_time='true')),
    ]
    if opt.localization:
        stack.append(Include('amr_localization', 'localization.launch.py', dict(
            namespace=name, map=opt.map, start_map_server='true' if index == 0 else 'false',
            use_imu=opt.use_imu, initial_x=mx, initial_y=my, initial_yaw=myaw, use_sim_time='true')))
    if opt.bt:
        stack.append(Include('amr_bt', 'amr_bt.launch.py', dict(
            namespace=name, relayed='true' if opt.fleet else 'false', use_sim_time='true',
            use_nav2='true' if opt.nav2 else 'false', use_docking='true' if opt.docking else 'false')))
    if opt.nav2:
        stack.append(Include('amr_bringup', 'nav2.launch.py', dict(
            namespace=name, cmd_vel_topic=chain['nav_out'], use_sim_time='true')))
    if opt.zone_client:
        # 서버와 같은 구역 목록. 시뮬레이션 프레임은 <ns>/base_footprint라 접두어를 넘긴다.
        # zones.yaml은 world 좌표라 TF map ← world(system.launch가 발행)로 바꿔 쓰게 한다 (구역 1.5 m 어긋남 수정)
        stack.append(Include('amr_zone_client', 'zone_client.launch.py', dict(
            namespace=name, robot_id=name, zones_file=opt.zones_file, zones_frame='world',
            base_frame=f'{name}/base_footprint', cmd_vel_out_topic=chain['zone_out'], use_sim_time='true')))
    if opt.docking:
        # 도킹 서버는 Nav2 컨트롤러 자리로 속도를 낸다 (같은 cmd_vel 체인)
        stack.append(Include('amr_docking', 'docking.launch.py', dict(
            namespace=name, cmd_vel_topic=chain['nav_out'], use_sim_time='true')))
    if opt.tracking:
        stack.append(Include('amr_tracking', 'tracking.launch.py', dict(namespace=name, use_sim_time='true')))
    if opt.detection:
        stack.append(Include('amr_detection', 'detection.launch.py', dict(namespace=name, use_sim_time='true')))
    if opt.safety:
        stack.append(Include('amr_safety', 'safety.launch.py', dict(
            namespace=name, cmd_vel_in_topic=chain['safety_in'], use_sim_time='true')))
    return stack
