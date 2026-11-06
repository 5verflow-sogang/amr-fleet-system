"""로봇 한 대마다 띄울 launch 목록. ROS 없이 돈다.

system.launch.py가 이 목록을 launch 동작(include)으로 바꾼다. "무엇을 어떤 인자로 띄우는가"를 여기에 모아
launch 라이브러리 객체를 뜯어보지 않고도 테스트할 수 있게 한다.
"""

from dataclasses import dataclass
from typing import List, NamedTuple, Optional

from amr_bringup.robots import cmd_vel_chain, encoder_seed

# 몸체(spawn_robot body) → Nav2 프로필(nav2.launch.py nav2_profile). serving이면 footprint가 서빙 로봇 크기가 된다
NAV2_PROFILE = {'amr': 'warehouse', 'serving': 'restaurant'}


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
    # true / false / 카메라를 켤 로봇 이름 목록 'amr_01,amr_03' (시연 로봇 시점: CPU로 5대를 다 그리면 RTF 0.37)
    enable_camera: object = False
    congestion_filter: str = 'none'
    payload_mass: str = '0.0'
    seed: int = 0
    map: str = ''
    use_imu: str = 'true'
    zones_file: str = ''
    body: str = 'amr'

    def camera_names(self) -> List[str]:
        """enable_camera가 로봇 이름 목록이면 그 이름들, true/false면 빈 목록."""
        value = str(self.enable_camera).replace(' ', '')
        return [] if value.lower() in ('true', 'false') else [n for n in value.split(',') if n]

    def has_camera(self, name: str) -> bool:
        return str(self.enable_camera).strip().lower() == 'true' or name in self.camera_names()

    def validate(self, robot_names: Optional[List[str]] = None) -> None:
        if robot_names is not None:
            unknown = sorted(set(self.camera_names()) - set(robot_names))
            if unknown:
                raise RuntimeError(f'enable_camera에 없는 로봇: {", ".join(unknown)} (로봇: {", ".join(robot_names)})')
        if self.body not in NAV2_PROFILE:
            raise RuntimeError(f'body는 {", ".join(NAV2_PROFILE)} 중 하나: {self.body!r}')
        if self.nav2 and not self.localization:
            raise RuntimeError('nav2:=true는 map → odom TF를 내는 위치추정이 필요하다 (localization:=true)')
        if self.docking and not self.nav2:
            # nav2:=false면 BT 이동이 자리채움이라 로봇이 스테이션에 가지 않고 제자리에서 도킹을 시도한다 (마커판이 안 보여 904)
            raise RuntimeError('docking:=true는 로봇이 스테이션까지 가야 해서 Nav2가 필요하다 (nav2:=true)')
        all_cameras = str(self.enable_camera).strip().lower() == 'true'
        if self.detection and not (all_cameras or self.camera_names()):
            raise RuntimeError('detection:=true는 카메라가 필요하다 (enable_camera:=true 또는 로봇 이름)')
        if self.docking and not all_cameras:
            # BT가 모든 로봇에서 도킹 서버를 부르므로 일부만 켜면 카메라 없는 로봇의 작업이 실패한다
            raise RuntimeError('docking:=true는 모든 로봇에 카메라가 필요하다 (enable_camera:=true)')


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
            enable_camera=str(opt.has_camera(name)).lower(), payload_mass=opt.payload_mass, body=opt.body,
            use_sim_time='true')),
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
            namespace=name, cmd_vel_topic=chain['nav_out'], nav2_profile=NAV2_PROFILE[opt.body],
            congestion_filter=opt.congestion_filter, use_sim_time='true')))
    if opt.zone_client:
        # 서버와 같은 구역 목록. zones.yaml은 world 좌표라 TF map ← world(system.launch가 발행)로
        # 바꿔 쓰게 한다 (구역 1.5 m 어긋남 수정). 로봇 위치는 네임스페이스 안 상대 토픽
        # (robot_pose_topic 기본값)이라 접두어를 따로 넘길 게 없다 (perf/zone-client-pose-topic)
        stack.append(Include('amr_zone_client', 'zone_client.launch.py', dict(
            namespace=name, robot_id=name, zones_file=opt.zones_file, zones_frame='world',
            cmd_vel_out_topic=chain['zone_out'], use_sim_time='true')))
    if opt.docking:
        # 도킹 서버는 Nav2 컨트롤러 자리로 속도를 낸다 (같은 cmd_vel 체인)
        stack.append(Include('amr_docking', 'docking.launch.py', dict(
            namespace=name, cmd_vel_topic=chain['nav_out'], use_sim_time='true')))
    if opt.tracking:
        stack.append(Include('amr_tracking', 'tracking.launch.py', dict(namespace=name, use_sim_time='true')))
    if opt.detection and opt.has_camera(name):
        stack.append(Include('amr_detection', 'detection.launch.py', dict(namespace=name, use_sim_time='true')))
    if opt.safety:
        stack.append(Include('amr_safety', 'safety.launch.py', dict(
            namespace=name, cmd_vel_in_topic=chain['safety_in'], use_sim_time='true')))
    return stack
