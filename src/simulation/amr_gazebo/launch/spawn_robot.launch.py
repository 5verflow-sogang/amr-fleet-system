"""실행 중인 Gazebo에 AMR 1대를 스폰하고 ROS와 잇는다.

    ros2 launch amr_gazebo spawn_robot.launch.py namespace:=amr_01 x:=0 y:=1.5 yaw:=0

ROS 토픽 (<ns> = namespace):
    /<ns>/cmd_vel       geometry_msgs/Twist     ROS → Gazebo
    /<ns>/diff_drive/odom  nav_msgs/Odometry    Gazebo → ROS (DiffDrive 이상적 odom, 노이즈 없음)
    /<ns>/joint_states  sensor_msgs/JointState  Gazebo → ROS
    /<ns>/ground_truth/odom  nav_msgs/Odometry  Gazebo → ROS (진짜 위치, frame "world", 평가 전용)
    /<ns>/scan_raw      sensor_msgs/LaserScan   Gazebo → ROS (노이즈 포함 원본, 10 Hz)
    /<ns>/imu/data_raw  sensor_msgs/Imu         Gazebo → ROS (노이즈·바이어스 포함 원본, 100 Hz)
    /<ns>/camera/color/image_raw   sensor_msgs/Image        Gazebo → ROS (rgb8, 30 Hz)
    /<ns>/camera/depth/image_raw   sensor_msgs/Image        Gazebo → ROS (32FC1, m, 30 Hz)
    /<ns>/camera/camera_info       sensor_msgs/CameraInfo   Gazebo → ROS (RGB·Depth 공통)
                        (camera/*는 enable_camera:=true일 때만)
    /tf                 <ns>/odom → <ns>/base_footprint (publish_odom_tf:=true일 때)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn(context):
    ns = LaunchConfiguration('namespace').perform(context)
    topic_ns = f'/{ns}' if ns else ''
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    enable_camera = LaunchConfiguration('enable_camera').perform(context).lower()

    bridge_args = [
        f'{topic_ns}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        f'{topic_ns}/diff_drive/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        f'{topic_ns}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        f'{topic_ns}/ground_truth/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        f'{topic_ns}/scan_raw@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        f'{topic_ns}/imu/data_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU',
    ]
    remappings = []
    if enable_camera == 'true':
        bridge_args += [
            f'{topic_ns}/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            f'{topic_ns}/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            f'{topic_ns}/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ]
        # rgbd_camera의 Gazebo 토픽 이름을 RealSense ROS 드라이버와 같은 관례로 바꾼다.
        # Gazebo camera/points는 브리지하지 않는다: Fortress는 점을 x 전방 좌표로 내면서 frame_id는
        # 광학 프레임(z 전방)을 붙여 RViz에서 90° 돌아간다. 포인트클라우드는 depth 영상 + camera_info로 만든다.
        remappings += [
            (f'{topic_ns}/camera/image', f'{topic_ns}/camera/color/image_raw'),
            (f'{topic_ns}/camera/depth_image', f'{topic_ns}/camera/depth/image_raw'),
        ]
    if LaunchConfiguration('publish_odom_tf').perform(context).lower() == 'true':
        # DiffDrive가 내는 odom → base_footprint를 전역 /tf로 합친다 (프레임 이름에 네임스페이스가 붙어 있다)
        bridge_args.append(f'{topic_ns}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V')
        remappings.append((f'{topic_ns}/tf', '/tf'))

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('amr_description'), 'launch', 'description.launch.py')),
            launch_arguments={
                'namespace': ns,
                'payload_mass': LaunchConfiguration('payload_mass'),
                'gt_rate': LaunchConfiguration('gt_rate'),
                'enable_camera': enable_camera,
                'max_linear_acceleration': LaunchConfiguration('max_linear_acceleration'),
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', ns or 'amr',
                '-topic', f'{topic_ns}/robot_description',
                '-x', LaunchConfiguration('x').perform(context),
                '-y', LaunchConfiguration('y').perform(context),
                '-z', LaunchConfiguration('z').perform(context),
                '-Y', LaunchConfiguration('yaw').perform(context),
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            namespace=ns,
            arguments=bridge_args,
            remappings=remappings,
            parameters=[{'use_sim_time': use_sim_time == 'true'}],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01', description='로봇 네임스페이스'),
        DeclareLaunchArgument('x', default_value='0.0', description='스폰 위치 x [m]'),
        DeclareLaunchArgument('y', default_value='1.5', description='스폰 위치 y [m] (기본: A–B 열 사이 통로)'),
        DeclareLaunchArgument('z', default_value='0.01', description='스폰 위치 z [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='스폰 방향 [rad]'),
        DeclareLaunchArgument('payload_mass', default_value='0.0', description='적재물 질량 [kg]'),
        DeclareLaunchArgument(
            'max_linear_acceleration', default_value='1.0',
            description='DiffDrive 선가속 상한 [m/s²]. 크게 주면 바퀴 토크 한계가 질량에 따라 가감속을 정한다'),
        DeclareLaunchArgument('gt_rate', default_value='100', description='GT 토픽 발행 주기 [Hz]'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'publish_odom_tf', default_value='true',
            description='odom → base_footprint TF를 DiffDrive에서 받을지. EKF가 발행하면 false'),
        DeclareLaunchArgument(
            'enable_camera', default_value='true',
            description='RGBD 카메라 센서와 브리지를 만들지. CPU sim에서 여러 대를 띄울 때 false'),
        OpaqueFunction(function=_spawn),
    ])
