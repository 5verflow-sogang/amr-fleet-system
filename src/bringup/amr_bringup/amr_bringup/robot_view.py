"""로봇 시점 RViz 설정(rviz/robot_view.rviz)을 다른 로봇용으로 바꾼다 (launch/robot_view.launch.py). ROS 없이 돈다."""

TEMPLATE_ROBOT = 'amr_01'


def robot_config(template: str, robot: str) -> str:
    """robot_view.rviz 내용에서 기준 로봇 이름(amr_01)을 robot으로 바꾼다 (토픽 /amr_01/...·프레임 amr_01/...)."""
    return template.replace(f'{TEMPLATE_ROBOT}/', f'{robot}/')
