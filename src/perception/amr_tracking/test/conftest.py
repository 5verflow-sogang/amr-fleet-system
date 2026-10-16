"""이 패키지 테스트의 ROS 도메인 (17).

CI의 colcon test는 패키지 테스트를 병렬로 돌리고, ROS 노드를 띄우는 테스트들이 같은 토픽(/fleet/*)·로봇 이름(amr_01)을
쓴다. 같은 도메인이면 다른 패키지 테스트의 메시지를 받아 섞인다 (10/16 amr_bt 테스트의 작업을 amr_fleet_manager
테스트의 가짜 로봇이 같이 수행). 패키지마다 도메인을 나눈다. 번호 표는 .github/workflows/ci.yml Test 단계 주석.
pytest가 테스트보다 먼저 읽으므로, 테스트의 rclpy.init과 launch_testing이 띄우는 노드 모두 이 도메인을 쓴다.
"""

import os

os.environ['ROS_DOMAIN_ID'] = '17'
