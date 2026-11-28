"""이 패키지 테스트의 ROS 도메인 (19).

CI의 colcon test는 패키지 테스트를 병렬로 돌린다. test_tf_defer.py는 인식 노드를 띄우고 /tf·카메라 토픽을 주고받는다.
같은 도메인이면 다른 패키지 테스트의 메시지(/tf 등)를 받아 섞인다. 패키지마다 도메인을 나눈다.
번호 표는 .github/workflows/ci.yml Test 단계 주석. pytest가 테스트보다 먼저 읽으므로, 테스트의 rclpy.init이 이 도메인을 쓴다.
"""

import os

os.environ['ROS_DOMAIN_ID'] = '19'
