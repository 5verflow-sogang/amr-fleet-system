"""시뮬레이션 시계(/clock) 발행 속도를 줄이는 규칙 (rclpy 없이 테스트한다).

Gazebo Fortress는 물리 스텝(1 ms)마다 /clock을 낸다. 그대로 ROS로 넘기면 use_sim_time인 노드 전부(로봇 1대에
16개쯤)가 초당 약 900번씩 시계 콜백을 돌려, 하는 일 없는 노드도 0.2~0.4코어를 먹었다(GCP 5대: 16코어 98 %,
Mac 2대 실측 893 Hz·구독 33개). 시간이 period만큼 흘렀을 때만 내보내 노드가 받는 횟수를 줄인다.
"""

from typing import Optional


def should_publish(last_sent_ns: Optional[int], now_ns: int, period_ns: int) -> bool:
    """마지막으로 보낸 시각에서 period 이상 흘렀으면 보낸다. 처음이거나 시간이 거꾸로 가면(리셋) 바로 보낸다."""
    if last_sent_ns is None or now_ns < last_sent_ns:
        return True
    return now_ns - last_sent_ns >= period_ns
