"""경매 로직 파라미터. config/task_auction.yaml의 ros__parameters와 1:1로 맞춘다."""
from dataclasses import dataclass


@dataclass(frozen=True)
class AuctionParams:
    bid_window_s: float = 2.0  # TaskAuction.bid_window 기본값 - 공고부터 마감까지
    # 10/7 하연·이진 결정: 중앙집중(fleet_manager.yaml min_battery_percent)과 같은
    # 20% - 분산·중앙 비교군 조건을 같게 둔다.
    min_battery_percent: float = 20.0
    # 완료 예상 시각(estimated_finish_time) 계산에 쓰는 단순 등속 추정 속도.
    robot_speed_mps: float = 1.0
    # 10/7 결정: 중앙 Ack 재전송과 같게 1초 간격 x 5회, 그래도 ACCEPT가 없으면 재경매.
    award_resend_interval_s: float = 1.0
    award_max_attempts: int = 5
    # 10/9 하연·이진 결정(#107·#108): ACCEPT 뒤 TaskStatus FAILED를 받으면 최대 1회
    # 재경매하고, 그래도 실패하면 실패로 남긴다. 중앙(fleet_manager.yaml
    # assignment.failed_retries)과 같은 규칙·이름 - 비교군 조건을 같게 둔다.
    failed_retries: int = 1
    # 낙찰 로봇 장애 회수 (중앙 #132 assignment.reclaim_timeout_s와 같은 규칙): ACCEPT한 낙찰 로봇의
    # RobotState가 이 시간(실제 시간, s) 동안 안 오면 죽은 것으로 보고 재경매한다. 0 이하면 끈다.
    reclaim_timeout_s: float = 5.0
    # 회수 재경매 상한. 음수면 상한 없음(#132와 같음). FAILED 재시도(failed_retries)와 따로 센다.
    max_reclaims: int = -1
