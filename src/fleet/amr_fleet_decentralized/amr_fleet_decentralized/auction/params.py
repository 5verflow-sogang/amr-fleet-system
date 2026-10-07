"""경매 로직 파라미터. config/task_auction.yaml의 ros__parameters와 1:1로 맞춘다."""
from dataclasses import dataclass


@dataclass(frozen=True)
class AuctionParams:
    bid_window_s: float = 2.0  # TaskAuction.bid_window 기본값 - 공고부터 마감까지
    # 이 미만이면 입찰하지 않는다. fleet_manager.yaml의 min_battery_percent(중앙
    # 할당, 하연)와 같은 의도지만 분산 쪽 임계값은 아직 팀 확인 전이다 - README
    # "확인 필요" 절 참고.
    min_battery_percent: float = 20.0
    # 완료 예상 시각(estimated_finish_time) 계산에 쓰는 단순 등속 추정 속도.
    robot_speed_mps: float = 1.0
