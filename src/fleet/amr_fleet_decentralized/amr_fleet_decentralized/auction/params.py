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
    # #155·#171: BID는 1회성 발행이라 DDS 디스커버리가 아직 안 끝난 상태에서 나가면
    # 그대로 유실되고 다시 안 온다. 마감(bid_window_s)까지 이 간격으로 같은 입찰을
    # 다시 보낸다 - AWARD를 받거나 마감되면 멈춘다. 횟수 상한은 없다(마감이 상한 역할을 한다).
    bid_resend_period_s: float = 0.5
    # 10/9 하연·이진 결정(#107·#108): ACCEPT 뒤 TaskStatus FAILED를 받으면 최대 1회
    # 재경매하고, 그래도 실패하면 실패로 남긴다. 중앙(fleet_manager.yaml
    # assignment.failed_retries)과 같은 규칙·이름 - 비교군 조건을 같게 둔다.
    failed_retries: int = 1
    # 낙찰 로봇 장애 회수 (중앙 #132 assignment.reclaim_timeout_s와 같은 규칙): ACCEPT한 낙찰 로봇의
    # RobotState가 이 시간(실제 시간, s) 동안 안 오면 죽은 것으로 보고 재경매한다. 0 이하면 끈다.
    reclaim_timeout_s: float = 5.0
    # 회수 재경매 상한. 음수면 상한 없음(#132와 같음). FAILED 재시도(failed_retries)와 따로 센다.
    max_reclaims: int = -1
    # 맡은 작업 해제 안전장치(동시 낙찰 경합): ACCEPT 뒤 이 시간(실제 시간, s) 동안 내 RobotState가
    # 한 번도 BUSY가 안 되면(작업이 시작 안 됨) 맡은 작업을 풀고 다시 입찰한다. 0 이하면 끈다.
    commit_timeout_s: float = 30.0
    # #227 민지 리뷰 2(가장 앞선 AWARD만 수락): AWARD를 받아도 더 급한(sort_by_priority로 앞선)
    # 입찰 결과를 아직 모르면(마감 전이거나, 마감은 지났지만 그 입찰의 AWARD를 못 받음) 이
    # 시간(실제 시간, s) 동안 수락을 미루고 기다린다. 다른 로봇이 그 작업을 맡은 걸 알면
    # (_taken_by_other) 그 전에 바로 포기하고 수락한다.
    # 경매인이 ACCEPT 없이 재경매하기까지 걸리는 최대 시간((award_max_attempts - 1) x
    # award_resend_interval_s, 기본 4 x 1.0 = 4.0 s)보다 반드시 짧아야 한다 - 그보다 길면
    # 기다리는 동안 경매인이 먼저 재경매해 버려, 다 기다리고 수락하려 할 때는 그 경매가 이미
    # 무효일 수 있다. 1 s 여유를 두고 3.0으로 둔다.
    award_wait_s: float = 3.0
