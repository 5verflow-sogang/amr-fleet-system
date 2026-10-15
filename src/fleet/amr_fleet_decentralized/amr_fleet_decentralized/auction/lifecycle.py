"""경매 뒷단계(AWARD 재전송·WITHDRAW 범위·FAILED 재시도·낙찰 로봇 장애 회수) 판단. ROS 비의존.

상태(재전송 횟수·마지막 전송 시각, 수락 여부)는 호출 쪽(노드)이 들고 있고, 이 함수들은
그 상태를 보고 "지금 뭘 해야 하는지"만 답한다 - ZoneClientCore처럼 상태를 직접 들고
있지 않는다(경매는 노드가 auction_id별로 여러 개를 동시에 관리해야 해서, 여기서
하나의 상태 클래스로 묶기보다 노드의 dict 값 하나하나에 쓰기 편한 순수 함수가 낫다).
예외로 작업별 재시도·회수 횟수는 task_id로 이어 세야 해서 TaskRetryLedger가 들고 있다.
"""

from typing import Dict, Optional


def should_resend_award(
    attempts_sent: int, last_sent_time: float, now_s: float,
    resend_interval_s: float, max_attempts: int,
) -> bool:
    """AWARD를 지금 다시 보내야 하는지.

    10/7 하연·이진 결정: 중앙 Ack 재전송과 같게 1초 간격 x 5회 - 재전송 정책이
    같아야 지연 내성 비교가 공정하다. 이미 max_attempts번 다 보냈으면 더 안
    보낸다(그 판단은 award_exhausted가 맡는다 - 재경매로 넘어가야 하므로).
    """
    if attempts_sent >= max_attempts:
        return False
    return now_s - last_sent_time >= resend_interval_s


def should_resend_bid(last_sent_time: float, now_s: float, resend_period_s: float, deadline_s: float) -> bool:
    """BID를 지금 다시 보내야 하는지.

    #155·#171: BID는 원래 1회성 발행이라, DDS 디스커버리가 아직 안 끝난 상태(또는
    그 밖의 일시적 유실)에서 나가면 그대로 사라지고 다시 안 온다. 마감(deadline_s,
    호출 쪽이 "공고를 받은 시각 + bid_window_s"로 잰다) 전까지 resend_period_s
    간격으로 같은 입찰(같은 auction_id·robot_id·cost)을 다시 보낸다. AWARD를
    받았는지는 여기서 모르므로(호출 쪽 상태) 판단하지 않는다 - 노드가 AWARD를
    받으면 재전송을 바로 멈춘다.
    """
    if now_s >= deadline_s:
        return False
    return now_s - last_sent_time >= resend_period_s


def award_exhausted(attempts_sent: int, max_attempts: int) -> bool:
    """재전송을 다 썼는데도(ACCEPT가 안 와서) 재경매해야 하는지."""
    return attempts_sent >= max_attempts


def can_withdraw(accepted: bool) -> bool:
    """TYPE_WITHDRAW를 보낼 수 있는지.

    10/7 하연·이진 결정: ACCEPT 전까지만 - 수락 뒤 못 하게 되면(예: 도중에 장애)
    TYPE_WITHDRAW가 아니라 TaskStatus FAILED로 보고하고, 경매인이 그걸 보고
    다시 경매한다(이 모듈은 TaskStatus 발행까지는 안 다룬다 - 노드 몫).
    """
    return not accepted


# amr_msgs/msg/ErrorInfo.ASSIGN_CANCELED 와 같은 값 (amr_fleet_manager dispatcher.ERROR_CANCELED와 같음)
ERROR_CANCELED = 103


def can_retry_failed_task(failures: int, failed_retries: int, error_code: int = 0) -> bool:
    """ACCEPT 뒤 TaskStatus FAILED를 받았을 때 재경매해도 되는지.

    10/9 하연·이진 결정(#107·#108): 최대 failed_retries번까지만 재경매하고,
    그래도 또 실패하면 실패로 남긴다 - 중앙(assignment.failed_retries)과 같은
    규칙이어야 비교군 조건이 같다. failures는 이 작업이 지금까지 FAILED로
    보고된 횟수(재경매 전 값)다.

    취소(error_code ERROR_CANCELED)로 온 FAILED는 재경매하지 않는다 - 일부러 멈춘
    작업이라 다시 맡기면 안 된다. 중앙 #107(dispatcher.on_status)과 같은 규칙이고,
    호출 쪽은 이 실패를 failures에 세지 않는다.
    """
    if error_code == ERROR_CANCELED:
        return False
    return failures < failed_retries


def winner_lost(
    last_seen: Optional[float], accepted_at: float, now_s: float, reclaim_timeout_s: float,
) -> bool:
    """ACCEPT한 낙찰 로봇이 죽었다고 보고 작업을 회수(재경매)해야 하는지.

    중앙 #132(assignment.reclaim_timeout_s)와 같은 규칙: 그 로봇의 RobotState가
    reclaim_timeout_s 동안 안 오면 죽은 것으로 본다. ACCEPT도 살아 있다는 증거라
    기준 시각은 마지막 RobotState와 ACCEPT 시각 중 늦은 쪽이다(RobotState를 한 번도
    못 받은 로봇이 수락하자마자 회수되지 않게). 시각은 모두 같은 시계(실제 시간)여야 한다.
    reclaim_timeout_s가 0 이하면 회수하지 않는다.
    """
    if reclaim_timeout_s <= 0:
        return False
    reference = accepted_at if last_seen is None else max(last_seen, accepted_at)
    return now_s - reference >= reclaim_timeout_s


def can_reclaim(reclaims: int, max_reclaims: int) -> bool:
    """이 작업을 장애 회수로 또 재경매해도 되는지. max_reclaims가 음수면 상한 없음(#132와 같음)."""
    return max_reclaims < 0 or reclaims < max_reclaims


class TaskRetryLedger:
    """작업별 FAILED 재시도·장애 회수 횟수. task_id로 세어 재경매로 auction_id가 바뀌어도 이어서 센다.

    중앙의 TaskEntry.failures·reclaims와 같은 역할이고, 둘은 따로 센다(#132): 회수는
    작업이 아니라 로봇의 문제라서 FAILED 재시도(failed_retries) 횟수를 쓰지 않는다.
    """

    def __init__(self, failed_retries: int, max_reclaims: int = -1):
        self._failed_retries = failed_retries
        self._max_reclaims = max_reclaims
        self._failures: Dict[str, int] = {}
        self._reclaims: Dict[str, int] = {}

    def failures(self, task_id: str) -> int:
        return self._failures.get(task_id, 0)

    def reclaims(self, task_id: str) -> int:
        return self._reclaims.get(task_id, 0)

    def on_failed(self, task_id: str, error_code: int = 0) -> bool:
        """ACCEPT 뒤 FAILED 보고. 재경매하면 True(횟수를 하나 쓴다)."""
        if not can_retry_failed_task(self.failures(task_id), self._failed_retries, error_code):
            return False
        self._failures[task_id] = self.failures(task_id) + 1
        return True

    def on_winner_lost(self, task_id: str) -> bool:
        """낙찰 로봇 장애. 재경매하면 True(회수 횟수를 하나 쓴다, failures는 그대로)."""
        if not can_reclaim(self.reclaims(task_id), self._max_reclaims):
            return False
        self._reclaims[task_id] = self.reclaims(task_id) + 1
        return True
