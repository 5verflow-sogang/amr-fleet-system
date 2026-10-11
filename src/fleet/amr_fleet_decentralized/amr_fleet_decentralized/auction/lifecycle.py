"""경매 뒷단계(AWARD 재전송·WITHDRAW 범위) 상태 전이. ROS 비의존, 순수 함수만 있다.

상태(재전송 횟수·마지막 전송 시각, 수락 여부)는 호출 쪽(노드)이 들고 있고, 이 함수들은
그 상태를 보고 "지금 뭘 해야 하는지"만 답한다 - ZoneClientCore처럼 상태를 직접 들고
있지 않는다(경매는 노드가 auction_id별로 여러 개를 동시에 관리해야 해서, 여기서
하나의 상태 클래스로 묶기보다 노드의 dict 값 하나하나에 쓰기 편한 순수 함수가 낫다).
"""


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


def can_retry_failed_task(failures: int, failed_retries: int) -> bool:
    """ACCEPT 뒤 TaskStatus FAILED를 받았을 때 재경매해도 되는지.

    10/9 하연·이진 결정(#107·#108): 최대 failed_retries번까지만 재경매하고,
    그래도 또 실패하면 실패로 남긴다 - 중앙(assignment.failed_retries)과 같은
    규칙이어야 비교군 조건이 같다. failures는 이 작업이 지금까지 FAILED로
    보고된 횟수(재경매 전 값)다.
    """
    return failures < failed_retries
