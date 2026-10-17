"""분산 모드 경매 트리거와 ACCEPT 뒤 작업 실행(TaskAssignment) 규칙. ROS 비의존.

트리거(하연 제안): 모든 로봇의 경매 노드가 작업 방송(/fleet/task_requests)을 받는다. 살아 있는
로봇(RobotState를 최근 alive_timeout_s 안에 받은 로봇)을 robot_id 순으로 줄 세우고,
crc32(task_id) mod 로봇 수 번째 로봇 하나만 그 작업의 경매를 연다. Python hash()는 프로세스마다
값이 달라서(PYTHONHASHSEED) 쓰면 안 된다 - 모든 로봇이 같은 답을 내야 한 번만 열린다.

ACCEPT 뒤 실행(하연 조건): ACCEPT한 로봇의 경매 노드가 자기에게 TaskAssignment를 보낸다.
assignment_id는 서버 번호와 안 겹치게 1,000,000 + 로봇 번호 x 100,000 + 로봇별 순번이다.
"""
import re
import zlib
from typing import Dict, List, Optional, Sequence

# 경매 쪽 assignment_id 번호 공간. 서버(amr_fleet_manager)는 1부터 차례로 쓴다.
ASSIGNMENT_ID_BASE = 1_000_000
ASSIGNMENT_ID_ROBOT_STRIDE = 100_000
_UINT32_MAX = 2**32 - 1


def live_robots(last_seen: Dict[str, float], now_s: float, alive_timeout_s: float) -> List[str]:
    """RobotState를 최근 alive_timeout_s 안에 받은 로봇들, robot_id 순. 시각은 같은 시계여야 한다."""
    return sorted(r for r, t in last_seen.items() if now_s - t <= alive_timeout_s)


def task_hash(task_id: str) -> int:
    """프로세스·실행마다 같은 작업 해시(crc32). hash()는 PYTHONHASHSEED에 따라 달라 쓰지 않는다."""
    return zlib.crc32(task_id.encode('utf-8'))


def auction_opener(task_id: str, robots: Sequence[str]) -> Optional[str]:
    """이 작업의 경매를 열 로봇. robots는 live_robots 결과(정렬된 목록). 아무도 없으면 None."""
    if not robots:
        return None
    return robots[task_hash(task_id) % len(robots)]


def robot_index(robot_id: str) -> int:
    """robot_id 끝의 숫자(amr_03 -> 3). assignment_id 번호 공간을 로봇별로 나누는 데 쓴다."""
    match = re.search(r'(\d+)$', robot_id)
    if match is None:
        raise ValueError(f'robot_id 끝에 번호가 없다: {robot_id!r} (파라미터 robot_index로 직접 준다)')
    return int(match.group(1))


def assignment_id_for(index: int, seq: int) -> int:
    """경매 쪽 assignment_id = 1,000,000 + 로봇 번호 x 100,000 + 로봇별 순번(1부터).

    로봇별 순번은 1~99,999 - 그보다 크면 다음 로봇 번호의 공간과 겹친다. 서버는 1부터 세므로
    1,000,000을 넘기 전에는 안 겹친다. TaskAssignment.assignment_id가 uint32라 그 범위도 본다.
    """
    if not 1 <= seq < ASSIGNMENT_ID_ROBOT_STRIDE:
        raise ValueError(f'로봇별 순번은 1~{ASSIGNMENT_ID_ROBOT_STRIDE - 1}: {seq}')
    if index < 0:
        raise ValueError(f'로봇 번호는 0 이상: {index}')
    value = ASSIGNMENT_ID_BASE + index * ASSIGNMENT_ID_ROBOT_STRIDE + seq
    if value > _UINT32_MAX:
        raise ValueError(f'assignment_id가 uint32를 넘는다: {value}')
    return value


def assignment_topic(robot_id: str, relayed: bool = True) -> str:
    """ACCEPT한 로봇이 자기에게 TaskAssignment를 보낼 토픽.

    relayed(기본)면 amr_bt·가짜 로봇이 relayed:=true로 실제 구독하는 중계본 이름
    /<robot_id>/fleet/task_assignments로 바로 보낸다 - 원본 /fleet/task_assignments로 보내면
    장애 주입기가 중계하면서 지연·손실을 한 번 더 건다(경매 메시지는 이미 relay_auction으로
    걸린다). relayed가 아니면(장애 주입기 없이 원본을 구독하는 로봇) 원본 이름.
    """
    return f'/{robot_id}/fleet/task_assignments' if relayed else '/fleet/task_assignments'
