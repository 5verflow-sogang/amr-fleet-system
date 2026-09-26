# 분산 모드에서 로봇끼리 작업을 나눠 갖는 입찰 메시지.
# 토픽: /fleet/task_bids
# 규칙 (초안): 정해진 시간 동안 BID를 모으고, cost가 가장 작은 로봇이 CLAIM한다.

uint8 TYPE_BID=0        # 이 비용에 하겠다
uint8 TYPE_CLAIM=1      # 내가 가져간다 (확정)
uint8 TYPE_RELEASE=2    # 가져간 작업을 포기한다

std_msgs/Header header

string task_id

# 보낸 로봇 (예: "amr_01")
string robot_id

# 메시지 종류 (TYPE_*)
uint8 type

# 입찰 비용. 기본: 현재 위치에서 픽업 지점까지 예상 이동 거리 (m)
float32 cost
