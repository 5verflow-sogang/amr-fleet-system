# 분산 모드 경매(TaskAuction)의 입찰·낙찰·수락·포기 메시지.
# 토픽: /fleet/task_bids
#
# 흐름: 경매인이 TaskAuction 공지 → 참여 로봇 BID → 경매인 AWARD → 낙찰 로봇 ACCEPT (이게 와야 할당 확정)
# 낙찰 통보가 유실되면 경매인만 할당됐다고 믿게 되므로 수락까지 받아야 확정한다.
# 동점이면 robot_id가 사전순으로 앞선 로봇이 이긴다.
# 최신 정보는 도착 순서가 아니라 header.stamp로 판단한다.

uint8 TYPE_BID=0        # 참여 로봇: 입찰
uint8 TYPE_AWARD=1      # 경매인: 낙찰 통보
uint8 TYPE_ACCEPT=2     # 낙찰 로봇: 수락
uint8 TYPE_WITHDRAW=3   # 참여·낙찰 로봇: 포기

# stamp: 보낸 시각
std_msgs/Header header

# 어느 경매인지: 경매를 연 로봇과 경매 번호 (TaskAuction과 같은 값)
string auctioneer_id
uint32 auction_id

# 대상 작업
string task_id

# 이 메시지를 보낸 로봇 (예: "amr_05")
string robot_id

# 메시지 종류 (TYPE_*)
uint8 type

# 입찰가 (m): 지금 위치 → 작업 시작점(pickup, 없으면 dropoff) 직선 거리. 작업이 없는(IDLE) 로봇만 입찰한다.
# 중앙 할당 nearest(amr_fleet_core euclidean_cost)와 같은 규칙이라 지연·손실이 없으면 같은 로봇이 뽑힌다 (연구 1 비교 공정성, #91)
float32 cost

# 예상 완료 시각. Task.deadline을 넘기면 무효 입찰
builtin_interfaces/Time estimated_finish_time

# 보낸 쪽이 알고 있는 현재 승자와 그 비용. AWARD에서는 필수, 모르면 빈 문자열과 0.0
string winner_id
float32 winning_cost
