# 서버가 없을 때 로봇이 여는 경매 공지. 서버가 없으므로 작업 원문을 함께 싣는다.
# 토픽: /fleet/task_auctions. 입찰·낙찰·수락은 TaskBid로 주고받는다.

# stamp: 공지한 시각
std_msgs/Header header

# 경매를 연 로봇 (예: "amr_03")
string auctioneer_id

# 경매 번호. auctioneer_id와 조합해 유일하다
uint32 auction_id

# 경매에 부칠 작업 원문
Task task

# 이 시간 안에 온 입찰만 유효하다 (예: 2초)
builtin_interfaces/Duration bid_window
