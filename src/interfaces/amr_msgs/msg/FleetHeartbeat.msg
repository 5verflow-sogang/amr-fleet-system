# Fleet 서버가 살아 있다는 신호. 고정 주기로 발행한다.
# 토픽: /fleet/heartbeat
# 로봇은 정해진 시간 동안 못 받으면 서버 장애로 보고 분산 모드로 바꾼다 (시간은 파라미터).

uint8 MODE_CENTRALIZED=0     # 중앙 서버가 할당
uint8 MODE_DECENTRALIZED=1   # 로봇끼리 협상

std_msgs/Header header

# 서버 이름
string server_id

# 1씩 늘어나는 번호. 빠진 번호로 손실을 센다
uint32 seq

# 서버가 지시하는 운용 모드 (MODE_*)
uint8 mode
