# Fleet 서버가 살아 있다는 신호. 고정 주기로 발행한다.
# 토픽: /fleet/heartbeat (장애 주입기가 중계하며 지연·손실을 건다)
# 로봇은 정해진 시간 동안 못 받으면 서버 장애로 보고 분산 모드로 바꾼다.
# 주기와 타임아웃은 연구 1의 분산 전환 임계점과 직결되므로 메시지에 넣지 않고 파라미터로 둔다.

uint8 MODE_CENTRALIZED=0     # 중앙 서버가 할당
uint8 MODE_DECENTRALIZED=1   # 로봇끼리 협상 (서버가 살아 있어도 의도적으로 분산 운용, 하이브리드 비교군)

# stamp: 서버가 보낸 시각. 도착 시각과 비교해 지연을 잰다. 장애 주입기는 바꾸지 않는다
std_msgs/Header header

# 서버 이름
string server_id

# 서버 시작 번호. 서버가 재시작할 때마다 1씩 는다.
# 바뀌면 로봇은 assignment_id·seq 기준을 초기화한다 (FleetTaskTable.epoch와 같은 값)
uint32 epoch

# epoch 안에서 1씩 늘어나는 번호. 빠진 번호로 손실을 센다
uint32 seq

# 서버가 지시하는 운용 모드 (MODE_*)
uint8 mode
