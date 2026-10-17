# 하이브리드에서 로봇(경매 노드)이 지금 중앙·분산 중 어느 모드로 도는지 (architecture.md 5-2절).
# 토픽: /fleet/robot_modes (로봇 → 서버·다른 로봇. 장애를 주입하지 않는다, D8)
# 1초마다, 그리고 모드가 바뀔 때 바로 발행한다.
# - 서버는 분산 모드 로봇에게 작업을 맡기지 않는다 (명령이 닿지 않는다)
# - 분산 모드 로봇들은 분산 모드인 로봇끼리만 경매를 연다
# - 하네스는 전환 횟수와 분산 모드 시간을 센다

std_msgs/Header header

string robot_id

# 지금 모드 (FleetHeartbeat.MODE_CENTRALIZED / MODE_DECENTRALIZED)
uint8 mode

# 노드 시작 뒤 모드가 바뀐 횟수 (중앙 → 분산, 분산 → 중앙 모두 센다)
uint32 transitions
