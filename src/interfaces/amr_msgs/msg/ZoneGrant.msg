# ZoneRequest에 대한 서버의 답.
# 토픽: /fleet/zone_grants (장애 주입기가 중계하며 지연·손실을 건다)
# 서버가 죽으면 연장이 안 되므로 허가가 자연히 풀린다. 분산 폴백과 충돌하지 않는다.

uint8 GRANT_GRANTED=0    # 진입 가능 (lease_expiry까지)
uint8 GRANT_QUEUED=1     # 구역 앞에서 대기. 차례가 오면 서버가 GRANTED를 보낸다
uint8 GRANT_REJECTED=2   # 이 구역으로는 불가. 우회 경로를 다시 계획한다
uint8 GRANT_REVOKED=3    # 이미 받은 허가를 회수. 교착 해소용

uint8 REASON_NONE=0                  # GRANTED
uint8 REASON_OCCUPIED=1              # 다른 로봇이 지나가는 중
uint8 REASON_PRIORITY_YIELD=2        # 급한 작업을 가진 로봇에게 양보
uint8 REASON_DEADLOCK_RESOLUTION=3   # 교착을 풀기 위해 후퇴

# stamp: 응답을 보낸 시각
std_msgs/Header header

# 받은 요청의 request_id를 그대로 되돌린다
uint32 request_id

# 받을 로봇 (예: "amr_04")
string robot_id

# 대상 구역 (예: "junction_03")
string zone_id

# 허가 결과 (GRANT_*)
uint8 grant

# 이 시각이 지나면 새로 들어갈 수 없다. 이미 구역 안이면 빠져나간 뒤 반납한다 (ZoneRequest.msg). GRANTED일 때만 의미가 있다
builtin_interfaces/Time lease_expiry

# 이유 (REASON_*)
uint8 reason

# 구역을 점유 중이거나 우선권을 가진 로봇. 없으면 빈 문자열
string holder_robot_id
