# 로봇이 교차로·좁은 통로 같은 구역에 들어가도 되는지 서버에 묻고, 나올 때 반납한다.
# 토픽: /fleet/zone_requests. 서버 답은 ZoneGrant (TrafficCommand를 대체한다)
#
# 로봇이 지킬 규칙
# - 유효한 허가(ZoneGrant.GRANT_GRANTED)가 있을 때만 구역에 들어간다. 응답이 없으면 들어가지 않는다.
# - 기다리는 동안이나 구역 안에 있는 동안 같은 request_id로 주기적으로 다시 요청한다 (허가 연장).
# - 구역을 벗어나면 TYPE_RELEASE를 보낸다. 못 보내도 lease_expiry가 지나면 자동으로 풀린다.

uint8 TYPE_ACCESS=0    # 진입 요청 (연장 포함)
uint8 TYPE_RELEASE=1   # 반납

# stamp: 요청을 보낸 시각
std_msgs/Header header

# 요청 번호. 로봇이 요청마다 1씩 늘리고, 재전송·연장할 때는 같은 번호를 쓴다
uint32 request_id

# 요청한 로봇 (예: "amr_04")
string robot_id

# 대상 구역 (예: "junction_03")
string zone_id

# 요청 종류 (TYPE_*)
uint8 type
