# Fleet 서버의 교통 제어 명령 (교차로 우선순위, 교착 해소).
# 토픽: /fleet/traffic_commands

uint8 CMD_PROCEED=0     # 구역에 들어가도 된다
uint8 CMD_WAIT=1        # 구역 앞에서 기다린다 (우선순위 양보)
uint8 CMD_REROUTE=2     # 구역을 피해 경로를 다시 짠다 (대체 경로)

std_msgs/Header header

# 받을 로봇 (예: "amr_01")
string robot_id

# 명령 종류 (CMD_*)
uint8 command

# 대상 구역: 교차로·좁은 통로 (예: "junction_03")
string zone_id

# 사람이 읽는 사유 (예: "yield to amr_03")
string reason
