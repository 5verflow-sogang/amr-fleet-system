# Fleet 서버가 로봇에게 작업을 맡기거나 취소하는 명령.
# 토픽: /fleet/task_assignments (장애 주입기가 중계하며 지연·손실을 건다)

uint8 ACTION_ASSIGN=0   # 작업 할당
uint8 ACTION_CANCEL=1   # 작업 취소

# stamp: 명령 발행 시각. 응답 시간 KPI의 명령 시각(cmd_time)으로 쓴다
std_msgs/Header header

# 받을 로봇 (예: "amr_01"). 다른 로봇은 무시한다
string robot_id

# 명령 종류 (ACTION_*)
uint8 action

# ASSIGN이면 작업 전체, CANCEL이면 task_id만 채운다
Task task
