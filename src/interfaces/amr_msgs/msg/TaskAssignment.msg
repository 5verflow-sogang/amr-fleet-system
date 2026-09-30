# Fleet 서버가 로봇에게 작업을 맡기거나 취소하는 명령.
# 토픽: /fleet/task_assignments (장애 주입기가 중계하며 지연·손실을 건다)
#
# 로봇이 지킬 규칙
# - 받으면 TaskAssignmentAck로 assignment_id를 되돌려 보낸다. 서버는 Ack가 없으면 같은 번호로 재전송한다.
# - 이미 처리한 assignment_id가 다시 오면 작업을 다시 시작하지 않고 Ack만 다시 보낸다 (멱등 처리).
# - 마지막으로 처리한 것보다 작은 assignment_id는 무시한다 (지연으로 순서가 뒤바뀐 경우).
#   서버가 재시작하면 번호가 다시 시작하므로, FleetHeartbeat.epoch가 바뀌면 이 기준을 초기화한다.

uint8 ACTION_ASSIGN=0   # 작업 할당
uint8 ACTION_CANCEL=1   # 작업 취소

# stamp: 서버가 처음 보낸 시각. 응답 시간 KPI의 명령 시각(cmd_time)으로 쓴다
#        재전송해도 바꾸지 않고, 장애 주입기도 바꾸지 않는다
std_msgs/Header header

# 명령 번호. 서버가 명령마다 1씩 늘리고, 재전송할 때는 같은 번호를 쓴다
uint32 assignment_id

# 받을 로봇 (예: "amr_01"). 다른 로봇은 무시한다
string robot_id

# 명령 종류 (ACTION_*)
uint8 action

# ASSIGN이면 작업 전체, CANCEL이면 task_id만 채운다
Task task
