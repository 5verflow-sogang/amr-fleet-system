# Fleet 서버가 로봇에게 작업을 맡기거나 취소하는 명령.
# 토픽: /fleet/task_assignments (장애 주입기가 중계하며 지연·손실을 건다)
#
# 로봇이 지킬 규칙
# - 받으면 TaskAssignmentAck로 assignment_id를 되돌려 보낸다. 서버는 Ack가 없으면 같은 번호로 재전송한다.
# - 이미 처리한 assignment_id가 다시 오면 작업을 다시 시작하지 않고 Ack만 다시 보낸다 (멱등 처리).
# - 마지막으로 처리한 것보다 작은 assignment_id는 무시한다 (지연으로 순서가 뒤바뀐 경우).
#   서버가 재시작하면 번호가 다시 시작하므로, FleetHeartbeat.epoch가 바뀌면 이 기준을 초기화한다.
# - 번호는 출처마다 따로 늘어나므로 "같은 출처끼리만" 크기를 비교한다 (하이브리드에서 서버·경매 명령이 섞인다).
#   출처는 번호대로 가린다: 1,000,000 미만 = Fleet 서버, 1,000,000 + k × 100,000 + 순번 = 분산 경매로 작업을
#   수락한 로봇 k (amr_fleet_decentralized). epoch가 바뀌면 서버 출처의 기준만 초기화한다 (api.md 4-3절).

uint8 ACTION_ASSIGN=0   # 작업 할당
uint8 ACTION_CANCEL=1   # 작업 취소

# stamp: 서버가 처음 보낸 시각 (노드 시계 = 시뮬레이션 시간). 재전송해도 바꾸지 않고, 장애 주입기도 바꾸지 않는다
#        응답 시간 KPI는 실제 시간으로 재므로 이 값을 쓰지 않는다. cmd_time은 하네스 기록기가
#        이 메시지를 받은 시각이다 (tools/harness/README.md, response_time.csv)
std_msgs/Header header

# 명령 번호. 서버가 명령마다 1씩 늘리고, 재전송할 때는 같은 번호를 쓴다
uint32 assignment_id

# 받을 로봇 (예: "amr_01"). 다른 로봇은 무시한다
string robot_id

# 명령 종류 (ACTION_*)
uint8 action

# ASSIGN이면 작업 전체, CANCEL이면 task_id만 채운다
Task task
