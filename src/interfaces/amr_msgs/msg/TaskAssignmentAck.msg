# 로봇이 TaskAssignment를 받았다고 서버에 알리는 확인.
# 토픽: /fleet/task_assignment_acks
# Ack 자체가 유실될 때를 대비해 RobotState.last_assignment_id로도 확인할 수 있다.

uint8 RESULT_ACCEPTED=0   # 수락 (CANCEL이면 취소 완료)
uint8 RESULT_REJECTED=1   # 거부. 이유는 error

# stamp: 로봇이 Ack를 보낸 시각
std_msgs/Header header

# 보낸 로봇 (예: "amr_02")
string robot_id

# 받은 명령의 assignment_id를 그대로 되돌린다
uint32 assignment_id

# 받은 명령의 header.stamp를 그대로 되돌린다. header.stamp - assignment_stamp가 확인까지 걸린 시간
builtin_interfaces/Time assignment_stamp

# 처리 결과 (RESULT_*)
uint8 result

# 거부 이유 (100번대). 수락이면 code = NONE
ErrorInfo error
