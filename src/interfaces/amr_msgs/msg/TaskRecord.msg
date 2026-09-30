# FleetTaskTable의 한 행: 작업 1건과 할당 현황.

# 작업 내용
Task task

# 맡은 로봇. 미할당이면 빈 문자열
string assignee_id

# 할당한 명령 번호 (TaskAssignment.assignment_id). 미할당이면 0
uint32 assignment_id

# 작업 상태 (TaskStatus.STATE_*와 같은 값)
uint8 state
