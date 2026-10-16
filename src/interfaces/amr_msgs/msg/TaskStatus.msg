# 작업 상태 이벤트. 상태나 세부 단계가 바뀔 때마다 발행한다.
# 토픽: /fleet/task_status
# 마지막 이벤트 (COMPLETED·FAILED)가 작업 로그 한 줄이 된다.

uint8 STATE_PENDING=0       # 할당받음, 시작 전
uint8 STATE_IN_PROGRESS=1   # 수행 중
uint8 STATE_COMPLETED=2     # 완료
uint8 STATE_FAILED=3        # 실패

std_msgs/Header header

string task_id

# 수행하는 로봇 (예: "amr_01")
string robot_id

# 작업 상태 (STATE_*)
uint8 state

# 세부 단계 (BT 서브트리 이름). 예: "move_to_pickup", "docking", "loading", "unloading"
string phase

# 실패 원인. FAILED가 아니면 code = NONE
ErrorInfo error

# 시작·종료 시각. 아직이면 0
# start_time은 로봇이 작업을 수락한 시각이다. PENDING부터 채운다. 여러 로봇이 같은 작업을 보고하면 이 값이 가장
# 이른 로봇이 주인이다 (먼저 수락한 쪽이 주인, architecture.md 5-2절). 0이면 그 보고의 header.stamp로 본다
builtin_interfaces/Time start_time
builtin_interfaces/Time end_time

# 작업 시작 후 누적 이동 거리 (m)
float32 distance_traveled
