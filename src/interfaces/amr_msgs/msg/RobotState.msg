# 로봇 1대의 현재 상태 요약. 주기적으로 발행한다 (1~5Hz).
# 토픽: /fleet/robot_states

uint8 MODE_IDLE=0       # 작업 없음, 할당 가능
uint8 MODE_BUSY=1       # 작업 수행 중
uint8 MODE_CHARGING=2   # 충전 중
uint8 MODE_ERROR=3      # 복구 못 한 오류, 할당 불가
uint8 MODE_ESTOP=4      # E-Stop 정지

# frame_id: pose의 좌표계 ("map")
std_msgs/Header header

# 로봇 이름 (예: "amr_01")
string robot_id

# 지도 위 위치·방향과 현재 속도
geometry_msgs/Pose pose
geometry_msgs/Twist velocity

# 운용 상태 (MODE_*)
uint8 mode

# 배터리 잔량 (0.0~100.0 %)
float32 battery_percent

# 수행 중인 작업 ID. 없으면 빈 문자열
string current_task_id

# 싣고 있는 물품 (Task.ITEM_*)
uint8 item_type

# 마지막으로 처리한 TaskAssignment.assignment_id. Ack가 유실돼도 서버가 이 값으로 확인한다
uint32 last_assignment_id
