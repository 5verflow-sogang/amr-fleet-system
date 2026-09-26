# 작업 1건의 내용. JSON Task 파일을 이 형식으로 바꿔 쓴다.
# 토픽: /fleet/task_requests (새 작업 요청). TaskAssignment 안에 담겨 로봇에게 전달되기도 한다.

uint8 TYPE_DELIVERY=0   # 픽업 → 배송
uint8 TYPE_CHARGE=1     # 충전소로 이동
uint8 TYPE_RETURN=2     # 대기 위치로 복귀

uint8 ITEM_NONE=0
uint8 ITEM_SMALL=1      # 소형 박스 2kg
uint8 ITEM_MEDIUM=2     # 중형 박스 10kg
uint8 ITEM_LARGE=3      # 대형 박스 25kg

# 작업 ID. 시스템 전체에서 유일 (예: "task_0042")
string task_id

# 작업 종류 (TYPE_*)
uint8 task_type

# 운반 물품 (ITEM_*). DELIVERY가 아니면 ITEM_NONE
uint8 item_type

# 출발지·목적지: 스테이션 이름과 map 기준 도착 자세. CHARGE·RETURN은 pickup을 비운다
string pickup_station_id
geometry_msgs/Pose pickup_pose
string dropoff_station_id
geometry_msgs/Pose dropoff_pose

# 우선순위. 클수록 급하다 (0~255)
uint8 priority

# 마감 시각. 0이면 마감 없음
builtin_interfaces/Time deadline

# 작업이 요청된 시각
builtin_interfaces/Time created_time
