# 로봇 안전 상태. level·reason·speed_limit이 바뀌면 즉시, 그 외에는 10 Hz로 발행한다.
# 토픽: safety/status (로봇 네임스페이스 아래, 예: /amr_01/safety/status)

uint8 LEVEL_NORMAL=0     # 제한 없음
uint8 LEVEL_WARNING=1    # Warning 존: 감속
uint8 LEVEL_CRITICAL=2   # Critical 존 또는 긴급정지 거리(0.3m) 이내: 정지
uint8 LEVEL_ESTOP=3      # E-Stop: 해제 요청 전까지 정지 유지

uint8 REASON_NONE=0
uint8 REASON_OBSTACLE_PROXIMITY=1   # 장애물이 Safety Zone 안에 있음
uint8 REASON_COLLISION_RISK=2       # CollisionRisk의 TTC가 임계값 이하
uint8 REASON_ESTOP_REQUEST=3        # 외부 E-Stop 요청
uint8 REASON_SENSOR_FAULT=4         # 센서 고장 감지

std_msgs/Header header

# 안전 단계 (LEVEL_*)
uint8 level

# 현재 단계의 원인 (REASON_*)
uint8 reason

# 사람이 읽는 설명. 예: "person at 1.0m (track 17)"
string detail

# 위험을 일으킨 장애물 (TrackedObstacle.track_id). 장애물과 무관하면 -1
# 원인이 여러 개면 가장 심각한 하나를 담는다 (SENSOR_FAULT는 detail에 함께 적는다)
int32 related_track_id

# 가장 가까운 장애물까지 거리 (m). 없으면 inf
float32 min_obstacle_distance

# 지금 허용되는 최대 선속도 (m/s). 제어는 min(명령 속도, speed_limit)으로 쓴다
#   제한 없음(LEVEL_NORMAL): inf (공통 규칙 "inf = 해당 없음", docs/api.md 4-3절)
#   감속(LEVEL_WARNING): 0.6. 반응 지연 0.2 s + 감속 1.0 m/s²로 0.3 m 안에 서려면
#     CRITICAL에 들어갈 때 속도가 0.6 m/s 이하여야 한다 (0.6×0.2 + 0.6²/2 = 0.30 m)
#   정지(LEVEL_CRITICAL, LEVEL_ESTOP): 0.0
# 음수는 쓰지 않는다
float32 speed_limit
