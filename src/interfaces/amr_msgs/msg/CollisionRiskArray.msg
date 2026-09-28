# 한 계산 주기의 장애물별 충돌 위험. 위험 대상이 없어도 빈 배열로 발행한다.
# 토픽: perception/collision_risks (로봇 네임스페이스 아래, 예: /amr_01/perception/collision_risks)

# stamp: 계산에 쓴 추적 결과(TrackedObstacleArray)의 stamp
# frame_id: 계산 좌표계 (기본 "map")
std_msgs/Header header

CollisionRisk[] risks
