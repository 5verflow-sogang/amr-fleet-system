# 한 추적 주기의 장애물 목록. 장애물이 없으면 빈 배열로 발행한다.
# 토픽: perception/obstacles (로봇 네임스페이스 아래, 예: /amr_01/perception/obstacles)

# stamp: 추정값이 가리키는 시각
# frame_id: obstacles[]의 좌표계 (기본 "map")
std_msgs/Header header

TrackedObstacle[] obstacles
