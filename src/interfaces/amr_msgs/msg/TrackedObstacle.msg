# 추적 중인 장애물 1개. LiDAR로 추적하고, 카메라에 보이면 YOLO 검출을 짝지어 클래스를 붙인다.
# DetectedObject3D와 달리 프레임이 바뀌어도 유지되는 ID와 속도가 있다.

uint8 SOURCE_LIDAR=0          # LiDAR만 (카메라 시야 밖이거나 YOLO 미검출)
uint8 SOURCE_LIDAR_CAMERA=1   # LiDAR + 카메라 검출

# 트랙 ID. 같은 장애물이면 유지되고, 사라진 트랙의 ID는 다시 쓰지 않는다
uint32 track_id

# 측정 출처 (SOURCE_*)
uint8 source

# 짝지어진 카메라 검출의 클래스·confidence. SOURCE_LIDAR이면 "unknown", 0.0
string class_name
float32 confidence

# 움직이는 장애물이면 true
bool is_dynamic

# 위치·속도 추정값과 공분산. 좌표계는 TrackedObstacleArray.header.frame_id
# 방향·각속도는 추정하지 않는다: DetectedObject3D.pose와 같은 규칙
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

# 장애물 크기: LiDAR 점들을 감싸는 원의 반경 (m)
float32 radius

# 이 로봇과의 충돌 예측. 충돌 경로가 아니면 ttc·ttc_p5는 inf, 확률은 0.0. 계산하지 않았으면 NaN
float32 ttc                    # 충돌까지 남은 시간 (s), 현재 속도 유지 가정
float32 ttc_p5                 # 불확실성을 반영한 TTC의 하위 5% 값 (s)
float32 collision_probability  # 충돌 확률 (0.0~1.0)
