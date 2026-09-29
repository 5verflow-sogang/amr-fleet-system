# 추적 중인 장애물 1개. LiDAR로 추적하고, 카메라에 보이면 YOLO 검출을 짝지어 클래스를 붙인다.
# DetectedObject3D와 달리 프레임이 바뀌어도 유지되는 ID와 속도가 있다.
# 이 로봇과의 충돌 예측(TTC, safety margin)은 CollisionRisk로 분리했다 (장애물의 성질이 아니라 관계라서).

uint8 SOURCE_UNKNOWN=0        # 출처를 모르거나 아직 구분하지 않음
uint8 SOURCE_LIDAR=1          # LiDAR만 (카메라 시야 밖이거나 YOLO 미검출)
uint8 SOURCE_CAMERA=2         # 카메라만
uint8 SOURCE_LIDAR_CAMERA=3   # LiDAR + 카메라 검출

# 트랙 ID. 같은 장애물이면 유지되고, 사라진 트랙의 ID는 다시 쓰지 않는다. 0 이상
int32 track_id

# 측정 출처 (SOURCE_*)
uint8 source

# 짝지어진 카메라 검출의 클래스·confidence. 카메라 검출이 없으면 "unknown", 0.0
string class_name
float32 confidence

# 움직이는 장애물이면 true
bool is_dynamic

# 위치·속도 추정값과 KF 상태 추정 공분산 P. 좌표계는 TrackedObstacleArray.header.frame_id
# 방향·각속도는 추정하지 않는다: DetectedObject3D.pose와 같은 규칙
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

# 장애물 크기: LiDAR 점들을 감싸는 원의 반경 (m). ORCA 충돌 반경으로도 쓴다
float32 radius
