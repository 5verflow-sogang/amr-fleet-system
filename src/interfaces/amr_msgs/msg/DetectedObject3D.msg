# YOLO가 카메라 한 프레임에서 검출한 객체 1개의 인식 결과와 3D 위치.
# 한 번의 관측이라 ID·속도가 없다. 한 프레임의 객체들은 DetectedObject3DArray로 묶는다.

# 클래스 라벨. YOLO 라벨 그대로. 허용 값은 docs/api.md 4-3절 (예: "cargo", "person", "sign")
string class_name

# class_name의 YOLO confidence (0.0~1.0). 보정하지 않은 원본 값
float32 confidence

# 이미지 속 2D bbox (픽셀, 원점은 이미지 왼쪽 위, 중심 좌표와 크기)
float32 bbox_center_x
float32 bbox_center_y
float32 bbox_width
float32 bbox_height

# 3D 위치와 측정 공분산. 좌표계는 DetectedObject3DArray.header.frame_id
# 방향은 추정하지 않는다: orientation은 (0, 0, 0, 1), 공분산의 회전 성분 대각은 1e6
geometry_msgs/PoseWithCovariance pose

# 카메라에서 객체까지 직선거리 (m)
float32 distance
