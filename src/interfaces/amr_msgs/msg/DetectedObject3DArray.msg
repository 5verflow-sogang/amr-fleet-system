# 카메라 한 프레임의 인식 결과 전체. 검출이 없으면 빈 배열로 발행한다.
# 토픽: perception/objects (로봇 네임스페이스 아래, 예: /amr_01/perception/objects)

# stamp: 영상 촬영 시각 (추론이 끝난 시각이 아님)
# frame_id: objects[].pose의 좌표계 (기본 "map")
std_msgs/Header header

DetectedObject3D[] objects
