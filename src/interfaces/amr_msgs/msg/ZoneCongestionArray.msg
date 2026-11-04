# 한 계산 주기의 구역별 혼잡도. 사람이 없어도 구역 파일의 모든 구역을 순서대로 담아 발행한다.
# 토픽: /fleet/congestion

# stamp: 계산 시각 (시뮬레이션 시간)
# frame_id: 구역 좌표계 (기본 "map")
std_msgs/Header header

ZoneCongestion[] zones
