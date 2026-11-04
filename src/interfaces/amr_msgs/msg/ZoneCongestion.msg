# 구역 1개의 혼잡도. 계산식과 구역 파일 형식은 docs/api.md 4-5절.

# 구역 이름 (혼잡도 구역 파일의 id, 예: "buffet_line")
string zone_id

# 구역 범위 [x_min, x_max, y_min, y_max] (m, ZoneCongestionArray.header.frame_id 기준).
# 대시보드가 구역 파일을 따로 읽지 않고 그릴 수 있게 함께 보낸다
float32[4] area

# 구역 안 사람 수 (이번 계산 주기, 평활 전)
uint16 people_count

# 혼잡도 0~1. min(1, people_count / capacity)를 지수 평활한 값
float32 value
