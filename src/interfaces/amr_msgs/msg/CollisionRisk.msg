# 추적 장애물 1개와 이 로봇 사이의 충돌 위험.
# 충돌 경로가 아니면 ttc·ttc_p5는 inf, 확률은 0.0. 계산하지 않았으면 NaN (연구 2 기능을 끈 baseline)

# 어느 추적 장애물에 대한 결과인지 (TrackedObstacle.track_id)
int32 track_id

# 현재 상대 위치·속도가 유지된다고 할 때 충돌까지 남은 시간 (s)
float32 ttc

# 추정 불확실성(KF의 P)을 반영한 TTC의 하위 5% 값 (s)
float32 ttc_p5

# 충돌 확률 (0.0~1.0)
float32 collision_probability

# KF의 P와 TTC로 계산한 Adaptive Safety Margin (m)
float32 safety_margin
