#ifndef AMR_CONTROL__CORE__STANLEY_HPP_
#define AMR_CONTROL__CORE__STANLEY_HPP_

// Stanley: heading 오차 + atan(k·CTE / v)로 조향각을 정한다 (Thrun et al., 2006).
// 차동구동은 앞바퀴가 없으므로 로봇 중심에서 front_offset만큼 앞의 점을 기준점으로 삼고,
// 그 점의 속도 방향이 조향각 δ만큼 꺾이도록 각속도를 고른다: atan2(ω·d, v) = δ → ω = v·tan(δ)/d.
// 이는 축거 d인 자전거 모델과 같은 식이라 원 논문의 수렴 성질이 그대로 남는다.
// ROS 비의존 코어 (CONTRIBUTING 4-4).

#include "amr_control/core/path_geometry.hpp"

namespace amr_control
{

struct StanleyConfig
{
  // CTE 게인 k [1/s]. 기준점 횡오차가 대략 exp(−k·t)로 준다
  double cte_gain = 2.0;
  // 저속에서 atan(k·e/v)가 튀지 않게 분모에 더하는 속도 [m/s]
  double softening_speed = 0.2;
  // 로봇 중심에서 기준점까지 앞쪽 거리 d [m]. 0보다 커야 한다
  double front_offset = 0.3;
  // 조향각 상한 [rad] (π/2 미만)
  double max_steer = 1.2;
};

// 로봇 중심 앞 front_offset 위치 (yaw는 로봇과 같다)
Pose2D stanleyReferencePoint(const Pose2D & pose, double front_offset);

// 조향각 δ [rad] = heading_error − atan2(k·cte, softening_speed + max(speed, 0)), ±max_steer로 자름.
// heading_error = 경로 방향 − 로봇 방향, cte는 기준점의 부호 있는 횡오차(왼쪽 +).
double stanleySteeringAngle(
  const StanleyConfig & config, double heading_error, double cte, double speed);

// 조향각에 해당하는 곡률 [1/m] = tan(δ) / front_offset. ω = v × 곡률
double stanleyCurvature(const StanleyConfig & config, double steer);

}  // namespace amr_control

#endif  // AMR_CONTROL__CORE__STANLEY_HPP_
