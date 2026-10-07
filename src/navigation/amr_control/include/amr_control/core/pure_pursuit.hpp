#ifndef AMR_CONTROL__CORE__PURE_PURSUIT_HPP_
#define AMR_CONTROL__CORE__PURE_PURSUIT_HPP_

// Pure Pursuit: 경로 위 lookahead 점을 지나는 원호의 곡률로 각속도를 낸다.
// ROS 비의존 코어 (CONTRIBUTING 4-4).

#include "amr_control/core/path_geometry.hpp"

namespace amr_control
{

struct PurePursuitConfig
{
  // true면 lookahead = lookahead_time × |v|를 [min_lookahead, max_lookahead]로 자른다 (속도 비례).
  // false면 lookahead_distance 고정.
  bool adaptive_lookahead = true;
  double lookahead_distance = 0.6;
  double lookahead_time = 1.0;
  double min_lookahead = 0.3;
  double max_lookahead = 1.5;
};

// 현재 선속도 [m/s]에서 쓸 lookahead 거리 [m]
double purePursuitLookahead(const PurePursuitConfig & config, double speed);

// 로봇 자세에서 target을 지나는 원호의 곡률 [1/m]. 왼쪽(반시계) 회전이 +.
// κ = 2·y_l / L², (x_l, y_l)은 로봇 좌표계의 target, L은 로봇–target 거리.
// target이 로봇과 겹치면 0.
double purePursuitCurvature(const Pose2D & pose, const PathPoint & target);

}  // namespace amr_control

#endif  // AMR_CONTROL__CORE__PURE_PURSUIT_HPP_
