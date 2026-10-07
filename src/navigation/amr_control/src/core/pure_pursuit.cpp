#include "amr_control/core/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>

namespace amr_control
{

double purePursuitLookahead(const PurePursuitConfig & config, double speed)
{
  if (!config.adaptive_lookahead) {
    return config.lookahead_distance;
  }
  return std::clamp(
    config.lookahead_time * std::abs(speed), config.min_lookahead, config.max_lookahead);
}

double purePursuitCurvature(const Pose2D & pose, const PathPoint & target)
{
  const double dx = target.x - pose.x;
  const double dy = target.y - pose.y;
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  const double y_local = -s * dx + c * dy;
  const double dist2 = dx * dx + dy * dy;
  if (dist2 <= 1e-12) {
    return 0.0;
  }
  return 2.0 * y_local / dist2;
}

}  // namespace amr_control
