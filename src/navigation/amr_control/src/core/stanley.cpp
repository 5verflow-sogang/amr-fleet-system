#include "amr_control/core/stanley.hpp"

#include <algorithm>
#include <cmath>

namespace amr_control
{

Pose2D stanleyReferencePoint(const Pose2D & pose, double front_offset)
{
  return Pose2D{
    pose.x + front_offset * std::cos(pose.yaw),
    pose.y + front_offset * std::sin(pose.yaw),
    pose.yaw};
}

double stanleySteeringAngle(
  const StanleyConfig & config, double heading_error, double cte, double speed)
{
  const double steer = normalizeAngle(heading_error) -
    std::atan2(config.cte_gain * cte, config.softening_speed + std::max(speed, 0.0));
  return std::clamp(steer, -config.max_steer, config.max_steer);
}

double stanleyCurvature(const StanleyConfig & config, double steer)
{
  return std::tan(steer) / config.front_offset;
}

}  // namespace amr_control
