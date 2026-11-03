#include "amr_nav2_plugins/plan_path.hpp"

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace amr_nav2_plugins
{

nav_msgs::msg::Path toPlanPath(
  const std::vector<amr_planning_core::Point2D> & points, double origin_x, double origin_y,
  double resolution, const std::string & frame_id, const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  if (points.empty()) {
    return path;
  }

  path.poses.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = origin_x + (points[i].x + 0.5) * resolution;
    pose.pose.position.y = origin_y + (points[i].y + 0.5) * resolution;
    if (i + 1 < points.size()) {
      const double yaw = std::atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x);
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
    } else {
      pose.pose.position = goal.pose.position;
      pose.pose.orientation = goal.pose.orientation;
    }
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace amr_nav2_plugins
