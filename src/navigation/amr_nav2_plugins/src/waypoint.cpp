#include "amr_nav2_plugins/waypoint.hpp"

#include <cmath>
#include <cstddef>
#include <limits>

namespace amr_nav2_plugins
{

namespace
{

double distance(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

// 경로에서 로봇과 가장 가까운 점의 인덱스. 뒤로 지나친 점을 다시 lookahead 기준으로
// 잡지 않게 한다(경로가 로봇을 지나 되돌아오는 모양이어도 항상 "앞"을 본다).
std::size_t closestIndex(
  const nav_msgs::msg::Path & global_plan, const geometry_msgs::msg::Point & robot_point)
{
  std::size_t best_index = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < global_plan.poses.size(); ++i) {
    const double d = distance(global_plan.poses[i].pose.position, robot_point);
    if (d < best_distance) {
      best_distance = d;
      best_index = i;
    }
  }
  return best_index;
}

}  // namespace

geometry_msgs::msg::PoseStamped selectLookaheadWaypoint(
  const nav_msgs::msg::Path & global_plan,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double lookahead_distance)
{
  if (global_plan.poses.empty()) {
    return robot_pose;
  }

  const std::size_t start = closestIndex(global_plan, robot_pose.pose.position);

  double accumulated = 0.0;
  geometry_msgs::msg::Point previous = global_plan.poses[start].pose.position;
  for (std::size_t i = start; i < global_plan.poses.size(); ++i) {
    const geometry_msgs::msg::Point & current = global_plan.poses[i].pose.position;
    accumulated += distance(previous, current);
    if (accumulated >= lookahead_distance) {
      return global_plan.poses[i];
    }
    previous = current;
  }

  return global_plan.poses.back();
}

}  // namespace amr_nav2_plugins
