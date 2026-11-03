#ifndef AMR_NAV2_PLUGINS__PLAN_PATH_HPP_
#define AMR_NAV2_PLUGINS__PLAN_PATH_HPP_

#include <string>
#include <vector>

#include "amr_planning_core/grid.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace amr_nav2_plugins
{

// 셀 단위 점(셀 (x, y)의 중심이 (x, y), amr_planning_core::smoothPath의 출력)을 nav_msgs/Path로 바꾸는
// 순수 함수 - rclcpp::Node 없이 단위 테스트할 수 있도록 AStarGlobalPlanner에서 분리했다.
// - 위치: costmap 원점(origin_x, origin_y = 셀 (0,0)의 모서리) + (점 + 0.5) * resolution
// - 방향: 다음 점을 향하는 yaw. 마지막 점은 goal 자세(위치·방향) 그대로 - NavFn처럼 경로가 요청한 목표에서 끝난다
// 점이 비어 있으면 빈 경로(헤더만)를 돌려준다.
nav_msgs::msg::Path toPlanPath(
  const std::vector<amr_planning_core::Point2D> & points, double origin_x, double origin_y,
  double resolution, const std::string & frame_id, const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::PoseStamped & goal);

}  // namespace amr_nav2_plugins

#endif  // AMR_NAV2_PLUGINS__PLAN_PATH_HPP_
