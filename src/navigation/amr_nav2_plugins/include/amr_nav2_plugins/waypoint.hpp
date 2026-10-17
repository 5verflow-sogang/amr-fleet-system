#ifndef AMR_NAV2_PLUGINS__WAYPOINT_HPP_
#define AMR_NAV2_PLUGINS__WAYPOINT_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace amr_nav2_plugins
{

// 전역 경로(setPlan으로 받은 global_plan)에서 로봇 바로 앞의 추종 지점을 고르는 순수
// 함수 - rclcpp::Node 없이 단위 테스트할 수 있도록 분리했다. DwaController는 이제 경로를
// 통째로 amr_planning_core::DwaPlanner에 넘기고, 코어의 pathPointAhead가 같은 규칙으로
// 기준점을 고른다(adaptive_lookahead).
//
// 로봇에 가장 가까운 점부터 시작해, 누적 거리가 lookahead_distance를 넘는 첫 점을
// 돌려준다(Pure Pursuit의 lookahead point 선택과 같은 방식). 경로가 lookahead보다
// 짧으면 마지막 점(최종 목표)을 돌려준다. 경로가 비어 있으면 로봇 위치 그대로.
geometry_msgs::msg::PoseStamped selectLookaheadWaypoint(
  const nav_msgs::msg::Path & global_plan,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double lookahead_distance);

}  // namespace amr_nav2_plugins

#endif  // AMR_NAV2_PLUGINS__WAYPOINT_HPP_
