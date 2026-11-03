#ifndef AMR_NAV2_PLUGINS__ASTAR_PLANNER_HPP_
#define AMR_NAV2_PLUGINS__ASTAR_PLANNER_HPP_

#include <memory>
#include <string>

#include "amr_planning_core/astar.hpp"
#include "amr_planning_core/path_smoothing.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace amr_nav2_plugins
{

// amr_planning_core::AStarPlanner + smoothPath(ROS 비의존)를 nav2_core::GlobalPlanner로 감싼다.
// 이 클래스는 세 가지만 한다:
// 1. 전역 costmap 버퍼를 복사 없이 Grid로 감싸고, 출발·목표 world 좌표를 셀로 바꿈
// 2. 탐색(AStarPlanner)·평활화(smoothPath)를 코어에 맡김
// 3. 셀 단위 결과를 nav_msgs/Path로 바꿈(toPlanPath)
// 파라미터는 nav2_params.yaml planner_server.AStar 블록 (docs/algorithms/navigation.md 3절).
// 기본 planner(GridBased = NavFn)는 그대로이고, nav2.launch.py planner:=astar로 고를 때만 쓴다.
class AStarGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarGlobalPlanner() = default;
  ~AStarGlobalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // 실패(범위 밖, 출발·목표가 막힘, 경로 없음)면 빈 경로를 돌려준다 - planner_server가 실패로 처리한다
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("AStarGlobalPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  std::unique_ptr<amr_planning_core::AStarPlanner> planner_;
  bool smooth_path_ = true;
  // smoothing_point_spacing_m(미터)는 configure에서 costmap 해상도로 나눠 셀로 바꿔 둔다
  double smoothing_point_spacing_m_ = 0.1;
  amr_planning_core::SmoothingConfig smoothing_;
};

}  // namespace amr_nav2_plugins

#endif  // AMR_NAV2_PLUGINS__ASTAR_PLANNER_HPP_
