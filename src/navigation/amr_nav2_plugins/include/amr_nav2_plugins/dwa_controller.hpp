#ifndef AMR_NAV2_PLUGINS__DWA_CONTROLLER_HPP_
#define AMR_NAV2_PLUGINS__DWA_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "amr_msgs/msg/collision_risk_array.hpp"
#include "amr_msgs/msg/tracked_obstacle_array.hpp"
#include "amr_nav2_plugins/avoid_mode.hpp"
#include "amr_planning_core/dwa.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"

namespace amr_nav2_plugins
{

// amr_planning_core::DwaPlanner(ROS 비의존)를 nav2_core::Controller로 감싼다.
// 탐색 로직은 전부 DwaPlanner에 위임하고, 이 클래스는 세 가지만 한다:
// 1. costmap <-> DwaPlanner의 Grid-로컬 좌표 변환 (costmap origin을 빼고 더함)
// 2. perception/obstacles(TrackedObstacleArray)를 DynamicObstacle로 변환
// 3. 전역 경로를 Grid 좌표 점 목록으로 바꿔 넘김(추종 기준점은 코어가 고른다)
// 4. perception/collision_risks의 동적 트랙 최소 TTC로 회피 모드를 켜고 끔(AvoidModeSwitch, 4-10절)
class DwaController : public nav2_core::Controller
{
public:
  DwaController() = default;
  ~DwaController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  void onObstacles(const amr_msgs::msg::TrackedObstacleArray::SharedPtr msg);
  void onCollisionRisks(const amr_msgs::msg::CollisionRiskArray::SharedPtr msg);
  // 제어 주기마다: 회피 모드를 갱신하고, 바뀌면 코어에 넘기고 dwa/avoid_mode를 발행한다.
  void updateAvoidMode();

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("DwaController")};
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::unique_ptr<amr_planning_core::DwaPlanner> planner_;
  double lookahead_distance_ = 1.5;

  // 전역 경로 끝(최종 목표)과의 위치 오차가 이 안이면 waypoint 추종을 멈추고
  // rotateToGoalYaw로 넘어간다 - general_goal_checker(nav2_controller)의
  // xy_goal_tolerance/yaw_goal_tolerance와 같은 값으로 둬야 한다(안 그러면 여기선
  // "도착"으로 보고 회전하는데 goal_checker는 아직 멀다고 보거나, 반대로 goal_checker는
  // 이미 통과시켰는데 여기선 계속 회전하는 불일치가 생긴다).
  double xy_goal_tolerance_ = 0.25;
  double yaw_goal_tolerance_ = 0.25;

  nav_msgs::msg::Path global_plan_;

  rclcpp::Subscription<amr_msgs::msg::TrackedObstacleArray>::SharedPtr obstacles_sub_;
  std::mutex obstacles_mutex_;
  std::vector<amr_planning_core::DynamicObstacle> obstacles_;
  std::unordered_set<int32_t> dynamic_track_ids_;
  std::string obstacles_frame_id_;

  rclcpp::Subscription<amr_msgs::msg::CollisionRiskArray>::SharedPtr risks_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr avoid_mode_pub_;
  std::mutex avoid_mutex_;
  AvoidModeSwitch avoid_switch_;

  // Nav2 speed_limit 관례: 음수면 제한 없음. percentage면 config().max_v에 대한 비율.
  double speed_limit_ = -1.0;
  bool speed_limit_is_percentage_ = false;
};

}  // namespace amr_nav2_plugins

#endif  // AMR_NAV2_PLUGINS__DWA_CONTROLLER_HPP_
