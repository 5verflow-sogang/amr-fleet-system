#include "amr_nav2_plugins/astar_planner.hpp"

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "amr_nav2_plugins/plan_path.hpp"
#include "amr_planning_core/grid.hpp"
#include "nav2_util/node_utils.hpp"

namespace amr_nav2_plugins
{

namespace
{

template<typename FieldT>
void declareAndGet(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name,
  FieldT & field)
{
  nav2_util::declare_parameter_if_not_declared(node, name, rclcpp::ParameterValue(field));
  node->get_parameter(name, field);
}

const char * statusName(amr_planning_core::PlanStatus status)
{
  switch (status) {
    case amr_planning_core::PlanStatus::kSuccess: return "성공";
    case amr_planning_core::PlanStatus::kStartOutOfBounds: return "출발이 지도 밖";
    case amr_planning_core::PlanStatus::kGoalOutOfBounds: return "목표가 지도 밖";
    case amr_planning_core::PlanStatus::kStartOccupied: return "출발 셀이 막힘";
    case amr_planning_core::PlanStatus::kGoalOccupied: return "목표 셀이 막힘";
    case amr_planning_core::PlanStatus::kNoPathFound: return "경로 없음";
  }
  return "알 수 없음";
}

}  // namespace

void AStarGlobalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  auto node = node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  const std::string prefix = name_ + ".";
  amr_planning_core::AStarConfig config;
  std::string heuristic = "octile";
  declareAndGet(node, prefix + "heuristic", heuristic);
  if (heuristic == "euclidean") {
    config.heuristic = amr_planning_core::HeuristicType::kEuclidean;
  } else if (heuristic != "octile") {
    RCLCPP_WARN(logger_, "%sheuristic '%s'를 모름, octile로 씀", prefix.c_str(), heuristic.c_str());
  }
  int lethal = config.lethal_cost_threshold;
  declareAndGet(node, prefix + "lethal_cost_threshold", lethal);
  config.lethal_cost_threshold = static_cast<uint8_t>(std::clamp(lethal, 1, 255));
  declareAndGet(node, prefix + "cost_weight", config.cost_weight);
  declareAndGet(node, prefix + "prevent_corner_cutting", config.prevent_corner_cutting);
  planner_ = std::make_unique<amr_planning_core::AStarPlanner>(config);

  declareAndGet(node, prefix + "smooth_path", smooth_path_);
  declareAndGet(node, prefix + "smoothing_shortcut", smoothing_.shortcut);
  declareAndGet(node, prefix + "smoothing_gradient", smoothing_.gradient);
  declareAndGet(node, prefix + "smoothing_weight_data", smoothing_.weight_data);
  declareAndGet(node, prefix + "smoothing_weight_smooth", smoothing_.weight_smooth);
  declareAndGet(node, prefix + "smoothing_max_iterations", smoothing_.max_iterations);
  declareAndGet(node, prefix + "smoothing_point_spacing_m", smoothing_point_spacing_m_);
  smoothing_.lethal_cost_threshold = config.lethal_cost_threshold;

  RCLCPP_INFO(
    logger_, "AStarGlobalPlanner '%s': heuristic %s, cost_weight %.2f, lethal %d, 평활화 %s",
    name_.c_str(), heuristic.c_str(), config.cost_weight, static_cast<int>(config.lethal_cost_threshold),
    smooth_path_ ? "켬" : "끔");
}

void AStarGlobalPlanner::cleanup()
{
  planner_.reset();
}

void AStarGlobalPlanner::activate()
{
}

void AStarGlobalPlanner::deactivate()
{
}

nav_msgs::msg::Path AStarGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  const std::string global_frame = costmap_ros_->getGlobalFrameID();
  nav_msgs::msg::Path empty;
  empty.header.frame_id = global_frame;
  empty.header.stamp = clock_->now();
  // planner_server가 출발·목표를 전역 프레임으로 바꿔서 넘긴다. 다르면 좌표를 섞지 않고 실패로 둔다
  if (start.header.frame_id != global_frame || goal.header.frame_id != global_frame) {
    RCLCPP_WARN(
      logger_, "출발(%s)·목표(%s) 프레임이 전역 costmap 프레임(%s)과 다름", start.header.frame_id.c_str(),
      goal.header.frame_id.c_str(), global_frame.c_str());
    return empty;
  }

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());

  unsigned int sx = 0, sy = 0, gx = 0, gy = 0;
  if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
    !costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy))
  {
    RCLCPP_WARN(logger_, "출발 또는 목표가 전역 costmap 밖");
    return empty;
  }

  const double resolution = costmap->getResolution();
  const double origin_x = costmap->getOriginX();
  const double origin_y = costmap->getOriginY();
  const amr_planning_core::Grid grid(
    costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), resolution, costmap->getCharMap());
  const amr_planning_core::PlanResult result = planner_->plan(
    grid, amr_planning_core::Cell{static_cast<int>(sx), static_cast<int>(sy)},
    amr_planning_core::Cell{static_cast<int>(gx), static_cast<int>(gy)});
  if (result.status != amr_planning_core::PlanStatus::kSuccess) {
    RCLCPP_WARN(logger_, "A* 실패: %s", statusName(result.status));
    return empty;
  }

  std::vector<amr_planning_core::Point2D> points;
  if (smooth_path_) {
    // 지도가 바뀌면(static layer가 해상도를 맞춤) 해상도도 바뀌므로 셀 간격은 매번 계산한다
    amr_planning_core::SmoothingConfig smoothing = smoothing_;
    smoothing.point_spacing = smoothing_point_spacing_m_ / resolution;
    points = amr_planning_core::smoothPath(grid, result.path, smoothing);
  } else {
    points.reserve(result.path.size());
    for (const amr_planning_core::Cell & cell : result.path) {
      points.push_back({static_cast<double>(cell.x), static_cast<double>(cell.y)});
    }
  }
  lock.unlock();

  return toPlanPath(points, origin_x, origin_y, resolution, global_frame, empty.header.stamp, goal);
}

}  // namespace amr_nav2_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(amr_nav2_plugins::AStarGlobalPlanner, nav2_core::GlobalPlanner)
