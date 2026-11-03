#include "amr_nav2_plugins/dwa_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "amr_nav2_plugins/obstacles.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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

}  // namespace

void DwaController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  logger_ = node->get_logger();

  amr_planning_core::DwaConfig config;
  const std::string prefix = name_ + ".";
  declareAndGet(node, prefix + "max_v", config.max_v);
  declareAndGet(node, prefix + "min_v", config.min_v);
  declareAndGet(node, prefix + "max_w", config.max_w);
  declareAndGet(node, prefix + "acc_v", config.acc_v);
  declareAndGet(node, prefix + "acc_w", config.acc_w);
  declareAndGet(node, prefix + "dt", config.dt);
  declareAndGet(node, prefix + "sim_time", config.sim_time);
  declareAndGet(node, prefix + "footprint_half_length", config.footprint_half_length);
  declareAndGet(node, prefix + "footprint_half_width", config.footprint_half_width);
  declareAndGet(node, prefix + "heading_weight", config.heading_weight);
  declareAndGet(node, prefix + "clearance_weight", config.clearance_weight);
  declareAndGet(node, prefix + "velocity_weight", config.velocity_weight);
  declareAndGet(node, prefix + "goal_tolerance", config.goal_tolerance);
  declareAndGet(node, prefix + "predict_dynamic_obstacles", config.predict_dynamic_obstacles);
  declareAndGet(
    node, prefix + "dynamic_obstacle_safety_margin", config.dynamic_obstacle_safety_margin);
  declareAndGet(
    node, prefix + "dynamic_obstacle_influence_radius", config.dynamic_obstacle_influence_radius);
  declareAndGet(node, prefix + "dynamic_weight", config.dynamic_weight);
  declareAndGet(node, prefix + "use_admissible_velocity", config.use_admissible_velocity);
  declareAndGet(node, prefix + "use_goal_slowdown", config.use_goal_slowdown);
  declareAndGet(node, prefix + "v_samples", config.v_samples);
  declareAndGet(node, prefix + "w_samples", config.w_samples);
  declareAndGet(node, prefix + "lookahead_distance", lookahead_distance_);
  config.lookahead_distance = lookahead_distance_;
  declareAndGet(node, prefix + "adaptive_lookahead", config.adaptive_lookahead);
  declareAndGet(node, prefix + "rotate_in_place_min_angle", config.rotate_in_place_min_angle);
  declareAndGet(node, prefix + "rotate_in_place_exit_angle", config.rotate_in_place_exit_angle);
  declareAndGet(node, prefix + "rotate_in_place_clearance", config.rotate_in_place_clearance);
  declareAndGet(node, prefix + "avoid_mode_enabled", config.avoid_mode_enabled);
  declareAndGet(node, prefix + "avoid_margin_add", config.avoid_margin_add);
  declareAndGet(node, prefix + "avoid_weight_scale", config.avoid_weight_scale);
  declareAndGet(node, prefix + "avoid_rollout_time", config.avoid_rollout_time);
  AvoidModeParams avoid;
  avoid.enabled = config.avoid_mode_enabled;
  declareAndGet(node, prefix + "avoid_ttc_enter_s", avoid.enter_ttc_s);
  declareAndGet(node, prefix + "avoid_exit_hold_s", avoid.exit_hold_s);
  declareAndGet(node, prefix + "avoid_risk_timeout_s", avoid.risk_timeout_s);
  avoid_switch_ = AvoidModeSwitch(avoid);
  if (config.rotate_in_place_exit_angle > config.rotate_in_place_min_angle) {
    RCLCPP_WARN(
      logger_, "DWA: rotate_in_place_exit_angle(%.3f) > min_angle(%.3f) - min_angle로 줄인다",
      config.rotate_in_place_exit_angle, config.rotate_in_place_min_angle);
  }
  declareAndGet(node, prefix + "xy_goal_tolerance", xy_goal_tolerance_);
  declareAndGet(node, prefix + "yaw_goal_tolerance", yaw_goal_tolerance_);

  // lethal_cost_threshold는 uint8_t라 ParameterValue가 바로 지원하지 않는다 - int로
  // 우회한다. nav2 costmap 관례(LETHAL_OBSTACLE=254, INSCRIBED_INFLATED_OBSTACLE=253)
  // 범위라 0~255 안에서만 쓰므로 안전하다.
  int lethal_cost_threshold = static_cast<int>(config.lethal_cost_threshold);
  declareAndGet(node, prefix + "lethal_cost_threshold", lethal_cost_threshold);
  config.lethal_cost_threshold = static_cast<uint8_t>(lethal_cost_threshold);

  planner_ = std::make_unique<amr_planning_core::DwaPlanner>(config);

  obstacles_sub_ = node->create_subscription<amr_msgs::msg::TrackedObstacleArray>(
    "perception/obstacles", rclcpp::QoS(10),
    std::bind(&DwaController::onObstacles, this, std::placeholders::_1));
  if (config.avoid_mode_enabled) {
    risks_sub_ = node->create_subscription<amr_msgs::msg::CollisionRiskArray>(
      "perception/collision_risks", rclcpp::QoS(10),
      std::bind(&DwaController::onCollisionRisks, this, std::placeholders::_1));
  }
  // 상태가 바뀔 때만 발행한다 - 늦게 붙은 구독자(rosbag, I17 기록)도 지금 상태를 받도록 transient local
  avoid_mode_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "dwa/avoid_mode", rclcpp::QoS(1).reliable().transient_local());

  RCLCPP_INFO(
    logger_,
    "DwaController(%s) 설정: max_v=%.2f max_w=%.2f v_samples=%d w_samples=%d "
    "dynamic_weight=%.2f lookahead=%.2fm avoid_mode=%s(TTC<%.1fs)",
    name_.c_str(), config.max_v, config.max_w, config.v_samples, config.w_samples,
    config.dynamic_weight, lookahead_distance_, config.avoid_mode_enabled ? "on" : "off",
    avoid.enter_ttc_s);
}

void DwaController::cleanup()
{
  obstacles_sub_.reset();
  risks_sub_.reset();
  avoid_mode_pub_.reset();
  planner_.reset();
}

void DwaController::activate()
{
  avoid_mode_pub_->on_activate();
  // 처음 상태(꺼짐)를 한 번 내 둔다 - 기록하는 쪽이 첫 전환 전 상태를 알게
  std_msgs::msg::Bool msg;
  msg.data = planner_->avoidMode();
  avoid_mode_pub_->publish(msg);
}

void DwaController::deactivate()
{
  avoid_mode_pub_->on_deactivate();
}

void DwaController::setPlan(const nav_msgs::msg::Path & path)
{
  const std::string & costmap_frame = costmap_ros_->getGlobalFrameID();
  if (path.poses.empty() || path.header.frame_id.empty() ||
    path.header.frame_id == costmap_frame)
  {
    global_plan_ = path;
    return;
  }

  // controller_server가 이미 costmap 전역 프레임으로 맞춰 줄 수도 있지만, 보장되지
  // 않는 경우를 대비해 직접 변환한다(RegulatedPurePursuit 등 다른 플러그인도 같은
  // 패턴). 실패하면 경고만 남기고 원본을 그대로 쓴다 - 다음 setPlan에서 복구된다.
  try {
    const geometry_msgs::msg::TransformStamped transform =
      tf_->lookupTransform(costmap_frame, path.header.frame_id, tf2::TimePointZero);

    nav_msgs::msg::Path transformed;
    transformed.header.frame_id = costmap_frame;
    transformed.header.stamp = path.header.stamp;
    transformed.poses.reserve(path.poses.size());
    for (const geometry_msgs::msg::PoseStamped & pose : path.poses) {
      geometry_msgs::msg::PoseStamped out;
      tf2::doTransform(pose, out, transform);
      transformed.poses.push_back(out);
    }
    global_plan_ = transformed;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(
      logger_, "전역 경로를 %s -> %s로 변환 못 함, 원본 그대로 씀: %s",
      path.header.frame_id.c_str(), costmap_frame.c_str(), e.what());
    global_plan_ = path;
  }
}

void DwaController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

void DwaController::onObstacles(const amr_msgs::msg::TrackedObstacleArray::SharedPtr msg)
{
  if (costmap_ros_ && !msg->header.frame_id.empty() &&
    msg->header.frame_id != costmap_ros_->getGlobalFrameID())
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_.lock()->get_clock(), 5000,
      "perception/obstacles 좌표계(%s)가 costmap 전역 좌표계(%s)와 다르다 - 변환 없이 그대로 쓴다",
      msg->header.frame_id.c_str(), costmap_ros_->getGlobalFrameID().c_str());
  }

  std::vector<amr_planning_core::DynamicObstacle> obstacles = toDynamicObstacles(*msg);
  std::unordered_set<int32_t> dynamic_ids = dynamicTrackIds(*msg);

  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  obstacles_ = std::move(obstacles);
  dynamic_track_ids_ = std::move(dynamic_ids);
}

void DwaController::onCollisionRisks(const amr_msgs::msg::CollisionRiskArray::SharedPtr msg)
{
  double min_ttc = 0.0;
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    min_ttc = minDynamicTtc(*msg, dynamic_track_ids_);
  }
  const double now = node_.lock()->now().seconds();
  std::lock_guard<std::mutex> lock(avoid_mutex_);
  avoid_switch_.onRisks(min_ttc, now);
}

void DwaController::updateAvoidMode()
{
  auto node = node_.lock();
  const double now = node->now().seconds();
  bool active = false;
  bool stale = false;
  {
    std::lock_guard<std::mutex> lock(avoid_mutex_);
    active = avoid_switch_.update(now);
    stale = avoid_switch_.stale(now);
  }
  if (stale) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 2000,
      "DWA: perception/collision_risks가 끊겼다 - 회피 모드를 지금 상태(%s)로 유지한다",
      active ? "켜짐" : "꺼짐");
  }
  if (active == planner_->avoidMode()) {
    return;
  }
  planner_->setAvoidMode(active);
  const amr_planning_core::DwaConfig & config = planner_->config();
  RCLCPP_INFO(
    logger_, "DWA 회피 모드 %s (safety_margin=%.2fm dynamic_weight=%.2f rollout=%.1fs)",
    active ? "켬" : "끔", config.dynamic_obstacle_safety_margin, config.dynamic_weight,
    planner_->rolloutTime());
  std_msgs::msg::Bool msg;
  msg.data = active;
  avoid_mode_pub_->publish(msg);
}

geometry_msgs::msg::TwistStamped DwaController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = node_.lock()->now();

  updateAvoidMode();

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  const double origin_x = costmap->getOriginX();
  const double origin_y = costmap->getOriginY();

  const amr_planning_core::Grid grid(
    static_cast<std::size_t>(costmap->getSizeInCellsX()),
    static_cast<std::size_t>(costmap->getSizeInCellsY()),
    costmap->getResolution(), costmap->getCharMap());

  amr_planning_core::RobotState state;
  state.x = pose.pose.position.x - origin_x;
  state.y = pose.pose.position.y - origin_y;
  state.yaw = tf2::getYaw(pose.pose.orientation);
  state.v = velocity.linear.x;
  state.w = velocity.angular.z;

  // 최종 접근: 전역 경로 끝(최종 목표)과의 위치 오차가 xy_goal_tolerance_ 안이면
  // waypoint 추종을 멈추고 제자리 회전으로 목표 각도를 맞춘다. 보통 흐름
  // (selectLookaheadWaypoint + computeVelocityCommand)은 지역 waypoint의 "위치"만
  // 보고 각도는 전혀 안 보므로(DwaConfig::goal_tolerance 주석: 최종 도킹 정밀도는
  // 범위 밖), 위치는 맞았는데 각도가 안 맞아 goal_checker를 못 넘기면 로봇이 멈춘
  // 채 아무 것도 안 해서 progress_checker가 "Failed to make progress"로 중단시키는
  // 문제가 있었다(실제로 겪음 - docs/algorithms/navigation.md 참고).
  if (!global_plan_.poses.empty()) {
    const geometry_msgs::msg::PoseStamped & final_pose = global_plan_.poses.back();
    const double dx = final_pose.pose.position.x - pose.pose.position.x;
    const double dy = final_pose.pose.position.y - pose.pose.position.y;
    if (std::hypot(dx, dy) <= xy_goal_tolerance_) {
      const double goal_yaw = tf2::getYaw(final_pose.pose.orientation);
      const amr_planning_core::DwaCommand rotate_command =
        planner_->rotateToGoalYaw(state, goal_yaw, yaw_goal_tolerance_);
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = rotate_command.w;
      return cmd_vel;
    }
  }

  // 전역 경로를 Grid 좌표로 넘긴다. heading 기준점(lookahead_distance, adaptive_lookahead)과
  // 목표 앞 감속의 최종 목표(경로 마지막 점)는 코어가 이 경로에서 고른다.
  std::vector<amr_planning_core::Point2D> path;
  path.reserve(global_plan_.poses.size());
  for (const geometry_msgs::msg::PoseStamped & plan_pose : global_plan_.poses) {
    path.push_back(amr_planning_core::Point2D{
      plan_pose.pose.position.x - origin_x, plan_pose.pose.position.y - origin_y});
  }

  std::vector<amr_planning_core::DynamicObstacle> obstacles;
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    obstacles = obstacles_;
  }
  // onObstacles는 costmap 전역 좌표계 그대로 저장한다(롤링 윈도우라 origin이
  // 매 호출마다 바뀌므로, origin을 빼는 건 콜백 시점이 아니라 여기서 매번 한다).
  for (amr_planning_core::DynamicObstacle & obstacle : obstacles) {
    obstacle.x -= origin_x;
    obstacle.y -= origin_y;
  }

  const amr_planning_core::DwaCommand command =
    planner_->computeVelocityCommand(state, path, grid, obstacles);

  if (command.status == amr_planning_core::DwaStatus::kNoValidTrajectory) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_.lock()->get_clock(), 1000, "DWA: 유효한 궤적을 못 찾아 정지한다");
    return cmd_vel;
  }

  // kGoalReached는 "다음 추종 지점"(지역 waypoint) 기준이라 전역 목표 도달과는
  // 다르다 - 전역 목표 판정은 goal_checker(nav2_controller)가 맡는다. 경로 끝
  // 근처에서만 실제로 나오므로 정지가 안전한 기본 동작이다.
  double v = command.v;
  if (command.status == amr_planning_core::DwaStatus::kGoalReached) {
    v = 0.0;
  } else if (speed_limit_ >= 0.0) {
    const double limit = speed_limit_is_percentage_ ?
      planner_->config().max_v * speed_limit_ / 100.0 : speed_limit_;
    v = std::min(v, limit);
  }

  cmd_vel.twist.linear.x = v;
  cmd_vel.twist.angular.z = (command.status == amr_planning_core::DwaStatus::kGoalReached) ?
    0.0 : command.w;
  return cmd_vel;
}

}  // namespace amr_nav2_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(amr_nav2_plugins::DwaController, nav2_core::Controller)
