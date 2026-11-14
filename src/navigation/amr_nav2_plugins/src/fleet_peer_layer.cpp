#include "amr_nav2_plugins/fleet_peer_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace amr_nav2_plugins
{

void FleetPeerLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("FleetPeerLayer: costmap 노드가 없다");
  }
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("states_topic", rclcpp::ParameterValue(std::string("/fleet/robot_states")));
  // 비우면 노드 네임스페이스(/amr_01/...)의 첫 칸
  declareParameter("robot_id", rclcpp::ParameterValue(std::string("")));
  // 다른 로봇 차체 반길이·반폭 (600 x 400 mm, global/local costmap footprint와 같다)
  declareParameter("peer_half_length", rclcpp::ParameterValue(0.3));
  declareParameter("peer_half_width", rclcpp::ParameterValue(0.2));
  // 위치 지연 여유: 상태 2 Hz + 위치추정 오차. 외삽이 지연 대부분을 메운다
  declareParameter("padding", rclcpp::ParameterValue(0.1));
  declareParameter("timeout", rclcpp::ParameterValue(2.0));
  declareParameter("max_extrapolation", rclcpp::ParameterValue(1.0));

  std::string topic;
  PeerTracker::Config config;
  node->get_parameter(getFullName("enabled"), enabled_);
  node->get_parameter(getFullName("states_topic"), topic);
  node->get_parameter(getFullName("robot_id"), robot_id_);
  node->get_parameter(getFullName("peer_half_length"), half_length_);
  node->get_parameter(getFullName("peer_half_width"), half_width_);
  node->get_parameter(getFullName("padding"), padding_);
  node->get_parameter(getFullName("timeout"), config.timeout_s);
  node->get_parameter(getFullName("max_extrapolation"), config.max_extrapolation_s);
  if (robot_id_.empty()) {
    robot_id_ = robotIdFromNamespace(node->get_namespace());
  }
  tracker_ = PeerTracker(config);

  // amr_bt가 RELIABLE depth 10으로 낸다
  sub_ = node->create_subscription<amr_msgs::msg::RobotState>(
    topic, rclcpp::QoS(10).reliable(),
    std::bind(&FleetPeerLayer::onState, this, std::placeholders::_1));
  current_ = true;
  RCLCPP_INFO(
    node->get_logger(), "FleetPeerLayer: %s 구독, 나(%s)는 빼고 그린다 (enabled %d)",
    topic.c_str(), robot_id_.c_str(), enabled_);
}

void FleetPeerLayer::onState(amr_msgs::msg::RobotState::ConstSharedPtr msg)
{
  if (msg->robot_id == robot_id_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  peer_frame_ = msg->header.frame_id;
  tracker_.update(
    msg->robot_id, rclcpp::Time(msg->header.stamp).seconds(), msg->pose.position.x,
    msg->pose.position.y, tf2::getYaw(msg->pose.orientation));
}

void FleetPeerLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  polygons_.clear();
  own_footprint_.clear();
  auto node = node_.lock();
  if (!enabled_ || !node) {
    return;
  }

  std::vector<PeerPose> peers;
  std::string peer_frame;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    peers = tracker_.predict(node->now().seconds());
    peer_frame = peer_frame_;
  }

  // 지난 자리를 지우도록 지난 영역을 먼저 넣는다
  double bmin_x = std::numeric_limits<double>::max();
  double bmin_y = std::numeric_limits<double>::max();
  double bmax_x = std::numeric_limits<double>::lowest();
  double bmax_y = std::numeric_limits<double>::lowest();
  if (has_last_bounds_) {
    *min_x = std::min(*min_x, last_min_x_);
    *min_y = std::min(*min_y, last_min_y_);
    *max_x = std::max(*max_x, last_max_x_);
    *max_y = std::max(*max_y, last_max_y_);
    has_last_bounds_ = false;
  }
  if (peers.empty() || peer_frame.empty()) {
    return;
  }

  const std::string costmap_frame = layered_costmap_->getGlobalFrameID();
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_->lookupTransform(costmap_frame, peer_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 10000, "FleetPeerLayer: %s -> %s TF 없음: %s",
      peer_frame.c_str(), costmap_frame.c_str(), e.what());
    return;
  }
  const double tx = tf_msg.transform.translation.x;
  const double ty = tf_msg.transform.translation.y;
  const double tyaw = tf2::getYaw(tf_msg.transform.rotation);
  const double c = std::cos(tyaw);
  const double s = std::sin(tyaw);

  for (const auto & p : peers) {
    const PeerPose in_costmap{tx + c * p.x - s * p.y, ty + s * p.x + c * p.y, p.yaw + tyaw};
    auto polygon = footprintCorners(in_costmap, half_length_, half_width_, padding_);
    for (const auto & pt : polygon) {
      bmin_x = std::min(bmin_x, pt.x);
      bmin_y = std::min(bmin_y, pt.y);
      bmax_x = std::max(bmax_x, pt.x);
      bmax_y = std::max(bmax_y, pt.y);
    }
    polygons_.push_back(std::move(polygon));
  }

  // 내 차체(costmap footprint, 패딩 포함)를 costmap 좌표로
  const double rc = std::cos(robot_yaw);
  const double rs = std::sin(robot_yaw);
  for (const auto & pt : getFootprint()) {
    own_footprint_.push_back(
      Point2{robot_x + rc * pt.x - rs * pt.y, robot_y + rs * pt.x + rc * pt.y});
  }

  *min_x = std::min(*min_x, bmin_x);
  *min_y = std::min(*min_y, bmin_y);
  *max_x = std::max(*max_x, bmax_x);
  *max_y = std::max(*max_y, bmax_y);
  has_last_bounds_ = true;
  last_min_x_ = bmin_x;
  last_min_y_ = bmin_y;
  last_max_x_ = bmax_x;
  last_max_y_ = bmax_y;
}

void FleetPeerLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  std::vector<nav2_costmap_2d::MapLocation> corners;
  std::vector<nav2_costmap_2d::MapLocation> cells;
  for (const auto & polygon : polygons_) {
    corners.clear();
    cells.clear();
    for (const auto & pt : polygon) {
      int mx = 0;
      int my = 0;
      master_grid.worldToMapEnforceBounds(pt.x, pt.y, mx, my);
      corners.push_back({static_cast<unsigned int>(mx), static_cast<unsigned int>(my)});
    }
    master_grid.convexFillCells(corners, cells);
    for (const auto & cell : cells) {
      const int i = static_cast<int>(cell.x);
      const int j = static_cast<int>(cell.y);
      if (i < min_i || i >= max_i || j < min_j || j >= max_j) {
        continue;
      }
      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(cell.x, cell.y, wx, wy);
      if (pointInPolygon(own_footprint_, wx, wy)) {
        continue;
      }
      master_grid.setCost(cell.x, cell.y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

}  // namespace amr_nav2_plugins

PLUGINLIB_EXPORT_CLASS(amr_nav2_plugins::FleetPeerLayer, nav2_costmap_2d::Layer)
