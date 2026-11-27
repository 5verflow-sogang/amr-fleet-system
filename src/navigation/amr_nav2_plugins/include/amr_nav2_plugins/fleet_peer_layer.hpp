#ifndef AMR_NAV2_PLUGINS__FLEET_PEER_LAYER_HPP_
#define AMR_NAV2_PLUGINS__FLEET_PEER_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "amr_msgs/msg/robot_state.hpp"
#include "amr_nav2_plugins/fleet_peers.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_nav2_plugins
{

// 다른 로봇을 costmap에 그리는 레이어. LiDAR 스캔면(지면 0.38 m)이 다른 로봇 차체(0.03~0.33 m) 위를
// 지나가 로봇끼리 서로를 못 보고 부딪혀 밀고 있었다(11/13 fleet_stack resp50, 중심 거리 0.55 m로 200 s).
// /fleet/robot_states(2 Hz)의 위치를 지금 시각으로 외삽해 차체 사각형을 LETHAL로 그린다. 매 주기 새로 그리고
// 지난 자리는 bounds에 넣어 LayeredCostmap이 지우게 한다 - obstacle_layer처럼 표시가 남지 않는다.
// 내 차체 안쪽 셀은 비워 둔다: 이미 붙어 있을 때 내 자리까지 LETHAL이면 DWA가 멀어지는 궤적도 못 고른다.
class FleetPeerLayer : public nav2_costmap_2d::Layer
{
public:
  FleetPeerLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void reset() override {}
  bool isClearable() override {return false;}

private:
  void onState(amr_msgs::msg::RobotState::ConstSharedPtr msg);

  std::mutex mutex_;
  PeerTracker tracker_;
  std::string robot_id_;
  std::string peer_frame_;
  double half_length_{0.3};
  double half_width_{0.2};
  double padding_{0.1};
  rclcpp::Subscription<amr_msgs::msg::RobotState>::SharedPtr sub_;

  // updateBounds가 계산해 updateCosts가 쓴다 (costmap 좌표계)
  std::vector<std::vector<Point2>> polygons_;
  std::vector<Point2> own_footprint_;
  // 지난 주기에 그린 영역. 이번 bounds에 넣어 지난 자리를 지운다
  bool has_last_bounds_{false};
  double last_min_x_{0.0};
  double last_min_y_{0.0};
  double last_max_x_{0.0};
  double last_max_y_{0.0};
};

}  // namespace amr_nav2_plugins

#endif  // AMR_NAV2_PLUGINS__FLEET_PEER_LAYER_HPP_
