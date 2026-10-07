// 경로 추종 노드: plan(nav_msgs/Path) + odom을 받아 Pure Pursuit 또는 Stanley로 cmd_vel_nav를 낸다.
//
// cmd_vel 체인 (docs/api.md, D6): 이 노드 → cmd_vel_nav → amr_zone_client → cmd_vel_zone_limited
// → amr_safety → cmd_vel. Nav2 controller_server와 같은 토픽을 내므로 둘을 같이 띄우지 않는다.
// 알고리즘은 ROS 비의존 코어(include/amr_control/core)에 있고, 이 노드는 메시지 변환·주기·기록만 한다.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "amr_control/core/cte_log.hpp"
#include "amr_control/core/path_tracker.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace amr_control
{

namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class PathTrackerNode : public rclcpp::Node
{
public:
  PathTrackerNode()
  : rclcpp::Node("path_tracker")
  {
    PathTrackerConfig config;
    const auto controller = declare_parameter<std::string>("controller", "pure_pursuit");
    if (!parseControllerType(controller, config.controller)) {
      throw std::invalid_argument("controller는 pure_pursuit 또는 stanley: " + controller);
    }
    config.desired_speed = declare_parameter<double>("desired_speed", config.desired_speed);
    config.max_linear_speed = declare_parameter<double>("max_linear_speed", config.max_linear_speed);
    config.max_angular_speed =
      declare_parameter<double>("max_angular_speed", config.max_angular_speed);
    config.max_linear_accel = declare_parameter<double>("max_linear_accel", config.max_linear_accel);
    config.max_linear_decel = declare_parameter<double>("max_linear_decel", config.max_linear_decel);
    config.min_approach_speed =
      declare_parameter<double>("min_approach_speed", config.min_approach_speed);
    config.goal_tolerance = declare_parameter<double>("goal_tolerance", config.goal_tolerance);
    config.rotate_in_place_angle =
      declare_parameter<double>("rotate_in_place_angle", config.rotate_in_place_angle);
    config.rotate_in_place_speed =
      declare_parameter<double>("rotate_in_place_speed", config.rotate_in_place_speed);
    config.search_window = declare_parameter<double>("search_window", config.search_window);
    config.curvature_half_span =
      declare_parameter<double>("curvature_half_span", config.curvature_half_span);

    auto & pp = config.pure_pursuit;
    pp.adaptive_lookahead =
      declare_parameter<bool>("pure_pursuit.adaptive_lookahead", pp.adaptive_lookahead);
    pp.lookahead_distance =
      declare_parameter<double>("pure_pursuit.lookahead_distance", pp.lookahead_distance);
    pp.lookahead_time = declare_parameter<double>("pure_pursuit.lookahead_time", pp.lookahead_time);
    pp.min_lookahead = declare_parameter<double>("pure_pursuit.min_lookahead", pp.min_lookahead);
    pp.max_lookahead = declare_parameter<double>("pure_pursuit.max_lookahead", pp.max_lookahead);

    auto & st = config.stanley;
    st.cte_gain = declare_parameter<double>("stanley.cte_gain", st.cte_gain);
    st.softening_speed = declare_parameter<double>("stanley.softening_speed", st.softening_speed);
    st.front_offset = declare_parameter<double>("stanley.front_offset", st.front_offset);
    st.max_steer = declare_parameter<double>("stanley.max_steer", st.max_steer);
    if (st.front_offset <= 0.0) {
      throw std::invalid_argument("stanley.front_offset는 0보다 커야 한다");
    }
    tracker_ = PathTracker(config);

    const double control_rate_hz = declare_parameter<double>("control_rate_hz", 20.0);
    odom_timeout_s_ = declare_parameter<double>("odom_timeout_s", 0.5);
    base_frame_ = declare_parameter<std::string>("base_frame", "");

    // cte.csv: 비우면 기록하지 않는다. 하네스가 {run_dir}/cte.csv를 넘긴다
    const auto cte_csv_path = declare_parameter<std::string>("cte_csv_path", "");
    curve_threshold_ = declare_parameter<double>("curve_curvature_threshold", 0.1);
    // 비우면 네임스페이스(/amr_01 → amr_01)를 쓴다
    robot_id_ = declare_parameter<std::string>("robot_id", "");
    if (robot_id_.empty()) {
      const std::string ns = get_namespace();
      robot_id_ = ns.substr(ns.find_last_of('/') + 1);
    }
    if (!cte_csv_path.empty()) {
      cte_csv_.open(cte_csv_path, std::ios::out | std::ios::trunc);
      if (!cte_csv_) {
        throw std::runtime_error("cte.csv를 열 수 없다: " + cte_csv_path);
      }
      writeCteCsvHeader(cte_csv_);
      cte_csv_.flush();
    }
    // 강제 종료돼도 남도록 약 1초마다 flush한다
    cte_flush_every_ = std::max(1, static_cast<int>(std::lround(control_rate_hz)));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // 이 노드에 구독을 붙이고 별도 스레드를 띄우지 않는다 (amr_bt_node와 같은 방식)
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", 10);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "plan", 10, [this](const nav_msgs::msg::Path & msg) {onPath(msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](const nav_msgs::msg::Odometry & msg) {odom_ = msg;});

    // 노드 시계 기준 (use_sim_time이면 시뮬레이션 시간)
    control_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / control_rate_hz),
      [this]() {onControl();});

    RCLCPP_INFO(
      get_logger(), "path_tracker 시작: %s, %.1f Hz, cte.csv %s", controller.c_str(),
      control_rate_hz, cte_csv_path.empty() ? "기록 안 함" : cte_csv_path.c_str());
  }

private:
  void onPath(const nav_msgs::msg::Path & msg)
  {
    Path path;
    path.reserve(msg.poses.size());
    for (const auto & p : msg.poses) {
      path.push_back(PathPoint{p.pose.position.x, p.pose.position.y});
    }
    path_frame_ = msg.header.frame_id;
    tracker_.setPath(path);
  }

  // 경로 프레임에서 로봇 자세. 경로와 odom 프레임이 같으면 odom을 그대로, 다르면 TF로 구한다
  std::optional<Pose2D> robotPose()
  {
    const auto & pose = odom_->pose.pose;
    if (path_frame_.empty() || path_frame_ == odom_->header.frame_id) {
      return Pose2D{pose.position.x, pose.position.y, yawFromQuaternion(pose.orientation)};
    }
    const std::string base = base_frame_.empty() ? odom_->child_frame_id : base_frame_;
    try {
      const auto t = tf_buffer_->lookupTransform(path_frame_, base, tf2::TimePointZero);
      return Pose2D{
        t.transform.translation.x, t.transform.translation.y,
        yawFromQuaternion(t.transform.rotation)};
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "%s → %s TF 없음: %s", path_frame_.c_str(), base.c_str(),
        e.what());
      return std::nullopt;
    }
  }

  void onControl()
  {
    const rclcpp::Time now = this->now();
    const double dt = last_control_ ? (now - *last_control_).seconds() : 0.0;
    last_control_ = now;

    const bool odom_fresh = odom_ &&
      (now - rclcpp::Time(odom_->header.stamp, now.get_clock_type())).seconds() <= odom_timeout_s_;
    if (!tracker_.hasPath() || !odom_fresh) {
      if (tracker_.hasPath()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "odom이 없거나 오래돼 정지");
      }
      stop();
      return;
    }
    const auto pose = robotPose();
    if (!pose) {
      stop();
      return;
    }

    const auto result = tracker_.compute(*pose, odom_->twist.twist.linear.x, dt);
    if (result.status == TrackingStatus::kGoalReached) {
      if (moving_) {
        RCLCPP_INFO(get_logger(), "경로 끝 도착");
      }
      stop();
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = result.v;
    cmd.angular.z = result.w;
    cmd_pub_->publish(cmd);
    moving_ = true;

    if (cte_csv_.is_open()) {
      CteSample sample;
      sample.timestamp = rclcpp::Time(odom_->header.stamp).seconds();
      sample.planned_x = result.projection.x;
      sample.planned_y = result.projection.y;
      sample.actual_x = pose->x;
      sample.actual_y = pose->y;
      sample.cte = result.projection.cte;
      sample.path_curvature = result.path_curvature;
      writeCteCsvRow(cte_csv_, sample, curve_threshold_, robot_id_);
      if (++cte_rows_since_flush_ >= cte_flush_every_) {
        cte_csv_.flush();
        cte_rows_since_flush_ = 0;
      }
    }
  }

  // 정지 명령은 멈출 때 한 번만 낸다. 경로가 없는 동안 cmd_vel_nav를 계속 차지하지 않는다
  void stop()
  {
    if (!moving_) {
      return;
    }
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    moving_ = false;
    tracker_.resetSpeed();
    if (cte_csv_.is_open()) {
      cte_csv_.flush();
    }
  }

  PathTracker tracker_;
  std::string path_frame_;
  std::string base_frame_;
  std::string robot_id_;
  double odom_timeout_s_ = 0.5;
  double curve_threshold_ = 0.1;
  bool moving_ = false;
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::optional<rclcpp::Time> last_control_;
  std::ofstream cte_csv_;
  int cte_flush_every_ = 20;
  int cte_rows_since_flush_ = 0;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace amr_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_control::PathTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
