// 로봇 1대의 작업 수행 노드: Fleet 서버와 메시지를 주고받고 BT(task_cycle.xml)를 돌린다.
//
// 서버 → 로봇: <prefix>/task_assignments, <prefix>/heartbeat
//   prefix = server_topic_prefix. 기본 /fleet (서버 원본), 장애 주입기를 거치면 fleet (→ /<ns>/fleet/...)
// 로봇 → 서버: /fleet/task_assignment_acks, /fleet/task_status, /fleet/robot_states
//
// 수신 규칙은 TaskAssignment.msg 주석과 amr_fleet_manager 가짜 로봇(robot_sim.AssignmentFilter)을 따른다.

#include <memory>
#include <stdexcept>
#include <string>

#include "amr_bt/assignment_filter.hpp"
#include "amr_bt/bt_nodes.hpp"
#include "amr_bt/task_context.hpp"
#include "amr_msgs/msg/error_info.hpp"
#include "amr_msgs/msg/fleet_heartbeat.hpp"
#include "amr_msgs/msg/robot_state.hpp"
#include "amr_msgs/msg/task_assignment.hpp"
#include "amr_msgs/msg/task_assignment_ack.hpp"
#include "amr_msgs/msg/task_status.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace amr_bt
{

using amr_msgs::msg::ErrorInfo;
using amr_msgs::msg::FleetHeartbeat;
using amr_msgs::msg::RobotState;
using amr_msgs::msg::TaskAssignment;
using amr_msgs::msg::TaskAssignmentAck;
using amr_msgs::msg::TaskStatus;

class AmrBtNode : public rclcpp::Node
{
public:
  AmrBtNode()
  : rclcpp::Node("amr_bt")
  {
    // 비우면 네임스페이스(/amr_01 → amr_01)를 쓴다. config yaml의 ""가 기본값을 덮어쓰므로 빈 값을 따로 처리한다
    robot_id_ = declare_parameter<std::string>("robot_id", "");
    if (robot_id_.empty()) {
      const std::string ns = get_namespace();
      robot_id_ = ns.substr(ns.find_last_of('/') + 1);
    }
    if (robot_id_.empty()) {
      throw std::invalid_argument("robot_id가 비었다: 네임스페이스(예: amr_01)로 띄우거나 robot_id를 준다");
    }
    const auto prefix = declare_parameter<std::string>("server_topic_prefix", "/fleet");
    const auto tree_file = declare_parameter<std::string>(
      "tree_file",
      ament_index_cpp::get_package_share_directory("amr_bt") + "/behavior_trees/task_cycle.xml");
    const double tick_hz = declare_parameter<double>("tick_hz", 10.0);
    const double state_rate_hz = declare_parameter<double>("state_rate_hz", 2.0);
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "");
    if (base_frame_.empty()) {
      base_frame_ = robot_id_ + "/base_footprint";
    }
    // TODO(작업 9): 가상 배터리 모델 (이동 거리·시간에 따른 소모, 충전)
    battery_percent_ = declare_parameter<double>("battery_percent", 100.0);

    ctx_ = std::make_shared<TaskContext>();
    ctx_->now = [this]() {return now().seconds();};
    ctx_->report = [this](uint8_t state, const std::string & phase, const ErrorInfo & error,
        bool end) {publish_status(state, phase, error, end);};
    ctx_->stub_move_s = declare_parameter<double>("stub_move_s", 5.0);
    ctx_->stub_dwell_s = declare_parameter<double>("stub_dwell_s", 2.0);

    BT::BehaviorTreeFactory factory;
    register_nodes(factory, ctx_);
    tree_ = factory.createTreeFromFile(tree_file);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // TODO(하연, api.md 6절): QoS 기본값이 정해지면 맞춘다. 지금은 Fleet 서버와 같은 RELIABLE depth 10
    const auto qos = rclcpp::QoS(10).reliable();
    ack_pub_ = create_publisher<TaskAssignmentAck>("/fleet/task_assignment_acks", qos);
    status_pub_ = create_publisher<TaskStatus>("/fleet/task_status", qos);
    state_pub_ = create_publisher<RobotState>("/fleet/robot_states", qos);
    assignment_sub_ = create_subscription<TaskAssignment>(
      prefix + "/task_assignments", qos,
      [this](const TaskAssignment & msg) {on_assignment(msg);});
    heartbeat_sub_ = create_subscription<FleetHeartbeat>(
      prefix + "/heartbeat", qos,
      [this](const FleetHeartbeat & msg) {
        if (filter_.on_heartbeat(msg.epoch)) {
          RCLCPP_INFO(get_logger(), "서버 재시작 감지 (epoch=%u), 명령 번호 기준 초기화", msg.epoch);
        }
      });

    // 노드 시계 기준 (use_sim_time이면 시뮬레이션 시간)
    tick_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / tick_hz),
      [this]() {tree_.tickRoot();});
    state_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / state_rate_hz),
      [this]() {publish_state();});

    RCLCPP_INFO(
      get_logger(), "amr_bt %s 시작: 구독 %s/task_assignments, 트리 %s", robot_id_.c_str(),
      prefix.c_str(), tree_file.c_str());
  }

  ~AmrBtNode() override {tree_.haltTree();}

private:
  void on_assignment(const TaskAssignment & msg)
  {
    if (msg.robot_id != robot_id_) {
      return;
    }
    AckReply reply;
    switch (filter_.classify(msg.assignment_id)) {
      case AssignmentKind::kStale:
        return;
      case AssignmentKind::kDuplicate:
        reply = *filter_.last_reply();
        break;
      case AssignmentKind::kNew:
        reply = handle(msg);
        filter_.record(msg.assignment_id, reply);
        break;
    }
    TaskAssignmentAck ack;
    ack.header.stamp = now();
    ack.robot_id = robot_id_;
    ack.assignment_id = msg.assignment_id;
    ack.assignment_stamp = msg.header.stamp;
    ack.result = reply.result;
    ack.error.code = reply.error_code;
    ack.error.message = reply.error_message;
    ack_pub_->publish(ack);
  }

  AckReply handle(const TaskAssignment & msg)
  {
    AckReply reply;
    reply.result = TaskAssignmentAck::RESULT_ACCEPTED;
    if (msg.action == TaskAssignment::ACTION_CANCEL) {
      // 하던 작업이 아니면 이미 끝난 것이므로 그대로 수락한다
      if (ctx_->busy() && ctx_->task->task_id == msg.task.task_id) {
        ctx_->request_cancel("canceled");
        RCLCPP_INFO(get_logger(), "작업 취소 요청: %s", msg.task.task_id.c_str());
      }
      return reply;
    }
    if (ctx_->busy()) {
      reply.result = TaskAssignmentAck::RESULT_REJECTED;
      reply.error_code = ErrorInfo::ASSIGN_ALREADY_BUSY;
      reply.error_message = "busy with " + ctx_->task->task_id;
      return reply;
    }
    ctx_->assign(msg.task);
    start_time_ = now();
    publish_status(TaskStatus::STATE_PENDING, "", ErrorInfo(), false);
    RCLCPP_INFO(
      get_logger(), "작업 수락: %s (assignment_id=%u)", msg.task.task_id.c_str(),
      msg.assignment_id);
    return reply;
  }

  void publish_status(uint8_t state, const std::string & phase, const ErrorInfo & error, bool end)
  {
    if (!ctx_->task) {
      return;
    }
    TaskStatus msg;
    msg.header.stamp = now();
    msg.task_id = ctx_->task->task_id;
    msg.robot_id = robot_id_;
    msg.state = state;
    msg.phase = phase;
    msg.error = error;
    msg.start_time = start_time_;
    if (end) {
      msg.end_time = msg.header.stamp;
    }
    // TODO(작업 9): odom으로 누적 이동 거리. 지금은 이동이 자리채움이라 0
    msg.distance_traveled = 0.0F;
    status_pub_->publish(msg);
  }

  void publish_state()
  {
    RobotState msg;
    msg.header.stamp = now();
    msg.header.frame_id = global_frame_;
    msg.robot_id = robot_id_;
    try {
      const auto t = tf_buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
      msg.pose.position.x = t.transform.translation.x;
      msg.pose.position.y = t.transform.translation.y;
      msg.pose.position.z = t.transform.translation.z;
      msg.pose.orientation = t.transform.rotation;
    } catch (const tf2::TransformException & e) {
      // 위치 추정(map → odom)이 아직 없으면 원점을 보낸다
      msg.pose.orientation.w = 1.0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000, "%s → %s TF 없음, pose를 원점으로 보낸다: %s",
        global_frame_.c_str(), base_frame_.c_str(), e.what());
    }
    if (ctx_->busy()) {
      msg.mode = RobotState::MODE_BUSY;
      msg.current_task_id = ctx_->task->task_id;
      msg.item_type = ctx_->task->item_type;
    } else {
      msg.mode = RobotState::MODE_IDLE;
    }
    msg.battery_percent = static_cast<float>(battery_percent_);
    msg.last_assignment_id = filter_.last_id();
    state_pub_->publish(msg);
  }

  std::string robot_id_;
  std::string global_frame_;
  std::string base_frame_;
  double battery_percent_ = 100.0;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};

  AssignmentFilter filter_;
  std::shared_ptr<TaskContext> ctx_;
  BT::Tree tree_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<TaskAssignmentAck>::SharedPtr ack_pub_;
  rclcpp::Publisher<TaskStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<RobotState>::SharedPtr state_pub_;
  rclcpp::Subscription<TaskAssignment>::SharedPtr assignment_sub_;
  rclcpp::Subscription<FleetHeartbeat>::SharedPtr heartbeat_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

}  // namespace amr_bt

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_bt::AmrBtNode>());
  rclcpp::shutdown();
  return 0;
}
