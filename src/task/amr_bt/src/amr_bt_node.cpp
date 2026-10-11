// 로봇 1대의 작업 수행 노드: Fleet 서버와 메시지를 주고받고 BT(task_cycle.xml)를 돌린다.
//
// 서버 → 로봇: <prefix>/task_assignments, <prefix>/heartbeat
//   prefix = server_topic_prefix. 기본 /fleet (서버 원본), 장애 주입기를 거치면 fleet (→ /<ns>/fleet/...)
// 로봇 → 서버: /fleet/task_assignment_acks, /fleet/task_status, /fleet/robot_states
// 로봇 안: navigate_to_pose (Nav2, use_nav2), docking/dock (Dock.action, use_docking) 액션 클라이언트.
//   zone_client/state (#137)·kidnap_recovery/state (#79)를 구독해 MoveTo가 멈추고 다시 보낸다
//
// 수신 규칙은 TaskAssignment.msg 주석과 amr_fleet_manager 가짜 로봇(robot_sim.AssignmentFilter)을 따른다.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "amr_bt/assignment_filter.hpp"
#include "amr_bt/battery.hpp"
#include "amr_bt/bt_nodes.hpp"
#include "amr_bt/task_context.hpp"
#include "amr_msgs/action/dock.hpp"
#include "amr_msgs/msg/error_info.hpp"
#include "amr_msgs/msg/fleet_heartbeat.hpp"
#include "amr_msgs/msg/robot_state.hpp"
#include "amr_msgs/msg/task_assignment.hpp"
#include "amr_msgs/msg/task_assignment_ack.hpp"
#include "amr_msgs/msg/task_status.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
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
using DockAction = amr_msgs::action::Dock;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

// 액션 클라이언트를 TaskContext의 ActionPort(start·poll·cancel)로 감싼다.
// - start: 이전 목표를 취소하고 새 목표를 보낸다. 서버가 아직 없으면 server_wait_s까지 poll마다 다시 본다
// - 수락·결과 콜백은 목표 번호(gen)가 지금 것일 때만 반영한다. 버린 옛 목표가 늦게 수락되면 바로 취소한다
template<typename ActionT>
class ActionClientPort
{
public:
  using Goal = typename ActionT::Goal;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using ToStatus = std::function<ActionStatus(const typename GoalHandle::WrappedResult &)>;

  ActionClientPort(
    rclcpp::Node * node, const std::string & name, double server_wait_s, ToStatus to_status)
  : node_(node), name_(name), server_wait_s_(server_wait_s), to_status_(std::move(to_status)),
    client_(rclcpp_action::create_client<ActionT>(node, name)) {}

  void start(const Goal & goal)
  {
    cancel();
    pending_ = goal;
    wait_since_ = node_->now().seconds();
    try_send();
  }

  ActionStatus poll()
  {
    if (pending_) {
      try_send();
    }
    return status_;
  }

  void cancel()
  {
    ++gen_;
    pending_.reset();
    if (handle_) {
      client_->async_cancel_goal(handle_);
      handle_.reset();
    }
    status_ = ActionStatus();
  }

private:
  void try_send()
  {
    if (!client_->action_server_is_ready()) {
      if (node_->now().seconds() - wait_since_ > server_wait_s_) {
        pending_.reset();
        status_.state = ActionStatus::kFailed;
        status_.message = name_ + " server not available";
      }
      return;
    }
    typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
    const uint64_t gen = gen_;
    options.goal_response_callback = [this, gen](typename GoalHandle::SharedPtr handle) {
        if (gen != gen_) {
          if (handle) {
            client_->async_cancel_goal(handle);
          }
          return;
        }
        if (!handle) {
          status_.state = ActionStatus::kFailed;
          status_.message = name_ + " goal rejected";
          return;
        }
        handle_ = handle;
      };
    options.result_callback = [this, gen](const typename GoalHandle::WrappedResult & result) {
        if (gen != gen_) {
          return;
        }
        handle_.reset();
        status_ = to_status_(result);
      };
    client_->async_send_goal(*pending_, options);
    pending_.reset();
  }

  rclcpp::Node * node_;
  std::string name_;
  double server_wait_s_;
  ToStatus to_status_;
  typename rclcpp_action::Client<ActionT>::SharedPtr client_;
  typename GoalHandle::SharedPtr handle_;
  std::optional<Goal> pending_;
  double wait_since_ = 0.0;
  uint64_t gen_ = 0;
  ActionStatus status_;
};

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
    // 이동 거리(distance_traveled·배터리)는 odom 기준으로 잰다. map 기준이면 AMCL 보정·kidnap 순간 이동만큼
    // 부풀어서다 (#141 하연 리뷰). odom → base_footprint는 연속이다 (EKF 로컬)
    odom_frame_ = declare_parameter<std::string>("odom_frame", "");
    if (odom_frame_.empty()) {
      odom_frame_ = robot_id_ + "/odom";
    }
    BatteryConfig battery;
    battery.initial_percent = declare_parameter<double>("battery.initial_percent", 100.0);
    battery.move_per_m = declare_parameter<double>("battery.move_per_m", 0.05);
    battery.idle_per_s = declare_parameter<double>("battery.idle_per_s", 0.002);
    battery.charge_per_s = declare_parameter<double>("battery.charge_per_s", 0.5);
    battery_ = Battery(battery);
    // 이보다 낮으면 충전이 아닌 작업을 ASSIGN_BATTERY_LOW로 거부한다 (서버 min_battery_percent 20 %보다 낮게)
    reject_below_percent_ = declare_parameter<double>("battery.reject_below_percent", 10.0);
    task_log_file_ = declare_parameter<std::string>("task_log_file", "");

    ctx_ = std::make_shared<TaskContext>();
    ctx_->now = [this]() {return now().seconds();};
    ctx_->report = [this](uint8_t state, const std::string & phase, const ErrorInfo & error,
        bool end) {publish_status(state, phase, error, end);};
    ctx_->dwell_s = declare_parameter<double>("dwell_s", 2.0);
    ctx_->nav_timeout_s = declare_parameter<double>("nav_timeout_s", 300.0);
    ctx_->nav_retries = static_cast<int>(declare_parameter<int64_t>("nav_retries", 2));
    ctx_->pause_timeout_s = declare_parameter<double>("pause_timeout_s", 60.0);
    ctx_->resume_hold_s = declare_parameter<double>("resume_hold_s", 0.3);
    ctx_->charge_target = declare_parameter<double>("battery.charge_target_percent", 100.0);
    ctx_->charge_timeout_s = declare_parameter<double>("battery.charge_timeout_s", 600.0);
    ctx_->battery_percent = battery_.percent();
    setup_actions();

    BT::BehaviorTreeFactory factory;
    register_nodes(factory, ctx_);
    tree_ = factory.createTreeFromFile(tree_file);
    // Groot 실시간 모니터링 (ZMQ). 로봇마다 다른 포트를 준다 (publisher = groot_port, server = groot_port + 1)
    const int groot_port = static_cast<int>(declare_parameter<int64_t>("groot_port", 0));
    if (groot_port > 0) {
      groot_ = std::make_unique<BT::PublisherZMQ>(tree_, 25, groot_port, groot_port + 1);
      RCLCPP_INFO(get_logger(), "Groot 모니터링: 포트 %d/%d", groot_port, groot_port + 1);
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // 이 노드에 구독을 붙이고 별도 스레드를 띄우지 않는다 (기본값은 내부 노드 + 스레드를 하나 더 만든다)
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);

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
    // 로봇 안 상태 토픽 (네임스페이스 안 상대 이름). 내는 노드가 없으면 기본값(진행)을 그대로 둔다
    zone_state_sub_ = create_subscription<std_msgs::msg::String>(
      "zone_client/state", 10,
      [this](const std_msgs::msg::String & msg) {
        on_state("zone_client", ctx_->zone_state, msg.data);
      });
    kidnap_state_sub_ = create_subscription<std_msgs::msg::String>(
      "kidnap_recovery/state", 10,
      [this](const std_msgs::msg::String & msg) {
        on_state("kidnap_recovery", ctx_->kidnap_state, msg.data);
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
  // use_nav2·use_docking이면 액션 클라이언트, 아니면 시간만 흘리는 자리채움 (1단계 하네스는 Nav2 없이 돈다)
  void setup_actions()
  {
    const bool use_nav2 = declare_parameter<bool>("use_nav2", false);
    const bool use_docking = declare_parameter<bool>("use_docking", false);
    const auto nav_action = declare_parameter<std::string>("nav_action", "navigate_to_pose");
    const auto dock_action = declare_parameter<std::string>("dock_action", "docking/dock");
    const double server_wait_s = declare_parameter<double>("server_wait_s", 10.0);
    const double stub_move_s = declare_parameter<double>("stub_move_s", 5.0);
    const double stub_dock_s = declare_parameter<double>("stub_dock_s", 1.0);

    if (use_nav2) {
      nav_client_ = std::make_unique<ActionClientPort<NavigateToPose>>(
        this, nav_action, server_wait_s,
        [](const auto & result) {
          ActionStatus s;
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            s.state = ActionStatus::kSucceeded;
            return s;
          }
          // Humble NavigateToPose 결과에는 오류 코드가 없다. 경로를 못 찾았거나 제어·복구가 다 실패한 경우다
          s.state = ActionStatus::kFailed;
          s.code = ErrorInfo::NAV_NO_PATH;
          s.message = result.code == rclcpp_action::ResultCode::ABORTED ?
            "navigate_to_pose aborted" : "navigate_to_pose canceled";
          return s;
        });
      ctx_->nav.start = [this](const geometry_msgs::msg::Pose & pose) {
          NavigateToPose::Goal goal;
          goal.pose.header.frame_id = global_frame_;
          goal.pose.header.stamp = now();
          goal.pose.pose = pose;
          nav_client_->start(goal);
        };
      ctx_->nav.poll = [this]() {return nav_client_->poll();};
      ctx_->nav.cancel = [this]() {nav_client_->cancel();};
    } else {
      ctx_->nav = make_stub_port<geometry_msgs::msg::Pose>(ctx_->now, stub_move_s);
    }

    if (use_docking) {
      dock_client_ = std::make_unique<ActionClientPort<DockAction>>(
        this, dock_action, server_wait_s,
        [](const auto & result) {
          ActionStatus s;
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
            result.result->success)
          {
            s.state = ActionStatus::kSucceeded;
            return s;
          }
          s.state = ActionStatus::kFailed;
          if (result.result) {
            s.code = result.result->error_code;
            s.message = result.result->message;
          }
          if (s.code == ErrorInfo::NONE) {
            s.code = DockAction::Result::UNKNOWN;
          }
          return s;
        });
      ctx_->dock.start = [this](const std::string & station_id) {
          DockAction::Goal goal;
          goal.station_id = station_id;
          dock_client_->start(goal);
        };
      ctx_->dock.poll = [this]() {return dock_client_->poll();};
      ctx_->dock.cancel = [this]() {dock_client_->cancel();};
    } else {
      ctx_->dock = make_stub_port<std::string>(ctx_->now, stub_dock_s);
    }
    RCLCPP_INFO(
      get_logger(), "이동 %s, 도킹 %s", use_nav2 ? nav_action.c_str() : "자리채움",
      use_docking ? dock_action.c_str() : "자리채움");
  }

  bool zone_failed() const {return ctx_->zone_state == "failed";}

  void on_state(const char * source, std::string & slot, const std::string & value)
  {
    if (slot != value) {
      RCLCPP_INFO(get_logger(), "%s 상태 %s → %s", source, slot.c_str(), value.c_str());
      slot = value;
    }
  }

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
      // Ack보다 먼저 정리를 끝낸다 (FAILED 발행 → 작업 비움). 그래야 바로 뒤 새 할당을 ALREADY_BUSY로 거부하지 않는다.
      // 하던 작업이 아니면 이미 끝난 것이므로 그대로 수락한다. 가짜 로봇(fake_robot_node)과 같은 순서다.
      // FAILED에 ASSIGN_CANCELED를 붙여 서버가 다시 할당하지 않게 한다 (#107, 가짜 로봇과 같음)
      if (cancel_task(tree_, *ctx_, msg.task.task_id, canceled_error())) {
        RCLCPP_INFO(get_logger(), "작업 취소: %s", msg.task.task_id.c_str());
      }
      return reply;
    }
    if (battery_.percent() < reject_below_percent_ && msg.task.task_type != amr_msgs::msg::Task::TYPE_CHARGE) {
      reply.result = TaskAssignmentAck::RESULT_REJECTED;
      reply.error_code = ErrorInfo::ASSIGN_BATTERY_LOW;
      reply.error_message = "battery " + std::to_string(static_cast<int>(battery_.percent())) + " %";
      return reply;
    }
    if (zone_failed()) {
      // zone_client가 후진 안전 한도로 멈춰 선 상태(#137 failed): 사람이 확인하기 전엔 움직일 수 없다.
      // 받으면 MoveTo가 바로 FAILED를 내고 서버가 또 할당하는 것을 되풀이하므로 거부한다
      reply.result = TaskAssignmentAck::RESULT_REJECTED;
      reply.error_code = ErrorInfo::TRAFFIC_REVOKED;
      reply.error_message = "zone_client failed, manual check needed";
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
    distance_ = 0.0;
    last_xy_.reset();
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
    msg.distance_traveled = static_cast<float>(distance_);
    status_pub_->publish(msg);
    if (end) {
      write_task_log(msg);
    }
    if (state == TaskStatus::STATE_FAILED) {
      RCLCPP_WARN(
        get_logger(), "작업 %s FAILED (%s, code=%u): %s", msg.task_id.c_str(), phase.c_str(),
        error.code, error.message.c_str());
    }
  }

  // 작업 로그 (명세: 시작·종료 시각, 이동 거리, 소요 시간, 결과를 분석 가능한 형태로). 파일을 주면 끝날 때 한 줄씩
  void write_task_log(const TaskStatus & msg)
  {
    if (task_log_file_.empty() || !ctx_->task) {
      return;
    }
    const bool fresh = !std::ifstream(task_log_file_).good();
    std::ofstream out(task_log_file_, std::ios::app);
    if (!out) {
      RCLCPP_WARN_ONCE(get_logger(), "작업 로그 파일을 열 수 없다: %s", task_log_file_.c_str());
      return;
    }
    if (fresh) {
      out << "robot_id,task_id,task_type,item_type,state,phase,error_code,error_message,"
        "start_s,end_s,duration_s,distance_m,battery_percent\n";
    }
    const double start = rclcpp::Time(msg.start_time).seconds();
    const double end = rclcpp::Time(msg.end_time).seconds();
    std::string message = msg.error.message;
    std::replace(message.begin(), message.end(), ',', ';');
    out << robot_id_ << ',' << msg.task_id << ',' << static_cast<int>(ctx_->task->task_type) << ','
        << static_cast<int>(ctx_->task->item_type) << ','
        << (msg.state == TaskStatus::STATE_COMPLETED ? "COMPLETED" : "FAILED") << ',' << msg.phase << ','
        << msg.error.code << ',' << message << ',' << std::fixed << start << ',' << end << ','
        << (end - start) << ',' << msg.distance_traveled << ',' << battery_.percent() << '\n';
  }

  void publish_state()
  {
    RobotState msg;
    msg.header.stamp = now();
    double step = 0.0;   // 지난 주기 뒤 이동 거리 (배터리 모델)
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
    // TaskStatus.distance_traveled·배터리: 이 주기(state_rate_hz)마다 odom 위 이동 거리를 더한다
    try {
      const auto t = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
      const double x = t.transform.translation.x;
      const double y = t.transform.translation.y;
      if (last_xy_) {
        step = std::hypot(x - last_xy_->first, y - last_xy_->second);
        if (ctx_->busy()) {
          distance_ += step;
        }
      }
      last_xy_ = std::make_pair(x, y);
    } catch (const tf2::TransformException &) {
      // odom이 아직 없으면 이번 주기는 거리 0 (경고는 위 pose 쪽에서 낸다)
    }
    const double t = now().seconds();
    if (last_state_t_ > 0.0) {
      battery_.update(t - last_state_t_, step, ctx_->charging);
    }
    last_state_t_ = t;
    ctx_->battery_percent = battery_.percent();
    if (ctx_->charging) {
      msg.mode = RobotState::MODE_CHARGING;
      msg.current_task_id = ctx_->task ? ctx_->task->task_id : "";
    } else if (ctx_->busy()) {
      msg.mode = RobotState::MODE_BUSY;
      msg.current_task_id = ctx_->task->task_id;
      msg.item_type = ctx_->task->item_type;
    } else if (zone_failed()) {
      msg.mode = RobotState::MODE_ERROR;   // 서버는 MODE_IDLE에만 할당한다
    } else {
      msg.mode = RobotState::MODE_IDLE;
    }
    msg.battery_percent = static_cast<float>(battery_.percent());
    msg.last_assignment_id = filter_.last_id();
    state_pub_->publish(msg);
  }

  std::string robot_id_;
  std::string global_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  Battery battery_;
  double reject_below_percent_ = 10.0;
  double last_state_t_ = 0.0;
  std::string task_log_file_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  double distance_ = 0.0;
  std::optional<std::pair<double, double>> last_xy_;

  AssignmentFilter filter_;
  std::shared_ptr<TaskContext> ctx_;
  std::unique_ptr<ActionClientPort<NavigateToPose>> nav_client_;
  std::unique_ptr<ActionClientPort<DockAction>> dock_client_;
  BT::Tree tree_;
  std::unique_ptr<BT::PublisherZMQ> groot_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<TaskAssignmentAck>::SharedPtr ack_pub_;
  rclcpp::Publisher<TaskStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<RobotState>::SharedPtr state_pub_;
  rclcpp::Subscription<TaskAssignment>::SharedPtr assignment_sub_;
  rclcpp::Subscription<FleetHeartbeat>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr zone_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr kidnap_state_sub_;
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
