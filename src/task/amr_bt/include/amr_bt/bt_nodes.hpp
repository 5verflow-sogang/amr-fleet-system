// amr_bt의 BehaviorTree.CPP(v3) 노드. 트리는 behavior_trees/task_cycle.xml.
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "amr_bt/task_context.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace amr_bt
{

// 수락한 작업이 생길 때까지 RUNNING. 생기면 SUCCESS.
// 작업이 끝나면(FinishTask·FailTask·cancel_task) 항상 비워지므로 같은 작업을 두 번 집지 않는다
class WaitForTask : public BT::StatefulActionNode
{
public:
  WaitForTask(const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<TaskContext> ctx);
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  std::shared_ptr<TaskContext> ctx_;
};

// 배송(픽업 있음) 작업인지. CHARGE·RETURN은 목적지만 있다
class IsDelivery : public BT::ConditionNode
{
public:
  IsDelivery(const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<TaskContext> ctx);
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<TaskContext> ctx_;
};

// 충전 작업인지. 충전은 목적지(충전소)에서 도킹한다
class IsCharge : public BT::ConditionNode
{
public:
  IsCharge(const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<TaskContext> ctx);
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<TaskContext> ctx_;
};

// 작업에 그 스테이션 이름(pickup_station_id·dropoff_station_id)이 있는지. 없으면 도킹을 건너뛴다
class HasStation : public BT::ConditionNode
{
public:
  HasStation(const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<TaskContext> ctx);
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("station", "pickup 또는 dropoff")};
  }
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<TaskContext> ctx_;
};

// TaskStatus(IN_PROGRESS, phase)를 발행한다. 작업이 없으면(취소·종료 뒤) 보고하지 않고 FAILURE
class ReportPhase : public BT::SyncActionNode
{
public:
  ReportPhase(const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<TaskContext> ctx);
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("phase", "TaskStatus.phase (architecture.md 5-1절)")};
  }
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<TaskContext> ctx_;
};

// 이동을 멈춰야 하는지. 구역(zone_client/state)과 위치 복구(kidnap_recovery/state) 상태로 정한다.
// pause면 Nav2 목표를 취소하고 기다린다. abort면 기다려도 안 풀리는 상태라 바로 실패한다
struct PauseReason
{
  bool pause = false;
  bool abort = false;
  uint16_t code = 0;    // 오래 멈추거나(abort면 바로) 실패할 때의 ErrorInfo 코드
  std::string why;      // 로그·실패 설명
  double timeout_s = 0.0;   // 한 번 멈춘 채 이만큼 지나면 실패 (구역 pause_timeout_s, 위치 kidnap_pause_timeout_s)
};
PauseReason pause_reason(const TaskContext & ctx);

// 작업의 pickup_pose 또는 dropoff_pose로 Nav2 이동 (ctx.nav).
// - 멈춤 상태(pause_reason)면 목표를 취소하고 RUNNING으로 기다린다. 진행 상태가 resume_hold_s 이어지면
//   같은 목표를 다시 보낸다 (Nav2가 지금 위치에서 경로를 다시 짠다)
// - Nav2가 실패하면 nav_retries번 다시 보낸다. 그래도 실패하면 Nav2가 준 코드로 FAILURE
// - 멈춘 시간을 뺀 이동 시간이 nav_timeout_s를 넘으면 NAV_TIMEOUT, 한 번 멈춘 채 상한을 넘으면 멈춘 이유의
//   코드로 FAILURE (구역 pause_timeout_s → TRAFFIC_LEASE_TIMEOUT, 위치 kidnap_pause_timeout_s → LOCALIZATION_LOST)
// - 구역·위치 복구가 failed면 기다리지 않고 바로 FAILURE (구역 TRAFFIC_BACKING_FAILED, 위치 LOCALIZATION_LOST)
class MoveTo : public BT::StatefulActionNode
{
public:
  MoveTo(const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<TaskContext> ctx);
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("goal", "pickup 또는 dropoff")};
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus fail(uint16_t code, const std::string & message);
  void send(double now);

  std::shared_ptr<TaskContext> ctx_;
  std::string goal_name_;
  geometry_msgs::msg::Pose goal_;
  bool active_ = false;           // Nav2에 목표가 나가 있음. false면 멈춤
  int retries_ = 0;
  double moving_s_ = 0.0;         // 지난 진행 구간 시간 합
  double active_since_ = 0.0;
  double paused_since_ = 0.0;
  std::optional<double> go_since_;   // 멈춤 중 진행 상태가 시작된 시각
};

// 작업의 pickup_station_id 또는 dropoff_station_id에 한 번 도킹한다 (ctx.dock, Dock.action).
// 재시도는 트리의 RetryUntilSuccessful이 한다. 스테이션 이름이 비면 도킹할 곳이 없어 바로 SUCCESS
class Dock : public BT::StatefulActionNode
{
public:
  Dock(const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<TaskContext> ctx);
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("station", "pickup 또는 dropoff")};
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<TaskContext> ctx_;
  std::string station_id_;
  std::string attempt_key_;   // 작업·스테이션이 바뀌면 시도 횟수를 새로 센다
  int attempt_ = 0;
  bool active_ = false;
};

// seconds(비우면 ctx.dwell_s) 동안 RUNNING 뒤 SUCCESS. 작업이 없어지면 FAILURE.
// 싣기·내리기는 명세상 도킹 뒤 가상 적재·하역 이벤트라 시간만 흘린다
class Dwell : public BT::StatefulActionNode
{
public:
  Dwell(const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<TaskContext> ctx);
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("seconds", "걸리는 시간 [s]. 비우면 dwell_s")};
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  std::shared_ptr<TaskContext> ctx_;
  double until_ = 0.0;
};

// 충전소에 도킹한 뒤 배터리가 charge_target까지 찰 때까지 RUNNING (ctx.charging을 켠다).
// charge_timeout_s를 넘으면 FAILURE
class Charge : public BT::StatefulActionNode
{
public:
  Charge(const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<TaskContext> ctx);
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<TaskContext> ctx_;
  double since_ = 0.0;
};

// COMPLETED를 발행하고 작업을 비운다
class FinishTask : public BT::SyncActionNode
{
public:
  FinishTask(const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<TaskContext> ctx);
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<TaskContext> ctx_;
};

// FAILED(ctx.failure: 코드 + 설명)를 발행하고 작업을 비운다. 트리는 SUCCESS로 끝나 다음 작업을 기다린다.
// 작업이 이미 없으면(취소로 끝남) 아무것도 보내지 않는다
class FailTask : public BT::SyncActionNode
{
public:
  FailTask(const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<TaskContext> ctx);
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<TaskContext> ctx_;
};

// 위 노드를 factory에 등록한다
void register_nodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<TaskContext> ctx);

// 하던 작업(task_id)을 그 자리에서 취소한다: 트리를 멈추고 FAILED(error)를 보내고 작업을 비운다.
// 다음 tick을 기다리지 않으므로, 호출 뒤 바로 들어온 새 할당을 받을 수 있고 취소된 작업에 IN_PROGRESS가 더 나가지 않는다.
// 하던 작업이 아니면(이미 끝남·다른 작업) 아무것도 하지 않고 false
bool cancel_task(
  BT::Tree & tree, TaskContext & ctx, const std::string & task_id,
  const amr_msgs::msg::ErrorInfo & error);

// 작업 전체 시간 상한(task_timeout_s)을 넘었으면 cancel_task처럼 그 자리에서 끝낸다: FAILED(TASK_TIMEOUT).
// 매 tick 전에 부른다. 끝냈으면 true
bool expire_task(BT::Tree & tree, TaskContext & ctx);

// 서버 취소(ACTION_CANCEL)로 멈춘 작업의 FAILED 오류: ASSIGN_CANCELED + "canceled".
// 서버는 이 코드가 붙은 FAILED를 다시 할당하지 않는다 (#107)
amr_msgs::msg::ErrorInfo canceled_error();

}  // namespace amr_bt
