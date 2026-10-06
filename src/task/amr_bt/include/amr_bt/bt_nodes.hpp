// amr_bt의 BehaviorTree.CPP(v3) 노드. 트리는 behavior_trees/task_cycle.xml.
#pragma once

#include <memory>
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

// 자리채움: seconds 동안 RUNNING 뒤 SUCCESS. 작업이 없어지면 FAILURE.
// MoveTo(이동)·Dwell(싣기·내리기)이 이 동작을 쓴다. Nav2 navigate_to_pose·docking/dock을 붙이면 바꾼다
class StubWait : public BT::StatefulActionNode
{
public:
  StubWait(const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<TaskContext> ctx, double TaskContext::* default_seconds);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("goal", "", "pickup 또는 dropoff (MoveTo만, 로그용)"),
      BT::InputPort<double>("seconds", "걸리는 시간 [s]. 비우면 stub_move_s / stub_dwell_s")};
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  std::shared_ptr<TaskContext> ctx_;
  double TaskContext::* default_seconds_;
  double until_ = 0.0;
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

// 위 노드를 factory에 등록한다 (MoveTo, Dwell은 StubWait)
void register_nodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<TaskContext> ctx);

// 하던 작업(task_id)을 그 자리에서 취소한다: 트리를 멈추고 FAILED(error)를 보내고 작업을 비운다.
// 다음 tick을 기다리지 않으므로, 호출 뒤 바로 들어온 새 할당을 받을 수 있고 취소된 작업에 IN_PROGRESS가 더 나가지 않는다.
// 하던 작업이 아니면(이미 끝남·다른 작업) 아무것도 하지 않고 false
bool cancel_task(
  BT::Tree & tree, TaskContext & ctx, const std::string & task_id,
  const amr_msgs::msg::ErrorInfo & error);

}  // namespace amr_bt
