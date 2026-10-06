// amr_bt의 BehaviorTree.CPP(v3) 노드. 트리는 behavior_trees/task_cycle.xml.
#pragma once

#include <memory>
#include <string>

#include "amr_bt/task_context.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace amr_bt
{

// 수락한 작업이 생길 때까지 RUNNING. 생기면 집어 들고 SUCCESS
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

// TaskStatus(IN_PROGRESS, phase)를 발행한다
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

// 자리채움: seconds 동안 RUNNING 뒤 SUCCESS. 취소 요청이 오면 FAILURE.
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

// FAILED(ctx.failure)를 발행하고 작업을 비운다. 트리는 SUCCESS로 끝나 다음 작업을 기다린다
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

}  // namespace amr_bt
