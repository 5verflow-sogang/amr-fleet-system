#include "amr_bt/bt_nodes.hpp"

#include "amr_msgs/msg/task_status.hpp"

namespace amr_bt
{

using amr_msgs::msg::ErrorInfo;
using amr_msgs::msg::TaskStatus;

WaitForTask::WaitForTask(
  const std::string & name, const BT::NodeConfiguration & config,
  std::shared_ptr<TaskContext> ctx)
: BT::StatefulActionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus WaitForTask::onStart() {return onRunning();}

BT::NodeStatus WaitForTask::onRunning()
{
  return ctx_->busy() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

IsDelivery::IsDelivery(
  const std::string & name, const BT::NodeConfiguration & config,
  std::shared_ptr<TaskContext> ctx)
: BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus IsDelivery::tick()
{
  return ctx_->is_delivery() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

ReportPhase::ReportPhase(
  const std::string & name, const BT::NodeConfiguration & config,
  std::shared_ptr<TaskContext> ctx)
: BT::SyncActionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus ReportPhase::tick()
{
  if (!ctx_->busy()) {
    return BT::NodeStatus::FAILURE;
  }
  auto phase = getInput<std::string>("phase");
  if (!phase) {
    throw BT::RuntimeError("ReportPhase: phase 포트가 없다: ", phase.error());
  }
  ctx_->phase = phase.value();
  ctx_->report(TaskStatus::STATE_IN_PROGRESS, ctx_->phase, ErrorInfo(), false);
  return BT::NodeStatus::SUCCESS;
}

StubWait::StubWait(
  const std::string & name, const BT::NodeConfiguration & config,
  std::shared_ptr<TaskContext> ctx, double TaskContext::* default_seconds)
: BT::StatefulActionNode(name, config), ctx_(std::move(ctx)), default_seconds_(default_seconds) {}

BT::NodeStatus StubWait::onStart()
{
  const auto seconds = getInput<double>("seconds");
  until_ = ctx_->now() + (seconds ? seconds.value() : (*ctx_).*default_seconds_);
  return onRunning();
}

BT::NodeStatus StubWait::onRunning()
{
  if (!ctx_->busy()) {
    return BT::NodeStatus::FAILURE;
  }
  return ctx_->now() >= until_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

FinishTask::FinishTask(
  const std::string & name, const BT::NodeConfiguration & config,
  std::shared_ptr<TaskContext> ctx)
: BT::SyncActionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus FinishTask::tick()
{
  ctx_->report(TaskStatus::STATE_COMPLETED, "", ErrorInfo(), true);
  ctx_->clear();
  return BT::NodeStatus::SUCCESS;
}

FailTask::FailTask(
  const std::string & name, const BT::NodeConfiguration & config,
  std::shared_ptr<TaskContext> ctx)
: BT::SyncActionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus FailTask::tick()
{
  if (!ctx_->busy()) {
    return BT::NodeStatus::SUCCESS;
  }
  // 코드는 실패한 동작 노드가 ctx.fail()로 남긴 값. 원인을 모르면 NONE
  ErrorInfo error = ctx_->failure;
  if (error.message.empty()) {
    error.message = "failed in phase " + ctx_->phase;
  }
  ctx_->report(TaskStatus::STATE_FAILED, ctx_->phase, error, true);
  ctx_->clear();
  return BT::NodeStatus::SUCCESS;
}

bool cancel_task(
  BT::Tree & tree, TaskContext & ctx, const std::string & task_id, const ErrorInfo & error)
{
  if (!ctx.busy() || ctx.task->task_id != task_id) {
    return false;
  }
  // 멈춘 뒤 다음 tick은 WaitForTask부터 다시 시작한다 (진행 중이던 MoveTo 등은 halt)
  tree.haltTree();
  ctx.report(TaskStatus::STATE_FAILED, ctx.phase, error, true);
  ctx.clear();
  return true;
}

ErrorInfo canceled_error()
{
  ErrorInfo error;
  error.code = ErrorInfo::ASSIGN_CANCELED;
  error.message = "canceled";
  return error;
}

namespace
{

// ctx를 생성자로 넘기는 노드 등록
template<typename T>
void register_with_context(
  BT::BehaviorTreeFactory & factory, const std::string & id, std::shared_ptr<TaskContext> ctx)
{
  BT::NodeBuilder builder = [ctx](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<T>(name, config, ctx);
    };
  factory.registerBuilder<T>(id, builder);
}

}  // namespace

void register_nodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<TaskContext> ctx)
{
  register_with_context<WaitForTask>(factory, "WaitForTask", ctx);
  register_with_context<IsDelivery>(factory, "IsDelivery", ctx);
  register_with_context<ReportPhase>(factory, "ReportPhase", ctx);
  register_with_context<FinishTask>(factory, "FinishTask", ctx);
  register_with_context<FailTask>(factory, "FailTask", ctx);

  auto stub = [&factory, ctx](const std::string & id, double TaskContext::* seconds) {
      BT::NodeBuilder builder =
        [ctx, seconds](const std::string & name, const BT::NodeConfiguration & config) {
          return std::make_unique<StubWait>(name, config, ctx, seconds);
        };
      factory.registerBuilder<StubWait>(id, builder);
    };
  stub("MoveTo", &TaskContext::stub_move_s);
  stub("Dwell", &TaskContext::stub_dwell_s);
}

}  // namespace amr_bt
