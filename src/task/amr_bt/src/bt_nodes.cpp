#include "amr_bt/bt_nodes.hpp"

#include "amr_msgs/action/dock.hpp"
#include "amr_msgs/msg/task_status.hpp"

namespace amr_bt
{

using amr_msgs::msg::ErrorInfo;
using amr_msgs::msg::TaskStatus;
using DockResult = amr_msgs::action::Dock::Result;

namespace
{

// MoveTo·Dock의 goal/station 포트 값 ("pickup" 또는 "dropoff")
std::string station_port(const BT::TreeNode & node, const char * port)
{
  auto value = node.getInput<std::string>(port);
  if (!value || (value.value() != "pickup" && value.value() != "dropoff")) {
    throw BT::RuntimeError(node.name(), ": ", port, " 포트는 pickup 또는 dropoff");
  }
  return value.value();
}

// 작업의 스테이션 이름 (which: pickup·dropoff)
const std::string & station_id(const TaskContext & ctx, const std::string & which)
{
  return which == "pickup" ? ctx.task->pickup_station_id : ctx.task->dropoff_station_id;
}

}  // namespace

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

IsCharge::IsCharge(
  const std::string & name, const BT::NodeConfiguration & config,
  std::shared_ptr<TaskContext> ctx)
: BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus IsCharge::tick()
{
  const bool charge = ctx_->busy() && ctx_->task->task_type == amr_msgs::msg::Task::TYPE_CHARGE;
  return charge ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

HasStation::HasStation(
  const std::string & name, const BT::NodeConfiguration & config,
  std::shared_ptr<TaskContext> ctx)
: BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus HasStation::tick()
{
  if (!ctx_->busy()) {
    return BT::NodeStatus::FAILURE;
  }
  return station_id(*ctx_, station_port(*this, "station")).empty() ?
         BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
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

PauseReason pause_reason(const TaskContext & ctx)
{
  PauseReason r;
  // zone_client/state (#137): waiting(허가 대기)·blocked(거부)·backing(회수돼 후진 중)이면 멈춘다.
  // failed는 후진 안전 한도를 넘어 그 자리에 선 상태라 기다려도 안 풀린다
  const auto & zone = ctx.zone_state;
  if (zone == "failed") {
    r.pause = r.abort = true;
    r.code = ErrorInfo::TRAFFIC_REVOKED;
    r.why = "zone_client failed (backing safety stop)";
    return r;
  }
  if (zone == "waiting" || zone == "blocked" || zone == "backing") {
    r.pause = true;
    r.code = ErrorInfo::TRAFFIC_LEASE_TIMEOUT;
    r.why = "zone " + zone;
    return r;
  }
  // kidnap_recovery/state (#79): tracking이 아니면 위치를 다시 잡는 중이라 멈춘다.
  // failed도 위치가 다시 잡히면 tracking으로 돌아오므로 abort가 아니라 pause_timeout_s까지 기다린다
  if (ctx.kidnap_state != "tracking") {
    r.pause = true;
    r.code = ErrorInfo::NAV_TIMEOUT;
    r.why = "localization " + ctx.kidnap_state;
  }
  return r;
}

MoveTo::MoveTo(
  const std::string & name, const BT::NodeConfiguration & config,
  std::shared_ptr<TaskContext> ctx)
: BT::StatefulActionNode(name, config), ctx_(std::move(ctx)) {}

void MoveTo::send(double now)
{
  ctx_->nav.start(goal_);
  active_ = true;
  active_since_ = now;
}

BT::NodeStatus MoveTo::fail(uint16_t code, const std::string & message)
{
  if (active_) {
    ctx_->nav.cancel();
    active_ = false;
  }
  ctx_->fail(code, "move to " + goal_name_ + ": " + message);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveTo::onStart()
{
  if (!ctx_->busy()) {
    return BT::NodeStatus::FAILURE;
  }
  goal_name_ = station_port(*this, "goal");
  goal_ = goal_name_ == "pickup" ? ctx_->task->pickup_pose : ctx_->task->dropoff_pose;
  retries_ = 0;
  moving_s_ = 0.0;
  go_since_.reset();
  const double now = ctx_->now();
  if (pause_reason(*ctx_).pause) {
    active_ = false;
    paused_since_ = now;
  } else {
    send(now);
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveTo::onRunning()
{
  if (!ctx_->busy()) {
    return fail(ErrorInfo::NONE, "task cleared");
  }
  const double now = ctx_->now();
  const PauseReason reason = pause_reason(*ctx_);

  if (active_) {
    if (reason.pause) {
      // 멈춤: Nav2 목표를 취소한다. 진행 상태로 돌아오면 같은 목표를 다시 보낸다
      ctx_->nav.cancel();
      active_ = false;
      moving_s_ += now - active_since_;
      paused_since_ = now;
      go_since_.reset();
      return BT::NodeStatus::RUNNING;
    }
    const ActionStatus s = ctx_->nav.poll();
    if (s.state == ActionStatus::kSucceeded) {
      active_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    if (s.state == ActionStatus::kFailed) {
      active_ = false;
      if (retries_ < ctx_->nav_retries) {
        ++retries_;
        send(now);   // 지난 진행 시간은 그대로 쌓는다 (재시도가 타임아웃을 늘리지 않게)
        return BT::NodeStatus::RUNNING;
      }
      const uint16_t code = s.code != ErrorInfo::NONE ? s.code : ErrorInfo::NAV_NO_PATH;
      return fail(code, s.message + " (" + std::to_string(retries_ + 1) + " attempts)");
    }
    if (moving_s_ + (now - active_since_) > ctx_->nav_timeout_s) {
      return fail(ErrorInfo::NAV_TIMEOUT, "not reached in " + std::to_string(ctx_->nav_timeout_s) + " s");
    }
    return BT::NodeStatus::RUNNING;
  }

  // 멈춤 중
  if (reason.abort) {
    return fail(reason.code, reason.why);
  }
  if (now - paused_since_ > ctx_->pause_timeout_s) {
    return fail(
      reason.pause ? reason.code : ErrorInfo::NAV_TIMEOUT,
      "paused over " + std::to_string(ctx_->pause_timeout_s) + " s (" +
      (reason.pause ? reason.why : "resuming") + ")");
  }
  if (reason.pause) {
    go_since_.reset();
    return BT::NodeStatus::RUNNING;
  }
  // 진행 상태가 resume_hold_s 이어져야 다시 보낸다. 후진 → 반납 → 재요청 사이 한 틱 idle 같은 것을 거른다
  if (!go_since_) {
    go_since_ = now;
  }
  if (now - *go_since_ >= ctx_->resume_hold_s) {
    send(now);
  }
  return BT::NodeStatus::RUNNING;
}

void MoveTo::onHalted()
{
  if (active_) {
    ctx_->nav.cancel();
    active_ = false;
  }
}

Dock::Dock(
  const std::string & name, const BT::NodeConfiguration & config,
  std::shared_ptr<TaskContext> ctx)
: BT::StatefulActionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus Dock::onStart()
{
  if (!ctx_->busy()) {
    return BT::NodeStatus::FAILURE;
  }
  const auto which = station_port(*this, "station");
  station_id_ = station_id(*ctx_, which);
  if (station_id_.empty()) {
    return BT::NodeStatus::SUCCESS;   // 스테이션 이름 없이 자세만 받은 작업: 도킹할 곳이 없다
  }
  const auto key = ctx_->task->task_id + "/" + which;
  if (key != attempt_key_) {
    attempt_key_ = key;
    attempt_ = 0;
  }
  ++attempt_;
  ctx_->dock.start(station_id_);
  active_ = true;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Dock::onRunning()
{
  if (!ctx_->busy()) {
    onHalted();
    return BT::NodeStatus::FAILURE;
  }
  const ActionStatus s = ctx_->dock.poll();
  if (s.state == ActionStatus::kRunning) {
    return BT::NodeStatus::RUNNING;
  }
  active_ = false;
  if (s.state == ActionStatus::kSucceeded) {
    return BT::NodeStatus::SUCCESS;
  }
  // 900번대는 Dock.action error_code 그대로 (ErrorInfo 900번대와 같은 값)
  const uint16_t code = s.code != ErrorInfo::NONE ? s.code : DockResult::UNKNOWN;
  ctx_->fail(
    code, "docking at " + station_id_ + " failed " + std::to_string(attempt_) + " times: " +
    s.message);
  return BT::NodeStatus::FAILURE;
}

void Dock::onHalted()
{
  if (active_) {
    ctx_->dock.cancel();
    active_ = false;
  }
}

Dwell::Dwell(
  const std::string & name, const BT::NodeConfiguration & config,
  std::shared_ptr<TaskContext> ctx)
: BT::StatefulActionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus Dwell::onStart()
{
  const auto seconds = getInput<double>("seconds");
  until_ = ctx_->now() + (seconds ? seconds.value() : ctx_->dwell_s);
  return onRunning();
}

BT::NodeStatus Dwell::onRunning()
{
  if (!ctx_->busy()) {
    return BT::NodeStatus::FAILURE;
  }
  return ctx_->now() >= until_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

Charge::Charge(
  const std::string & name, const BT::NodeConfiguration & config,
  std::shared_ptr<TaskContext> ctx)
: BT::StatefulActionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus Charge::onStart()
{
  if (!ctx_->busy()) {
    return BT::NodeStatus::FAILURE;
  }
  ctx_->charging = true;
  since_ = ctx_->now();
  return onRunning();
}

BT::NodeStatus Charge::onRunning()
{
  if (!ctx_->busy()) {
    ctx_->charging = false;
    return BT::NodeStatus::FAILURE;
  }
  if (ctx_->battery_percent >= ctx_->charge_target) {
    ctx_->charging = false;
    return BT::NodeStatus::SUCCESS;
  }
  if (ctx_->now() - since_ > ctx_->charge_timeout_s) {
    ctx_->charging = false;
    ctx_->fail(ErrorInfo::NONE, "charging not done in " + std::to_string(ctx_->charge_timeout_s) + " s");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void Charge::onHalted()
{
  ctx_->charging = false;
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
  // 멈춘 뒤 다음 tick은 WaitForTask부터 다시 시작한다 (진행 중이던 MoveTo·Dock은 halt에서 목표 취소)
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
  register_with_context<IsCharge>(factory, "IsCharge", ctx);
  register_with_context<HasStation>(factory, "HasStation", ctx);
  register_with_context<ReportPhase>(factory, "ReportPhase", ctx);
  register_with_context<MoveTo>(factory, "MoveTo", ctx);
  register_with_context<Dock>(factory, "Dock", ctx);
  register_with_context<Dwell>(factory, "Dwell", ctx);
  register_with_context<Charge>(factory, "Charge", ctx);
  register_with_context<FinishTask>(factory, "FinishTask", ctx);
  register_with_context<FailTask>(factory, "FailTask", ctx);
}

}  // namespace amr_bt
