#include "amr_bt/check_battery.hpp"

#include <cstdio>
#include <utility>

#include "amr_msgs/msg/error_info.hpp"

namespace amr_bt
{

CheckBattery::CheckBattery(
  const std::string & name, const BT::NodeConfiguration & config,
  std::shared_ptr<TaskContext> ctx)
: BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus CheckBattery::tick()
{
  if (!ctx_->busy()) {
    return BT::NodeStatus::FAILURE;
  }
  const double min_percent = getInput<double>("min_percent").value_or(5.0);
  if (ctx_->battery_percent >= min_percent) {
    return BT::NodeStatus::SUCCESS;
  }
  const std::string on_low = getInput<std::string>("on_low").value_or("fail");
  if (on_low != "fail" && on_low != "warn") {
    throw BT::RuntimeError("CheckBattery: on_low는 fail 또는 warn: ", on_low);
  }
  char message[96];
  std::snprintf(message, sizeof(message), "battery %.1f%% below %.1f%% before %s",
    ctx_->battery_percent, min_percent, ctx_->phase.c_str());
  if (on_low == "warn") {
    // BT 노드 라이브러리는 rclcpp를 쓰지 않아 stderr로 남긴다 (launch 출력에 그대로 나온다)
    std::fprintf(stderr, "[amr_bt] %s: %s, continuing (already loaded)\n",
      ctx_->task->task_id.c_str(), message);
    return BT::NodeStatus::SUCCESS;
  }
  ctx_->fail(amr_msgs::msg::ErrorInfo::ASSIGN_BATTERY_LOW, message);
  return BT::NodeStatus::FAILURE;
}

}  // namespace amr_bt
