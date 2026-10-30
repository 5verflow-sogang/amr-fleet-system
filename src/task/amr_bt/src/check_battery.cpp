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
  char message[96];
  std::snprintf(message, sizeof(message), "battery %.1f%% below %.1f%% before %s",
    ctx_->battery_percent, min_percent, ctx_->phase.c_str());
  ctx_->fail(amr_msgs::msg::ErrorInfo::ASSIGN_BATTERY_LOW, message);
  return BT::NodeStatus::FAILURE;
}

}  // namespace amr_bt
