// BT 확장 예시 노드: 스테이션으로 이동하기 전 배터리 확인 (명세 KPI "BT 확장 시 기존 코드 수정 3개 파일 이내").
// 노드를 이 파일과 check_battery.cpp에 따로 두고, 기존 파일은 CMakeLists.txt(소스)·bt_nodes.cpp(등록)·
// task_cycle.xml(트리)만 고친다 (README "노드 추가로 확장하기").
#pragma once

#include <memory>
#include <string>

#include "amr_bt/task_context.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace amr_bt
{

// 배터리(ctx.battery_percent)가 min_percent 이상이면 SUCCESS. 낮으면 ASSIGN_BATTERY_LOW로 실패 원인을 남기고 FAILURE
// → FailTask가 FAILED를 보내고 서버가 다른 로봇에 다시 할당한다. 작업을 받을 때의 거부(battery.reject_below_percent
// 10 %)는 받는 순간만 보므로, 긴 배송 도중 배터리가 떨어지는 경우를 여기서 막는다
class CheckBattery : public BT::ConditionNode
{
public:
  CheckBattery(const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<TaskContext> ctx);
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("min_percent", 5.0, "이보다 낮으면 이동하지 않고 실패 [%]")};
  }
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<TaskContext> ctx_;
};

}  // namespace amr_bt
