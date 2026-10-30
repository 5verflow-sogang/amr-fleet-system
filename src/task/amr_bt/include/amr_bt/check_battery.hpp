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

// 배터리(ctx.battery_percent)가 min_percent 이상이면 SUCCESS. 낮으면 on_low에 따라
// - fail: ASSIGN_BATTERY_LOW로 실패 원인을 남기고 FAILURE → FailTask가 FAILED를 보내고 서버가 다시 할당한다.
//   아직 싣기 전(픽업 쪽)에만 쓴다
// - warn: 경고만 남기고 SUCCESS. 이미 실은 뒤(하역 쪽)에 실패하면 서버가 작업을 픽업부터 다른 로봇에 다시 줘서
//   같은 물건을 두 번 싣게 되므로, 실린 물건은 끝까지 배송한다 (#253 하연 리뷰)
// 작업을 받을 때의 거부(battery.reject_below_percent 10 %)가 먼저 걸러서 실제로 걸리는 경우는 드물다
class CheckBattery : public BT::ConditionNode
{
public:
  CheckBattery(const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<TaskContext> ctx);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_percent", 5.0, "이보다 낮으면 on_low대로 [%]"),
      BT::InputPort<std::string>("on_low", "fail", "fail(실패, 싣기 전) 또는 warn(경고만, 실은 뒤)")};
  }
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<TaskContext> ctx_;
};

}  // namespace amr_bt
