// CheckBattery(BT 확장 예시 노드)를 task_cycle.xml 안에서 ROS 없이 돌린다.
// 이동·도킹은 시간만 흘리는 make_stub_port로 대신하고, 이동 목표를 보낸 횟수만 센다
#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "amr_bt/bt_nodes.hpp"
#include "amr_bt/task_context.hpp"
#include "amr_msgs/msg/error_info.hpp"
#include "amr_msgs/msg/task.hpp"
#include "amr_msgs/msg/task_status.hpp"

using amr_msgs::msg::ErrorInfo;
using amr_msgs::msg::Task;
using amr_msgs::msg::TaskStatus;

namespace
{

class CheckBatteryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ctx = std::make_shared<amr_bt::TaskContext>();
    ctx->now = [this]() {return clock;};
    ctx->report = [this](uint8_t state, const std::string & phase, const ErrorInfo & error, bool) {
        events.push_back(std::to_string(state) + ":" + phase);
        last_code = error.code;
      };
    auto nav = amr_bt::make_stub_port<geometry_msgs::msg::Pose>(ctx->now, 5.0);
    auto start = nav.start;
    nav.start = [this, start](const geometry_msgs::msg::Pose & goal) {
        ++nav_starts;
        start(goal);
      };
    ctx->nav = nav;
    ctx->dock = amr_bt::make_stub_port<std::string>(ctx->now, 1.0);
    ctx->dwell_s = 2.0;
    amr_bt::register_nodes(factory, ctx);
    tree = factory.createTreeFromFile(TREE_FILE);
  }

  // dt마다 tick. 작업이 끝나면(busy 해제) 멈춘다. on_tick은 tick마다 부른다 (배터리를 도중에 바꿀 때)
  void run(double seconds, const std::function<void()> & on_tick = {})
  {
    for (double t = 0; t < seconds; t += 0.1) {
      if (on_tick) {
        on_tick();
      }
      tree.tickRoot();
      clock += 0.1;
      if (started && !ctx->busy()) {
        return;
      }
    }
  }

  void assign_delivery()
  {
    Task t;
    t.task_id = "task_1";
    t.task_type = Task::TYPE_DELIVERY;
    t.pickup_station_id = "dock_1";
    t.dropoff_station_id = "rack_a1";
    ctx->assign(t);
    started = true;
  }

  double clock = 0.0;
  int nav_starts = 0;
  bool started = false;
  uint16_t last_code = 0;
  std::shared_ptr<amr_bt::TaskContext> ctx;
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  std::vector<std::string> events;
};

const std::string P = std::to_string(TaskStatus::STATE_IN_PROGRESS) + ":";
const std::string DONE = std::to_string(TaskStatus::STATE_COMPLETED) + ":";
const std::string FAILED = std::to_string(TaskStatus::STATE_FAILED) + ":";

TEST_F(CheckBatteryTest, EnoughBatteryCompletesDelivery)
{
  ctx->battery_percent = 50.0;
  assign_delivery();
  run(60.0);
  const std::vector<std::string> expected = {
    P + "move_to_pickup", P + "docking", P + "loading",
    P + "move_to_dropoff", P + "docking", P + "unloading", DONE};
  EXPECT_EQ(events, expected);
  EXPECT_EQ(nav_starts, 2);
}

// 받은 뒤 배터리가 이미 5 % 아래면 픽업으로 출발하지 않고 ASSIGN_BATTERY_LOW로 끝낸다
TEST_F(CheckBatteryTest, LowBatteryFailsBeforeMoving)
{
  ctx->battery_percent = 3.0;
  assign_delivery();
  run(10.0);
  const std::vector<std::string> expected = {P + "move_to_pickup", FAILED + "move_to_pickup"};
  EXPECT_EQ(events, expected);
  EXPECT_EQ(last_code, ErrorInfo::ASSIGN_BATTERY_LOW);
  EXPECT_EQ(nav_starts, 0);
  EXPECT_FALSE(ctx->busy());
}

// 픽업까지는 충분했는데 싣는 사이 떨어지면 하역 이동 전에 끝낸다 (작업을 받을 때의 거부로는 못 잡는 경우)
TEST_F(CheckBatteryTest, BatteryDropDuringTaskFailsBeforeDropoff)
{
  ctx->battery_percent = 50.0;
  assign_delivery();
  run(60.0, [this]() {
      if (ctx->phase == "loading") {
        ctx->battery_percent = 4.0;
      }
    });
  const std::vector<std::string> expected = {
    P + "move_to_pickup", P + "docking", P + "loading", P + "move_to_dropoff", FAILED + "move_to_dropoff"};
  EXPECT_EQ(events, expected);
  EXPECT_EQ(last_code, ErrorInfo::ASSIGN_BATTERY_LOW);
  EXPECT_EQ(nav_starts, 1);
}

// 충전 작업은 VisitStation을 지나지 않아 배터리가 낮아도 충전소로 간다
TEST_F(CheckBatteryTest, ChargeTaskIgnoresLowBattery)
{
  ctx->battery_percent = 3.0;
  ctx->charge_target = 3.0;
  Task t;
  t.task_id = "charge_1";
  t.task_type = Task::TYPE_CHARGE;
  t.dropoff_station_id = "charger_c1";
  ctx->assign(t);
  started = true;
  run(30.0);
  EXPECT_EQ(events.back(), DONE);
  EXPECT_EQ(nav_starts, 1);
}

}  // namespace
